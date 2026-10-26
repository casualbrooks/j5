"""Exchange replay reviews with stronger computers without trusting model output as laps."""

import asyncio
import hashlib
import json
import uuid
from datetime import datetime, timezone
from typing import Literal

import aiosqlite
from fastapi import HTTPException
from pydantic import BaseModel, Field, model_validator

from app import database
from app.camera import Camera


class ProposedCrossing(BaseModel):
    racer_id: str
    second: float = Field(ge=0, le=86400)
    confidence: float = Field(ge=0, le=1)
    evidence_seconds: list[float] = Field(min_length=2, max_length=15)


class ObservedPass(BaseModel):
    """A visible directed pass that has not necessarily been assigned to a racer."""

    second: float = Field(ge=0, le=86400)
    direction: Literal["outward", "return"]
    x: float = Field(ge=0, le=1)
    y: float = Field(ge=0, le=1)
    confidence: float = Field(ge=0, le=1)
    tracklet_id: int = Field(ge=1)
    appearance_cluster: int | None = Field(default=None, ge=0)


class ReviewGate(BaseModel):
    finish_x: float = Field(ge=0.1, le=0.9)
    y_min: float = Field(ge=0, le=1)
    y_max: float = Field(ge=0, le=1)
    direction: Literal[-1, 1]

    @model_validator(mode="after")
    def ordered_y(self):
        if self.y_min >= self.y_max:
            raise ValueError("Gate top must be above its bottom")
        return self


class ReviewProposal(BaseModel):
    race_id: str
    video_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    model: str = Field(min_length=1, max_length=160)
    analysis_gate: ReviewGate | None = None
    crossings: list[ProposedCrossing] = Field(max_length=10000)
    observations: list[ObservedPass] = Field(default_factory=list, max_length=10000)
    unassigned_crossings: int = Field(ge=0)
    notes: str = Field(default="", max_length=2000)

    @model_validator(mode="after")
    def sorted_crossings(self):
        seconds = [event.second for event in self.crossings]
        if seconds != sorted(seconds):
            raise ValueError("Crossings must be sorted by video time")
        observed_seconds = [event.second for event in self.observations]
        if observed_seconds != sorted(observed_seconds):
            raise ValueError("Observations must be sorted by video time")
        return self


class GroupAssignment(BaseModel):
    appearance_cluster: int = Field(ge=0)
    racer_id: str | None = None


class TrackPoint(BaseModel):
    second: float = Field(ge=0, le=86400)
    x: float = Field(ge=0, le=1)
    y: float = Field(ge=0, le=1)
    observed: bool = True


class TrackTrace(BaseModel):
    track_id: int = Field(ge=1)
    crossing_second: float = Field(ge=0, le=86400)
    appearance_cluster: int | None = Field(default=None, ge=0)
    assigned_racer_id: str | None = None
    excluded: bool = False
    points: list[TrackPoint] = Field(min_length=2, max_length=500)

    @model_validator(mode="after")
    def ordered_points(self):
        seconds = [point.second for point in self.points]
        if any(later <= earlier for earlier, later in zip(seconds, seconds[1:])):
            raise ValueError("Track points must be ordered by video time")
        return self


class TrackingPreview(BaseModel):
    race_id: str
    video_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    source: str = Field(min_length=1, max_length=160)
    analysis_gate: ReviewGate
    tracks: list[TrackTrace] = Field(max_length=1000)

    @model_validator(mode="after")
    def bounded_points(self):
        if len({track.track_id for track in self.tracks}) != len(self.tracks):
            raise ValueError("Track IDs must be unique")
        if sum(len(track.points) for track in self.tracks) > 50000:
            raise ValueError("Tracking preview has too many points")
        return self


class TrackAssignment(BaseModel):
    track_id: int = Field(ge=1)
    racer_id: str | None = None
    excluded: bool = False


def gate_matches(proposed, saved):
    return proposed is not None and all(
        abs(proposed[key] - saved[key]) < 0.00001
        for key in ("finish_x", "y_min", "y_max", "direction")
    )


def finish_observation(event, gate):
    return (
        event["direction"] == "outward"
        and gate["y_min"] <= event["y"] <= gate["y_max"]
        and abs(event["x"] - gate["finish_x"]) <= 0.02
    )


async def _state(key):
    row = await database.get_row("system_state", key, id_column="key")
    return json.loads(row["value"]) if row else None


def _video_hash(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _lap_checksum(laps):
    return hashlib.sha256(
        json.dumps(
            sorted(
                (row["racer_profile_id"], row["lap_number"], row["lap_time"])
                for row in laps
            ),
            separators=(",", ":"),
        ).encode()
    ).hexdigest()


async def manifest(race_id):
    """Return an immutable video identity and the saved configuration for a race."""
    race = await database.get_row("races", race_id)
    if not race:
        raise HTTPException(404, "Race not found")
    context = await _state(f"race_context:{race_id}") or {}
    current = await _state("race_context") or {}
    if not context and current.get("race_id") == race_id:
        context = current
    config = await _state(f"quick_config:{race_id}") or {}
    if not config and current.get("race_id") == race_id:
        config = await _state("quick_config") or {}
    source = config.get("source", "")
    if not context.get("server_timing") or not source.startswith("video:"):
        raise HTTPException(409, "This race has no saved replay video")
    try:
        path = Camera.video_path(source[6:])
    except ValueError as exc:
        raise HTTPException(409, str(exc)) from exc
    if not path.is_file():
        raise HTTPException(404, "Saved replay video is missing")
    digest = await asyncio.to_thread(_video_hash, path)
    laps = await database.get_all_rows("lap_records", {"race_id": race_id})
    baseline = _lap_checksum(laps)
    return {
        "race_id": race_id,
        "race_status": race["status"],
        "video": path.name,
        "video_sha256": digest,
        "video_bytes": path.stat().st_size,
        "video_start_s": config.get("video_start_s", 0),
        "video_end_s": config.get("video_end_s"),
        "finish_gate": {
            key: config[key] for key in ("finish_x", "y_min", "y_max", "direction")
        },
        "minimum_lap_ms": config.get("minimum_lap_ms", 2000),
        "total_laps": race["total_laps"],
        "racers": context.get("racers", []),
        "recorded_laps": {
            racer["id"]: sum(row["racer_profile_id"] == racer["id"] for row in laps)
            for racer in context.get("racers", [])
        },
        "baseline_sha256": baseline,
        "path": path,
    }


def proposal_summary(data, info):
    counts = {racer["id"]: 0 for racer in info["racers"]}
    for event in data.crossings:
        counts[event.racer_id] += 1
    return {
        "proposed_laps": {racer: max(0, count - 1) for racer, count in counts.items()},
        "unassigned_crossings": data.unassigned_crossings,
        "observed_outward": sum(
            event.direction == "outward" for event in data.observations
        ),
        "observed_return": sum(
            event.direction == "return" for event in data.observations
        ),
        "minimum_confidence": min(
            (event.confidence for event in data.crossings), default=0
        ),
        "complete": bool(data.crossings)
        and data.unassigned_crossings == 0
        and all(count >= 2 for count in counts.values())
        and all(event.confidence >= 0.9 for event in data.crossings),
    }


def candidate_summary(proposal, info):
    """Preview mapped passes as tentative laps, never as ledger entries."""
    assignments = proposal.get("group_assignments", {})
    minimum = info["minimum_lap_ms"] / 1000
    gate = info["finish_gate"]
    accepted = {racer["id"]: [] for racer in info["racers"]}
    ignored_close = {racer["id"]: 0 for racer in info["racers"]}
    unassigned = 0
    for event in proposal.get("observations", []):
        if not finish_observation(event, gate):
            continue
        racer_id = assignments.get(str(event.get("appearance_cluster")))
        if racer_id not in accepted:
            unassigned += 1
            continue
        times = accepted[racer_id]
        if times and event["second"] - times[-1] < minimum:
            ignored_close[racer_id] += 1
        else:
            times.append(event["second"])
    return {
        "candidate_crossings": {racer: len(times) for racer, times in accepted.items()},
        "candidate_laps": {
            racer: max(0, len(times) - 1) for racer, times in accepted.items()
        },
        "ignored_close_passes": ignored_close,
        "unassigned_passes": unassigned,
        "mapped_racers": sorted(set(assignments.values())),
    }


async def submit_proposal(data):
    info = await manifest(data.race_id)
    if info["race_status"] == "active":
        raise HTTPException(409, "Pause the race before reviewing its video")
    if data.video_sha256 != info["video_sha256"]:
        raise HTTPException(409, "Proposal video does not match the saved replay")
    if data.analysis_gate and not gate_matches(
        data.analysis_gate.model_dump(), info["finish_gate"]
    ):
        raise HTTPException(409, "Analysis gate does not match the saved finish line")
    roster = {racer["id"] for racer in info["racers"]}
    if any(event.racer_id not in roster for event in data.crossings):
        raise HTTPException(422, "Proposal contains a racer outside this race")
    start = info["video_start_s"]
    end = info["video_end_s"] or 86400
    if any(not start <= event.second <= end for event in data.observations):
        raise HTTPException(422, "Observed pass is outside the race interval")
    previous = {}
    counts = {racer: 0 for racer in roster}
    for event in data.crossings:
        if not start <= event.second <= end or any(
            not start <= second <= end for second in event.evidence_seconds
        ):
            raise HTTPException(422, "Proposal evidence is outside the race interval")
        earlier = previous.get(event.racer_id)
        if (
            earlier is not None
            and (event.second - earlier) * 1000 < info["minimum_lap_ms"]
        ):
            raise HTTPException(422, "A proposed lap is shorter than the saved minimum")
        previous[event.racer_id] = event.second
        counts[event.racer_id] += 1
        if counts[event.racer_id] > info["total_laps"] + 1:
            raise HTTPException(422, "Proposal exceeds the race lap target")
    summary = proposal_summary(data, info)
    proposal = {
        "id": str(uuid.uuid4()),
        "received_at": datetime.now(timezone.utc).isoformat(),
        "baseline_sha256": info["baseline_sha256"],
        "status": "ready" if summary["complete"] else "incomplete",
        "summary": summary,
        "group_assignments": {},
        **data.model_dump(),
    }
    proposal["candidate_summary"] = candidate_summary(proposal, info)
    async with aiosqlite.connect(database.DB_PATH) as db:
        await db.execute(
            "INSERT INTO system_state(key,value) VALUES(?,?) "
            "ON CONFLICT(key) DO UPDATE SET value=excluded.value",
            [f"replay_proposal:{data.race_id}", json.dumps(proposal)],
        )
        await db.execute(
            "INSERT INTO system_state(key,value) VALUES(?,?) "
            "ON CONFLICT(key) DO UPDATE SET value=excluded.value",
            [
                f"review_job:{data.race_id}",
                json.dumps(
                    {
                        "race_id": data.race_id,
                        "status": proposal["status"],
                        "proposal_id": proposal["id"],
                    }
                ),
            ],
        )
        await db.commit()
    return proposal


async def get_proposal(race_id):
    return await _state(f"replay_proposal:{race_id}")


async def submit_tracking_preview(data: TrackingPreview):
    """Store anonymous track evidence for replay inspection without scoring laps."""
    info = await manifest(data.race_id)
    if info["race_status"] == "active":
        raise HTTPException(409, "Pause the race before reviewing its video")
    if data.video_sha256 != info["video_sha256"]:
        raise HTTPException(409, "Tracking video does not match the saved replay")
    if not gate_matches(data.analysis_gate.model_dump(), info["finish_gate"]):
        raise HTTPException(409, "Tracking gate does not match the saved finish line")
    roster = {racer["id"] for racer in info["racers"]}
    if any(
        track.assigned_racer_id not in roster
        for track in data.tracks
        if track.assigned_racer_id
    ):
        raise HTTPException(422, "Tracking preview contains a racer outside this race")
    start = info["video_start_s"]
    end = info["video_end_s"] or 86400
    if any(
        not start <= track.crossing_second <= end
        or not track.points[0].second
        <= track.crossing_second
        <= track.points[-1].second
        or any(not start <= point.second <= end for point in track.points)
        for track in data.tracks
    ):
        raise HTTPException(
            422, "Tracking points must bracket a crossing in the race interval"
        )
    preview = {
        "id": str(uuid.uuid4()),
        "received_at": datetime.now(timezone.utc).isoformat(),
        **data.model_dump(),
    }
    async with aiosqlite.connect(database.DB_PATH) as db:
        await db.execute(
            "INSERT INTO system_state(key,value) VALUES(?,?) "
            "ON CONFLICT(key) DO UPDATE SET value=excluded.value",
            [f"tracking_preview:{data.race_id}", json.dumps(preview)],
        )
        await db.commit()
    return tracking_preview_summary(preview)


def tracking_preview_summary(preview):
    if preview is None:
        return None
    return {
        "id": preview["id"],
        "source": preview["source"],
        "track_count": len(preview["tracks"]),
        "point_count": sum(len(track["points"]) for track in preview["tracks"]),
    }


async def get_tracking_preview(race_id):
    return await _state(f"tracking_preview:{race_id}")


async def get_tracking_preview_summary(race_id):
    return tracking_preview_summary(await get_tracking_preview(race_id))


async def assign_track(race_id: str, assignment: TrackAssignment):
    """Save a human review of one crossing without touching the lap ledger."""
    info = await manifest(race_id)
    if info["race_status"] == "active":
        raise HTTPException(409, "Pause the race before reviewing a track")
    if assignment.racer_id and assignment.racer_id not in {
        racer["id"] for racer in info["racers"]
    }:
        raise HTTPException(422, "Racer is outside this race")
    key = f"tracking_preview:{race_id}"
    async with aiosqlite.connect(database.DB_PATH) as db:
        await db.execute("BEGIN IMMEDIATE")
        row = await (
            await db.execute("SELECT value FROM system_state WHERE key=?", [key])
        ).fetchone()
        if not row:
            raise HTTPException(404, "No tracking preview for this race")
        preview = json.loads(row[0])
        if preview["video_sha256"] != info["video_sha256"] or not gate_matches(
            preview["analysis_gate"], info["finish_gate"]
        ):
            raise HTTPException(409, "Tracking preview no longer matches this race")
        track = next(
            (
                item
                for item in preview["tracks"]
                if item["track_id"] == assignment.track_id
            ),
            None,
        )
        if track is None:
            raise HTTPException(404, "Track not found")
        track["assigned_racer_id"] = assignment.racer_id
        track["excluded"] = assignment.excluded
        preview["id"] = str(uuid.uuid4())
        preview["updated_at"] = datetime.now(timezone.utc).isoformat()
        await db.execute(
            "UPDATE system_state SET value=? WHERE key=?", [json.dumps(preview), key]
        )
        await db.commit()
    return tracking_preview_summary(preview)


async def assign_group(race_id, assignment: GroupAssignment):
    """Record a human identity hint without turning candidate passes into laps."""
    info = await manifest(race_id)
    if info["race_status"] == "active":
        raise HTTPException(409, "Pause the race before assigning an appearance group")
    key = f"replay_proposal:{race_id}"
    async with aiosqlite.connect(database.DB_PATH) as db:
        await db.execute("BEGIN IMMEDIATE")
        row = await (
            await db.execute("SELECT value FROM system_state WHERE key=?", [key])
        ).fetchone()
        if not row:
            raise HTTPException(404, "No proposal for this race")
        proposal = json.loads(row[0])
        if proposal["status"] == "applied":
            raise HTTPException(409, "This proposal has already been applied")
        if proposal["video_sha256"] != info["video_sha256"] or not gate_matches(
            proposal.get("analysis_gate"), info["finish_gate"]
        ):
            raise HTTPException(
                409, "Review the saved video at its current finish line"
            )
        groups = {
            event["appearance_cluster"]
            for event in proposal.get("observations", [])
            if event.get("appearance_cluster") is not None
            and finish_observation(event, info["finish_gate"])
        }
        if assignment.appearance_cluster not in groups:
            raise HTTPException(422, "Appearance group is not in this proposal")
        roster = {racer["id"] for racer in info["racers"]}
        if assignment.racer_id is not None and assignment.racer_id not in roster:
            raise HTTPException(422, "Racer is not in this race")
        mappings = proposal.setdefault("group_assignments", {})
        group = str(assignment.appearance_cluster)
        if assignment.racer_id is None:
            mappings.pop(group, None)
        else:
            mappings[group] = assignment.racer_id
        proposal["candidate_summary"] = candidate_summary(proposal, info)
        await db.execute(
            "UPDATE system_state SET value=? WHERE key=?", [json.dumps(proposal), key]
        )
        await db.commit()
    return proposal


async def request_review(race_id):
    info = await manifest(race_id)
    if info["race_status"] == "active":
        raise HTTPException(409, "Pause the race before requesting off-Pi review")
    job = {
        "race_id": race_id,
        "status": "requested",
        "video_sha256": info["video_sha256"],
        "requested_at": datetime.now(timezone.utc).isoformat(),
    }
    async with aiosqlite.connect(database.DB_PATH) as db:
        await db.execute(
            "INSERT INTO system_state(key,value) VALUES(?,?) "
            "ON CONFLICT(key) DO UPDATE SET value=excluded.value",
            [f"review_job:{race_id}", json.dumps(job)],
        )
        await db.commit()
    return job


async def list_review_jobs():
    rows = await database.query(
        "SELECT value FROM system_state WHERE key LIKE 'review_job:%' ORDER BY updated_at"
    )
    return [json.loads(row["value"]) for row in rows]


async def get_review_job(race_id):
    return await _state(f"review_job:{race_id}")


async def apply_proposal(race_id):
    """Atomically snapshot and replace laps; rescore an already finished race."""
    proposal = await get_proposal(race_id)
    if not proposal:
        raise HTTPException(404, "No proposal for this race")
    if proposal["status"] == "applied":
        return proposal
    if proposal["status"] != "ready":
        raise HTTPException(409, "Proposal has unresolved or low-confidence crossings")
    info = await manifest(race_id)
    if info["race_status"] not in ("paused", "finished"):
        raise HTTPException(409, "Pause or finish the race before applying a review")
    if proposal["video_sha256"] != info["video_sha256"]:
        raise HTTPException(409, "Saved replay video changed after analysis")
    if proposal.get("observations") and not gate_matches(
        proposal.get("analysis_gate"), info["finish_gate"]
    ):
        raise HTTPException(409, "Analysis did not use the saved finish line")
    if proposal["baseline_sha256"] != info["baseline_sha256"]:
        raise HTTPException(409, "Lap ledger changed after this proposal was prepared")
    by_racer = {racer["id"]: [] for racer in info["racers"]}
    for event in proposal["crossings"]:
        by_racer[event["racer_id"]].append(event["second"])
    async with aiosqlite.connect(database.DB_PATH) as db:
        db.row_factory = aiosqlite.Row
        await db.execute("PRAGMA foreign_keys=ON")
        await db.execute("BEGIN IMMEDIATE")
        race = await (
            await db.execute("SELECT * FROM races WHERE id=?", [race_id])
        ).fetchone()
        if race["status"] not in ("paused", "finished"):
            raise HTTPException(409, "Race status changed during review")
        old_laps = [
            dict(row)
            for row in await (
                await db.execute("SELECT * FROM lap_records WHERE race_id=?", [race_id])
            ).fetchall()
        ]
        if _lap_checksum(old_laps) != proposal["baseline_sha256"]:
            raise HTTPException(409, "Lap ledger changed during review")
        old_results = [
            dict(row)
            for row in await (
                await db.execute(
                    "SELECT * FROM race_results WHERE race_id=?", [race_id]
                )
            ).fetchall()
        ]
        backup = {"laps": old_laps, "results": old_results, "proposal": proposal}
        await db.execute(
            "INSERT INTO system_state(key,value) VALUES(?,?)",
            [f"review_backup:{race_id}:{proposal['id']}", json.dumps(backup)],
        )
        await db.execute("DELETE FROM lap_records WHERE race_id=?", [race_id])
        for racer_id, seconds in by_racer.items():
            for number, (earlier, later) in enumerate(zip(seconds, seconds[1:]), 1):
                await db.execute(
                    "INSERT INTO lap_records(id,race_id,racer_profile_id,lap_number,lap_time) "
                    "VALUES(?,?,?,?,?)",
                    [
                        str(uuid.uuid4()),
                        race_id,
                        racer_id,
                        number,
                        round((later - earlier) * 1000, 3),
                    ],
                )
        if race["status"] == "finished":
            from app.race_service import write_scored_results

            await write_scored_results(db, race)
        proposal["status"] = "applied"
        proposal["applied_at"] = datetime.now(timezone.utc).isoformat()
        await db.execute(
            "UPDATE system_state SET value=? WHERE key=?",
            [json.dumps(proposal), f"replay_proposal:{race_id}"],
        )
        await db.commit()
    return proposal
