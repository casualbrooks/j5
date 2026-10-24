"""A single-origin browser workflow using the existing Race Master database."""

import asyncio
import csv
import io
import json
import os
import re
import time
import uuid
from contextlib import suppress
from datetime import datetime, timezone
from pathlib import Path

import aiosqlite
import cv2
from fastapi import APIRouter, HTTPException, Request
from fastapi.responses import FileResponse, Response
from pydantic import BaseModel, Field, model_validator

from app import database
from app.camera import Camera
from app.color_timing import FinishGate, detect_colors, hue_for_color
from app.motion_timing import MotionFinishDetector
from app.models import LapRecordCreate

router = APIRouter()


async def pause_active_camera_race():
    """On restart or camera fault, pause only the active browser-timed race."""
    async with aiosqlite.connect(database.DB_PATH) as db:
        state = await (
            await db.execute("SELECT value FROM system_state WHERE key='race_context'")
        ).fetchone()
        context = json.loads(state[0]) if state else {}
        if not context.get("server_timing") or not context.get("race_id"):
            return
        await db.execute(
            "UPDATE races SET status='paused' WHERE id=? AND status='active'",
            [context["race_id"]],
        )
        await db.commit()


class RacerColor(BaseModel):
    name: str = Field(min_length=1, max_length=80)
    color: str = Field(pattern=r"^#[0-9a-fA-F]{6}$")


class SessionSetup(BaseModel):
    championship_id: str | None = None
    championship_name: str = Field(
        default="Club Championship", min_length=1, max_length=100
    )
    event_name: str = Field(default="Race day", min_length=1, max_length=100)
    total_laps: int = Field(default=10, ge=1, le=1000)
    source: str = "/dev/video0"
    racers: list[RacerColor] = Field(min_length=1, max_length=8)
    finish_x: float = Field(default=0.5, ge=0.1, le=0.9)
    y_min: float = Field(default=0.1, ge=0, le=1)
    y_max: float = Field(default=0.9, ge=0, le=1)
    direction: int = Field(default=1)
    minimum_lap_ms: int = Field(default=2000, ge=500, le=600000)

    @model_validator(mode="after")
    def validate_track(self):
        validate_source(self.source)
        if self.direction not in (-1, 1) or self.y_min >= self.y_max:
            raise ValueError("Choose a valid direction and finish-line range")
        names = [r.name.strip().casefold() for r in self.racers]
        if not all(names) or len(set(names)) != len(names):
            raise ValueError("Racer names must be distinct and nonempty")
        hues = [hue_for_color(r.color) for r in self.racers]
        for i, hue in enumerate(hues):
            if any(min(abs(hue - h), 180 - abs(hue - h)) <= 16 for h in hues[:i]):
                raise ValueError(
                    "Car colors overlap. Select more distinct colors before racing"
                )
        return self


def validate_source(source):
    if not re.fullmatch(
        r"(?:[0-9]|/dev/video[0-9]+|ros:/[A-Za-z0-9_/]+|video:(?:/home/racetime/)?[A-Za-z0-9_-]+\.mp4)",
        source,
    ):
        raise ValueError(
            "Choose a camera, upload an MP4, or select an existing replay video"
        )


class VisionGate(BaseModel):
    finish_x: float = Field(ge=0.1, le=0.9)
    y_min: float = Field(ge=0, le=1)
    y_max: float = Field(ge=0, le=1)
    direction: int

    @model_validator(mode="after")
    def valid_gate(self):
        if self.y_min >= self.y_max or self.direction not in (-1, 1):
            raise ValueError("Choose a valid finish-line range and direction")
        return self


class TimingRuntime:
    def __init__(self):
        self.camera = Camera()
        self.lock = asyncio.Lock()
        self.task = None
        self.gate = None
        self.race_key = None
        self.last_frame = 0
        self.detections = []
        self.error = None
        self.vision_config = VisionGate(finish_x=0.5, y_min=0.1, y_max=0.9, direction=1)
        self.motion = MotionFinishDetector(**self.vision_config.model_dump())
        self.motion_evidence = {}
        self.cached_state = None
        self.cache_until = 0

    async def start(self):
        self.lock = asyncio.Lock()
        self.race_key = None
        self.last_frame = 0
        self.motion = MotionFinishDetector(**self.vision_config.model_dump())
        self.motion_evidence.clear()
        self.cached_state = None
        self.cache_until = 0
        # A process restart invalidates in-flight timing, never saved laps.
        await pause_active_camera_race()
        self.task = asyncio.create_task(self.run())

    async def stop(self):
        if self.task:
            self.task.cancel()
            with suppress(asyncio.CancelledError):
                await self.task
            self.task = None
        await asyncio.to_thread(self.camera.stop)

    async def process_frame(self, frame, stamp, now=None):
        from app.main import _get_state_json, record_lap, pause_race

        now = time.monotonic() if now is None else now
        async with self.lock:
            new_frame = (
                frame is not None and stamp != self.last_frame and now - stamp <= 1
            )
            if new_frame:
                self.last_frame = stamp
                for event in self.motion.observe(frame, stamp):
                    event["observed_at"] = datetime.now(timezone.utc).isoformat(
                        timespec="milliseconds"
                    )
                    event["video_second"] = (
                        self.camera.video_second
                        if self.camera.source
                        and self.camera.source.startswith("video:")
                        and self.camera.frame_time == stamp
                        else None
                    )
                    ok, jpeg = cv2.imencode(".jpg", frame)
                    if ok:
                        self.motion_evidence[event["id"]] = jpeg.tobytes()
                        if len(self.motion_evidence) > 500:
                            del self.motion_evidence[next(iter(self.motion_evidence))]
            if self.cached_state is None or now >= self.cache_until:
                config = await _get_state_json("quick_config", {})
                context = await _get_state_json("race_context", {})
                race = (
                    await database.get_row("races", context.get("race_id", ""))
                    if context.get("server_timing")
                    else None
                )
                self.cached_state = (config, context, race)
                self.cache_until = now + 0.2
            else:
                config, context, race = self.cached_state
            if not context.get("server_timing"):
                self.detections = []
                return
            if not race:
                return
            key = (race["id"], race["status"])
            if key != self.race_key:
                self.gate = FinishGate(
                    config.get("finish_x", 0.5),
                    config.get("y_min", 0.1),
                    config.get("y_max", 0.9),
                    config.get("direction", 1),
                    config.get("minimum_lap_ms", 2000),
                )
                self.race_key = key
            if race["status"] == "active" and (frame is None or now - stamp > 2):
                await pause_race(race["id"])
                self.race_key = None
                self.cache_until = 0
                self.detections = []
                self.error = "Camera frames stopped. Race paused; reconnect and resume. Completed laps are saved."
                return
            if not new_frame:
                return
            self.detections = await asyncio.to_thread(
                detect_colors, frame, context.get("racers", [])
            )
            if race["status"] != "active":
                return
            for detection in self.detections:
                duration = self.gate.observe(
                    detection["racer_id"], detection["x"], detection["y"], stamp
                )
                if duration is None:
                    continue
                laps = await database.query(
                    "SELECT COALESCE(MAX(lap_number),0) n FROM lap_records WHERE race_id=? AND racer_profile_id=?",
                    [race["id"], detection["racer_id"]],
                )
                number = laps[0]["n"] + 1
                if number <= race["total_laps"]:
                    await record_lap(
                        LapRecordCreate(
                            race_id=race["id"],
                            racer_profile_id=detection["racer_id"],
                            lap_number=number,
                            lap_time=duration,
                        )
                    )

    async def run(self):
        import logging

        while True:
            try:
                frame, stamp = self.camera.latest()
                await self.process_frame(frame, stamp)
            except Exception as exc:
                self.error = str(exc)
                logging.getLogger(__name__).exception("Camera timing failed")
                # Never silently continue counting after a failed ledger write.
                try:
                    await pause_active_camera_race()
                except Exception:
                    logging.getLogger(__name__).exception(
                        "Could not persist paused status"
                    )
                self.race_key = None
                self.cache_until = 0
            await asyncio.sleep(0.025)


runtime = TimingRuntime()


@router.get("/race")
async def race_page():
    return FileResponse(Path(__file__).parent / "static" / "race.html")


@router.get("/api/quick/state")
async def state():
    from app.main import _get_state_json

    context = await _get_state_json("race_context", {})
    race_id = context.get("race_id", "")
    return {
        "config": await _get_state_json("quick_config", {}),
        "context": context,
        "race": await database.get_row("races", race_id),
        "laps": await database.get_all_rows(
            "lap_records", {"race_id": race_id}, "lap_number"
        ),
        "results": await database.get_all_rows(
            "race_results", {"race_id": race_id}, "finish_position"
        ),
        "camera": {
            "source": runtime.camera.source,
            "ready": (runtime.camera.paused and runtime.camera.frame is not None)
            or time.monotonic() - runtime.camera.frame_time < 2,
            "paused": runtime.camera.paused,
            "video_second": runtime.camera.video_second,
            "error": runtime.camera.error,
            "detections": runtime.detections,
        },
        "vision": {
            "config": runtime.vision_config.model_dump(),
            "calibrated": runtime.motion.calibrated,
            "processed_fps": runtime.motion.processed_fps,
            "total": runtime.motion.total,
            "boxes": runtime.motion.boxes,
            "recent": list(runtime.motion.recent)[-20:],
        },
        "error": runtime.error,
    }


@router.post("/api/quick/vision")
async def set_vision_gate(data: VisionGate):
    """Start an anonymous crossing test; no championship lap is written."""
    async with runtime.lock:
        runtime.vision_config = data
        runtime.motion = MotionFinishDetector(**data.model_dump())
        runtime.motion_evidence.clear()
    return {"calibrating": True}


def video_directory():
    return Path(database.DB_PATH).parent / "videos"


@router.get("/api/quick/videos")
async def list_videos():
    videos = []
    for directory, legacy in (
        (video_directory(), False),
        (Path("/home/racetime"), True),
    ):
        if not directory.is_dir():
            continue
        for path in sorted(directory.glob("*.mp4")):
            if path.is_file() and re.fullmatch(r"[A-Za-z0-9_-]+\.mp4", path.name):
                videos.append(
                    {
                        "name": path.name,
                        "source": f"video:{path}" if legacy else f"video:{path.name}",
                        "size": path.stat().st_size,
                    }
                )
    return videos


@router.post("/api/quick/videos/{filename}")
async def upload_video(filename: str, request: Request):
    """Stream a replay to persistent storage without replacing an existing file."""
    if not re.fullmatch(r"[A-Za-z0-9_-]+\.mp4", filename, re.IGNORECASE):
        raise HTTPException(400, "Choose an MP4 file with a simple filename")
    filename = f"{filename[:-4]}.mp4"
    directory = video_directory()
    directory.mkdir(parents=True, exist_ok=True)
    name = f"{uuid.uuid4().hex[:8]}-{filename}"
    destination = directory / name
    temporary = directory / f".{name}.upload"
    size = 0
    try:
        with temporary.open("xb") as output:
            async for chunk in request.stream():
                size += len(chunk)
                if size > 2 * 1024**3:
                    raise HTTPException(
                        413, "Video is larger than the 2 GB upload limit"
                    )
                output.write(chunk)
        if not size:
            raise HTTPException(400, "The selected video is empty")
        capture = await asyncio.to_thread(cv2.VideoCapture, str(temporary))
        try:
            if not capture.isOpened() or not capture.get(cv2.CAP_PROP_FRAME_COUNT):
                raise HTTPException(
                    400, "The selected file is not a readable MP4 video"
                )
        finally:
            capture.release()
        os.replace(temporary, destination)
    finally:
        temporary.unlink(missing_ok=True)
    return {"name": filename, "source": f"video:{name}", "size": size}


@router.get("/api/quick/crossing/{event_id}.jpg")
async def crossing_frame(event_id: int):
    jpeg = runtime.motion_evidence.get(event_id)
    if jpeg is None:
        raise HTTPException(404, "Crossing frame is no longer available")
    return Response(
        jpeg, media_type="image/jpeg", headers={"Cache-Control": "no-store"}
    )


@router.get("/api/quick/crossings.csv")
async def crossing_log():
    stream = io.StringIO()
    fields = ("id", "observed_at", "video_second", "x", "y", "box", "identity")
    writer = csv.DictWriter(stream, fieldnames=fields)
    writer.writeheader()
    for event in runtime.motion.recent:
        writer.writerow({field: event.get(field) for field in fields})
    return Response(
        stream.getvalue(),
        media_type="text/csv",
        headers={"Content-Disposition": "attachment; filename=crossing-candidates.csv"},
    )


class CameraSource(BaseModel):
    source: str


@router.post("/api/quick/camera")
async def open_camera(data: CameraSource):
    from app.main import _get_state_json

    async with runtime.lock:
        try:
            validate_source(data.source)
        except ValueError as exc:
            raise HTTPException(400, str(exc))
        context = await _get_state_json("race_context", {})
        race = await database.get_row("races", context.get("race_id", ""))
        if race and race["status"] == "active":
            raise HTTPException(409, "Pause the race before reconnecting the camera")
        if race and race["status"] == "paused" and data.source.startswith("video:"):
            laps = await database.get_all_rows("lap_records", {"race_id": race["id"]})
            if laps and (
                data.source != runtime.camera.source
                or not runtime.camera.thread
                or not runtime.camera.thread.is_alive()
            ):
                raise HTTPException(
                    409,
                    "This paused race has saved laps; restarting a replay would count earlier footage again",
                )
        try:
            if (
                data.source != runtime.camera.source
                and runtime.camera.thread
                and runtime.camera.thread.is_alive()
            ):
                await asyncio.to_thread(runtime.camera.stop)
            runtime.camera.start(data.source)
            if runtime.camera.paused and race and race["status"] == "paused":
                laps = await database.get_all_rows(
                    "lap_records", {"race_id": race["id"]}
                )
                if not laps:
                    runtime.camera.resume()
        except ValueError as exc:
            raise HTTPException(409, str(exc))
    return {"opening": True}


@router.delete("/api/quick/camera")
async def close_camera():
    from app.main import _get_state_json, pause_race

    async with runtime.lock:
        context = await _get_state_json("race_context", {})
        race = await database.get_row("races", context.get("race_id", ""))
        if race and race["status"] == "active":
            await pause_race(race["id"])
        try:
            await asyncio.to_thread(runtime.camera.stop)
        except ValueError as exc:
            raise HTTPException(409, str(exc))
    return {"stopped": True}


@router.get("/api/quick/frame.jpg")
async def frame_image():
    frame, stamp = runtime.camera.latest()
    if frame is None or (not runtime.camera.paused and time.monotonic() - stamp > 2):
        raise HTTPException(503, runtime.camera.error or "Waiting for camera frames")
    ok, jpeg = await asyncio.to_thread(cv2.imencode, ".jpg", frame)
    if not ok:
        raise HTTPException(503, "Could not encode camera frame")
    return Response(
        jpeg.tobytes(), media_type="image/jpeg", headers={"Cache-Control": "no-store"}
    )


@router.post("/api/quick/session")
async def create_session(data: SessionSetup):
    async with runtime.lock:
        async with aiosqlite.connect(database.DB_PATH) as db:
            db.row_factory = aiosqlite.Row
            await db.execute("PRAGMA foreign_keys=ON")
            await db.execute("BEGIN IMMEDIATE")
            saved_context = await (
                await db.execute(
                    "SELECT value FROM system_state WHERE key='race_context'"
                )
            ).fetchone()
            previous_context = json.loads(saved_context[0]) if saved_context else {}
            unfinished = None
            if previous_context.get("server_timing"):
                unfinished = await (
                    await db.execute(
                        "SELECT id,status FROM races WHERE id=? AND status IN ('active','paused')",
                        [previous_context.get("race_id")],
                    )
                ).fetchone()
            if unfinished and unfinished["status"] == "paused":
                has_laps = await (
                    await db.execute(
                        "SELECT 1 FROM lap_records WHERE race_id=? LIMIT 1",
                        [unfinished["id"]],
                    )
                ).fetchone()
            else:
                has_laps = None
            if unfinished and (unfinished["status"] == "active" or has_laps):
                raise HTTPException(
                    409,
                    "Pause before editing setup; finish a race with saved laps before starting another",
                )
            if previous_context.get("race_id"):
                await db.execute(
                    "INSERT OR IGNORE INTO system_state(key,value) VALUES(?,?)",
                    [
                        f"race_context:{previous_context['race_id']}",
                        json.dumps(previous_context),
                    ],
                )
            champ_id = data.championship_id
            if champ_id:
                if not await (
                    await db.execute(
                        "SELECT id FROM championships WHERE id=?", [champ_id]
                    )
                ).fetchone():
                    raise HTTPException(404, "Championship not found")
            else:
                year = datetime.now().year
                existing = await (
                    await db.execute(
                        "SELECT c.id FROM championships c JOIN seasons s ON s.id=c.season_id WHERE c.name=? AND s.year=?",
                        [data.championship_name.strip(), year],
                    )
                ).fetchone()
                if existing:
                    champ_id = existing[0]
                else:
                    season_id, champ_id = str(uuid.uuid4()), str(uuid.uuid4())
                    await db.execute(
                        "INSERT INTO seasons(id,name,year) VALUES(?,?,?)",
                        [season_id, str(year), year],
                    )
                    await db.execute(
                        "INSERT INTO championships(id,season_id,name) VALUES(?,?,?)",
                        [champ_id, season_id, data.championship_name.strip()],
                    )
            prepared = None
            if previous_context.get("server_timing") and previous_context.get(
                "race_id"
            ):
                prepared = await (
                    await db.execute(
                        "SELECT r.id,r.event_id FROM races r WHERE r.id=? AND r.status IN ('setup','paused') "
                        "AND NOT EXISTS (SELECT 1 FROM lap_records l WHERE l.race_id=r.id)",
                        [previous_context["race_id"]],
                    )
                ).fetchone()
            if prepared:
                race_id, event_id = prepared["id"], prepared["event_id"]
                await db.execute(
                    "UPDATE race_events SET championship_id=?,name=? WHERE id=?",
                    [champ_id, data.event_name.strip(), event_id],
                )
                await db.execute(
                    "UPDATE races SET total_laps=?,status='setup',start_time=NULL WHERE id=?",
                    [data.total_laps, race_id],
                )
            else:
                event_id, race_id = str(uuid.uuid4()), str(uuid.uuid4())
                number = (
                    await (
                        await db.execute(
                            "SELECT COUNT(*) FROM race_events WHERE championship_id=?",
                            [champ_id],
                        )
                    ).fetchone()
                )[0] + 1
                await db.execute(
                    "INSERT INTO race_events(id,championship_id,name,event_date,round_number) VALUES(?,?,?,?,?)",
                    [
                        event_id,
                        champ_id,
                        data.event_name.strip(),
                        datetime.now().date().isoformat(),
                        number,
                    ],
                )
                await db.execute(
                    "INSERT INTO races(id,event_id,total_laps) VALUES(?,?,?)",
                    [race_id, event_id, data.total_laps],
                )
            racers = []
            for racer in data.racers:
                row = await (
                    await db.execute(
                        "SELECT * FROM racer_profiles WHERE lower(name)=lower(?) ORDER BY created_at LIMIT 1",
                        [racer.name.strip()],
                    )
                ).fetchone()
                racer_id = row["id"] if row else str(uuid.uuid4())
                if not row:
                    await db.execute(
                        "INSERT INTO racer_profiles(id,name) VALUES(?,?)",
                        [racer_id, racer.name.strip()],
                    )
                racers.append(
                    {"id": racer_id, "name": racer.name.strip(), "color": racer.color}
                )
            context = {
                "race_id": race_id,
                "event_id": event_id,
                "championship_id": champ_id,
                "race_name": data.event_name,
                "racers": racers,
                "server_timing": True,
                "tracking_enabled": True,
                "log_stream": [],
            }
            for key, value in [
                ("race_context", context),
                (f"race_context:{race_id}", context),
                ("quick_config", data.model_dump()),
            ]:
                await db.execute(
                    "INSERT INTO system_state(key,value) VALUES(?,?) ON CONFLICT(key) DO UPDATE SET value=excluded.value",
                    [key, json.dumps(value)],
                )
            await db.commit()
        runtime.race_key = None
        runtime.cache_until = 0
        runtime.vision_config = VisionGate(
            finish_x=data.finish_x,
            y_min=data.y_min,
            y_max=data.y_max,
            direction=data.direction,
        )
        runtime.motion = MotionFinishDetector(**runtime.vision_config.model_dump())
        runtime.motion_evidence.clear()
        if runtime.camera.paused:
            runtime.camera.resume()
    return {"race_id": race_id, "championship_id": champ_id, "updated": bool(prepared)}


@router.post("/api/quick/control/{action}")
async def control(action: str):
    from app.main import _get_state_json, start_race, pause_race, finish_race

    async with runtime.lock:
        context = await _get_state_json("race_context", {})
        if not context.get("server_timing"):
            raise HTTPException(409, "Create a camera race first")
        if action == "start":
            config = await _get_state_json("quick_config", {})
            race = await database.get_row("races", context["race_id"])
            if not race:
                raise HTTPException(404, "Race not found")
            if race["status"] not in ("setup", "paused"):
                raise HTTPException(
                    409, f"Cannot start a race in {race['status']} status"
                )
            source = config.get("source")
            if race["status"] == "setup" and source and source.startswith("video:"):
                # Always begin a new race at the first frame, even if a different
                # replay was used to position the finish line in the preview.
                try:
                    await asyncio.to_thread(runtime.camera.stop)
                    runtime.camera.start(source)
                except ValueError as exc:
                    raise HTTPException(409, str(exc)) from exc
                deadline = time.monotonic() + 5
                while runtime.camera.frame_time == 0 and time.monotonic() < deadline:
                    if runtime.camera.error:
                        raise HTTPException(409, runtime.camera.error)
                    await asyncio.sleep(0.05)
                if runtime.camera.frame_time == 0:
                    raise HTTPException(
                        409, runtime.camera.error or "Video did not provide a frame"
                    )
            elif race["status"] == "paused" and runtime.camera.paused:
                if runtime.camera.source != source:
                    raise HTTPException(409, "Reconnect the video saved for this race")
                runtime.camera.resume()
                deadline = time.monotonic() + 5
                previous_frame = runtime.camera.frame_time
                while (
                    runtime.camera.frame_time == previous_frame
                    and time.monotonic() < deadline
                ):
                    await asyncio.sleep(0.05)
            if runtime.camera.source != config.get("source"):
                raise HTTPException(
                    409, "Connect the saved camera source before resuming this race"
                )
            if time.monotonic() - runtime.camera.frame_time > 2:
                raise HTTPException(
                    409, "Camera or replay is not providing frames; check the preview"
                )
            result = await start_race(context["race_id"])
        elif action == "pause":
            result = await pause_race(context["race_id"])
            runtime.camera.pause()
        elif action == "finish":
            result = await finish_race(context["race_id"])
        else:
            raise HTTPException(404, "Unknown race action")
        runtime.race_key = None
        runtime.cache_until = 0
        runtime.error = None
        return result
