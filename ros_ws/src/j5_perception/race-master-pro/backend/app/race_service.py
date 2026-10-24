"""Transactional race ledger. Lap times are milliseconds throughout the API."""

import json
import uuid
from datetime import datetime, timezone

import aiosqlite
from fastapi import HTTPException

from app import database


async def save_lap(data):
    row = {"id": str(uuid.uuid4()), **data.model_dump()}
    async with aiosqlite.connect(database.DB_PATH) as db:
        db.row_factory = aiosqlite.Row
        await db.execute("PRAGMA foreign_keys=ON")
        await db.execute("BEGIN IMMEDIATE")
        race = await (
            await db.execute("SELECT * FROM races WHERE id=?", [data.race_id])
        ).fetchone()
        if not race:
            raise HTTPException(404, "Race not found")
        previous = await (
            await db.execute(
                "SELECT * FROM lap_records WHERE race_id=? AND racer_profile_id=? AND lap_number=?",
                [data.race_id, data.racer_profile_id, data.lap_number],
            )
        ).fetchone()
        if previous:
            if previous["lap_time"] != data.lap_time:
                raise HTTPException(409, "This lap number already has a different time")
            return dict(previous), False
        if race["status"] != "active":
            raise HTTPException(409, "Start or resume the race before recording laps")
        state = await (
            await db.execute("SELECT value FROM system_state WHERE key='race_context'")
        ).fetchone()
        context = json.loads(state["value"]) if state else {}
        if context.get("race_id") != data.race_id or not context.get(
            "tracking_enabled"
        ):
            raise HTTPException(409, "Tracking is not enabled for this race")
        if data.racer_profile_id not in {r["id"] for r in context.get("racers", [])}:
            raise HTTPException(400, "Racer is not entered in this race")
        count = await (
            await db.execute(
                "SELECT COALESCE(MAX(lap_number),0) FROM lap_records WHERE race_id=? AND racer_profile_id=?",
                [data.race_id, data.racer_profile_id],
            )
        ).fetchone()
        if data.lap_number != count[0] + 1:
            raise HTTPException(409, "Lap numbers must be sequential")
        if data.lap_number > race["total_laps"]:
            raise HTTPException(409, "Racer has completed the lap target")
        columns = ",".join(row)
        await db.execute(
            f"INSERT INTO lap_records ({columns}) VALUES ({','.join('?' for _ in row)})",
            list(row.values()),
        )
        await db.commit()
    return row, True


async def finalize_race(race_id):
    """Finish and score once, in the same transaction as the race state change."""
    async with aiosqlite.connect(database.DB_PATH) as db:
        db.row_factory = aiosqlite.Row
        await db.execute("PRAGMA foreign_keys=ON")
        await db.execute("BEGIN IMMEDIATE")
        race = await (
            await db.execute("SELECT * FROM races WHERE id=?", [race_id])
        ).fetchone()
        if not race:
            raise HTTPException(404, "Race not found")
        if race["status"] == "finished":
            return dict(race)
        if race["status"] not in ("active", "paused"):
            raise HTTPException(409, "Only an active or paused race can finish")
        championship = await (
            await db.execute(
                "SELECT c.points_system FROM championships c JOIN race_events e ON e.championship_id=c.id WHERE e.id=?",
                [race["event_id"]],
            )
        ).fetchone()
        try:
            points = json.loads(championship[0]) if championship else {}
            if not isinstance(points, dict) or any(
                type(v) is not int or v < 0 for v in points.values()
            ):
                raise ValueError()
        except (ValueError, TypeError):
            raise HTTPException(
                409, "Championship points must be a JSON object of nonnegative integers"
            )
        rows = [
            dict(row)
            for row in await (
                await db.execute(
                    """SELECT racer_profile_id, COUNT(*) laps, SUM(lap_time) total,
               MIN(lap_time) best, AVG(lap_time) average FROM lap_records
               WHERE race_id=? GROUP BY racer_profile_id
               ORDER BY laps DESC, total ASC, racer_profile_id ASC""",
                    [race_id],
                )
            ).fetchall()
        ]
        saved = await (
            await db.execute(
                "SELECT value FROM system_state WHERE key=?",
                [f"race_context:{race_id}"],
            )
        ).fetchone()
        if not saved:
            saved = await (
                await db.execute(
                    "SELECT value FROM system_state WHERE key='race_context'"
                )
            ).fetchone()
        roster = json.loads(saved[0]) if saved else {}
        present = {row["racer_profile_id"] for row in rows}
        if roster.get("race_id") == race_id:
            rows.extend(
                {
                    "racer_profile_id": r["id"],
                    "laps": 0,
                    "total": 0,
                    "best": None,
                    "average": None,
                }
                for r in roster.get("racers", [])
                if r["id"] not in present
            )
        await db.execute("DELETE FROM race_results WHERE race_id=?", [race_id])
        for position, row in enumerate(rows, 1):
            completed = row["laps"] >= race["total_laps"]
            await db.execute(
                """INSERT INTO race_results
                (id,race_id,racer_profile_id,finish_position,total_time,best_lap_time,
                 avg_lap_time,laps_completed,status,points_earned)
                VALUES (?,?,?,?,?,?,?,?,?,?)""",
                [
                    str(uuid.uuid4()),
                    race_id,
                    row["racer_profile_id"],
                    position,
                    row["total"],
                    row["best"],
                    row["average"],
                    row["laps"],
                    "finished" if completed else "dnf",
                    (
                        points.get(str(position), 0)
                        if completed and race["type"] == "main"
                        else 0
                    ),
                ],
            )
        now = datetime.now(timezone.utc).isoformat()
        await db.execute(
            "UPDATE races SET status='finished', end_time=? WHERE id=?", [now, race_id]
        )
        await db.commit()
        return {**dict(race), "status": "finished", "end_time": now}
