"""
Race Master Pro — FastAPI Backend
Main application entry point with REST API + WebSocket endpoints.
"""

import json
import uuid
from datetime import datetime
from contextlib import asynccontextmanager

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware

from app.database import (
    init_db,
    insert_row,
    get_row,
    get_all_rows,
    update_row,
    delete_row,
    query,
)
from app.models import (
    SeasonCreate,
    ChampionshipCreate,
    TrackCreate,
    CheckpointCreate,
    CameraConfigCreate,
    RacerProfileCreate,
    RaceEventCreate,
    RaceCreate,
    RaceResultCreate,
    LapRecordCreate,
    DetectionCreate,
    PositionUpdate,
    LapComplete,
)
from app.websocket_manager import manager


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Initialize DB on startup."""
    await init_db()
    yield


app = FastAPI(
    title="Race Master Pro",
    description="Real-time race tracking with AI-powered computer vision for ROS2",
    version="1.0.0",
    lifespan=lifespan,
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Restrict in production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# ── Health ──────────────────────────────────────────────────


@app.get("/health")
async def health_check():
    return {
        "status": "healthy",
        "timestamp": datetime.now().isoformat(),
        "connections": manager.total_connections,
    }


# ── Seasons ─────────────────────────────────────────────────


@app.post("/api/seasons")
async def create_season(data: SeasonCreate):
    row = {"id": str(uuid.uuid4()), **data.model_dump()}
    return await insert_row("seasons", row)


@app.get("/api/seasons")
async def list_seasons():
    return await get_all_rows("seasons", order_by="year DESC")


@app.get("/api/seasons/{season_id}")
async def get_season(season_id: str):
    row = await get_row("seasons", season_id)
    if not row:
        raise HTTPException(404, "Season not found")
    return row


@app.put("/api/seasons/{season_id}")
async def update_season(season_id: str, data: SeasonCreate):
    return await update_row("seasons", season_id, data.model_dump())


@app.delete("/api/seasons/{season_id}")
async def delete_season(season_id: str):
    if not await delete_row("seasons", season_id):
        raise HTTPException(404, "Season not found")
    return {"deleted": True}


# ── Championships ───────────────────────────────────────────


@app.post("/api/championships")
async def create_championship(data: ChampionshipCreate):
    row = {"id": str(uuid.uuid4()), **data.model_dump()}
    return await insert_row("championships", row)


@app.get("/api/championships")
async def list_championships(season_id: str | None = None):
    where = {"season_id": season_id} if season_id else None
    return await get_all_rows("championships", where=where)


@app.get("/api/championships/{champ_id}")
async def get_championship(champ_id: str):
    row = await get_row("championships", champ_id)
    if not row:
        raise HTTPException(404, "Championship not found")
    return row


@app.delete("/api/championships/{champ_id}")
async def delete_championship(champ_id: str):
    if not await delete_row("championships", champ_id):
        raise HTTPException(404, "Championship not found")
    return {"deleted": True}


# ── Tracks ──────────────────────────────────────────────────


@app.post("/api/tracks")
async def create_track(data: TrackCreate):
    row = {"id": str(uuid.uuid4()), **data.model_dump()}
    return await insert_row("tracks", row)


@app.get("/api/tracks")
async def list_tracks():
    return await get_all_rows("tracks", order_by="name")


@app.get("/api/tracks/{track_id}")
async def get_track(track_id: str):
    row = await get_row("tracks", track_id)
    if not row:
        raise HTTPException(404, "Track not found")
    return row


@app.put("/api/tracks/{track_id}")
async def update_track(track_id: str, data: TrackCreate):
    return await update_row("tracks", track_id, data.model_dump())


@app.delete("/api/tracks/{track_id}")
async def delete_track(track_id: str):
    if not await delete_row("tracks", track_id):
        raise HTTPException(404, "Track not found")
    return {"deleted": True}


# ── Racer Profiles ──────────────────────────────────────────


@app.post("/api/racers")
async def create_racer(data: RacerProfileCreate):
    row = {"id": str(uuid.uuid4()), **data.model_dump()}
    return await insert_row("racer_profiles", row)


@app.get("/api/racers")
async def list_racers():
    return await get_all_rows("racer_profiles", order_by="name")


@app.get("/api/racers/{racer_id}")
async def get_racer(racer_id: str):
    row = await get_row("racer_profiles", racer_id)
    if not row:
        raise HTTPException(404, "Racer not found")
    return row


@app.put("/api/racers/{racer_id}")
async def update_racer(racer_id: str, data: RacerProfileCreate):
    return await update_row("racer_profiles", racer_id, data.model_dump())


@app.delete("/api/racers/{racer_id}")
async def delete_racer(racer_id: str):
    if not await delete_row("racer_profiles", racer_id):
        raise HTTPException(404, "Racer not found")
    return {"deleted": True}


# ── Race Events ─────────────────────────────────────────────


@app.post("/api/events")
async def create_event(data: RaceEventCreate):
    row = {"id": str(uuid.uuid4()), **data.model_dump()}
    return await insert_row("race_events", row)


@app.get("/api/events")
async def list_events(championship_id: str | None = None):
    where = {"championship_id": championship_id} if championship_id else None
    return await get_all_rows("race_events", where=where, order_by="round_number")


@app.delete("/api/events/{event_id}")
async def delete_event(event_id: str):
    if not await delete_row("race_events", event_id):
        raise HTTPException(404, "Event not found")
    return {"deleted": True}


# ── Races ───────────────────────────────────────────────────


@app.post("/api/races")
async def create_race(data: RaceCreate):
    row = {"id": str(uuid.uuid4()), "status": "setup", **data.model_dump()}
    return await insert_row("races", row)


@app.get("/api/races")
async def list_races(event_id: str | None = None, status: str | None = None):
    where = {}
    if event_id:
        where["event_id"] = event_id
    if status:
        where["status"] = status
    return await get_all_rows("races", where=where or None)


@app.get("/api/races/{race_id}")
async def get_race(race_id: str):
    row = await get_row("races", race_id)
    if not row:
        raise HTTPException(404, "Race not found")
    return row


@app.post("/api/races/{race_id}/start")
async def start_race(race_id: str):
    race = await get_row("races", race_id)
    if not race:
        raise HTTPException(404, "Race not found")
    if race["status"] not in ("setup", "paused"):
        raise HTTPException(400, f"Cannot start race in '{race['status']}' status")
    now = datetime.now().isoformat()
    updated = await update_row(
        "races", race_id, {"status": "active", "start_time": now}
    )
    await manager.broadcast_all(
        {
            "type": "raceStart",
            "data": {"race_id": race_id, "start_time": now},
            "timestamp": now,
        }
    )
    return updated


@app.post("/api/races/{race_id}/pause")
async def pause_race(race_id: str):
    race = await get_row("races", race_id)
    if not race:
        raise HTTPException(404, "Race not found")
    if race["status"] != "active":
        raise HTTPException(400, "Can only pause an active race")
    updated = await update_row("races", race_id, {"status": "paused"})
    await manager.broadcast_all(
        {
            "type": "racePause",
            "data": {"race_id": race_id},
            "timestamp": datetime.now().isoformat(),
        }
    )
    return updated


@app.post("/api/races/{race_id}/finish")
async def finish_race(race_id: str):
    race = await get_row("races", race_id)
    if not race:
        raise HTTPException(404, "Race not found")
    now = datetime.now().isoformat()
    updated = await update_row(
        "races", race_id, {"status": "finished", "end_time": now}
    )
    await manager.broadcast_all(
        {
            "type": "raceFinish",
            "data": {"race_id": race_id, "end_time": now},
            "timestamp": now,
        }
    )
    return updated


# ── Race Results ────────────────────────────────────────────


@app.post("/api/results")
async def create_result(data: RaceResultCreate):
    row = {"id": str(uuid.uuid4()), **data.model_dump()}
    return await insert_row("race_results", row)


@app.get("/api/results")
async def list_results(race_id: str | None = None, racer_profile_id: str | None = None):
    where = {}
    if race_id:
        where["race_id"] = race_id
    if racer_profile_id:
        where["racer_profile_id"] = racer_profile_id
    return await get_all_rows(
        "race_results", where=where or None, order_by="finish_position"
    )


@app.put("/api/results/{result_id}")
async def update_result(result_id: str, data: RaceResultCreate):
    return await update_row("race_results", result_id, data.model_dump())


# ── Lap Records ─────────────────────────────────────────────


@app.post("/api/laps")
async def record_lap(data: LapRecordCreate):
    row = {"id": str(uuid.uuid4()), **data.model_dump()}
    result = await insert_row("lap_records", row)
    # Broadcast lap complete
    await manager.broadcast_all(
        {
            "type": "lapComplete",
            "data": {
                "racer_id": data.racer_profile_id,
                "lap_number": data.lap_number,
                "lap_time": data.lap_time,
            },
            "timestamp": datetime.now().isoformat(),
        }
    )
    return result


@app.get("/api/laps")
async def list_laps(race_id: str | None = None, racer_profile_id: str | None = None):
    where = {}
    if race_id:
        where["race_id"] = race_id
    if racer_profile_id:
        where["racer_profile_id"] = racer_profile_id
    return await get_all_rows("lap_records", where=where or None, order_by="lap_number")


# ── Camera Configs ──────────────────────────────────────────


@app.post("/api/cameras")
async def create_camera(data: CameraConfigCreate):
    row = {"id": str(uuid.uuid4()), **data.model_dump()}
    return await insert_row("camera_configs", row)


@app.get("/api/cameras")
async def list_cameras():
    return await get_all_rows("camera_configs")


@app.put("/api/cameras/{camera_id}")
async def update_camera(camera_id: str, data: CameraConfigCreate):
    return await update_row("camera_configs", camera_id, data.model_dump())


@app.delete("/api/cameras/{camera_id}")
async def delete_camera(camera_id: str):
    if not await delete_row("camera_configs", camera_id):
        raise HTTPException(404, "Camera not found")
    return {"deleted": True}


# ── Detections ──────────────────────────────────────────────


@app.post("/api/detections")
async def save_detection(data: DetectionCreate):
    row = {"id": str(uuid.uuid4()), **data.model_dump()}
    return await insert_row("detections", row)


# ── Analytics ───────────────────────────────────────────────


@app.get("/api/analytics/racer/{racer_id}/summary")
async def racer_summary(racer_id: str):
    """Aggregate stats for a racer across all their races."""
    results = await query(
        """
        SELECT
            COUNT(*) as total_races,
            AVG(finish_position) as avg_position,
            MIN(best_lap_time) as all_time_best_lap,
            AVG(avg_lap_time) as career_avg_lap,
            SUM(points_earned) as total_points,
            SUM(CASE WHEN status = 'dnf' THEN 1 ELSE 0 END) as dnf_count,
            SUM(CASE WHEN finish_position = 1 THEN 1 ELSE 0 END) as wins,
            SUM(CASE WHEN finish_position <= 3 THEN 1 ELSE 0 END) as podiums
        FROM race_results
        WHERE racer_profile_id = ?
    """,
        [racer_id],
    )
    return results[0] if results else {}


@app.get("/api/analytics/track/{track_id}/records")
async def track_records(track_id: str):
    """Best lap times at a specific track."""
    return await query(
        """
        SELECT lr.*, rp.name as racer_name, rp.number as racer_number
        FROM lap_records lr
        JOIN race_results rr ON lr.race_id = rr.race_id AND lr.racer_profile_id = rr.racer_profile_id
        JOIN races r ON lr.race_id = r.id
        JOIN race_events re ON r.event_id = re.id
        JOIN racer_profiles rp ON lr.racer_profile_id = rp.id
        WHERE re.track_id = ?
        ORDER BY lr.lap_time ASC
        LIMIT 20
    """,
        [track_id],
    )


@app.get("/api/analytics/championship/{champ_id}/standings")
async def championship_standings(champ_id: str):
    """Championship standings with total points per racer."""
    return await query(
        """
        SELECT
            rp.id as racer_id,
            rp.name as racer_name,
            rp.number as racer_number,
            SUM(rr.points_earned) as total_points,
            SUM(CASE WHEN rr.finish_position = 1 THEN 1 ELSE 0 END) as wins,
            SUM(CASE WHEN rr.finish_position <= 3 THEN 1 ELSE 0 END) as podiums,
            COUNT(*) as races_completed
        FROM race_results rr
        JOIN races r ON rr.race_id = r.id
        JOIN race_events re ON r.event_id = re.id
        JOIN racer_profiles rp ON rr.racer_profile_id = rp.id
        WHERE re.championship_id = ?
        GROUP BY rp.id
        ORDER BY total_points DESC
    """,
        [champ_id],
    )


# ── WebSocket ───────────────────────────────────────────────


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket, client_type: str = "spectator"):
    await manager.connect(websocket, client_type)
    try:
        while True:
            data = await websocket.receive_text()
            try:
                message = json.loads(data)
            except json.JSONDecodeError:
                continue

            msg_type = message.get("type", "")

            if msg_type == "ping":
                await manager.send_personal(
                    websocket,
                    {
                        "type": "pong",
                        "timestamp": datetime.now().isoformat(),
                    },
                )
            elif msg_type in (
                "raceUpdate",
                "positionUpdate",
                "lapComplete",
                "raceStart",
                "racePause",
                "raceFinish",
            ):
                # Relay race events to all clients
                message["timestamp"] = datetime.now().isoformat()
                await manager.broadcast_all(message)
            elif msg_type == "visionDetection":
                # CV detections go to organizers
                message["timestamp"] = datetime.now().isoformat()
                await manager.broadcast_to_group(message, "organizer")
            else:
                # Unknown message type — ignore
                pass

    except WebSocketDisconnect:
        manager.disconnect(websocket, client_type)
        await manager.broadcast_connection_count()


# ── Entry Point ─────────────────────────────────────────────

if __name__ == "__main__":
    import uvicorn

    uvicorn.run(
        "app.main:app", host="0.0.0.0", port=8080, reload=True, log_level="info"
    )
