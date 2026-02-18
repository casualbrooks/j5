"""FastAPI race manager service for standalone and ROS-fed deployments."""

from __future__ import annotations

from datetime import datetime

try:
    from typing import Annotated
except ImportError:  # pragma: no cover
    from typing_extensions import Annotated

from fastapi import Depends, FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse

from .config import Settings, get_settings
from .repository import RaceRepository, get_repository
from .schemas import LapIngest, LapResponse, LeaderboardResponse

app = FastAPI(title="J5 Race Manager", version="0.2.0")


class ConnectionManager:
    def __init__(self) -> None:
        self._connections: list[WebSocket] = []

    async def connect(self, websocket: WebSocket) -> None:
        await websocket.accept()
        self._connections.append(websocket)

    def disconnect(self, websocket: WebSocket) -> None:
        if websocket in self._connections:
            self._connections.remove(websocket)

    async def broadcast_json(self, payload: dict) -> None:
        stale: list[WebSocket] = []
        for connection in self._connections:
            try:
                await connection.send_json(payload)
            except RuntimeError:
                stale.append(connection)
        for dead in stale:
            self.disconnect(dead)


class RepoProvider:
    def __init__(self) -> None:
        self._repo: RaceRepository | None = None

    def __call__(
        self, settings: Annotated[Settings, Depends(get_settings)]
    ) -> RaceRepository:
        if self._repo is None:
            self._repo = get_repository(
                settings.db_backend,
                mongo_uri=settings.atlas_uri,
                db_name=settings.race_db_name,
                laps_collection=settings.laps_collection,
                sqlite_path=settings.sqlite_path,
            )
        return self._repo


def calculate_speed_kph(track_distance_m: float, lap_time_ms: int) -> float:
    if lap_time_ms <= 0:
        raise ValueError("lap_time_ms must be positive")
    return round((track_distance_m / lap_time_ms) * 3600, 3)


repo_provider = RepoProvider()
ws_manager = ConnectionManager()


@app.get("/health")
def health(settings: Annotated[Settings, Depends(get_settings)]) -> JSONResponse:
    return JSONResponse(
        {"status": "ok", "config": settings.dict(include_sensitive=False)}
    )


@app.post("/ingest/lap", response_model=LapResponse)
async def ingest_lap(
    payload: LapIngest,
    repository: Annotated[RaceRepository, Depends(repo_provider)],
) -> LapResponse:
    speed = calculate_speed_kph(payload.trackDistanceM, payload.lapTimeMs)
    try:
        inserted_id = repository.insert_lap(payload, speed)
        leaderboard = repository.leaderboard(payload.raceId)
    except Exception as exc:  # pragma: no cover - runtime infrastructure failures
        raise HTTPException(status_code=500, detail=str(exc)) from exc

    await ws_manager.broadcast_json(
        {
            "type": "lap",
            "payload": {
                "insertedId": inserted_id,
                "raceId": payload.raceId,
                "carId": payload.carId,
                "sessionLap": payload.sessionLap,
                "lapTimeMs": payload.lapTimeMs,
                "speedKph": speed,
                "isValid": payload.validity.is_valid,
                "source": payload.source,
                "timestamp": payload.timestamp.isoformat(),
            },
        }
    )
    await ws_manager.broadcast_json(
        {"type": "leaderboard", "payload": leaderboard.dict()}
    )
    return LapResponse(
        insertedId=inserted_id, speedKph=speed, isValid=payload.validity.is_valid
    )


@app.get("/races/{race_id}/leaderboard", response_model=LeaderboardResponse)
def race_leaderboard(
    race_id: str,
    repository: Annotated[RaceRepository, Depends(repo_provider)],
) -> LeaderboardResponse:
    try:
        return repository.leaderboard(race_id)
    except Exception as exc:  # pragma: no cover
        raise HTTPException(status_code=500, detail=str(exc)) from exc


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket) -> None:
    await ws_manager.connect(websocket)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        ws_manager.disconnect(websocket)


@app.get("/")
def root() -> dict[str, str]:
    return {
        "message": "Race manager service running",
        "timestamp": datetime.utcnow().isoformat(),
    }
