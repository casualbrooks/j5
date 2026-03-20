"""FastAPI race manager service for standalone and ROS-fed deployments."""

from __future__ import annotations

import uuid
from datetime import datetime

from fastapi import (
    Depends,
    FastAPI,
    HTTPException,
    Response,
    WebSocket,
    WebSocketDisconnect,
)
from fastapi.responses import JSONResponse

from .config import Settings, get_settings
from .repository import RaceRepository, get_repository
from .schemas import (
    CarRecord,
    CarUpsert,
    DashboardEntry,
    LapIngest,
    LapResponse,
    LeaderboardResponse,
    RaceDashboardResponse,
    RaceEventIngest,
    RaceRecord,
    RaceUpsert,
    RacerRecord,
    RacerUpsert,
    TrackRecord,
    TrackUpsert,
)

app = FastAPI(title="J5 Race Manager", version="0.3.0")


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

    def __call__(self, settings: Settings = Depends(get_settings)) -> RaceRepository:
        if self._repo is None:
            self._repo = get_repository(
                settings.db_backend,
                mongo_uri=settings.atlas_uri,
                db_name=settings.race_db_name,
                laps_collection=settings.laps_collection,
                sqlite_path=settings.sqlite_path,
            )
        return self._repo


repo_provider = RepoProvider()
ws_manager = ConnectionManager()


def calculate_speed_kph(track_distance_m: float, lap_time_ms: int) -> float:
    if lap_time_ms <= 0:
        raise ValueError("lap_time_ms must be positive")
    return round((track_distance_m / lap_time_ms) * 3600, 3)


def _create_entity(kind: str, payload: dict, repository: RaceRepository) -> dict:
    record = {"id": str(uuid.uuid4()), **payload}
    return repository.create_entity(kind, record)


def _list_entities(kind: str, repository: RaceRepository) -> list[dict]:
    return repository.list_entities(kind)


def _get_entity_or_404(kind: str, entity_id: str, repository: RaceRepository) -> dict:
    record = repository.get_entity(kind, entity_id)
    if record is None:
        raise HTTPException(status_code=404, detail=f"{kind[:-1].title()} not found")
    return record


def _update_entity_or_404(
    kind: str, entity_id: str, payload: dict, repository: RaceRepository
) -> dict:
    updated = repository.update_entity(kind, entity_id, payload)
    if updated is None:
        raise HTTPException(status_code=404, detail=f"{kind[:-1].title()} not found")
    return updated


def _delete_entity_or_404(
    kind: str, entity_id: str, repository: RaceRepository
) -> Response:
    if not repository.delete_entity(kind, entity_id):
        raise HTTPException(status_code=404, detail=f"{kind[:-1].title()} not found")
    return Response(status_code=204)


def build_dashboard(race_id: str, repository: RaceRepository) -> RaceDashboardResponse:
    race = repository.get_entity("races", race_id)
    if race is None:
        raise HTTPException(status_code=404, detail="Race not found")

    track = None
    if race.get("trackId"):
        track = repository.get_entity("tracks", race["trackId"])

    cars = {item["id"]: item for item in repository.list_entities("cars")}
    racers = {item["id"]: item for item in repository.list_entities("racers")}
    leaderboard = repository.leaderboard(race_id)
    lb_by_car = {entry.carId: entry for entry in leaderboard.leaderboard}
    entries: list[DashboardEntry] = []

    for entry in race.get("entries", []):
        car = cars.get(entry.get("carId"), {})
        racer = racers.get(entry.get("racerId"), {})
        stats = lb_by_car.get(entry.get("carId"))
        entries.append(
            DashboardEntry(
                carId=entry.get("carId", ""),
                racerId=entry.get("racerId"),
                carName=car.get("name", entry.get("carId", "Unassigned Car")),
                racerName=racer.get("name", "Unassigned Racer"),
                carNumber=car.get("carNumber", ""),
                paintColor=car.get("paintColor", "#4ade80"),
                trackerId=entry.get("trackerId"),
                currentLap=stats.lapCount if stats else 0,
                bestLapMs=stats.bestLapMs if stats else None,
                avgSpeedKph=stats.avgSpeedKph if stats else None,
                gapToLeaderMs=stats.gapToLeaderMs if stats else 0,
                totalTimeMs=stats.totalTimeMs if stats else 0,
            )
        )

    return RaceDashboardResponse(
        race=RaceRecord(**race),
        track=TrackRecord(**track) if track else None,
        leaderboard=leaderboard,
        entries=entries,
        recentEvents=repository.list_events(race_id),
    )


async def broadcast_dashboard(race_id: str, repository: RaceRepository) -> None:
    dashboard = build_dashboard(race_id, repository)
    await ws_manager.broadcast_json({"type": "dashboard", "payload": dashboard.dict()})


@app.get("/health")
def health(settings: Settings = Depends(get_settings)) -> JSONResponse:
    return JSONResponse(
        {"status": "ok", "config": settings.dict(include_sensitive=False)}
    )


@app.post("/ingest/lap", response_model=LapResponse)
async def ingest_lap(
    payload: LapIngest,
    repository: RaceRepository = Depends(repo_provider),
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
    if repository.get_entity("races", payload.raceId) is not None:
        await broadcast_dashboard(payload.raceId, repository)
    return LapResponse(
        insertedId=inserted_id, speedKph=speed, isValid=payload.validity.is_valid
    )


@app.get("/races/{race_id}/leaderboard", response_model=LeaderboardResponse)
def race_leaderboard(
    race_id: str,
    repository: RaceRepository = Depends(repo_provider),
) -> LeaderboardResponse:
    try:
        return repository.leaderboard(race_id)
    except Exception as exc:  # pragma: no cover
        raise HTTPException(status_code=500, detail=str(exc)) from exc


@app.get("/api/tracks", response_model=list[TrackRecord])
def list_tracks(
    repository: RaceRepository = Depends(repo_provider),
) -> list[TrackRecord]:
    return [TrackRecord(**item) for item in _list_entities("tracks", repository)]


@app.post("/api/tracks", response_model=TrackRecord, status_code=201)
def create_track(
    payload: TrackUpsert, repository: RaceRepository = Depends(repo_provider)
) -> TrackRecord:
    return TrackRecord(**_create_entity("tracks", payload.dict(), repository))


@app.get("/api/tracks/{track_id}", response_model=TrackRecord)
def get_track(
    track_id: str, repository: RaceRepository = Depends(repo_provider)
) -> TrackRecord:
    return TrackRecord(**_get_entity_or_404("tracks", track_id, repository))


@app.put("/api/tracks/{track_id}", response_model=TrackRecord)
def update_track(
    track_id: str,
    payload: TrackUpsert,
    repository: RaceRepository = Depends(repo_provider),
) -> TrackRecord:
    return TrackRecord(
        **_update_entity_or_404("tracks", track_id, payload.dict(), repository)
    )


@app.delete("/api/tracks/{track_id}", status_code=204)
def delete_track(
    track_id: str, repository: RaceRepository = Depends(repo_provider)
) -> Response:
    return _delete_entity_or_404("tracks", track_id, repository)


@app.get("/api/cars", response_model=list[CarRecord])
def list_cars(repository: RaceRepository = Depends(repo_provider)) -> list[CarRecord]:
    return [CarRecord(**item) for item in _list_entities("cars", repository)]


@app.post("/api/cars", response_model=CarRecord, status_code=201)
def create_car(
    payload: CarUpsert, repository: RaceRepository = Depends(repo_provider)
) -> CarRecord:
    return CarRecord(**_create_entity("cars", payload.dict(), repository))


@app.get("/api/cars/{car_id}", response_model=CarRecord)
def get_car(
    car_id: str, repository: RaceRepository = Depends(repo_provider)
) -> CarRecord:
    return CarRecord(**_get_entity_or_404("cars", car_id, repository))


@app.put("/api/cars/{car_id}", response_model=CarRecord)
def update_car(
    car_id: str,
    payload: CarUpsert,
    repository: RaceRepository = Depends(repo_provider),
) -> CarRecord:
    return CarRecord(
        **_update_entity_or_404("cars", car_id, payload.dict(), repository)
    )


@app.delete("/api/cars/{car_id}", status_code=204)
def delete_car(
    car_id: str, repository: RaceRepository = Depends(repo_provider)
) -> Response:
    return _delete_entity_or_404("cars", car_id, repository)


@app.get("/api/racers", response_model=list[RacerRecord])
def list_racers(
    repository: RaceRepository = Depends(repo_provider),
) -> list[RacerRecord]:
    return [RacerRecord(**item) for item in _list_entities("racers", repository)]


@app.post("/api/racers", response_model=RacerRecord, status_code=201)
def create_racer(
    payload: RacerUpsert, repository: RaceRepository = Depends(repo_provider)
) -> RacerRecord:
    return RacerRecord(**_create_entity("racers", payload.dict(), repository))


@app.get("/api/racers/{racer_id}", response_model=RacerRecord)
def get_racer(
    racer_id: str, repository: RaceRepository = Depends(repo_provider)
) -> RacerRecord:
    return RacerRecord(**_get_entity_or_404("racers", racer_id, repository))


@app.put("/api/racers/{racer_id}", response_model=RacerRecord)
def update_racer(
    racer_id: str,
    payload: RacerUpsert,
    repository: RaceRepository = Depends(repo_provider),
) -> RacerRecord:
    return RacerRecord(
        **_update_entity_or_404("racers", racer_id, payload.dict(), repository)
    )


@app.delete("/api/racers/{racer_id}", status_code=204)
def delete_racer(
    racer_id: str, repository: RaceRepository = Depends(repo_provider)
) -> Response:
    return _delete_entity_or_404("racers", racer_id, repository)


@app.get("/api/races", response_model=list[RaceRecord])
def list_races(repository: RaceRepository = Depends(repo_provider)) -> list[RaceRecord]:
    return [RaceRecord(**item) for item in _list_entities("races", repository)]


@app.post("/api/races", response_model=RaceRecord, status_code=201)
def create_race(
    payload: RaceUpsert, repository: RaceRepository = Depends(repo_provider)
) -> RaceRecord:
    return RaceRecord(**_create_entity("races", payload.dict(), repository))


@app.get("/api/races/{race_id}", response_model=RaceRecord)
def get_race(
    race_id: str, repository: RaceRepository = Depends(repo_provider)
) -> RaceRecord:
    return RaceRecord(**_get_entity_or_404("races", race_id, repository))


@app.put("/api/races/{race_id}", response_model=RaceRecord)
def update_race(
    race_id: str,
    payload: RaceUpsert,
    repository: RaceRepository = Depends(repo_provider),
) -> RaceRecord:
    updated = RaceRecord(
        **_update_entity_or_404("races", race_id, payload.dict(), repository)
    )
    return updated


@app.delete("/api/races/{race_id}", status_code=204)
def delete_race(
    race_id: str, repository: RaceRepository = Depends(repo_provider)
) -> Response:
    return _delete_entity_or_404("races", race_id, repository)


@app.post("/api/races/{race_id}/events", status_code=201)
async def create_race_event(
    race_id: str,
    payload: RaceEventIngest,
    repository: RaceRepository = Depends(repo_provider),
) -> dict:
    _get_entity_or_404("races", race_id, repository)
    event_id = repository.insert_event(race_id, payload)
    event_payload = {
        "id": event_id,
        "raceId": race_id,
        "type": payload.type,
        "carId": payload.carId,
        "racerId": payload.racerId,
        "checkpointId": payload.checkpointId,
        "lapNumber": payload.lapNumber,
        "payload": payload.payload,
        "timestamp": payload.timestamp.isoformat(),
    }
    await ws_manager.broadcast_json({"type": "event", "payload": event_payload})
    await broadcast_dashboard(race_id, repository)
    return event_payload


@app.get("/api/races/{race_id}/dashboard", response_model=RaceDashboardResponse)
def get_dashboard(
    race_id: str, repository: RaceRepository = Depends(repo_provider)
) -> RaceDashboardResponse:
    return build_dashboard(race_id, repository)


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
