"""Minimal FastAPI race manager service aligned with lap counter outputs."""

from __future__ import annotations

from datetime import datetime

try:
    from typing import Annotated
except ImportError:  # pragma: no cover - fallback for older Python
    from typing_extensions import Annotated

from fastapi import Depends, FastAPI, HTTPException
from fastapi.responses import JSONResponse
from pymongo.errors import PyMongoError

from .config import Settings, get_settings
from .repository import RaceRepository, get_repository
from .schemas import LapIngest, LapResponse, LeaderboardResponse

app = FastAPI(title="J5 Race Manager", version="0.1.0")


class RepoProvider:
    def __init__(self) -> None:
        self._repo: RaceRepository | None = None

    def __call__(
        self, settings: Annotated[Settings, Depends(get_settings)]
    ) -> RaceRepository:
        if self._repo is None:
            self._repo = get_repository(
                settings.atlas_uri, settings.race_db_name, settings.laps_collection
            )
        return self._repo


def calculate_speed_kph(track_distance_m: float, lap_time_ms: int) -> float:
    if lap_time_ms <= 0:
        raise ValueError("lap_time_ms must be positive")
    meters_per_ms = track_distance_m / lap_time_ms
    return round(meters_per_ms * 3600, 3)  # convert to kph with three decimals


repo_provider = RepoProvider()


@app.get("/health")
def health(settings: Annotated[Settings, Depends(get_settings)]) -> JSONResponse:
    return JSONResponse(
        {"status": "ok", "config": settings.dict(include_sensitive=False)}
    )


@app.post("/ingest/lap", response_model=LapResponse)
def ingest_lap(
    payload: LapIngest,
    repository: Annotated[RaceRepository, Depends(repo_provider)],
) -> LapResponse:
    speed = calculate_speed_kph(payload.trackDistanceM, payload.lapTimeMs)
    try:
        inserted_id = repository.insert_lap(payload, speed)
    except PyMongoError as exc:  # pragma: no cover - passthrough for runtime failures
        raise HTTPException(status_code=500, detail=str(exc)) from exc
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
    except PyMongoError as exc:  # pragma: no cover
        raise HTTPException(status_code=500, detail=str(exc)) from exc


@app.get("/")
def root() -> dict[str, str]:
    return {
        "message": "Race manager service running",
        "timestamp": datetime.utcnow().isoformat(),
    }
