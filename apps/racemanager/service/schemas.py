"""Pydantic models for race manager CRUD, event ingestion, and dashboard responses."""

from __future__ import annotations

from datetime import datetime
from typing import Any, Literal, Optional

from pydantic import BaseModel, Field, validator


class Point(BaseModel):
    x: float
    y: float


class LineGate(BaseModel):
    a: Point
    b: Point
    label: Optional[str] = None


class CheckpointSpec(BaseModel):
    id: str
    name: str
    type: Literal["start", "finish", "checkpoint", "sector"] = "checkpoint"
    position: Point
    order: int = 0


class TrackUpsert(BaseModel):
    name: str
    lengthM: Optional[float] = Field(default=None, gt=0)
    imageUrl: Optional[str] = None
    layoutPoints: list[Point] = Field(default_factory=list)
    maskPolygon: list[Point] = Field(default_factory=list)
    startGate: Optional[LineGate] = None
    finishGate: Optional[LineGate] = None
    checkpoints: list[CheckpointSpec] = Field(default_factory=list)
    calibration: dict[str, Any] = Field(default_factory=dict)


class TrackRecord(TrackUpsert):
    id: str
    createdAt: datetime
    updatedAt: datetime


class CarUpsert(BaseModel):
    name: str
    carNumber: str = ""
    paintColor: str = "#4ade80"
    trackerHint: Optional[str] = None
    photoUrl: Optional[str] = None
    notes: Optional[str] = None


class CarRecord(CarUpsert):
    id: str
    createdAt: datetime
    updatedAt: datetime


class RacerUpsert(BaseModel):
    name: str
    team: Optional[str] = None
    badgeColor: Optional[str] = None


class RacerRecord(RacerUpsert):
    id: str
    createdAt: datetime
    updatedAt: datetime


class RaceEntry(BaseModel):
    carId: str
    racerId: str
    gridSlot: Optional[int] = None
    trackerId: Optional[str] = None


class RaceUpsert(BaseModel):
    name: str
    trackId: Optional[str] = None
    totalLaps: int = Field(default=10, ge=1)
    status: Literal["setup", "active", "paused", "finished"] = "setup"
    entries: list[RaceEntry] = Field(default_factory=list)


class RaceRecord(RaceUpsert):
    id: str
    createdAt: datetime
    updatedAt: datetime


class Validity(BaseModel):
    onTrack: bool = Field(
        ...,
        description="True when the perception mask confirms the car stayed on track",
    )
    directionOk: bool = Field(
        ..., description="True when heading is aligned with track direction"
    )
    minLapTimeOk: bool = Field(
        ..., description="True when lap duration is above the minimum threshold"
    )

    @property
    def is_valid(self) -> bool:
        return self.onTrack and self.directionOk and self.minLapTimeOk


class LapIngest(BaseModel):
    raceId: str = Field(..., description="Race/session identifier")
    carId: str = Field(..., description="Unique car ID; use transponder/camera ID")
    sessionLap: int = Field(..., ge=1, description="Lap number within this session")
    lapTimeMs: int = Field(..., gt=0, description="Lap duration in milliseconds")
    trackDistanceM: float = Field(
        ..., gt=0, description="Track length for speed calculations"
    )
    timestamp: datetime = Field(..., description="Wall-clock time the lap finished")
    validity: Validity
    source: str = Field(
        "live", description="Either 'live' or 'replay' to tag prerecorded runs"
    )
    replayId: Optional[str] = Field(
        None, description="Identifier for the replay/video used to generate this lap"
    )

    @validator("source")
    def validate_source(cls, value: str) -> str:
        normalized = value.lower()
        if normalized not in {"live", "replay"}:
            raise ValueError("source must be 'live' or 'replay'")
        return normalized


class RaceEventIngest(BaseModel):
    type: Literal[
        "lap_count",
        "checkpoint_crossing",
        "off_track",
        "finish_crossing",
        "car_assignment_changed",
        "incident",
    ]
    carId: Optional[str] = None
    racerId: Optional[str] = None
    checkpointId: Optional[str] = None
    lapNumber: Optional[int] = Field(default=None, ge=0)
    timestamp: datetime = Field(default_factory=datetime.utcnow)
    payload: dict[str, Any] = Field(default_factory=dict)


class LapResponse(BaseModel):
    insertedId: str
    speedKph: float
    isValid: bool


class LeaderboardEntry(BaseModel):
    carId: str
    lapCount: int
    bestLapMs: int
    avgSpeedKph: float
    totalTimeMs: int
    gapToLeaderMs: int


class LeaderboardResponse(BaseModel):
    raceId: str
    leaderboard: list[LeaderboardEntry]
    asOf: datetime


class DashboardEntry(BaseModel):
    carId: str
    racerId: Optional[str] = None
    carName: str
    racerName: str
    carNumber: str = ""
    paintColor: str = "#4ade80"
    trackerId: Optional[str] = None
    currentLap: int = 0
    bestLapMs: Optional[int] = None
    avgSpeedKph: Optional[float] = None
    gapToLeaderMs: int = 0
    totalTimeMs: int = 0


class RaceDashboardResponse(BaseModel):
    race: RaceRecord
    track: Optional[TrackRecord] = None
    leaderboard: LeaderboardResponse
    entries: list[DashboardEntry]
    recentEvents: list[RaceEventIngest | dict[str, Any]]
