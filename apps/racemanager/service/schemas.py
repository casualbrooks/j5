"""Pydantic models for lap ingestion and responses."""

from __future__ import annotations

from datetime import datetime
from typing import Optional

from pydantic import BaseModel, Field, validator


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
