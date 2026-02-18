"""
Pydantic models for request/response validation.
"""

from typing import Optional
from pydantic import BaseModel, Field
from datetime import datetime
from enum import Enum


class RaceStatusEnum(str, Enum):
    SETUP = "setup"
    ACTIVE = "active"
    PAUSED = "paused"
    FINISHED = "finished"
    CANCELLED = "cancelled"


class RacerStatusEnum(str, Enum):
    RACING = "racing"
    FINISHED = "finished"
    DNF = "dnf"


class RaceTypeEnum(str, Enum):
    PRACTICE = "practice"
    QUALIFYING = "qualifying"
    MAIN = "main"


# ── Request/Response Models ─────────────────────────────────


class SeasonCreate(BaseModel):
    name: str
    year: int
    status: str = "active"


class SeasonResponse(SeasonCreate):
    id: str
    created_at: Optional[str] = None


class ChampionshipCreate(BaseModel):
    season_id: str
    name: str
    points_system: str = (
        '{"1":25,"2":18,"3":15,"4":12,"5":10,"6":8,"7":6,"8":4,"9":2,"10":1}'
    )
    status: str = "active"


class ChampionshipResponse(ChampionshipCreate):
    id: str
    created_at: Optional[str] = None


class TrackCreate(BaseModel):
    name: str
    scale: str = "1:24"
    track_distance: Optional[float] = None
    layout_points: str = "[]"
    boundary_polygon: str = "[]"


class TrackResponse(TrackCreate):
    id: str
    created_at: Optional[str] = None


class CheckpointCreate(BaseModel):
    track_id: str
    name: str
    type: str = "checkpoint"
    position: str = '{"x":0,"y":0}'
    sort_order: int = 0


class CheckpointResponse(CheckpointCreate):
    id: str


class CameraConfigCreate(BaseModel):
    track_id: Optional[str] = None
    name: str
    source: str = "0"
    ros_topic: Optional[str] = None
    calibration: Optional[str] = None
    status: str = "disconnected"


class CameraConfigResponse(CameraConfigCreate):
    id: str


class RacerProfileCreate(BaseModel):
    name: str
    number: str = ""
    profile_pic: Optional[str] = None
    vehicle_description: Optional[str] = None


class RacerProfileResponse(RacerProfileCreate):
    id: str
    created_at: Optional[str] = None


class RaceEventCreate(BaseModel):
    championship_id: Optional[str] = None
    track_id: Optional[str] = None
    name: str
    event_date: Optional[str] = None
    round_number: int = 1


class RaceEventResponse(RaceEventCreate):
    id: str


class RaceCreate(BaseModel):
    event_id: Optional[str] = None
    type: str = "main"
    total_laps: int = 10


class RaceResponse(RaceCreate):
    id: str
    status: str = "setup"
    start_time: Optional[str] = None
    end_time: Optional[str] = None


class RaceResultCreate(BaseModel):
    race_id: str
    racer_profile_id: str
    finish_position: int = 0
    total_time: float = 0
    best_lap_time: Optional[float] = None
    avg_lap_time: Optional[float] = None
    laps_completed: int = 0
    status: str = "racing"
    points_earned: int = 0


class RaceResultResponse(RaceResultCreate):
    id: str


class LapRecordCreate(BaseModel):
    race_id: str
    racer_profile_id: str
    lap_number: int
    lap_time: float
    sector1_time: Optional[float] = None
    sector2_time: Optional[float] = None
    sector3_time: Optional[float] = None


class LapRecordResponse(LapRecordCreate):
    id: str
    recorded_at: Optional[str] = None


class DetectionCreate(BaseModel):
    race_id: Optional[str] = None
    camera_id: Optional[str] = None
    racer_profile_id: Optional[str] = None
    position_x: float
    position_y: float
    confidence: float


class DetectionResponse(DetectionCreate):
    id: str
    recorded_at: Optional[str] = None


# ── WebSocket Message ───────────────────────────────────────


class WSMessage(BaseModel):
    type: str
    data: dict = Field(default_factory=dict)
    timestamp: Optional[str] = None


# ── Position Update (from CV) ──────────────────────────────


class PositionUpdate(BaseModel):
    racer_id: str
    position: int
    lap: int


class LapComplete(BaseModel):
    racer_id: str
    lap_time: float
    total_time: float
