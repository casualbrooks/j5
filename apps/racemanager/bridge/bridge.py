"""HTTP bridge between the lap counter ROS topic and the race manager service.

The bridge is designed to be ROS-agnostic so it can be swapped between
live nodes and prerecorded video replays. Replace the `emit_lap` call
with ROS 2 subscription callbacks in deployment.
"""

from __future__ import annotations

import json
import logging
from datetime import datetime
from typing import TYPE_CHECKING, Optional

import requests

logger = logging.getLogger(__name__)


class LapEvent:
    """Lightweight Lap representation safe for older Python interpreters."""

    if TYPE_CHECKING:
        raceId: str
        carId: str
        sessionLap: int
        lapTimeMs: int
        trackDistanceM: float
        timestamp: datetime
        onTrack: bool
        directionOk: bool
        minLapTimeOk: bool
        source: str
        replayId: Optional[str]

    def __init__(
        self,
        raceId: str,
        carId: str,
        sessionLap: int,
        lapTimeMs: int,
        trackDistanceM: float,
        timestamp: datetime,
        onTrack: bool,
        directionOk: bool,
        minLapTimeOk: bool,
        *,
        source: str = "live",
        replayId: Optional[str] = None,
    ) -> None:
        self.raceId = raceId
        self.carId = carId
        self.sessionLap = sessionLap
        self.lapTimeMs = lapTimeMs
        self.trackDistanceM = trackDistanceM
        self.timestamp = timestamp
        self.onTrack = onTrack
        self.directionOk = directionOk
        self.minLapTimeOk = minLapTimeOk
        self.source = source
        self.replayId = replayId

    def as_payload(self) -> dict:
        validity = {
            "onTrack": self.onTrack,
            "directionOk": self.directionOk,
            "minLapTimeOk": self.minLapTimeOk,
        }
        base = dict(self.__dict__)
        for key in ["onTrack", "directionOk", "minLapTimeOk"]:
            base.pop(key)
        base["timestamp"] = self.timestamp.isoformat()
        base["validity"] = validity
        return base


class RaceManagerBridge:
    def __init__(self, service_base_url: str) -> None:
        self.service_base_url = service_base_url.rstrip("/")

    def emit_lap(self, lap: LapEvent) -> dict:
        url = f"{self.service_base_url}/ingest/lap"
        payload = lap.as_payload()
        logger.debug("Posting lap payload: %s", json.dumps(payload, default=str))
        response = requests.post(url, json=payload, timeout=5)
        response.raise_for_status()
        return response.json()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    demo = RaceManagerBridge("http://localhost:4000")
    sample_lap = LapEvent(
        raceId="demo-race",
        carId="car-42",
        sessionLap=1,
        lapTimeMs=72345,
        trackDistanceM=350.0,
        timestamp=datetime.utcnow(),
        onTrack=True,
        directionOk=True,
        minLapTimeOk=True,
        source="replay",
        replayId="mp4-smoke-test",
    )
    logger.info("Service response: %s", demo.emit_lap(sample_lap))
