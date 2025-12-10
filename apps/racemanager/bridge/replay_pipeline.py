"""Utilities for feeding prerecorded video through the lap pipeline.

The bridge keeps the interface narrow: provide a callback that accepts
frames and returns completed laps. The replay runner forwards those laps
into the race manager service using the same HTTP contract as live laps.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable

try:
    import cv2  # type: ignore
except ImportError:  # pragma: no cover - optional dependency
    cv2 = None  # type: ignore

from .bridge import LapEvent, RaceManagerBridge

logger = logging.getLogger(__name__)


@dataclass
class ReplayConfig:
    race_id: str
    replay_id: str
    track_distance_m: float
    service_base_url: str
    min_lap_ms: int


class ReplayRunner:
    def __init__(
        self, config: ReplayConfig, lap_callback: Callable[[bytes], Iterable[LapEvent]]
    ):
        self.config = config
        self.lap_callback = lap_callback
        self.bridge = RaceManagerBridge(config.service_base_url)

    def run(self, video_path: str) -> int:
        if cv2 is None:
            raise RuntimeError(
                "opencv-python is required for replay ingestion but is not installed"
            )
        path = Path(video_path)
        if not path.exists():
            raise FileNotFoundError(video_path)

        cap = cv2.VideoCapture(str(path))
        emitted = 0
        try:
            while cap.isOpened():
                success, frame = cap.read()
                if not success:
                    break
                for lap in self.lap_callback(frame):
                    self._forward_lap(lap)
                    emitted += 1
        finally:
            cap.release()
        logger.info("Emitted %s laps from replay %s", emitted, self.config.replay_id)
        return emitted

    def _forward_lap(self, lap: LapEvent) -> None:
        lap.replayId = self.config.replay_id
        lap.raceId = self.config.race_id
        lap.source = "replay"
        response = self.bridge.emit_lap(lap)
        logger.debug("Replay lap sent: %s", response)
