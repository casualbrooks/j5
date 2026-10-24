"""Color identity and directed finish crossings, independent of browser clients."""

import colorsys
from dataclasses import dataclass

import cv2
import numpy as np


def hue_for_color(color):
    rgb = tuple(int(color[i : i + 2], 16) / 255 for i in (1, 3, 5))
    hue, saturation, value = colorsys.rgb_to_hsv(*rgb)
    if saturation < 0.3 or value < 0.2:
        raise ValueError(
            "Choose a saturated car color; black, white and grey require another identification method"
        )
    return round(hue * 179)


def detect_colors(frame, racers, tolerance=8, min_area=40):
    """Reject ambiguous colors rather than guessing between two cars or scenery."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    detections = []
    for racer in racers:
        hue = hue_for_color(racer["color"])
        distance = np.abs(hsv[:, :, 0].astype(np.int16) - hue)
        distance = np.minimum(distance, 180 - distance)
        mask = (
            (distance <= tolerance) & (hsv[:, :, 1] >= 75) & (hsv[:, :, 2] >= 45)
        ).astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blobs = [c for c in contours if cv2.contourArea(c) >= min_area]
        if len(blobs) != 1:
            continue
        x, y, w, h = cv2.boundingRect(blobs[0])
        height, width = frame.shape[:2]
        detections.append(
            {
                "racer_id": racer["id"],
                "name": racer["name"],
                "x": (x + w / 2) / width,
                "y": (y + h / 2) / height,
                "box": [x, y, w, h],
            }
        )
    return detections


@dataclass
class CrossingState:
    side: int = 0
    last_seen: float = 0
    last_crossing: float | None = None


class FinishGate:
    """First forward crossing arms a lap. Missing frames never imply a crossing."""

    def __init__(self, x=0.5, y_min=0.1, y_max=0.9, direction=1, minimum_lap_ms=2000):
        self.x, self.y_min, self.y_max = x, y_min, y_max
        self.direction, self.minimum_lap_ms = direction, minimum_lap_ms
        self.states = {}

    def observe(self, racer_id, x, y, now):
        state = self.states.setdefault(racer_id, CrossingState())
        # After an occlusion, require a fresh observed approach to the line.
        if now - state.last_seen > 0.5:
            state.side = 0
        state.last_seen = now
        offset = (x - self.x) * self.direction
        side = -1 if offset < -0.06 else 1 if offset > 0.01 else 0
        if not side:
            return None
        crossed = state.side == -1 and side == 1 and self.y_min <= y <= self.y_max
        state.side = side
        if not crossed:
            return None
        if state.last_crossing is None:
            state.last_crossing = now
            return None
        duration = (now - state.last_crossing) * 1000
        if duration < self.minimum_lap_ms:
            return None
        state.last_crossing = now
        return round(duration, 3)
