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


def detect_colors(frame, racers, tolerance=8, min_area=40, region=None):
    """Reject ambiguous colors rather than guessing between two cars or scenery."""
    height, width = frame.shape[:2]
    if region is None:
        x0, y0, x1, y1 = 0, 0, width, height
    else:
        left, top, right, bottom = region
        x0, y0 = max(0, round(left * width)), max(0, round(top * height))
        x1, y1 = min(width, round(right * width)), min(height, round(bottom * height))
        if x0 >= x1 or y0 >= y1:
            return []
    hsv = cv2.cvtColor(frame[y0:y1, x0:x1], cv2.COLOR_BGR2HSV)
    detections = []
    usable = []
    for racer in racers:
        try:
            usable.append((racer, hue_for_color(racer["color"])))
        except ValueError:
            # Keep the racer in the race ledger, but never invent a color match.
            continue
    for racer, hue in usable:
        if any(
            other["id"] != racer["id"]
            and min(abs(hue - other_hue), 180 - abs(hue - other_hue)) <= 16
            for other, other_hue in usable
        ):
            continue
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
        detections.append(
            {
                "racer_id": racer["id"],
                "name": racer["name"],
                "x": (x0 + x + w / 2) / width,
                "y": (y0 + y + h / 2) / height,
                "box": [x0 + x, y0 + y, w, h],
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
