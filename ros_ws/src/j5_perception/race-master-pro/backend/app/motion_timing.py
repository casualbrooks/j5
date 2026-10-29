"""Camera-only finish-line observations, without guessing a car's identity."""

from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class _Track:
    x: float
    y: float
    box: tuple[int, int, int, int]
    last_seen: float
    hits: int = 1
    approached: bool = False
    emitted: bool = False


class MotionFinishDetector:
    """Track moving foreground blobs through one directed finish line.

    The first frames calibrate a stationary background. Observations are
    anonymous by design: lane changes and similar cars make identity uncertain.
    """

    def __init__(self, finish_x=0.5, y_min=0.1, y_max=0.9, direction=1):
        self.finish_x = finish_x
        self.y_min = y_min
        self.y_max = y_max
        self.direction = direction
        self.subtractor = cv2.createBackgroundSubtractorMOG2(
            history=200, varThreshold=18, detectShadows=False
        )
        self.calibration_frames = 0
        self.tracks = []
        self.next_id = 1
        self.recent = deque(maxlen=500)
        self.total = 0
        self.boxes = []
        self.frames_seen = 0
        self.first_stamp = None
        self.last_stamp = None

    @property
    def processed_fps(self):
        if self.first_stamp is None or self.last_stamp == self.first_stamp:
            return 0.0
        return round((self.frames_seen - 1) / (self.last_stamp - self.first_stamp), 1)

    @property
    def calibrated(self):
        return self.calibration_frames >= 21

    def observe(self, frame, stamp):
        self.frames_seen += 1
        self.first_stamp = stamp if self.first_stamp is None else self.first_stamp
        self.last_stamp = stamp
        height, width = frame.shape[:2]
        line = round(self.finish_x * width)
        half_width = max(40, round(width * 0.09))
        x0, x1 = max(0, line - half_width), min(width, line + half_width)
        y0, y1 = round(self.y_min * height), round(self.y_max * height)
        crop = frame[y0:y1, x0:x1]
        if crop.size == 0:
            return []
        mask = self.subtractor.apply(
            crop, learningRate=0.1 if not self.calibrated else 0.005
        )
        if not self.calibrated:
            self.calibration_frames += 1
            return []
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
        count, _, stats, centroids = cv2.connectedComponentsWithStats(mask)
        detections = []
        for index in range(1, count):
            x, y, w, h, area = map(int, stats[index])
            if not (9 <= area <= 500 and w <= 65 and h <= 20):
                continue
            detections.append(
                (
                    x0 + centroids[index, 0],
                    y0 + centroids[index, 1],
                    (x0 + x, y0 + y, w, h),
                )
            )
        self.boxes = [box for _, _, box in detections]
        self.tracks = [track for track in self.tracks if stamp - track.last_seen < 0.35]
        used = set()
        events = []
        for x, y, box in detections:
            choices = [
                (abs(x - track.x) + 2 * abs(y - track.y), track)
                for track in self.tracks
                if id(track) not in used
                and abs(x - track.x) <= 35
                and abs(y - track.y) <= 13
            ]
            if choices:
                _, track = min(choices, key=lambda item: item[0])
                used.add(id(track))
                track.x, track.y, track.box = x, y, box
                track.last_seen = stamp
                track.hits += 1
            else:
                track = _Track(x, y, box, stamp)
                self.tracks.append(track)
                used.add(id(track))
            side = (x - line) * self.direction
            if side < -max(8, width * 0.018):
                track.approached = True
            if (
                track.approached
                and not track.emitted
                and side > max(5, width * 0.01)
                and track.hits >= 2
            ):
                track.emitted = True
                event = {
                    "id": self.next_id,
                    "timestamp": stamp,
                    "x": round(x / width, 4),
                    "y": round(y / height, 4),
                    "box": list(box),
                    "identity": None,
                }
                self.next_id += 1
                self.total += 1
                self.recent.append(event)
                events.append(event)
        return events
