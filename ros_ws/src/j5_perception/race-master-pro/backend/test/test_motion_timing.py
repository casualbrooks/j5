import asyncio

import cv2
import numpy as np
from fastapi.testclient import TestClient

from app import database
from app.main import app
from app.motion_timing import MotionFinishDetector
from app.quickstart import TimingRuntime, validate_source


def _frame(*cars):
    frame = np.full((120, 200, 3), 35, dtype=np.uint8)
    for x, y in cars:
        cv2.rectangle(frame, (x, y), (x + 12, y + 7), (220, 220, 220), -1)
    return frame


def _calibrate(detector):
    for index in range(21):
        assert detector.observe(_frame(), index / 30) == []
    assert detector.calibrated


def test_two_unmarked_cars_cross_once_in_separate_track_bands():
    detector = MotionFinishDetector(finish_x=0.5, y_min=0.1, y_max=0.9)
    _calibrate(detector)
    events = []
    for index, x in enumerate((65, 76, 87, 98, 109, 120)):
        events.extend(detector.observe(_frame((x, 35), (x, 65)), 1 + index / 30))
    assert len(events) == 2
    assert detector.total == 2
    assert all(event["identity"] is None for event in events)
    assert all(
        detector.observe(_frame((130, 35), (130, 65)), 2 + i / 30) == []
        for i in range(3)
    )


def test_reverse_and_large_foreground_object_are_not_counted():
    detector = MotionFinishDetector(finish_x=0.5, y_min=0.1, y_max=0.9)
    _calibrate(detector)
    for index, x in enumerate((125, 114, 103, 92, 81, 70)):
        assert detector.observe(_frame((x, 50)), 1 + index / 30) == []
    large = _frame()
    cv2.rectangle(large, (50, 20), (150, 80), (220, 220, 220), -1)
    assert detector.observe(large, 2) == []
    assert detector.total == 0


def test_replay_source_is_limited_to_recordings_in_pi_home():
    validate_source("video:/home/piuser/videotest.mp4")
    for source in (
        "video:/etc/passwd",
        "video:/home/piuser/../secret.mp4",
        "http://example.com/video",
    ):
        try:
            validate_source(source)
        except ValueError:
            pass
        else:
            raise AssertionError(f"Accepted unsafe source: {source}")


def test_diagnostic_passes_do_not_write_championship_laps(tmp_path, monkeypatch):
    monkeypatch.setattr(database, "DB_PATH", str(tmp_path / "race.db"))
    with TestClient(app) as client:
        worker = TimingRuntime()

        async def feed():
            for index in range(21):
                await worker.process_frame(_frame(), 1 + index / 30, now=1 + index / 30)
            for index, x in enumerate((65, 76, 87, 98, 109, 120)):
                await worker.process_frame(
                    _frame((x, 50)), 2 + index / 30, now=2 + index / 30
                )

        client.portal.call(feed)
        assert worker.motion.total == 1
        assert len(worker.motion_evidence) == 1
        assert client.get("/api/laps").json() == []
