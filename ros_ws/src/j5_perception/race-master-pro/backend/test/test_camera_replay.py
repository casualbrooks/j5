import time

import cv2
import numpy as np

from app import database
from app.camera import Camera


def wait_for_frame(camera, after=0, timeout=3):
    deadline = time.monotonic() + timeout
    while camera.frame_time <= after and time.monotonic() < deadline:
        time.sleep(0.02)
    assert camera.frame_time > after, camera.error


def test_replay_pauses_and_resumes_at_same_position(tmp_path, monkeypatch):
    monkeypatch.setattr(database, "DB_PATH", str(tmp_path / "race.db"))
    directory = tmp_path / "videos"
    directory.mkdir()
    path = directory / "clip.mp4"
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), 10, (64, 48))
    for number in range(30):
        writer.write(np.full((48, 64, 3), number, dtype=np.uint8))
    writer.release()
    camera = Camera()
    try:
        camera.start("video:clip.mp4")
        wait_for_frame(camera)
        camera.pause()
        time.sleep(0.15)  # Let any read already underway finish.
        paused_frame = camera.frame_time
        paused_second = camera.video_second
        time.sleep(0.25)
        assert camera.paused
        assert camera.frame_time == paused_frame
        assert camera.video_second == paused_second
        camera.resume()
        wait_for_frame(camera, after=paused_frame)
        assert camera.video_second > paused_second
    finally:
        camera.stop()
