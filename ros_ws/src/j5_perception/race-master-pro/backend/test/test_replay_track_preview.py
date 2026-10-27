import hashlib
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
from replay_track_preview import build_preview  # noqa: E402


def test_tracker_keeps_frames_after_a_car_crosses_and_rejects_large_motion(tmp_path):
    video = tmp_path / "race.mp4"
    writer = cv2.VideoWriter(
        str(video), cv2.VideoWriter_fourcc(*"mp4v"), 30, (640, 360)
    )
    assert writer.isOpened()
    for index in range(75):
        frame = np.zeros((360, 640, 3), dtype=np.uint8)
        if 10 <= index <= 32:
            x = 480 - 14 * (index - 10)
            cv2.rectangle(frame, (x, 213), (x + 18, 222), (0, 0, 255), -1)
            cv2.rectangle(frame, (x, 228), (x + 90, 257), (200, 200, 200), -1)
        writer.write(frame)
    writer.release()
    manifest = {
        "race_id": "test-race",
        "video_sha256": hashlib.sha256(video.read_bytes()).hexdigest(),
        "video_start_s": 0,
        "video_end_s": 2.4,
        "finish_gate": {
            "finish_x": 0.55,
            "y_min": 0.59,
            "y_max": 0.66,
            "direction": -1,
        },
    }
    preview = build_preview(video, manifest)
    assert len(preview["tracks"]) == 1
    track = preview["tracks"][0]
    assert track["motion_evidence"]["consistent"]
    assert track["crossing_confidence"] > 0.9
    assert track["points"][0]["second"] < track["crossing_second"]
    assert track["points"][-1]["second"] > track["crossing_second"] + 0.1
