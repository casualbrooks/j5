import cv2
import numpy as np
from fastapi.testclient import TestClient

from app import database
from app.main import app
from app.replay_assessment import assess_replay


def test_replay_assessment_reports_anonymous_candidates(tmp_path):
    path = tmp_path / "practice.mp4"
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), 30, (200, 120))
    assert writer.isOpened()
    for index in range(90):
        frame = np.full((120, 200, 3), 35, dtype=np.uint8)
        if 30 <= index < 38:
            x = 65 + (index - 30) * 8
            cv2.rectangle(frame, (x, 45), (x + 12, 52), (230, 230, 230), -1)
        writer.write(frame)
    writer.release()

    result = assess_replay(
        path,
        0,
        3,
        {"finish_x": 0.5, "y_min": 0.1, "y_max": 0.9, "direction": 1},
        4,
    )
    assert result["status"] == "needs_labels"
    assert result["identified_cars"] == 0
    assert result["expected_cars"] == 4
    assert result["resolution"] == [200, 120]
    assert result["windows"][0]["frames"] == 90
    assert all(event["identity"] is None for event in result["windows"][0]["events"])


def test_replay_review_saves_a_label_without_moving_the_camera(tmp_path, monkeypatch):
    monkeypatch.setattr(database, "DB_PATH", str(tmp_path / "race.db"))
    directory = tmp_path / "videos"
    directory.mkdir()
    path = directory / "practice.mp4"
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), 30, (200, 120))
    for index in range(90):
        frame = np.full((120, 200, 3), 35, dtype=np.uint8)
        if 30 <= index < 38:
            x = 65 + (index - 30) * 8
            cv2.rectangle(frame, (x, 45), (x + 12, 52), (230, 230, 230), -1)
        writer.write(frame)
    writer.release()
    with TestClient(app) as client:
        response = client.post(
            "/api/quick/session",
            json={
                "source": "video:practice.mp4",
                "racers": [{"name": "A", "color": "#ff0000"}],
            },
        )
        assert response.status_code == 200, response.text
        assert client.post("/api/quick/replay/assess").status_code == 200
        import time

        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            state = client.get("/api/quick/state").json()
            if state["replay_assessment"]["status"] != "running":
                break
            time.sleep(0.02)
        report = state["replay_assessment"]
        assert report["status"] == "needs_labels", report
        event = report["windows"][0]["events"][0]
        frame = client.get(f"/api/quick/replay/frame.jpg?second={event['second']}")
        assert frame.status_code == 200
        assert frame.headers["content-type"] == "image/jpeg"
        racer_id = state["context"]["racers"][0]["id"]
        saved = client.post(
            "/api/quick/replay/label",
            json={"event_key": event["key"], "label": racer_id},
        )
        assert saved.status_code == 200, saved.text
        assert (
            client.get("/api/quick/state").json()["replay_labels"][event["key"]]
            == racer_id
        )
        assert state["camera"]["source"] is None
