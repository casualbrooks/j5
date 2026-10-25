import cv2
import numpy as np
from fastapi.testclient import TestClient

from app import database
from app.main import app


def _video(path):
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), 30, (100, 80))
    assert writer.isOpened()
    for index in range(120):
        writer.write(np.full((80, 100, 3), index % 200, dtype=np.uint8))
    writer.release()


def _proposal(manifest, shift=0):
    first, second = [racer["id"] for racer in manifest["racers"]]
    crossings = []
    for racer_id, times in (
        (first, (0.5, 1.5 + shift, 2.5)),
        (second, (0.8, 1.8, 2.8)),
    ):
        crossings.extend(
            {
                "racer_id": racer_id,
                "second": time,
                "confidence": 0.98,
                "evidence_seconds": [time - 0.06, time, time + 0.06],
            }
            for time in times
        )
    return {
        "race_id": manifest["race_id"],
        "video_sha256": manifest["video_sha256"],
        "model": "test-temporal-review",
        "crossings": sorted(crossings, key=lambda event: event["second"]),
        "unassigned_crossings": 0,
    }


def test_offboard_proposal_replaces_laps_and_rescores_finished_race(
    tmp_path, monkeypatch
):
    monkeypatch.setattr(database, "DB_PATH", str(tmp_path / "race.db"))
    directory = tmp_path / "videos"
    directory.mkdir()
    _video(directory / "heat.mp4")
    with TestClient(app) as client:
        session = client.post(
            "/api/quick/session",
            json={
                "source": "video:heat.mp4",
                "minimum_lap_ms": 500,
                "total_laps": 2,
                "racers": [
                    {"name": "Alpha", "color": "#ff0000"},
                    {"name": "Beta", "color": "#0000ff"},
                ],
            },
        ).json()
        race_id = session["race_id"]
        assert client.post(f"/api/races/{race_id}/start").status_code == 200
        assert client.post(f"/api/races/{race_id}/pause").status_code == 200
        job = client.post(f"/api/quick/review/{race_id}/request")
        assert job.status_code == 200, job.text
        assert client.get("/api/quick/review/jobs").json()[0]["race_id"] == race_id
        manifest = client.get(f"/api/quick/review/{race_id}/manifest").json()
        assert manifest["video_start_s"] == 0
        assert len(manifest["video_sha256"]) == 64
        assert client.get(f"/api/quick/review/{race_id}/video").status_code == 200

        proposal = _proposal(manifest)
        sent = client.post(f"/api/quick/review/{race_id}/proposal", json=proposal)
        assert sent.status_code == 200, sent.text
        assert sent.json()["status"] == "ready"
        assert sent.json()["summary"]["proposed_laps"] == {
            racer["id"]: 2 for racer in manifest["racers"]
        }
        applied = client.post(f"/api/quick/review/{race_id}/apply")
        assert applied.status_code == 200, applied.text
        assert applied.json()["status"] == "applied"
        laps = client.get(f"/api/laps?race_id={race_id}").json()
        assert len(laps) == 4
        assert sorted(row["lap_time"] for row in laps) == [1000.0] * 4
        assert client.post(f"/api/races/{race_id}/finish").status_code == 200

        revised = _proposal(manifest, shift=0.1)
        sent = client.post(f"/api/quick/review/{race_id}/proposal", json=revised)
        assert sent.status_code == 200, sent.text
        assert client.post(f"/api/quick/review/{race_id}/apply").status_code == 200
        laps = client.get(f"/api/laps?race_id={race_id}").json()
        assert sorted(row["lap_time"] for row in laps) == [
            900.0,
            1000.0,
            1000.0,
            1100.0,
        ]
        results = client.get(f"/api/results?race_id={race_id}").json()
        assert len(results) == 2
        assert sorted(result["points_earned"] for result in results) == [7, 10]
        assert (
            client.post(f"/api/quick/review/{race_id}/apply").json()["status"]
            == "applied"
        )


def test_incomplete_or_wrong_video_proposal_cannot_replace_ledger(
    tmp_path, monkeypatch
):
    monkeypatch.setattr(database, "DB_PATH", str(tmp_path / "race.db"))
    directory = tmp_path / "videos"
    directory.mkdir()
    _video(directory / "heat.mp4")
    with TestClient(app) as client:
        race_id = client.post(
            "/api/quick/session",
            json={
                "source": "video:heat.mp4",
                "racers": [{"name": "A", "color": "#ff0000"}],
            },
        ).json()["race_id"]
        assert client.post(f"/api/races/{race_id}/start").status_code == 200
        assert client.post(f"/api/races/{race_id}/pause").status_code == 200
        manifest = client.get(f"/api/quick/review/{race_id}/manifest").json()
        proposal = {
            "race_id": race_id,
            "video_sha256": "0" * 64,
            "model": "test",
            "crossings": [],
            "unassigned_crossings": 1,
        }
        assert (
            client.post(
                f"/api/quick/review/{race_id}/proposal", json=proposal
            ).status_code
            == 409
        )
        proposal["video_sha256"] = manifest["video_sha256"]
        sent = client.post(f"/api/quick/review/{race_id}/proposal", json=proposal)
        assert sent.status_code == 200
        assert sent.json()["status"] == "incomplete"
        assert client.post(f"/api/quick/review/{race_id}/apply").status_code == 409
        assert client.get(f"/api/laps?race_id={race_id}").json() == []
