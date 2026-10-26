import cv2
import numpy as np
from fastapi.testclient import TestClient

from app import database
from app.main import app


def _video(path, frames=120):
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), 30, (100, 80))
    assert writer.isOpened()
    for index in range(frames):
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


def test_tracking_preview_is_video_bound_and_does_not_score(tmp_path, monkeypatch):
    monkeypatch.setattr(database, "DB_PATH", str(tmp_path / "race.db"))
    directory = tmp_path / "videos"
    directory.mkdir()
    _video(directory / "heat.mp4")
    with TestClient(app) as client:
        race_id = client.post(
            "/api/quick/session",
            json={
                "source": "video:heat.mp4",
                "video_start_s": 0.5,
                "video_end_s": 3.0,
                "finish_x": 0.55,
                "y_min": 0.59,
                "y_max": 0.66,
                "direction": -1,
                "racers": [{"name": "A", "color": "#ff0000"}],
            },
        ).json()["race_id"]
        info = client.get(f"/api/quick/review/{race_id}/manifest").json()
        preview = {
            "race_id": race_id,
            "video_sha256": info["video_sha256"],
            "source": "test-lower-straight",
            "analysis_gate": info["finish_gate"],
            "tracks": [
                {
                    "track_id": 1,
                    "crossing_second": 1.5,
                    "points": [
                        {"second": 1.45, "x": 0.57, "y": 0.61},
                        {"second": 1.55, "x": 0.53, "y": 0.61},
                    ],
                }
            ],
        }
        endpoint = f"/api/quick/review/{race_id}/tracks"
        wrong_video = {**preview, "video_sha256": "0" * 64}
        assert client.post(endpoint, json=wrong_video).status_code == 409
        wrong_gate = {**preview, "analysis_gate": {**info["finish_gate"], "y_max": 0.7}}
        assert client.post(endpoint, json=wrong_gate).status_code == 409
        out_of_range = {
            **preview,
            "tracks": [
                {
                    **preview["tracks"][0],
                    "points": [
                        {"second": 0.4, "x": 0.57, "y": 0.61},
                        preview["tracks"][0]["points"][1],
                    ],
                }
            ],
        }
        assert client.post(endpoint, json=out_of_range).status_code == 422
        sent = client.post(endpoint, json=preview)
        assert sent.status_code == 200, sent.text
        assert sent.json()["track_count"] == 1
        assert sent.json()["point_count"] == 2
        stored = client.get(endpoint).json()["tracks"]
        assert stored[0]["track_id"] == 1
        assert [point["second"] for point in stored[0]["points"]] == [1.45, 1.55]
        assert (
            client.get("/api/quick/state").json()["tracking_preview"]["id"]
            == sent.json()["id"]
        )
        assignment = client.post(
            f"{endpoint}/assign",
            json={"track_id": 1, "racer_id": info["racers"][0]["id"]},
        )
        assert assignment.status_code == 200, assignment.text
        assert assignment.json()["id"] != sent.json()["id"]
        assert (
            client.get(endpoint).json()["tracks"][0]["assigned_racer_id"]
            == info["racers"][0]["id"]
        )
        assert (
            client.post(
                f"{endpoint}/assign", json={"track_id": 1, "racer_id": "unknown"}
            ).status_code
            == 422
        )
        assert (
            client.post(
                f"{endpoint}/assign", json={"track_id": 999, "excluded": True}
            ).status_code
            == 404
        )
        assert client.get(f"/api/laps?race_id={race_id}").json() == []


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
        video = client.get(f"/api/quick/review/{race_id}/video")
        assert video.status_code == 200
        assert video.headers["content-disposition"].startswith("inline;")

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
                "video_end_s": 3.0,
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
            "observations": [
                {
                    "second": 1.25,
                    "direction": "outward",
                    "x": 0.5,
                    "y": 0.25,
                    "confidence": 0.8,
                    "tracklet_id": 7,
                    "appearance_cluster": 2,
                }
            ],
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
        assert sent.json()["summary"]["observed_outward"] == 1
        assert (
            client.get(f"/api/quick/review/{race_id}/proposal").json()["observations"][
                0
            ]["tracklet_id"]
            == 7
        )
        assert client.post(f"/api/quick/review/{race_id}/apply").status_code == 409
        assert client.get(f"/api/laps?race_id={race_id}").json() == []
        proposal["observations"][0]["second"] = 3.2
        assert (
            client.post(
                f"/api/quick/review/{race_id}/proposal", json=proposal
            ).status_code
            == 422
        )


def test_finish_line_groups_can_be_mapped_without_scoring(tmp_path, monkeypatch):
    monkeypatch.setattr(database, "DB_PATH", str(tmp_path / "race.db"))
    directory = tmp_path / "videos"
    directory.mkdir()
    _video(directory / "heat.mp4", frames=450)
    with TestClient(app) as client:
        race_id = client.post(
            "/api/quick/session",
            json={
                "source": "video:heat.mp4",
                "finish_x": 0.55,
                "y_min": 0.59,
                "y_max": 0.66,
                "direction": -1,
                "racers": [
                    {"name": "A", "color": "#ff0000"},
                    {"name": "B", "color": "#0000ff"},
                ],
            },
        ).json()["race_id"]
        manifest = client.get(f"/api/quick/review/{race_id}/manifest").json()
        proposal = {
            "race_id": race_id,
            "video_sha256": manifest["video_sha256"],
            "model": "test-lower-gate",
            "analysis_gate": {**manifest["finish_gate"], "finish_x": 0.62},
            "crossings": [],
            "unassigned_crossings": 2,
            "observations": [
                {
                    "second": second,
                    "direction": "outward",
                    "x": 0.55,
                    "y": 0.62,
                    "confidence": 0.8,
                    "tracklet_id": index + 1,
                    "appearance_cluster": index,
                }
                for index, second in enumerate((1.0, 2.0, 12.0, 13.0))
            ],
        }
        proposal["observations"][2]["appearance_cluster"] = 0
        proposal["observations"][3]["appearance_cluster"] = 0
        proposal["observations"][0]["y"] = 0.60  # First racing lane.
        proposal["observations"][1]["y"] = 0.65  # Second racing lane.
        proposal["observations"][3]["y"] = 0.55  # Pit lane: outside the saved gate.
        proposal["unassigned_crossings"] = 4
        assert (
            client.post(
                f"/api/quick/review/{race_id}/proposal", json=proposal
            ).status_code
            == 409
        )
        proposal["analysis_gate"] = manifest["finish_gate"]
        sent = client.post(f"/api/quick/review/{race_id}/proposal", json=proposal)
        assert sent.status_code == 200, sent.text
        assert sent.json()["status"] == "incomplete"
        sample = client.get(f"/api/quick/review/{race_id}/sample/1.jpg")
        assert sample.status_code == 200
        assert sample.headers["content-type"] == "image/jpeg"
        first, second = [racer["id"] for racer in manifest["racers"]]
        mapped = client.post(
            f"/api/quick/review/{race_id}/groups",
            json={"appearance_cluster": 0, "racer_id": first},
        )
        assert mapped.status_code == 200, mapped.text
        assert mapped.json()["group_assignments"] == {"0": first}
        assert mapped.json()["status"] == "incomplete"
        assert mapped.json()["candidate_summary"]["candidate_laps"][first] == 1
        assert mapped.json()["candidate_summary"]["candidate_crossings"][first] == 2
        duplicate = client.post(
            f"/api/quick/review/{race_id}/groups",
            json={"appearance_cluster": 1, "racer_id": first},
        )
        assert duplicate.status_code == 200
        assert duplicate.json()["group_assignments"] == {"0": first, "1": first}
        assert duplicate.json()["candidate_summary"]["candidate_laps"][first] == 1
        assert duplicate.json()["candidate_summary"]["ignored_close_passes"][first] == 1
        assert (
            client.post(
                f"/api/quick/review/{race_id}/groups",
                json={"appearance_cluster": 1, "racer_id": second},
            ).status_code
            == 200
        )
        assert client.get(f"/api/laps?race_id={race_id}").json() == []
        assert client.post(f"/api/quick/review/{race_id}/apply").status_code == 409
