import asyncio
import time

import pytest
import numpy as np
import cv2
from fastapi.testclient import TestClient

from app import database
from app.main import app
from app.models import LapRecordCreate
from app.quickstart import runtime, TimingRuntime
from app.race_service import save_lap


@pytest.fixture
def client(tmp_path, monkeypatch):
    monkeypatch.setattr(database, "DB_PATH", str(tmp_path / "race.db"))
    monkeypatch.setattr(
        runtime.camera,
        "latest",
        lambda: (np.zeros((200, 300, 3), dtype=np.uint8), time.monotonic()),
    )
    runtime.race_key = None
    monkeypatch.setattr(runtime.camera, "source", "/dev/video0")
    with TestClient(app) as client:
        yield client


def prepare(client, laps=2):
    response = client.post(
        "/api/quick/session",
        json={
            "championship_name": "Club",
            "total_laps": laps,
            "racers": [
                {"name": "Red", "color": "#ff0000"},
                {"name": "Blue", "color": "#0000ff"},
            ],
        },
    )
    assert response.status_code == 200, response.text
    state = client.get("/api/quick/state").json()
    runtime.camera.frame_time = time.monotonic()
    assert client.post("/api/quick/control/start").status_code == 200
    return response.json(), state["context"]["racers"]


def lap(client, race_id, racer_id, number, duration):
    return client.post(
        "/api/laps",
        json={
            "race_id": race_id,
            "racer_profile_id": racer_id,
            "lap_number": number,
            "lap_time": duration,
        },
    )


def test_finish_scores_once_and_survives_restart(client):
    session, racers = prepare(client)
    rid = session["race_id"]
    for racer, durations in zip(racers, ([3100, 3200], [3500, 3400])):
        for number, duration in enumerate(durations, 1):
            assert lap(client, rid, racer["id"], number, duration).status_code == 200
    assert client.post("/api/quick/control/finish").status_code == 200
    assert client.post("/api/quick/control/finish").status_code == 200
    standings = client.get(
        f"/api/analytics/championship/{session['championship_id']}/standings"
    ).json()
    assert [s["total_points"] for s in standings] == [25, 18]
    assert len(client.get(f"/api/results?race_id={rid}").json()) == 2
    client.portal.call(runtime.stop)
    with TestClient(app) as restarted:
        assert len(restarted.get(f"/api/laps?race_id={rid}").json()) == 4
        assert restarted.get(f"/api/races/{rid}").json()["status"] == "finished"
        assert (
            restarted.get(
                f"/api/analytics/championship/{session['championship_id']}/standings"
            ).json()
            == standings
        )


def test_lap_retry_is_idempotent_and_conflicts_are_rejected(client):
    session, racers = prepare(client)
    args = (client, session["race_id"], racers[0]["id"], 1, 3000)
    first = lap(*args)
    assert lap(*args).json()["id"] == first.json()["id"]
    assert lap(*args[:-1], 3100).status_code == 409
    assert lap(client, session["race_id"], racers[0]["id"], 3, 3000).status_code == 409
    assert lap(client, session["race_id"], "unknown", 1, 3000).status_code == 400
    assert lap(*args[:-1], -1).status_code == 422
    assert client.post("/api/quick/control/pause").status_code == 200
    assert lap(client, session["race_id"], racers[0]["id"], 2, 3000).status_code == 409


def test_concurrent_delivery_creates_one_lap(client):
    session, racers = prepare(client)
    data = LapRecordCreate(
        race_id=session["race_id"],
        racer_profile_id=racers[0]["id"],
        lap_number=1,
        lap_time=3000,
    )

    async def deliver():
        return await asyncio.gather(save_lap(data), save_lap(data))

    results = asyncio.run(deliver())
    assert sum(created for _, created in results) == 1
    assert len(client.get(f"/api/laps?race_id={session['race_id']}").json()) == 1


def test_unfinished_car_is_dnf_and_scores_zero(client):
    session, racers = prepare(client)
    assert lap(client, session["race_id"], racers[0]["id"], 1, 3000).status_code == 200
    assert client.post("/api/quick/control/finish").status_code == 200
    results = client.get(f"/api/results?race_id={session['race_id']}").json()
    assert len(results) == 2
    assert all(r["status"] == "dnf" and r["points_earned"] == 0 for r in results)


def test_ambiguous_colors_and_invalid_sources_rejected(client):
    assert (
        client.post(
            "/api/quick/session",
            json={
                "racers": [
                    {"name": "A", "color": "#ff0000"},
                    {"name": "B", "color": "#ee0000"},
                ]
            },
        ).status_code
        == 422
    )
    assert (
        client.post(
            "/api/quick/camera", json={"source": "http://example.com/video"}
        ).status_code
        == 400
    )
    assert client.get("/race").status_code == 200


def test_second_round_reuses_racers_and_accumulates_points(client):
    first, racers = prepare(client, laps=1)
    for racer in racers:
        assert (
            lap(
                client,
                first["race_id"],
                racer["id"],
                1,
                3000 if racer == racers[0] else 3500,
            ).status_code
            == 200
        )
    client.post("/api/quick/control/finish")
    second, next_racers = prepare(client, laps=1)
    assert first["championship_id"] == second["championship_id"]
    assert [r["id"] for r in racers] == [r["id"] for r in next_racers]
    for racer in next_racers:
        assert (
            lap(
                client,
                second["race_id"],
                racer["id"],
                1,
                3000 if racer == next_racers[0] else 3500,
            ).status_code
            == 200
        )
    client.post("/api/quick/control/finish")
    standings = client.get(
        f"/api/analytics/championship/{first['championship_id']}/standings"
    ).json()
    assert [s["total_points"] for s in standings] == [50, 36]
    assert all(s["races_completed"] == 2 for s in standings)


def test_camera_frames_are_saved_without_a_browser_counter(client):
    session, racers = prepare(client, laps=1)
    worker = TimingRuntime()

    async def feed():
        for x, stamp in [(100, 10), (180, 10.1), (100, 12), (180, 12.1)]:
            image = np.zeros((200, 300, 3), dtype=np.uint8)
            cv2.rectangle(image, (x, 80), (x + 20, 100), (0, 0, 255), -1)
            await worker.process_frame(image, stamp, now=stamp)

    asyncio.run(feed())
    laps = client.get(f"/api/laps?race_id={session['race_id']}").json()
    assert len(laps) == 1
    assert laps[0]["racer_profile_id"] == racers[0]["id"]
    assert laps[0]["lap_time"] == 2000


def test_camera_loss_pauses_even_before_first_processed_frame(client):
    session, _ = prepare(client)
    worker = TimingRuntime()
    asyncio.run(worker.process_frame(None, 0, now=10))
    assert client.get(f"/api/races/{session['race_id']}").json()["status"] == "paused"
    assert "Camera frames stopped" in worker.error


def test_wrong_camera_cannot_resume_and_active_camera_cannot_reconnect(client):
    session, _ = prepare(client)
    assert (
        client.post("/api/quick/camera", json={"source": "/dev/video1"}).status_code
        == 409
    )
    assert client.post("/api/quick/control/pause").status_code == 200
    runtime.camera.source = "/dev/video1"
    runtime.camera.frame_time = time.monotonic()
    response = client.post("/api/quick/control/start")
    assert response.status_code == 409
    assert client.get(f"/api/races/{session['race_id']}").json()["status"] == "paused"


def test_legacy_paused_context_is_archived_without_blocking_new_camera_race(client):
    from app.main import _get_state_json, _set_state_json

    legacy, _ = prepare(client)
    assert client.post("/api/quick/control/pause").status_code == 200
    context = client.get("/api/quick/state").json()["context"]
    context.pop("server_timing")
    client.portal.call(_set_state_json, "race_context", context)
    next_session, _ = prepare(client)
    assert next_session["race_id"] != legacy["race_id"]
    assert client.get(f"/api/races/{legacy['race_id']}").json()["status"] == "paused"
    archived = client.portal.call(
        _get_state_json, f"race_context:{legacy['race_id']}", {}
    )
    assert archived["racers"] == context["racers"]


def test_camera_recovery_keeps_unrelated_active_race_running(client):
    from app.main import _set_state_json
    from app.quickstart import pause_active_camera_race

    legacy, _ = prepare(client)
    context = client.get("/api/quick/state").json()["context"]
    context.pop("server_timing")
    client.portal.call(_set_state_json, "race_context", context)
    client.portal.call(pause_active_camera_race)
    assert client.get(f"/api/races/{legacy['race_id']}").json()["status"] == "active"
