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
    assert [s["total_points"] for s in standings] == [10, 7]
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


def test_video_round_scores_completed_laps_even_below_target(client):
    session, racers = prepare(client)
    assert lap(client, session["race_id"], racers[0]["id"], 1, 3000).status_code == 200
    assert client.post("/api/quick/control/finish").status_code == 200
    results = client.get(f"/api/results?race_id={session['race_id']}").json()
    assert len(results) == 2
    ranked = sorted(results, key=lambda row: row["finish_position"])
    assert [(r["status"], r["points_earned"]) for r in ranked] == [
        ("finished", 10),
        ("dnf", 0),
    ]


def test_empty_round_awards_no_points_and_custom_points_are_saved(client):
    response = client.post(
        "/api/quick/session",
        json={
            "racers": [{"name": "Red", "color": "#ff0000"}],
            "points_by_place": [12, 8, 6, 3],
        },
    )
    assert response.status_code == 200, response.text
    championship = client.get(
        f"/api/championships/{response.json()['championship_id']}"
    ).json()
    assert championship["points_system"] == '{"1": 12, "2": 8, "3": 6, "4": 3}'
    runtime.camera.frame_time = time.monotonic()
    assert client.post("/api/quick/control/start").status_code == 200
    assert client.post("/api/quick/control/finish").status_code == 200
    result = client.get(f"/api/results?race_id={response.json()['race_id']}").json()[0]
    assert (result["status"], result["points_earned"]) == ("dnf", 0)


def test_ambiguous_colors_warn_and_invalid_sources_are_rejected(client):
    response = client.post(
        "/api/quick/session",
        json={
            "racers": [
                {"name": "A", "color": "#ff0000"},
                {"name": "B", "color": "#ee0000"},
            ]
        },
    )
    assert response.status_code == 200
    assert len(response.json()["color_warnings"]) == 1
    assert (
        client.post(
            "/api/quick/camera", json={"source": "http://example.com/video"}
        ).status_code
        == 400
    )
    assert client.get("/race").status_code == 200


def test_prepared_race_can_be_saved_again_without_creating_another_round(client):
    payload = {
        "championship_name": "Club",
        "event_name": "Practice",
        "racers": [{"name": "Red", "color": "#ff0000"}],
    }
    first = client.post("/api/quick/session", json=payload).json()
    changed = {**payload, "event_name": "Final", "total_laps": 25, "finish_x": 0.61}
    second = client.post("/api/quick/session", json=changed)
    assert second.status_code == 200, second.text
    assert second.json()["race_id"] == first["race_id"]
    assert second.json()["updated"] is True
    state = client.get("/api/quick/state").json()
    assert state["race"]["total_laps"] == 25
    assert state["config"]["finish_x"] == 0.61
    events = client.portal.call(database.query, "SELECT id FROM race_events")
    assert len(events) == 1


def test_uploaded_video_can_be_selected_and_invalid_video_is_rejected(client, tmp_path):
    path = tmp_path / "clip.mp4"
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), 10, (64, 48))
    writer.write(np.zeros((48, 64, 3), dtype=np.uint8))
    writer.release()
    response = client.post("/api/quick/videos/clip.mp4", content=path.read_bytes())
    assert response.status_code == 200, response.text
    source = response.json()["source"]
    assert source in [v["source"] for v in client.get("/api/quick/videos").json()]
    assert client.post("/api/quick/camera", json={"source": source}).status_code == 200
    assert (
        client.post("/api/quick/videos/broken.mp4", content=b"bad").status_code == 400
    )
    assert (
        client.post("/api/quick/videos/escape.txt", content=b"bad").status_code == 400
    )
    seek = client.post("/api/quick/camera/seek", json={"second": 0})
    assert seek.status_code == 200, seek.text
    assert seek.json()["paused"] is True
    assert client.get("/api/quick/frame.jpg").status_code == 200
    assert client.post("/api/quick/camera/preview/play").status_code == 200


def test_start_opens_saved_replay_from_beginning_but_resume_does_not_rewind(
    client, monkeypatch
):
    source = "video:/home/racetime/videotest_masked.mp4"
    calls = []

    def stop():
        calls.append("stop")
        runtime.camera.frame_time = 0

    def start(chosen, **options):
        calls.append(("start", chosen, options))
        runtime.camera.source = chosen
        runtime.camera.frame_time = time.monotonic()
        if options.get("paused"):
            runtime.camera.pause_event.set()

    monkeypatch.setattr(runtime.camera, "stop", stop)
    monkeypatch.setattr(runtime.camera, "start", start)
    response = client.post(
        "/api/quick/session",
        json={"source": source, "racers": [{"name": "Red", "color": "#ff0000"}]},
    )
    assert response.status_code == 200, response.text
    runtime.camera.source = "video:/home/racetime/videotest.mp4"
    runtime.camera.frame_time = 0
    assert client.post("/api/quick/control/start").status_code == 200
    assert calls == [
        "stop",
        ("start", source, {"start_second": 0, "end_second": None, "paused": True}),
    ]
    assert client.post("/api/quick/control/pause").status_code == 200
    calls.clear()
    assert client.post("/api/quick/control/start").status_code == 200
    assert calls == []


def test_zero_lap_paused_race_can_return_to_setup_without_new_round(client):
    payload = {
        "event_name": "Practice",
        "racers": [{"name": "Red", "color": "#ff0000"}],
    }
    first = client.post("/api/quick/session", json=payload).json()
    runtime.camera.frame_time = time.monotonic()
    assert client.post("/api/quick/control/start").status_code == 200
    assert client.post("/api/quick/control/pause").status_code == 200
    updated = client.post(
        "/api/quick/session", json={**payload, "event_name": "Final", "finish_x": 0.61}
    )
    assert updated.status_code == 200, updated.text
    assert updated.json()["race_id"] == first["race_id"]
    state = client.get("/api/quick/state").json()
    assert state["race"]["status"] == "setup"
    assert state["race"]["start_time"] is None
    assert state["config"]["finish_x"] == 0.61
    assert len(client.portal.call(database.query, "SELECT id FROM race_events")) == 1


def test_unidentifiable_colors_can_be_saved_with_explicit_warnings(client):
    response = client.post(
        "/api/quick/session",
        json={
            "event_name": "Color test",
            "racers": [
                {"name": "Black", "color": "#101010"},
                {"name": "Red", "color": "#ff0000"},
                {"name": "Other red", "color": "#ed2010"},
            ],
        },
    )
    assert response.status_code == 200, response.text
    assert len(response.json()["racers"]) == 3
    assert len(response.json()["color_warnings"]) == 2
    state = client.get("/api/quick/state").json()
    assert state["race"]["status"] == "setup"
    assert state["color_warnings"] == response.json()["color_warnings"]


def test_paused_race_with_saved_lap_cannot_change_setup(client):
    session, racers = prepare(client)
    assert lap(client, session["race_id"], racers[0]["id"], 1, 3000).status_code == 200
    assert client.post("/api/quick/control/pause").status_code == 200
    response = client.post(
        "/api/quick/session",
        json={"racers": [{"name": "Red", "color": "#ff0000"}]},
    )
    assert response.status_code == 409
    assert client.get("/api/quick/state").json()["race"]["status"] == "paused"


def test_pause_freezes_replay_and_resume_continues_it(client, tmp_path):
    directory = tmp_path / "videos"
    directory.mkdir()
    path = directory / "clip.mp4"
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), 10, (64, 48))
    for number in range(30):
        writer.write(np.full((48, 64, 3), number, dtype=np.uint8))
    writer.release()
    response = client.post(
        "/api/quick/session",
        json={
            "source": "video:clip.mp4",
            "racers": [{"name": "Red", "color": "#ff0000"}],
        },
    )
    assert response.status_code == 200, response.text
    assert client.post("/api/quick/control/start").status_code == 200
    assert client.post("/api/quick/control/pause").status_code == 200
    time.sleep(0.15)
    paused = client.get("/api/quick/state").json()
    assert paused["camera"]["paused"] is True
    video_second = paused["camera"]["video_second"]
    time.sleep(0.25)
    assert (
        client.get("/api/quick/state").json()["camera"]["video_second"] == video_second
    )
    assert client.post("/api/quick/control/start").status_code == 200
    deadline = time.monotonic() + 2
    while runtime.camera.video_second == video_second and time.monotonic() < deadline:
        time.sleep(0.02)
    assert runtime.camera.video_second > video_second


def test_marked_video_interval_pauses_then_supports_another_round(client, tmp_path):
    directory = tmp_path / "videos"
    directory.mkdir()
    writer = cv2.VideoWriter(
        str(directory / "heats.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 10, (64, 48)
    )
    for number in range(40):
        writer.write(np.full((48, 64, 3), number, dtype=np.uint8))
    writer.release()
    payload = {
        "source": "video:heats.mp4",
        "event_name": "Heat 1",
        "video_start_s": 1.0,
        "video_end_s": 1.4,
        "racers": [{"name": "Red", "color": "#ff0000"}],
    }
    assert (
        client.post(
            "/api/quick/session", json={**payload, "video_end_s": 0.5}
        ).status_code
        == 422
    )
    first = client.post("/api/quick/session", json=payload)
    assert first.status_code == 200, first.text
    assert client.post("/api/quick/control/start").status_code == 200
    deadline = time.monotonic() + 3
    while client.get("/api/quick/state").json()["race"]["status"] != "paused":
        assert time.monotonic() < deadline
        time.sleep(0.05)
    ended = client.get("/api/quick/state").json()
    assert ended["camera"]["end_reached"] is True
    assert ended["laps"] == []
    assert client.post("/api/quick/control/start").status_code == 409
    assert client.post("/api/quick/control/finish").status_code == 200
    second = client.post(
        "/api/quick/session",
        json={
            **payload,
            "event_name": "Heat 2",
            "video_start_s": 2.0,
            "video_end_s": 2.4,
        },
    )
    assert second.status_code == 200, second.text
    assert second.json()["race_id"] != first.json()["race_id"]


def test_replay_lap_time_uses_video_clock_across_a_pause(client):
    from app.main import pause_race, start_race

    response = client.post(
        "/api/quick/session",
        json={
            "source": "video:clock.mp4",
            "minimum_lap_ms": 1000,
            "racers": [{"name": "Red", "color": "#ff0000"}],
        },
    )
    assert response.status_code == 200, response.text
    race_id = response.json()["race_id"]
    client.portal.call(start_race, race_id)
    worker = TimingRuntime()
    worker.camera.source = "video:clock.mp4"

    async def feed(x, wall_second, video_second):
        image = np.zeros((200, 300, 3), dtype=np.uint8)
        cv2.rectangle(image, (x, 80), (x + 20, 100), (0, 0, 255), -1)
        worker.camera.frame_time = wall_second
        worker.camera.video_second = video_second
        await worker.process_frame(image, wall_second, now=wall_second)

    asyncio.run(feed(100, 100, 10))
    asyncio.run(feed(180, 100.1, 10.1))
    client.portal.call(pause_race, race_id)
    asyncio.run(worker.process_frame(None, 0, now=150))
    client.portal.call(start_race, race_id)
    asyncio.run(feed(100, 200, 12))
    asyncio.run(feed(180, 200.1, 12.1))
    laps = client.get(f"/api/laps?race_id={race_id}").json()
    assert len(laps) == 1
    assert laps[0]["lap_time"] == pytest.approx(2000)


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
    assert [s["total_points"] for s in standings] == [20, 14]
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


def test_old_worker_snapshot_after_start_uses_current_camera_frame(client):
    session, _ = prepare(client)
    worker = TimingRuntime()
    image = np.zeros((200, 300, 3), dtype=np.uint8)
    worker.camera.latest = lambda: (image, time.monotonic())
    asyncio.run(worker.process_frame(None, 0))
    assert client.get(f"/api/races/{session['race_id']}").json()["status"] == "active"


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
