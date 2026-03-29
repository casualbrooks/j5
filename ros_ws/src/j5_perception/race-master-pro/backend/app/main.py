"""
Race Master Pro — FastAPI Backend
Main application entry point with REST API + WebSocket endpoints.
"""

import asyncio
import json
import socket
import uuid
from shutil import which
from datetime import datetime
from contextlib import asynccontextmanager
from urllib.parse import urlparse

import httpx

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware

from app.database import (
    init_db,
    insert_row,
    get_row,
    get_all_rows,
    update_row,
    delete_row,
    query,
)
from app.models import (
    SeasonCreate,
    ChampionshipCreate,
    TrackCreate,
    CheckpointCreate,
    CameraConfigCreate,
    RacerProfileCreate,
    RaceEventCreate,
    RaceCreate,
    RaceResultCreate,
    LapRecordCreate,
    DetectionCreate,
    PositionUpdate,
    LapComplete,
)
from app.websocket_manager import manager


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Initialize DB on startup."""
    await init_db()
    yield


app = FastAPI(
    title="Race Master Pro",
    description="Real-time race tracking with AI-powered computer vision for ROS2",
    version="1.0.0",
    lifespan=lifespan,
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Restrict in production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

_SETUP_STEPS = [
    {
        "id": "ros2_cli",
        "title": "ROS2 CLI on PATH",
        "description": "Ensure ros2 and launch verbs are available in the active shell.",
        "check_commands": ["ros2 --help"],
        "check_contains": ["launch"],
        "connect_command": ". /opt/ros/iron/setup.bash && . ~/alive/j5/ros_ws/install/setup.bash",
        "stop_command": "pkill -f ros2 || true",
        "help": "The API process cannot mutate your terminal PATH. Run the source command in the shell where you launch ROS nodes, then click Verify.",
    },
    {
        "id": "pi_connectivity",
        "title": "Headless Raspberry Pi Reachability",
        "description": "Verify the configured Pi host resolves and SSH reachability is available.",
        "check_commands": [],
        "connect_command": "Verify host + API reachability (no interactive shell needed)",
        "stop_command": "echo 'No persistent process to stop for connectivity check'",
        "stop_commands": ["echo No persistent process to stop for connectivity check"],
        "help": "Set pi_host to an address that resolves from this machine and ensure the Pi is reachable over SSH. Backend, websocket, and preview are checked in their own steps.",
    },
    {
        "id": "backend_service",
        "title": "Race Master backend API",
        "description": "Backend must be reachable to drive setup + live controls.",
        "check_commands": ["GET {backend_url}/health or /docs or /openapi.json"],
        "connect_command": "cd backend && source .venv/bin/activate && python -m app.main",
        "stop_command": "pkill -f 'python -m app.main' || true",
        "manual_connect": True,
        "stop_commands": ["pkill -f python -m app.main"],
        "help": "If this fails, start the backend on the configured host/port and confirm at least one of /health, /docs, or /openapi.json returns HTTP 200.",
    },
    {
        "id": "vision_preview",
        "title": "Camera preview stream",
        "description": "Preview and track-image capture endpoint from the Pi.",
        "check_commands": ["GET {preview_url}/health or /stream.mjpg or /snapshot.jpg"],
        "connect_command": "python scripts/pi_preflight.py --camera-source /dev/video0 --capture-file track_snapshot.jpg --serve-preview --preview-host 0.0.0.0 --preview-port 8091",
        "stop_command": "pkill -f pi_preflight.py || true",
        "manual_connect": True,
        "stop_commands": ["pkill -f pi_preflight.py"],
        "help": "Start the preview service to restore the Computer Vision feed, then verify at least one of {preview_url}/health, /stream.mjpg, or /snapshot.jpg is reachable.",
    },
    {
        "id": "websocket_endpoint",
        "title": "WebSocket endpoint",
        "description": "Realtime events require a reachable WS endpoint based on backend_url.",
        "check_commands": [],
        "connect_command": "Optional: set VITE_API_BASE_URL=http://<backend-host>:<backend-port> and VITE_WS_URL=ws://<backend-host>:<backend-port>/ws before starting Vite; if omitted the UI auto-falls back to current-host:8080",
        "stop_command": "echo 'No persistent process to stop for websocket endpoint'",
        "stop_commands": ["echo No persistent process to stop for websocket endpoint"],
        "help": "These env vars are frontend startup config, not runtime shell commands. They should point to the FastAPI backend host/port (usually :8080), not the Vite dev server (:5173).",
    },
    {
        "id": "race_state",
        "title": "Race configured",
        "description": "Race metadata + racers + lap target saved and ready to initialize.",
        "check_commands": [],
        "connect_command": "POST /api/setup/wizard/initialize",
        "stop_command": "POST /api/setup/wizard/reset",
        "manual_connect": True,
    },
]

_INITIALIZE_REQUIRED_STEP_IDS = {
    "pi_connectivity",
    "backend_service",
}

_DEFAULT_SETUP_CONFIG = {
    "pi_host": "raspberrypi.local",
    "pi_user": "pi",
    "backend_url": "http://localhost:8080",
    "preview_url": "http://localhost:8091",
    "race_name": "Main Event",
    "event_name": "Weekend Session",
    "total_laps": 20,
    "racer_names": ["Racer 1", "Racer 2"],
}


async def _get_state_json(key: str, default: dict | list | None = None):
    row = await get_row("system_state", key, id_column="key")
    if not row:
        return default
    try:
        return json.loads(row["value"])
    except json.JSONDecodeError:
        return default


async def _set_state_json(key: str, value):
    payload = {"key": key, "value": json.dumps(value)}
    existing = await get_row("system_state", key, id_column="key")
    if existing:
        await update_row(
            "system_state",
            key,
            {"value": payload["value"], "updated_at": datetime.now().isoformat()},
            id_column="key",
        )
        return
    await insert_row("system_state", payload)


def _validate_setup_config(config: dict) -> dict:
    merged = {**_DEFAULT_SETUP_CONFIG, **config}

    def _safe_token(name: str, value: str):
        if not value or any(
            ch in value for ch in [";", "&", "|", "$", "`", "\n", "\r", "\t"]
        ):
            raise HTTPException(400, f"Invalid characters in {name}")

    pi_host = str(merged.get("pi_host", "")).strip()
    pi_user = str(merged.get("pi_user", "")).strip()
    _safe_token("pi_host", pi_host)
    _safe_token("pi_user", pi_user)

    for field in ("backend_url", "preview_url"):
        parsed = urlparse(str(merged.get(field, "")).strip())
        if parsed.scheme not in {"http", "https"} or not parsed.netloc:
            raise HTTPException(400, f"Invalid URL for {field}")
        merged[field] = f"{parsed.scheme}://{parsed.netloc}"

    merged["pi_host"] = pi_host
    merged["pi_user"] = pi_user
    return merged


async def _run_shell_command(command: str):
    argv = command.split()
    process = await asyncio.create_subprocess_exec(
        *argv,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE,
    )
    stdout, stderr = await process.communicate()
    return {
        "command": command,
        "return_code": process.returncode,
        "stdout": stdout.decode().strip(),
        "stderr": stderr.decode().strip(),
        "ok": process.returncode == 0,
    }


async def _run_exec_command(argv: list[str], label: str):
    process = await asyncio.create_subprocess_exec(
        *argv,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE,
    )
    stdout, stderr = await process.communicate()
    return {
        "command": label,
        "return_code": process.returncode,
        "stdout": stdout.decode().strip(),
        "stderr": stderr.decode().strip(),
        "ok": process.returncode == 0,
    }


async def _check_host_resolution(host: str):
    loop = asyncio.get_running_loop()
    try:
        resolved = await asyncio.wait_for(loop.getaddrinfo(host, None), timeout=2.0)
        addresses = sorted({item[4][0] for item in resolved if item[4]})
        return {
            "command": f"resolve {host}",
            "ok": bool(addresses),
            "stdout": ", ".join(addresses),
            "stderr": "",
        }
    except (socket.gaierror, TimeoutError, asyncio.TimeoutError) as exc:
        return {
            "command": f"resolve {host}",
            "ok": False,
            "stdout": "",
            "stderr": str(exc),
        }


async def _check_tcp_port(host: str, port: int):
    try:
        connection = await asyncio.wait_for(
            asyncio.open_connection(host, port), timeout=2.0
        )
        _, writer = connection
        writer.close()
        await writer.wait_closed()
        return {
            "command": f"tcp://{host}:{port}",
            "ok": True,
            "stdout": "reachable",
            "stderr": "",
        }
    except Exception as exc:
        return {
            "command": f"tcp://{host}:{port}",
            "ok": False,
            "stdout": "",
            "stderr": str(exc),
        }


async def _check_http_health(url: str):
    return await _check_http_endpoints(url, ["/health"])


async def _check_http_endpoints(url: str, paths: list[str]):
    base = url.rstrip("/")
    errors: list[str] = []
    async with httpx.AsyncClient(timeout=2.0) as client:
        for path in paths:
            endpoint = f"{base}{path}"
            try:
                response = await client.get(endpoint)
                if response.status_code == 200:
                    return {
                        "command": f"GET {endpoint}",
                        "ok": True,
                        "stdout": response.text.strip(),
                        "stderr": "",
                    }
                errors.append(f"{endpoint} -> HTTP {response.status_code}")
            except Exception as exc:
                errors.append(f"{endpoint} -> {exc}")
    first_endpoint = f"{base}{paths[0]}"
    return {
        "command": f"GET {first_endpoint}",
        "ok": False,
        "stdout": "",
        "stderr": "; ".join(errors),
    }


async def _check_ws_port(url: str):
    parsed = urlparse(url)
    host = parsed.hostname or "localhost"
    port = parsed.port or (443 if parsed.scheme == "https" else 80)
    return await _check_tcp_port(host, port)


def _render_command(template: str, config: dict):
    return template.format(**config)


async def _build_setup_status():
    config = await _get_state_json("setup_config", _DEFAULT_SETUP_CONFIG)
    config = _validate_setup_config(config)
    overrides = await _get_state_json("setup_step_overrides", {})
    race_context = await _get_state_json("race_context", {})
    steps = []
    for step in _SETUP_STEPS:
        state = overrides.get(step["id"], {})
        checks = []
        check_ok = True
        if step["id"] == "race_state":
            check_ok = bool(race_context.get("race_id"))
            checks = [{"command": "race context exists", "ok": check_ok}]
        elif step["id"] == "ros2_cli":
            ros2_path = which("ros2")
            if not ros2_path:
                checks = [
                    {
                        "command": "which ros2",
                        "ok": False,
                        "stdout": "",
                        "stderr": "ros2 not found in PATH",
                    }
                ]
                check_ok = False
            else:
                checks = await asyncio.gather(
                    _run_exec_command(["ros2", "--help"], "ros2 --help"),
                    _run_exec_command(
                        ["ros2", "launch", "--help"], "ros2 launch --help"
                    ),
                )
                check_ok = all(item["ok"] for item in checks)
        elif step["id"] == "pi_connectivity":
            pi_host = config["pi_host"]
            checks = [await _check_host_resolution(pi_host)]
            checks.append(await _check_tcp_port(pi_host, 22))
            check_ok = all(item["ok"] for item in checks)
        elif step["id"] == "backend_service":
            checks = [
                await _check_http_endpoints(
                    config["backend_url"], ["/health", "/docs", "/openapi.json"]
                )
            ]
            check_ok = all(item["ok"] for item in checks)
        elif step["id"] == "vision_preview":
            checks = [
                await _check_http_endpoints(
                    config["preview_url"], ["/health", "/stream.mjpg", "/snapshot.jpg"]
                )
            ]
            check_ok = all(item["ok"] for item in checks)
        elif step["id"] == "websocket_endpoint":
            ws_result = await _check_ws_port(config["backend_url"])
            ws_result["command"] = (
                f"ws tcp://{urlparse(config['backend_url']).hostname}:{urlparse(config['backend_url']).port or 80}"
            )
            checks = [ws_result]
            check_ok = all(item["ok"] for item in checks)
        else:
            for cmd_template in step["check_commands"]:
                command = _render_command(cmd_template, config)
                result = await _run_shell_command(command)
                expected_tokens = step.get("check_contains", [])
                if expected_tokens and result["ok"]:
                    result["ok"] = all(
                        token in result["stdout"] for token in expected_tokens
                    )
                checks.append(result)
                if not result["ok"]:
                    check_ok = False
        connected = bool(check_ok)
        step_data = {
            "id": step["id"],
            "title": step["title"],
            "description": step["description"],
            "connected": connected,
            "checks": checks,
            "next_command": _render_command(step["connect_command"], config),
            "stop_command": _render_command(step["stop_command"], config),
            "last_action": state.get("last_action"),
            "last_error": state.get("last_error"),
            "help": _render_command(step.get("help", ""), config),
        }
        steps.append(step_data)
    for idx, step in enumerate(steps):
        if not step["connected"]:
            step["is_current"] = True
            if idx + 1 < len(steps):
                step["next_step_command"] = steps[idx + 1]["next_command"]
            break
    return {"config": config, "steps": steps, "race_context": race_context}


# ── Health ──────────────────────────────────────────────────


@app.get("/health")
async def health_check():
    return {
        "status": "healthy",
        "timestamp": datetime.now().isoformat(),
        "connections": manager.total_connections,
    }


@app.get("/")
async def root():
    return {
        "service": "Race Master Pro backend",
        "status_endpoint": "/health",
        "docs_endpoint": "/docs",
    }


# ── Seasons ─────────────────────────────────────────────────


@app.post("/api/seasons")
async def create_season(data: SeasonCreate):
    row = {"id": str(uuid.uuid4()), **data.model_dump()}
    return await insert_row("seasons", row)


@app.get("/api/seasons")
async def list_seasons():
    return await get_all_rows("seasons", order_by="year DESC")


@app.get("/api/seasons/{season_id}")
async def get_season(season_id: str):
    row = await get_row("seasons", season_id)
    if not row:
        raise HTTPException(404, "Season not found")
    return row


@app.put("/api/seasons/{season_id}")
async def update_season(season_id: str, data: SeasonCreate):
    return await update_row("seasons", season_id, data.model_dump())


@app.delete("/api/seasons/{season_id}")
async def delete_season(season_id: str):
    if not await delete_row("seasons", season_id):
        raise HTTPException(404, "Season not found")
    return {"deleted": True}


# ── Championships ───────────────────────────────────────────


@app.post("/api/championships")
async def create_championship(data: ChampionshipCreate):
    row = {"id": str(uuid.uuid4()), **data.model_dump()}
    return await insert_row("championships", row)


@app.get("/api/championships")
async def list_championships(season_id: str | None = None):
    where = {"season_id": season_id} if season_id else None
    return await get_all_rows("championships", where=where)


@app.get("/api/championships/{champ_id}")
async def get_championship(champ_id: str):
    row = await get_row("championships", champ_id)
    if not row:
        raise HTTPException(404, "Championship not found")
    return row


@app.delete("/api/championships/{champ_id}")
async def delete_championship(champ_id: str):
    if not await delete_row("championships", champ_id):
        raise HTTPException(404, "Championship not found")
    return {"deleted": True}


# ── Tracks ──────────────────────────────────────────────────


@app.post("/api/tracks")
async def create_track(data: TrackCreate):
    row = {"id": str(uuid.uuid4()), **data.model_dump()}
    return await insert_row("tracks", row)


@app.get("/api/tracks")
async def list_tracks():
    return await get_all_rows("tracks", order_by="name")


@app.get("/api/tracks/{track_id}")
async def get_track(track_id: str):
    row = await get_row("tracks", track_id)
    if not row:
        raise HTTPException(404, "Track not found")
    return row


@app.put("/api/tracks/{track_id}")
async def update_track(track_id: str, data: TrackCreate):
    return await update_row("tracks", track_id, data.model_dump())


@app.delete("/api/tracks/{track_id}")
async def delete_track(track_id: str):
    if not await delete_row("tracks", track_id):
        raise HTTPException(404, "Track not found")
    return {"deleted": True}


# ── Racer Profiles ──────────────────────────────────────────


@app.post("/api/racers")
async def create_racer(data: RacerProfileCreate):
    row = {"id": str(uuid.uuid4()), **data.model_dump()}
    return await insert_row("racer_profiles", row)


@app.get("/api/racers")
async def list_racers():
    return await get_all_rows("racer_profiles", order_by="name")


@app.get("/api/racers/{racer_id}")
async def get_racer(racer_id: str):
    row = await get_row("racer_profiles", racer_id)
    if not row:
        raise HTTPException(404, "Racer not found")
    return row


@app.put("/api/racers/{racer_id}")
async def update_racer(racer_id: str, data: RacerProfileCreate):
    return await update_row("racer_profiles", racer_id, data.model_dump())


@app.delete("/api/racers/{racer_id}")
async def delete_racer(racer_id: str):
    if not await delete_row("racer_profiles", racer_id):
        raise HTTPException(404, "Racer not found")
    return {"deleted": True}


# ── Race Events ─────────────────────────────────────────────


@app.post("/api/events")
async def create_event(data: RaceEventCreate):
    row = {"id": str(uuid.uuid4()), **data.model_dump()}
    return await insert_row("race_events", row)


@app.get("/api/events")
async def list_events(championship_id: str | None = None):
    where = {"championship_id": championship_id} if championship_id else None
    return await get_all_rows("race_events", where=where, order_by="round_number")


@app.delete("/api/events/{event_id}")
async def delete_event(event_id: str):
    if not await delete_row("race_events", event_id):
        raise HTTPException(404, "Event not found")
    return {"deleted": True}


# ── Races ───────────────────────────────────────────────────


@app.post("/api/races")
async def create_race(data: RaceCreate):
    row = {"id": str(uuid.uuid4()), "status": "setup", **data.model_dump()}
    return await insert_row("races", row)


@app.get("/api/races")
async def list_races(event_id: str | None = None, status: str | None = None):
    where = {}
    if event_id:
        where["event_id"] = event_id
    if status:
        where["status"] = status
    return await get_all_rows("races", where=where or None)


@app.get("/api/races/{race_id}")
async def get_race(race_id: str):
    row = await get_row("races", race_id)
    if not row:
        raise HTTPException(404, "Race not found")
    return row


@app.post("/api/races/{race_id}/start")
async def start_race(race_id: str):
    race = await get_row("races", race_id)
    if not race:
        raise HTTPException(404, "Race not found")
    if race["status"] not in ("setup", "paused"):
        raise HTTPException(400, f"Cannot start race in '{race['status']}' status")
    now = datetime.now().isoformat()
    updated = await update_row(
        "races", race_id, {"status": "active", "start_time": now}
    )
    await manager.broadcast_all(
        {
            "type": "raceStart",
            "data": {"race_id": race_id, "start_time": now},
            "timestamp": now,
        }
    )
    return updated


@app.post("/api/races/{race_id}/pause")
async def pause_race(race_id: str):
    race = await get_row("races", race_id)
    if not race:
        raise HTTPException(404, "Race not found")
    if race["status"] != "active":
        raise HTTPException(400, "Can only pause an active race")
    updated = await update_row("races", race_id, {"status": "paused"})
    await manager.broadcast_all(
        {
            "type": "racePause",
            "data": {"race_id": race_id},
            "timestamp": datetime.now().isoformat(),
        }
    )
    return updated


@app.post("/api/races/{race_id}/finish")
async def finish_race(race_id: str):
    race = await get_row("races", race_id)
    if not race:
        raise HTTPException(404, "Race not found")
    now = datetime.now().isoformat()
    updated = await update_row(
        "races", race_id, {"status": "finished", "end_time": now}
    )
    await manager.broadcast_all(
        {
            "type": "raceFinish",
            "data": {"race_id": race_id, "end_time": now},
            "timestamp": now,
        }
    )
    return updated


# ── Race Results ────────────────────────────────────────────


@app.post("/api/results")
async def create_result(data: RaceResultCreate):
    row = {"id": str(uuid.uuid4()), **data.model_dump()}
    return await insert_row("race_results", row)


@app.get("/api/results")
async def list_results(race_id: str | None = None, racer_profile_id: str | None = None):
    where = {}
    if race_id:
        where["race_id"] = race_id
    if racer_profile_id:
        where["racer_profile_id"] = racer_profile_id
    return await get_all_rows(
        "race_results", where=where or None, order_by="finish_position"
    )


@app.put("/api/results/{result_id}")
async def update_result(result_id: str, data: RaceResultCreate):
    return await update_row("race_results", result_id, data.model_dump())


# ── Lap Records ─────────────────────────────────────────────


@app.post("/api/laps")
async def record_lap(data: LapRecordCreate):
    row = {"id": str(uuid.uuid4()), **data.model_dump()}
    result = await insert_row("lap_records", row)
    # Broadcast lap complete
    await manager.broadcast_all(
        {
            "type": "lapComplete",
            "data": {
                "racer_id": data.racer_profile_id,
                "lap_number": data.lap_number,
                "lap_time": data.lap_time,
            },
            "timestamp": datetime.now().isoformat(),
        }
    )
    return result


@app.get("/api/laps")
async def list_laps(race_id: str | None = None, racer_profile_id: str | None = None):
    where = {}
    if race_id:
        where["race_id"] = race_id
    if racer_profile_id:
        where["racer_profile_id"] = racer_profile_id
    return await get_all_rows("lap_records", where=where or None, order_by="lap_number")


# ── Camera Configs ──────────────────────────────────────────


@app.post("/api/cameras")
async def create_camera(data: CameraConfigCreate):
    row = {"id": str(uuid.uuid4()), **data.model_dump()}
    return await insert_row("camera_configs", row)


@app.get("/api/cameras")
async def list_cameras():
    return await get_all_rows("camera_configs")


@app.put("/api/cameras/{camera_id}")
async def update_camera(camera_id: str, data: CameraConfigCreate):
    return await update_row("camera_configs", camera_id, data.model_dump())


@app.delete("/api/cameras/{camera_id}")
async def delete_camera(camera_id: str):
    if not await delete_row("camera_configs", camera_id):
        raise HTTPException(404, "Camera not found")
    return {"deleted": True}


# ── Detections ──────────────────────────────────────────────


@app.post("/api/detections")
async def save_detection(data: DetectionCreate):
    row = {"id": str(uuid.uuid4()), **data.model_dump()}
    return await insert_row("detections", row)


# ── Analytics ───────────────────────────────────────────────


@app.get("/api/analytics/racer/{racer_id}/summary")
async def racer_summary(racer_id: str):
    """Aggregate stats for a racer across all their races."""
    results = await query(
        """
        SELECT
            COUNT(*) as total_races,
            AVG(finish_position) as avg_position,
            MIN(best_lap_time) as all_time_best_lap,
            AVG(avg_lap_time) as career_avg_lap,
            SUM(points_earned) as total_points,
            SUM(CASE WHEN status = 'dnf' THEN 1 ELSE 0 END) as dnf_count,
            SUM(CASE WHEN finish_position = 1 THEN 1 ELSE 0 END) as wins,
            SUM(CASE WHEN finish_position <= 3 THEN 1 ELSE 0 END) as podiums
        FROM race_results
        WHERE racer_profile_id = ?
    """,
        [racer_id],
    )
    return results[0] if results else {}


@app.get("/api/analytics/track/{track_id}/records")
async def track_records(track_id: str):
    """Best lap times at a specific track."""
    return await query(
        """
        SELECT lr.*, rp.name as racer_name, rp.number as racer_number
        FROM lap_records lr
        JOIN race_results rr ON lr.race_id = rr.race_id AND lr.racer_profile_id = rr.racer_profile_id
        JOIN races r ON lr.race_id = r.id
        JOIN race_events re ON r.event_id = re.id
        JOIN racer_profiles rp ON lr.racer_profile_id = rp.id
        WHERE re.track_id = ?
        ORDER BY lr.lap_time ASC
        LIMIT 20
    """,
        [track_id],
    )


@app.get("/api/analytics/championship/{champ_id}/standings")
async def championship_standings(champ_id: str):
    """Championship standings with total points per racer."""
    return await query(
        """
        SELECT
            rp.id as racer_id,
            rp.name as racer_name,
            rp.number as racer_number,
            SUM(rr.points_earned) as total_points,
            SUM(CASE WHEN rr.finish_position = 1 THEN 1 ELSE 0 END) as wins,
            SUM(CASE WHEN rr.finish_position <= 3 THEN 1 ELSE 0 END) as podiums,
            COUNT(*) as races_completed
        FROM race_results rr
        JOIN races r ON rr.race_id = r.id
        JOIN race_events re ON r.event_id = re.id
        JOIN racer_profiles rp ON rr.racer_profile_id = rp.id
        WHERE re.championship_id = ?
        GROUP BY rp.id
        ORDER BY total_points DESC
    """,
        [champ_id],
    )


# ── Setup Wizard + Runtime State ──────────────────────────


@app.get("/api/setup/wizard")
async def get_setup_wizard_status():
    return await _build_setup_status()


@app.put("/api/setup/wizard/config")
async def update_setup_wizard_config(config: dict):
    merged = _validate_setup_config(config)
    await _set_state_json("setup_config", merged)
    return await _build_setup_status()


@app.post("/api/setup/wizard/steps/{step_id}/verify")
async def verify_setup_step(step_id: str):
    status = await _build_setup_status()
    step = next((item for item in status["steps"] if item["id"] == step_id), None)
    if not step:
        raise HTTPException(404, "Setup step not found")
    overrides = await _get_state_json("setup_step_overrides", {})
    overrides[step_id] = {
        "connected": step["connected"],
        "last_action": {
            "type": "verify",
            "timestamp": datetime.now().isoformat(),
            "result": step["checks"],
        },
        "last_error": None if step["connected"] else "Verification failed",
    }
    await _set_state_json("setup_step_overrides", overrides)
    return await _build_setup_status()


@app.post("/api/setup/wizard/steps/{step_id}/connect")
async def connect_setup_step(step_id: str):
    config = await _get_state_json("setup_config", _DEFAULT_SETUP_CONFIG)
    config = _validate_setup_config(config)
    step = next((item for item in _SETUP_STEPS if item["id"] == step_id), None)
    if not step:
        raise HTTPException(404, "Setup step not found")
    command = _render_command(step["connect_command"], config)
    if step_id == "pi_connectivity":
        wizard = await _build_setup_status()
        current_step = next(
            (item for item in wizard["steps"] if item["id"] == step_id), None
        )
        result = {
            "ok": bool(current_step and current_step["connected"]),
            "stdout": "Connectivity checks completed via backend API.",
            "stderr": "",
            "return_code": 0 if current_step and current_step["connected"] else 1,
        }
        overrides = await _get_state_json("setup_step_overrides", {})
        overrides[step_id] = {
            "connected": result["ok"],
            "last_action": {
                "type": "connect",
                "timestamp": datetime.now().isoformat(),
                "command": command,
                "result": result,
            },
            "last_error": (
                None
                if result["ok"]
                else "Pi host/API ports are not reachable. Verify host/IP and services."
            ),
        }
        await _set_state_json("setup_step_overrides", overrides)
        return {
            "step_id": step_id,
            "command": command,
            "result": result,
            "wizard": wizard,
        }
    result = {
        "ok": False,
        "stdout": "Run the command shown and then click Verify.",
        "stderr": "",
        "return_code": 1,
    }
    connect_commands = step.get("connect_commands", [])
    if connect_commands:
        rendered = [_render_command(cmd, config) for cmd in connect_commands]
        last_result = None
        for cmd in rendered:
            last_result = await _run_shell_command(cmd)
            if not last_result["ok"]:
                break
        if last_result:
            result = last_result
    overrides = await _get_state_json("setup_step_overrides", {})
    overrides[step_id] = {
        "connected": bool(result["ok"]),
        "last_action": {
            "type": "connect",
            "timestamp": datetime.now().isoformat(),
            "command": command,
            "result": result,
        },
        "last_error": (
            None
            if result["ok"]
            else result.get("stderr") or result.get("stdout") or "Connect failed"
        ),
    }
    await _set_state_json("setup_step_overrides", overrides)
    return {
        "step_id": step_id,
        "command": command,
        "result": result,
        "wizard": await _build_setup_status(),
    }


@app.post("/api/setup/wizard/steps/{step_id}/stop")
async def stop_setup_step(step_id: str):
    config = await _get_state_json("setup_config", _DEFAULT_SETUP_CONFIG)
    config = _validate_setup_config(config)
    step = next((item for item in _SETUP_STEPS if item["id"] == step_id), None)
    if not step:
        raise HTTPException(404, "Setup step not found")
    command = _render_command(step["stop_command"], config)
    stop_commands = step.get("stop_commands", [command])
    result = {"ok": True, "stdout": "", "stderr": "", "return_code": 0}
    for raw_cmd in stop_commands:
        rendered = _render_command(raw_cmd, config)
        result = await _run_shell_command(rendered)
        if not result["ok"]:
            break
    overrides = await _get_state_json("setup_step_overrides", {})
    overrides[step_id] = {
        "connected": False,
        "last_action": {
            "type": "stop",
            "timestamp": datetime.now().isoformat(),
            "command": command,
            "result": result,
        },
        "last_error": (
            None
            if result["ok"]
            else result.get("stderr") or result.get("stdout") or "Stop failed"
        ),
    }
    await _set_state_json("setup_step_overrides", overrides)
    return {
        "step_id": step_id,
        "command": command,
        "result": result,
        "wizard": await _build_setup_status(),
    }


@app.post("/api/setup/wizard/initialize")
async def initialize_race_from_wizard():
    status = await _build_setup_status()
    blocking_step = next(
        (
            item
            for item in status["steps"]
            if item["id"] in _INITIALIZE_REQUIRED_STEP_IDS and not item["connected"]
        ),
        None,
    )
    if blocking_step:
        raise HTTPException(
            400,
            f"Setup blocked by '{blocking_step['title']}'. Run: {blocking_step['next_command']} and verify this step before initializing race state.",
        )

    config = await _get_state_json("setup_config", _DEFAULT_SETUP_CONFIG)
    event = await insert_row(
        "race_events",
        {
            "id": str(uuid.uuid4()),
            "name": config.get("event_name", "Setup Wizard Event"),
            "championship_id": None,
            "track_id": None,
            "event_date": datetime.now().date().isoformat(),
            "round_number": 1,
        },
    )
    race = await insert_row(
        "races",
        {
            "id": str(uuid.uuid4()),
            "event_id": event["id"],
            "type": "main",
            "status": "setup",
            "total_laps": int(config.get("total_laps", 20)),
            "start_time": None,
            "end_time": None,
        },
    )
    racers = []
    for idx, name in enumerate(config.get("racer_names", []), start=1):
        racer = await insert_row(
            "racer_profiles",
            {
                "id": str(uuid.uuid4()),
                "name": name,
                "number": str(idx),
                "profile_pic": None,
                "vehicle_description": None,
            },
        )
        racers.append(racer)
    context = {
        "race_id": race["id"],
        "event_id": event["id"],
        "race_name": config.get("race_name", "Main Event"),
        "track_image_captured": False,
        "tracking_enabled": False,
        "racers": racers,
        "log_stream": [],
    }
    await _set_state_json("race_context", context)
    overrides = await _get_state_json("setup_step_overrides", {})
    overrides["race_state"] = {
        "connected": True,
        "last_action": {"type": "initialize", "timestamp": datetime.now().isoformat()},
        "last_error": None,
    }
    await _set_state_json("setup_step_overrides", overrides)
    return {
        "race": race,
        "event": event,
        "racers": racers,
        "wizard": await _build_setup_status(),
    }


@app.post("/api/setup/wizard/reset")
async def reset_wizard_race_context():
    await _set_state_json("race_context", {})
    overrides = await _get_state_json("setup_step_overrides", {})
    overrides["race_state"] = {
        "connected": False,
        "last_action": {"type": "reset", "timestamp": datetime.now().isoformat()},
        "last_error": None,
    }
    await _set_state_json("setup_step_overrides", overrides)
    return await _build_setup_status()


@app.get("/api/races/{race_id}/state")
async def get_race_runtime_state(race_id: str):
    race = await get_row("races", race_id)
    if not race:
        raise HTTPException(404, "Race not found")
    context = await _get_state_json("race_context", {})
    snapshots = await _get_state_json("race_snapshots", {})
    return {"race": race, "context": context, "snapshot": snapshots.get(race_id)}


@app.post("/api/races/{race_id}/snapshot")
async def snapshot_race_state(race_id: str):
    race = await get_row("races", race_id)
    if not race:
        raise HTTPException(404, "Race not found")
    laps = await get_all_rows("lap_records", where={"race_id": race_id})
    results = await get_all_rows("race_results", where={"race_id": race_id})
    context = await _get_state_json("race_context", {})
    snapshot = {
        "race": race,
        "laps": laps,
        "results": results,
        "context": context,
        "saved_at": datetime.now().isoformat(),
    }
    snapshots = await _get_state_json("race_snapshots", {})
    snapshots[race_id] = snapshot
    await _set_state_json("race_snapshots", snapshots)
    return snapshot


@app.post("/api/races/{race_id}/resume")
async def resume_race_from_snapshot(race_id: str):
    snapshots = await _get_state_json("race_snapshots", {})
    snapshot = snapshots.get(race_id)
    if not snapshot:
        raise HTTPException(404, "No snapshot found for race")
    await update_row("races", race_id, {"status": "active"})
    await _set_state_json("race_context", snapshot.get("context", {}))
    await manager.broadcast_all(
        {
            "type": "raceResume",
            "data": {"race_id": race_id, "snapshot_time": snapshot.get("saved_at")},
            "timestamp": datetime.now().isoformat(),
        }
    )
    return {"resumed": True, "snapshot": snapshot}


@app.post("/api/races/{race_id}/tracking/start")
async def start_tracking(race_id: str):
    context = await _get_state_json("race_context", {})
    context["tracking_enabled"] = True
    context.setdefault("log_stream", []).append(
        f"{datetime.now().isoformat()} Tracking started for race {race_id}"
    )
    await _set_state_json("race_context", context)
    return context


@app.post("/api/races/{race_id}/tracking/stop")
async def stop_tracking(race_id: str):
    context = await _get_state_json("race_context", {})
    context["tracking_enabled"] = False
    context.setdefault("log_stream", []).append(
        f"{datetime.now().isoformat()} Tracking stopped for race {race_id}"
    )
    await _set_state_json("race_context", context)
    return context


@app.post("/api/races/{race_id}/logs")
async def append_race_log(race_id: str, payload: dict):
    context = await _get_state_json("race_context", {})
    entry = payload.get("entry", f"{datetime.now().isoformat()} log")
    context.setdefault("log_stream", []).append(entry)
    await _set_state_json("race_context", context)
    await manager.broadcast_all(
        {
            "type": "lapLog",
            "data": {"race_id": race_id, "entry": entry},
            "timestamp": datetime.now().isoformat(),
        }
    )
    return {"ok": True, "entry": entry}


# ── WebSocket ───────────────────────────────────────────────


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket, client_type: str = "spectator"):
    await manager.connect(websocket, client_type)
    try:
        while True:
            data = await websocket.receive_text()
            try:
                message = json.loads(data)
            except json.JSONDecodeError:
                continue

            msg_type = message.get("type", "")

            if msg_type == "ping":
                await manager.send_personal(
                    websocket,
                    {
                        "type": "pong",
                        "timestamp": datetime.now().isoformat(),
                    },
                )
            elif msg_type in (
                "raceUpdate",
                "positionUpdate",
                "lapComplete",
                "raceStart",
                "racePause",
                "raceFinish",
                "raceResume",
                "lapLog",
            ):
                # Relay race events to all clients
                message["timestamp"] = datetime.now().isoformat()
                await manager.broadcast_all(message)
            elif msg_type == "visionDetection":
                # CV detections go to organizers
                message["timestamp"] = datetime.now().isoformat()
                await manager.broadcast_to_group(message, "organizer")
            else:
                # Unknown message type — ignore
                pass

    except WebSocketDisconnect:
        manager.disconnect(websocket, client_type)
        await manager.broadcast_connection_count()


# ── Entry Point ─────────────────────────────────────────────

if __name__ == "__main__":
    import uvicorn

    uvicorn.run(
        "app.main:app", host="0.0.0.0", port=8080, reload=True, log_level="info"
    )
