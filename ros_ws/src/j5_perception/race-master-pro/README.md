# Race Master Pro

A comprehensive real-time race tracking system with AI-powered computer vision, designed as a ROS2 perception module. Uses cameras as data sources and AI models (YOLO + DeepSORT) to track racing vehicles, compute lap times, and visualize race data.

## Prerequisites (verify first)

Run from a fresh shell before starting backend/frontend/perception:

```bash
# source ROS2 underlay + J5 overlay (source-build flow)
source ~/ros2_kilted/install/setup.bash
source ~/j5/ros_ws/install/setup.bash

# verify CLI + package visibility
command -v ros2
ros2 -h | rg -w launch
ros2 pkg list | rg '^racetracker_perception$'
```

If `ros2` or `launch` is missing, fix underlay/overlay first (do not continue to backend/frontend until this passes).

## Quick Start (source-build workflow)

### 1. Start the Backend (Python deps in venv)

```bash
cd backend
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python -m app.main
```

Backend starts at `http://localhost:8080` (API docs at `/docs`).

For headless/remote access from another device on the same network, use the Pi host IP
(for example `http://<pi-ip>:8080`) instead of `localhost`.

> `ModuleNotFoundError: No module named 'fastapi'` means the backend venv is not active
> (or requirements were not installed in that interpreter).

### 2. Start the Frontend

```bash
cd frontend
npm install
npm run dev
```

Frontend starts at `http://localhost:5173`.

For remote browser access, open `http://<pi-ip>:5173` and run Vite with host binding
(for example `npm run dev -- --host 0.0.0.0`) if needed.

### 3. Run the Perception System

```bash
# Standalone mode (no ROS2 needed, uses mock cameras)
python -m perception.standalone.standalone_runner

# ROS2 node mode (requires sourced underlay + overlay in THIS shell)
ros2 run racetracker_perception perception_node
```

If you see `ros2: command not found` here, re-source before running the node:

```bash
source ~/ros2_kilted/install/setup.bash
source ~/j5/ros_ws/install/setup.bash
```

## Features

| Feature | Status |
|---|---|
| Real-time race dashboard | ✅ |
| Leaderboard with position/timing | ✅ |
| Track visualization (canvas) | ✅ |
| Season/championship management | ✅ |
| Racer profiles & behavioral metrics | ✅ |
| SQLite database (offline-first) | ✅ |
| WebSocket real-time updates | ✅ |
| REST API (30+ endpoints) | ✅ |
| Computer vision integration | ✅ |
| AI model selector (YOLO family) | ✅ |
| ROS2 perception node | ✅ |
| Standalone dev mode | ✅ |
| Docker deployment | ✅ |

## AI Model Setup

```bash
# List available models
python -m perception.ai_models.model_manager list

# Download a model
python -m perception.ai_models.model_manager download yolov8n
```

## Project Structure

```
├── frontend/          React 19 + TypeScript + Tailwind CSS
├── backend/           FastAPI + SQLite (async)
├── perception/        AI pipeline + ROS2 node + standalone mode
├── config/            YAML configuration files
├── docs/              Detailed documentation
└── scripts/           Setup and utility scripts
```

## Configuration

Edit `config/default.yaml` for:
- Server ports
- Database path
- AI model selection
- Camera sources
- Perception mode (standalone vs ROS2)


## Raspberry Pi Deployment Workflow

Run the preflight checker from your SSH session before each track setup:

```bash
python3 scripts/pi_preflight.py --backend-host <api-host> --backend-port 8080 --health-url http://<api-host>:8080/health --wizard
```

This verifies:
- ROS2 CLI availability
- Backend TCP/HTTP connectivity
- Camera discovery
- Optional track snapshot capture
- Interactive race setup summary (racers, laps, scale assumptions)

See `docs/PI_DEPLOYMENT_GAP_ANALYSIS.md` for architecture status and priority roadmap.

## Documentation

- [Quick Start](docs/QUICKSTART.md)
- [Camera Setup](docs/CAMERA_SETUP.md)
- [AI Models](docs/AI_MODELS.md)
- [Cloud Deployment](docs/CLOUD_DEPLOY.md)
- [ROS2 Integration](docs/ROS2_INTEGRATION.md)

## License

MIT
