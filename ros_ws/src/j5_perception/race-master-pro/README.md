# Race Master Pro

A comprehensive real-time race tracking system with AI-powered computer vision, designed as a ROS2 perception module. Uses cameras as data sources and AI models (YOLO + DeepSORT) to track racing vehicles, compute lap times, and visualize race data.

## Prerequisites (verify first)

Run from a fresh shell before starting backend/frontend/perception:

```bash
# source ROS2 underlay
source ~/ros2_kilted/install/setup.bash

# build + source the J5 overlay so racetracker_perception is registered
cd ~/j5/ros_ws
colcon build --symlink-install \
  --base-paths src/j5_perception/race-master-pro/perception/ros2_node \
  --packages-select racetracker_perception
source ~/j5/ros_ws/install/setup.bash

# build/install the race-master-pro ROS2 node into the overlay
cd ~/j5/ros_ws
colcon build --symlink-install \
  --base-paths src/j5_perception/race-master-pro/perception/ros2_node \
  --packages-select racetracker_perception
source ~/j5/ros_ws/install/setup.bash

# verify CLI + package visibility
command -v ros2
ros2 -h | rg -w launch

# verify the package exists in your source tree first
find src -name package.xml | rg 'racetracker_perception'

# then verify it's registered in ROS after build+source
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

If you see `Package 'racetracker_perception' not found`, build and re-source the overlay:

```bash
cd ~/j5/ros_ws
colcon build --symlink-install \
  --base-paths src/j5_perception/race-master-pro/perception/ros2_node \
  --packages-select racetracker_perception
source ~/j5/ros_ws/install/setup.bash
ros2 pkg list | rg '^racetracker_perception$'
```

If `ros2 run racetracker_perception perception_node` fails with `ModuleNotFoundError: No module named 'sensor_msgs'`, your underlay is missing ROS message packages in the active environment. Re-source first, then verify:

```bash
source ~/ros2_kilted/install/setup.bash
source ~/j5/ros_ws/install/setup.bash
ros2 pkg list | rg '^sensor_msgs$'
python3 -c "from sensor_msgs.msg import Image; print('sensor_msgs OK')"
```

If `sensor_msgs` is still missing, make sure you are building the underlay (not `~/j5/ros_ws`), then re-source both layers:

```bash
cd ~/ros2_kilted
find src -name package.xml | rg 'sensor_msgs'

# if listed, build it from the underlay
colcon build --symlink-install --packages-select sensor_msgs \
  || colcon build --symlink-install \
       --base-paths src/common_interfaces/sensor_msgs \
       --packages-select sensor_msgs
source ~/ros2_kilted/install/setup.bash

cd ~/j5/ros_ws
source ~/j5/ros_ws/install/setup.bash
ros2 pkg list | rg '^sensor_msgs$'
python3 -c "from sensor_msgs.msg import Image; print('sensor_msgs OK')"
ros2 run racetracker_perception perception_node
```

If `find src -name package.xml | rg 'sensor_msgs'` returns nothing in `~/ros2_kilted`, your underlay sources are incomplete; sync/update `common_interfaces` in that workspace, rebuild underlay, then retry.

If `colcon` prints `ignoring unknown package 'racetracker_perception'`, that build did not run (no-op). `ros2 pkg list` may still show a previously-installed copy, so force a targeted rebuild from `--base-paths` before trusting runtime behavior.

If `colcon` says the package is unknown, confirm package metadata and env paths first:

```bash
cd ~/j5/ros_ws
find src -name package.xml | rg 'racetracker_perception'
echo $AMENT_PREFIX_PATH
echo $CMAKE_PREFIX_PATH
echo $COLCON_CURRENT_PREFIX
```

If `package.xml` exists but `--packages-select racetracker_perception` is still unknown, build from the ROS2 node directory explicitly (the parent `src/j5_perception` package can prevent nested package discovery):

```bash
cd ~/j5/ros_ws
colcon build --symlink-install \
  --base-paths src/j5_perception/race-master-pro/perception/ros2_node \
  --packages-select racetracker_perception
source ~/j5/ros_ws/install/setup.bash
ros2 pkg list | rg '^racetracker_perception$'
```

If `find` returns nothing, your workspace checkout is missing the Race Master Pro ROS2 package; sync/update `~/j5/ros_ws/src/j5_perception/race-master-pro` and retry.

If the traceback still shows `self.subscriptions = []`, first verify your *source file* is updated:

```bash
cd ~/j5/ros_ws
rg 'image_subscriptions|self\.subscriptions' \
  src/j5_perception/race-master-pro/perception/ros2_node/racetracker_perception/perception_node.py

# expected: image_subscriptions only
# if source still shows self.subscriptions, your local file is old
```

Confirm the tracked version at current HEAD too (catches accidental edits / wrong branch):

```bash
cd ~/j5
git status --short ros_ws/src/j5_perception/race-master-pro/perception/ros2_node/racetracker_perception/perception_node.py
git show HEAD:ros_ws/src/j5_perception/race-master-pro/perception/ros2_node/racetracker_perception/perception_node.py | rg 'image_subscriptions|self\.subscriptions'
```

If HEAD still shows `self.subscriptions`, your current branch does not include the fix yet. Sync to a branch/commit that contains it, or apply this local patch and rebuild:

```bash
cd ~/j5
python3 - <<'PY2'
from pathlib import Path
p = Path('ros_ws/src/j5_perception/race-master-pro/perception/ros2_node/racetracker_perception/perception_node.py')
s = p.read_text()
s = s.replace('self.subscriptions = []', 'self.image_subscriptions = []')
s = s.replace('self.subscriptions.append(sub)', 'self.image_subscriptions.append(sub)')
p.write_text(s)
print('patched perception_node.py')
PY2
```

Also verify which module path your current shell will import (to catch wrong/old overlays):

```bash
python3 -c "import racetracker_perception.perception_node as p; print(p.__file__)"
ros2 pkg prefix racetracker_perception
```

If either path points outside `~/j5/ros_ws`, re-source the intended underlay/overlay and retry.

Then rebuild from a clean state:

```bash
cd ~/j5/ros_ws
rm -rf build/racetracker_perception install/racetracker_perception

# optional: clear stale deleted-path warning in current shell
export AMENT_PREFIX_PATH=$(echo "$AMENT_PREFIX_PATH" | tr ':' '\n' | rg -v 'install/racetracker_perception$' | paste -sd: -)

colcon build --symlink-install \
  --base-paths src/j5_perception/race-master-pro/perception/ros2_node \
  --packages-select racetracker_perception
source ~/j5/ros_ws/install/setup.bash

# verify rebuilt artifact now matches source
rg 'image_subscriptions|self\.subscriptions' \
  build/racetracker_perception/racetracker_perception/perception_node.py
ros2 run racetracker_perception perception_node
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

To also auto-create baseline backend records (track, camera config, racers, race,
and race results placeholders) from the wizard answers, use:

```bash
python3 scripts/pi_preflight.py \
  --backend-host <api-host> \
  --backend-port 8080 \
  --backend-base-url http://<api-host>:8080 \
  --wizard --apply-backend --start-race
```

This helps the frontend Settings/Vision/Race views show data immediately on
first boot in headless Pi workflows.

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
