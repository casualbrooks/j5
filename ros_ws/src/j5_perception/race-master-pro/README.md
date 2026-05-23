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

### One-command launcher

From the repo root you can now start the Race Master Pro backend plus the Vite
walkthrough UI with:

```bash
./scripts/run_race_master_pro.sh --mode ros2 --host 0.0.0.0 --pi-ip <pi-lan-ip>
```

That launcher keeps the Settings/Computer Vision/Dashboard workflow on the
legacy Vite port (`5173`) while pointing the UI at the FastAPI backend on
`8080`.

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

By default the Vite UI tries same-origin `/api` first and falls back to
`http://<current-host>:8080`. Websocket traffic stays on same-origin `/ws` for
HTTPS/reverse-proxy deployments and falls back to
`ws://<current-host>:8080/ws?client_type=organizer` during local Vite dev. Override
those targets with `VITE_API_BASE_URL=http://<api-host>:8080` and
`VITE_WS_URL=ws://<api-host>:8080/ws` before starting `npm run dev` when the
FastAPI service lives on a different host or port.

### 3. Run the Perception System

```bash
# Standalone mode with a real USB camera (no ROS2 required)
python -m perception.standalone.standalone_runner \
  --camera-source /dev/video0 \
  --camera-id cam1 \
  --ws-url ws://<api-host>:8080/ws?client_type=cv_system

# Optional: mock mode for quick UI testing only
python -m perception.standalone.standalone_runner --use-mock

# ROS2 node mode (requires sourced underlay + overlay in this shell)
ros2 run racetracker_perception perception_node
```

When standalone camera mode is running, it publishes `visionDetection` websocket
events with object IDs like `cv-cam1-track-1`, `cv-cam1-track-2`, etc. The **Computer Vision**
panel will show those IDs in **Recent object detections**, and you can assign
them to racers in **Settings → Racer tracking assignments**.

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

### 4. Operator flow for camera-based object tracking

1. Start backend + frontend.
2. Start camera preview (`scripts/pi_preflight.py --serve-preview ...`) so you can position camera framing.
3. Start standalone perception camera runner (`python -m perception.standalone.standalone_runner --camera-source /dev/video0 ...`).
4. In UI **Settings**, initialize race state and assign each racer a live object ID from detections.
5. In **Computer Vision**, click **Start Tracking**.
6. In **Dashboard**, verify racer markers move with incoming detections.

If detections do not appear:
- Confirm the runner is connected to `ws://<api-host>:8080/ws?client_type=cv_system`.
- Confirm only one process has exclusive access to `/dev/video0`.
- Move cars clearly across frame (motion-based detector filters static background).

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
head -n 1 install/racetracker_perception/lib/racetracker_perception/perception_node
python3 -c "import racetracker_perception.perception_node as p; print(p.__file__)"
```

## Hardened ROS2 + YOLO runtime (system Python path)

If you intentionally launch with:

```bash
ros2 run racetracker_perception perception_node
```

the generated console script may use `/usr/bin/python3` directly. On Raspberry Pi
images that mix apt-provided scientific packages (for example matplotlib) with user-site
pip installs, NumPy 2.x can cause ABI import failures while importing ultralytics:

```text
A module that was compiled using NumPy 1.x cannot be run in NumPy 2.x
```

Use this hardened sequence in a fresh shell:

```bash
source ~/ros2_kilted/install/setup.bash
cd ~/j5/ros_ws

# optional cleanup if you changed overlays recently
rm -rf build/racetracker_perception install/racetracker_perception log

colcon build --symlink-install \
  --base-paths src/j5_perception/race-master-pro/perception/ros2_node \
  --packages-select racetracker_perception
source ~/j5/ros_ws/install/setup.bash

# verify ros2 entrypoint interpreter
head -n 1 install/racetracker_perception/lib/racetracker_perception/perception_node
```

Install runtime deps for `/usr/bin/python3` (the interpreter used by that shebang):

```bash
/usr/bin/python3 -m pip install --user --break-system-packages ultralytics "numpy<2"

# verify imports in system python
/usr/bin/python3 -c "import numpy; print(numpy.__version__)"
/usr/bin/python3 -c "import matplotlib; print('matplotlib OK')"
/usr/bin/python3 -c "from ultralytics import YOLO; print('ultralytics OK')"
```

Launch and confirm YOLO initialization:

```bash
ros2 run racetracker_perception perception_node --ros-args --log-level info \
  -p camera_topics:=[/camera/cam1/image_raw] \
  -p auto_discover_camera_topics:=true \
  -p confidence_threshold:=0.35
```

Expected startup signal:

```text
Loaded YOLO model from .../perception/models/yolov8n.pt
```

If you see repeated model downloads, run once while connected to the internet so
`perception/models/yolov8n.pt` is cached for future runs.


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
python3 scripts/pi_preflight.py \
  --backend-host <api-host> \
  --backend-port 8080 \
  --health-url http://<api-host>:8080/health \
  --camera-source /dev/video0 \
  --capture-file track_snapshot.jpg \
  --serve-preview --preview-host 0.0.0.0 --preview-port 8091
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

After the wizard finishes:
- Open `preflight_summary.json` and use `wizard.track.meters_per_pixel` as your baseline calibration value.
- If you used `--apply-backend`, keep the printed Race/Track/Camera IDs for API debugging and race operations.
- Verify racers and lap count in the UI match the wizard values before live heats.
- Before pressing **Start Tracking**, capture the latest track photo, mark the start line / finish line / checkpoints, and confirm each detected car is mapped to the correct racer entry for this race.
- If MongoDB is running on the Pi, verify `mongosh --eval 'db.adminCommand({ ping: 1 })'` succeeds and point the backend service at `mongodb://<pi-ip>:27017/j5_racing` (or your configured authenticated URI) so setup records and live events persist outside local SQLite.
- Treat the dashboard as a consumer of validated backend state: the API should persist checkpoint/lap/incident events first, then fan them out over websocket so the track map, leaderboard, and racer cards stay in sync.

If snapshot capture says `ffmpeg not installed`, install it on Raspberry Pi:

```bash
sudo apt update
sudo apt install -y ffmpeg
```

Then rerun preflight without `--skip-capture`.

To view the camera feed from another device on your network and manually trigger
snapshot capture from a browser, start preview mode:

```bash
python3 scripts/pi_preflight.py \
  --backend-host <api-host> \
  --backend-port 8080 \
  --health-url http://<api-host>:8080/health \
  --camera-source /dev/video0 \
  --capture-file track_snapshot.jpg \
  --serve-preview --preview-host 0.0.0.0 --preview-port 8091
```

Then open `http://<pi-ip>:8091/` from another device. Use **Capture Track Photo**
to write the snapshot to `track_snapshot.jpg` on the Pi.

If you need preview + tracking at the same time, use a single camera owner (`camera_ros_publisher.py`) and let the perception node subscribe to `/camera/cam1/image_raw`.

Run the publisher + preview stream:

```bash
python3 scripts/camera_ros_publisher.py \
  --device /dev/video0 \
  --topic /camera/cam1/image_raw \
  --width 1280 --height 720 --fps 15 \
  --pixel-format MJPG \
  --serve-preview --preview-host 0.0.0.0 --preview-port 8091
```

Then start the perception node in another shell (same host):

```bash
python -m racetracker_perception.perception_node   --ros-args   -p camera_topics:=[/camera/cam1/image_raw]   -p auto_discover_camera_topics:=true   -p confidence_threshold:=0.25
```

Calibration/tuning notes:
- `scripts/pi_preflight.py` is **preview + capture only**. It does not run object tracking.
- For race tracking and lap counting, run `camera_ros_publisher.py` + `racetracker_perception.perception_node`.
- Start at `--width 960 --height 540 --fps 15` to reduce motion blur + ID switching.
- Keep `confidence_threshold` around `0.25-0.35` and tune upward only if false positives are excessive.
- Use only one camera owner process for `/dev/video0` at a time to avoid blank preview or camera-open failures.

This is useful when `usb_cam` is unavailable in the current source-built environment.

You can also use Race Manager Pro UI from another machine at
`http://<pi-ip>:5173` -> **Computer Vision** tab:
- set **Preview server URL** to `http://<pi-ip>:8091`
- verify live feed appears
- click **Capture Track Photo** to trigger remote snapshot save on Pi

If you only want to bring the camera feed back for the **Computer Vision** tab
and do **not** want to re-initialize the race, rerun just this preview command
on the Pi:

```bash
python3 scripts/pi_preflight.py \
  --camera-source /dev/video0 \
  --capture-file track_snapshot.jpg \
  --serve-preview --preview-host 0.0.0.0 --preview-port 8091
```

That command is safe to use independently of the race setup flow. Replace
`/dev/video0` if your camera is exposed at a different device path.

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


## Web Setup Wizard (Settings + Vision tabs)

Race setup can now be driven from the web UI in ordered steps under **Settings**:

1. Verify ROS2 CLI availability (`ros2`, `launch`)
2. Verify Raspberry Pi reachability
3. Verify backend health endpoint
4. Initialize race state (event/race/racers + championship context)
5. Verify camera preview service before live tracking
6. Verify websocket/event flow before live race operations

The Settings wizard shows for each step:

- verification status (connected/not connected)
- exact command needed to continue
- one-click buttons to verify, connect/reconnect, and stop
- the next-step command so operators can progress linearly

Only **Pi reachability** and **backend health** block `Initialize Race State`. The
preview + websocket checks remain visible in the wizard, but they are intended
to be completed after race metadata is created and before live tracking begins.

Backend endpoints added for this workflow:

- `GET /api/setup/wizard`
- `PUT /api/setup/wizard/config`
- `POST /api/setup/wizard/steps/{step_id}/verify|connect|stop`
- `POST /api/setup/wizard/initialize`
- `POST /api/setup/wizard/reset`

### Vision flow updates

After setup is connected:

- use **Vision** tab to capture the track image
- start/stop tracking from the same panel
- inspect live lap/tracking logs streamed from backend state

### Pause / Resume with state restore

Race state persistence now supports snapshot and resume:

- `POST /api/races/{race_id}/snapshot` to persist race + laps + results + setup context
- `POST /api/races/{race_id}/resume` to restore snapshot context and continue race
- `GET /api/races/{race_id}/state` for current runtime state

This enables preserving laps counted, track image workflow state, racer metadata, and log context when pausing/resuming operations through the web interface.
