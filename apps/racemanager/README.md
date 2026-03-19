# Race Manager (apps/racemanager)

This folder holds the race-management web UI, backend service, and ROS bridge that connect the lap counter to MongoDB Atlas and keep live race/championship views updated. A minimal FastAPI service implementation now lives in `service/` and is ready to ingest laps from the bridge.

## Layout
- `ui/`: Web client (e.g., React/Next.js) that subscribes to websocket updates and renders live leaderboards + championship tables.
- `service/`: API + websocket fan-out (FastAPI starter) that ingests validated lap events, writes to Atlas, and computes derived stats.
- `bridge/`: ROS 2/vision-side process that consumes lap counter topics and posts vetted laps to the service.
- `scripts/`: Seeders/loaders for fake races, migrations, and local fixtures.

## Environment
Create per-component `.env` files; the service needs at minimum:

```
ATLAS_URI=mongodb+srv://<user>:<password>@cluster...
RACE_DB_NAME=j5_racing
LAPS_COLLECTION=laps_live
WS_PUBLIC_URL=ws://localhost:4000/ws
HTTP_PORT=4000
# Optional auth
JWT_SECRET=change-me
```

For the FastAPI service, copy `apps/racemanager/service/.env.example` to `.env` in
the same folder and fill in your MongoDB connection string and collection
names. The config loader reads that file automatically on startup.

Quickstart for the service (from the repo root):

1. `cp apps/racemanager/service/.env.example apps/racemanager/service/.env` and
   edit the values
2. `python -m venv apps/racemanager/service/.venv && source apps/racemanager/service/.venv/bin/activate`
3. `pip install -r apps/racemanager/service/requirements.txt`
4. `python -m uvicorn apps.racemanager.service.main:app --reload --env-file apps/racemanager/service/.env`

The UI should point to the same HTTP/websocket endpoints (e.g.,
`NEXT_PUBLIC_API_BASE`, `NEXT_PUBLIC_WS_URL`).

## Data flow (live + replay)
1. **Lap counter node (ROS 2)** publishes validated laps: `{ carId, gateTime, lapTimeMs, headingOk, onTrack, minLapTimeOk }`.
2. **Bridge** (`bridge/`) subscribes to the lap topic, attaches `raceId`/`seasonId`, and posts to `service/ingest` (HTTP or gRPC). Reject laps that fail anti-cheat flags. The bridge can also stream laps detected from prerecorded videos into the same endpoint with `source="replay"`.
3. **Service** persists to Atlas collections (`laps_live`, `races`, `championships`, `telemetry_archive`) and emits websocket events `{ type: "lap", payload: {...} }` to UI clients.
4. **UI** listens for `lap`, `race_status`, and `standings` events to repaint per-car cards and leaderboards in real time.

## Derived stats (track distance + lap times)
The service should enrich each lap with:
- `trackDistanceM`: pulled from race config.
- `speedKph`: `(trackDistanceM / lapTimeMs) * 3.6`.
- `isValid`: `onTrack && headingOk && minLapTimeOk`.

Expose per-car aggregates that the UI can bind to existing proof-of-concept fields:
- Lap count, last lap, best lap, average lap time, and rolling median.
- Average speed per lap and session average.
- Delta to leader: `gapMs = carElapsedMs - leaderElapsedMs`.
- Consistency: standard deviation of recent N lap times + a percentage (e.g., `1 - (stdev/mean)`).
- Championship rollups: accumulate points/podiums/wins/best-lap counts into `championships.standings` after each race.

## Suggested endpoints/events (implemented in FastAPI starter)
- `POST /ingest/lap`: accepts the payload above plus `raceId`, `sessionLap`, and optional `sectorTimes`.
- `GET /races/:raceId/standings`: returns leaderboard with computed gaps and best laps.
- `GET /health` / root: sanity endpoints for probes.
- Websocket events: `lap`, `leaderboard`, `race_status`, `championship` (stubbed; add via FastAPI websockets or a sidecar socket server).

## Local development
- Start Atlas tunnel or use a local Mongo instance with the same schemas.
- **Run the service**: `python -m venv apps/racemanager/service/.venv && source apps/racemanager/service/.venv/bin/activate && pip install -r apps/racemanager/service/requirements.txt && python -m uvicorn apps.racemanager.service.main:app --reload --env-file apps/racemanager/service/.env`.
- **Smoke test ingest**: from another shell run `python apps/racemanager/bridge/bridge.py` to push a demo lap into the service (requires Mongo running).
- **Run the UI**: `cd apps/racemanager/ui && npm install && npm run dev`, pointing to the service URLs above.
- **ROS-side testing without hardware**: feed recorded lap messages through the bridge (bag replay) and confirm the UI updates instantly.
- **Perception regression testing**: point the bridge replay runner to a prerecorded MP4; forward the resulting laps with `source="replay"`/`replayId` so the service/UI can compare them against live runs.

### MP4 replay ingestion on Raspberry Pi 5 (copy/paste commands)

1. Copy an MP4 from another machine to the Pi:
   ```bash
   scp /path/to/session_2026-02-25.mp4 racetime@<pi5-ip>:~/j5/replays/
   ```

2. Or sync a whole replay folder from another machine:
   ```bash
   rsync -avP /path/to/replay_folder/ racetime@<pi5-ip>:~/j5/replays/
   ```

3. Or copy from a USB stick plugged into the Pi:
   ```bash
   mkdir -p ~/usb ~/j5/replays
   sudo mount /dev/sda1 ~/usb
   cp ~/usb/session_2026-02-25.mp4 ~/j5/replays/
   sudo umount ~/usb
   ```

4. Run your lap extractor against the MP4 and emit JSON lines (one lap per line):
   ```bash
   python /path/to/your_lap_extractor.py \
     --video ~/j5/replays/session_2026-02-25.mp4 \
     --race-id demo-race \
     --replay-id pi5-session-2026-02-25 \
     > /tmp/replay_laps.jsonl
   ```

5. Ingest replay laps into the Race Manager service via the bridge:
   ```bash
   cat /tmp/replay_laps.jsonl | python apps/racemanager/bridge/bridge.py --mode stdin --service-url http://localhost:4000
   ```

6. Optional quick smoke test without a real extractor (single replay lap):
   ```bash
   printf '%s\n' '{"raceId":"demo-race","carId":"car-7","sessionLap":1,"lapTimeMs":70234,"trackDistanceM":350.0,"source":"replay","replayId":"pi5-manual-test","validity":{"onTrack":true,"directionOk":true,"minLapTimeOk":true}}' \
   | python apps/racemanager/bridge/bridge.py --mode stdin --service-url http://localhost:4000
   ```

## Pushing the `race-manager` branch to GitHub
If the branch was recreated locally and you need to publish it to your GitHub fork or the upstream origin, do the following from the repo root:

1. Fetch the latest remotes so your local branch has the current refs: `git fetch --all --prune`.
2. Ensure you are on the branch you want to publish (recreate if needed): `git checkout race-manager` (or `git switch race-manager`).
3. If the branch should track `origin/race-manager`, set upstream on first push: `git push -u origin race-manager`.
4. For subsequent pushes just run `git push` and Git will use the tracked upstream.
5. Verify the branch exists on GitHub: `git ls-remote --heads origin race-manager` should show the new ref.

## Using `racetrack-master-pro` with this layout
- **Where to place it**: keep the upstream project under `apps/racemanager/racetrack-master-pro/` (submodule/subtree) or fold its UI/service code into `apps/racemanager/ui` and `apps/racemanager/service` so the bridge path stays unchanged.
- **Endpoints to align**: ensure it exposes `POST /ingest/lap` plus websocket topics `lap`, `leaderboard`, `race_status`, and `championship` so the bridge and UI continue to interoperate.
- **Schema mapping**: map incoming lap fields to the lap counter payload `{ carId, raceId, lapTimeMs, onTrack, headingOk, minLapTimeOk, trackDistanceM, speedKph }` and store them in the Atlas collections noted above.
- **Replay parity**: feed prerecorded MP4s through the same ingestion endpoint with `source: "replay"` and `replayId` to validate against live sessions.

## Standalone + ROS 2 bridge modes

The bridge now supports both non-ROS and ROS 2 execution so you can run on a
single mini PC with only a USB camera and add ROS later.

### Standalone (no ROS 2 required)
1. Start the API service with SQLite (default):
   ```bash
   cp apps/racemanager/service/.env.example apps/racemanager/service/.env
   python -m venv apps/racemanager/service/.venv
   source apps/racemanager/service/.venv/bin/activate
   pip install -r apps/racemanager/service/requirements.txt
   uvicorn apps.racemanager.service.main:app --reload --env-file apps/racemanager/service/.env
   ```
2. In another shell, run a demo lap feed:
   ```bash
   python apps/racemanager/bridge/bridge.py --mode demo --service-url http://localhost:4000
   ```
3. Query leaderboard:
   ```bash
   curl http://localhost:4000/races/demo-race/leaderboard
   ```

### ROS 2 mode (source-build workflow)
1. Build and source your ROS 2 workspace from source:
   ```bash
   cd ~/j5/ros_ws
   colcon build --symlink-install
   source ~/j5/ros_ws/install/setup.bash
   ```
2. Verify CLI exists in this shell:
   ```bash
   command -v ros2
   ```
3. Run bridge subscriber:
   ```bash
   python apps/racemanager/bridge/bridge.py --mode ros2 --topic /race/lap_event
   ```
4. Publish JSON lap event on topic for smoke test:
   ```bash
   ros2 topic pub /race/lap_event std_msgs/msg/String '{data: "{\"raceId\":\"demo-race\",\"carId\":\"car-7\",\"sessionLap\":1,\"lapTimeMs\":70000,\"trackDistanceM\":350.0,\"validity\":{\"onTrack\":true,\"directionOk\":true,\"minLapTimeOk\":true}}"}' --once
   ```

### Websocket updates for GUI
- Connect GUI clients to `ws://<host>:4000/ws`.
- Service broadcasts:
  - `type="lap"` after each ingest
  - `type="leaderboard"` with recalculated standings


## One-command launcher (headless/server friendly)

Use `scripts/run_racemanager.sh` from the repo root to start backend + frontend and
(optionally) the bridge in standalone/demo or ROS 2 mode.

This launcher is the **all-in-one equivalent** of the manual service/UI/bridge steps above
(so you can run without `tmux` if preferred).

Before starting processes, run:

```bash
./scripts/run_racemanager.sh --mode ros2 --doctor
```

Examples:

```bash
# Standalone mode (no ROS 2), reachable from other devices
./scripts/run_racemanager.sh --mode standalone --host 0.0.0.0 --pi-ip 192.168.1.50

# ROS 2 mode (sources your workspace install/setup.bash)
./scripts/run_racemanager.sh --mode ros2 --host 0.0.0.0 --pi-ip 192.168.1.50
```

The script prints browser URLs for local and LAN access (for example,
`http://192.168.1.50:3000`) and sets:
- `NEXT_PUBLIC_API_BASE=http://<pi-ip>:4000`
- `NEXT_PUBLIC_WS_URL=ws://<pi-ip>:4000/ws`

By default the Race Manager UI runs on port `3000`, because the frontend uses
Next.js (`npm run dev` → `next dev`) rather than Vite. If you try `5173`, the
page will not respond unless you explicitly override `--ui-port 5173`.

Typical healthy startup logs include `✓ Ready`, `✓ Compiled /`, and `GET / 200`
from Next.js. Those lines mean the UI did build and is serving traffic; if the
browser is blank, double-check that you are opening port `3000` (or your
explicit `--ui-port` override) rather than the legacy Vite port `5173`.

If ROS 2 is not installed system-wide, that is OK for this workflow. Build ROS 2 + dependencies from source, source your workspace `install/setup.bash`, then retry `--mode ros2`.


### SSH and multiple shells
- Yes, you can SSH in multiple times (multiple terminal windows/tabs).
- Alternative: use `tmux` in one SSH session and split panes for service/UI/bridge logs.


### ROS 2 build gotcha: app virtualenv, underlay, and `ament_cmake`
If `colcon build` reports `No module named 'catkin_pkg'` or cannot find `ament_cmake`, use a fresh shell and run:

Known-good build command:
```bash
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

If CMake warns that `Python3_EXECUTABLE` was not used, that warning is often benign for packages that do not use Python in configure steps. Focus on whether failures still reference `.../venv/bin/python3`.

```bash
deactivate 2>/dev/null || true
unset VIRTUAL_ENV
unset PYTHONPATH
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_CURRENT_PREFIX
export COLCON_PYTHON_EXECUTABLE=/usr/bin/python3
export Python3_EXECUTABLE=/usr/bin/python3
sudo apt update && sudo apt install -y python3-catkin-pkg

# rebuild/source ROS 2 core underlay with explicit system python
cd ~/ros2_kilted
rm -rf build install log
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash

# then build j5 overlay
cd ~/j5/ros_ws
rm -rf build install log
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

`CC`/`CXX` (Clang) only selects the compiler; it does not replace missing underlay setup or missing Python deps.


If diagnostics already show `/usr/bin/python3` and `catkin_pkg` works, but `ament_cmake` is still missing, your ROS 2 underlay build is incomplete. Bootstrap it first:

```bash
cd ~/ros2_kilted
colcon build --symlink-install --packages-up-to ament_cmake
source install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Then rebuild `~/j5/ros_ws` as the overlay.


If you see warnings about missing paths in `AMENT_PREFIX_PATH` / `CMAKE_PREFIX_PATH`, those are stale values from a previous environment; clear them before running `colcon` as shown above.


### Headless Pi on a new network (SSH-only)
- Prefer SSH keys + hostname/mDNS (`<hostname>.local`) or DHCP reservation instead of re-enabling password auth long-term.
- From any active SSH shell, get current address with `hostname -I` or `ip -4 addr show`.
- To switch Wi-Fi over SSH (NetworkManager):
  ```bash
  nmcli dev wifi list
  nmcli dev wifi connect "<SSID>" password "<PASSWORD>"
  nmcli con show --active
  ip -4 addr show
  ```
- If you temporarily turn on password login for recovery, disable it again immediately after restoring key-based access.


If ROS underlay sourcing fails due to missing `rmw_zenoh_cpp` local setup scripts, rebuild `~/ros2_kilted` from clean (`rm -rf build install log`) with `--packages-skip rmw_zenoh_cpp`, then source `~/ros2_kilted/install/setup.bash` (not `local_setup.bash`) before launching in `--mode ros2`.


If ROS underlay builds fail at `rmw_implementation` after skipping zenoh, rebuild with dependency-based skipping: `colcon build --symlink-install --packages-skip-by-dep rmw_zenoh_cpp --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3`, then source `~/ros2_kilted/install/setup.bash` and verify `command -v ros2`.


If your J5 overlay build finishes all packages and only shows `stderr output` warnings (unused `CATKIN_*` vars, `pytest-repeat` egg warning), treat it as non-fatal unless there are actual `Failed <<<` entries.


If `command -v ros2` is empty after sourcing `~/j5/ros_ws/install/setup.bash`, source your ROS underlay first (`source ~/ros2_kilted/install/setup.bash`), then source the J5 overlay again.


If `command -v ros2` is still empty after sourcing underlay then overlay, verify underlay CLI binary exists (for example: `find ~/ros2_kilted/install -type f -path "*/bin/ros2" | head -n 1`). If missing, rebuild underlay with at least `ros2cli` before launching `--mode ros2`.


If overlay rosdep install fails with `Cannot locate rosdep definition for [ament_python]`, source underlay first and run: `rosdep install --from-paths src --ignore-src -r -y --rosdistro kilted --skip-keys "ament_python"`.


If a `ros2` binary exists under `~/ros2_kilted/install` (for example `*/bin/ros2`) but `command -v ros2` stays empty, prepend PATH manually before launching `--mode ros2`: `ROS2_BIN="$(find ~/ros2_kilted/install -type f -path '*/bin/ros2' | head -n 1)" && export PATH="$(dirname "$ROS2_BIN"):$PATH"`.


If no `ros2` binary is found under `~/ros2_kilted/install`, verify underlay sources include `ros2cli` (`colcon list | rg '^ros2cli\b'`) and rebuild CLI targets before launching `--mode ros2`.


If `colcon list | rg '^ros2cli\b'` returns `ros2cli` but no `ros2` binary appears under `~/ros2_kilted/install`, rebuild from clean using dependency-aware targets (`rm -rf build install log && colcon build --symlink-install --packages-up-to ros2cli ros2run ros2topic demo_nodes_cpp --packages-skip rmw_zenoh_cpp --packages-skip-by-dep rmw_zenoh_cpp`) before `--mode ros2`.


If ROS underlay summary shows `rmw_zenoh_cpp` failed and core interfaces (`example_interfaces`, `geometry_msgs`) aborted, clean and rebuild underlay with `--packages-skip rmw_zenoh_cpp`, then source underlay again before `--mode ros2`.


If `rmw_zenoh_cpp` fails with GCC 13 `std::optional<zenoh::CancellationToken>` errors, skip the zenoh RMW package for underlay builds and proceed with Fast DDS/CycloneDDS middleware for `--mode ros2` workflows.


If `zenoh_cpp_vendor` builds but `rmw_zenoh_cpp` fails (GCC13 optional signature), skip the zenoh RMW package and rerun the same underlay build command once so previously not-processed packages complete before `--mode ros2`.


To avoid long zenoh vendor compile time on affected toolchains, use `--packages-skip rmw_zenoh_cpp` from the first underlay build and export `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` (or `rmw_cyclonedds_cpp`) before ROS-mode runs.


If `AMENT_PREFIX_PATH` / `CMAKE_PREFIX_PATH` are empty, check them again only after sourcing underlay then overlay; empty values in a fresh shell are expected.


### Copy-paste quick recovery (ROS 2 mode)
```bash
# 1) rebuild underlay (skip zenoh dependency chain)
cd ~/ros2_kilted
rm -rf build install log
colcon build --symlink-install   --packages-skip rmw_zenoh_cpp rmw_test_fixture_implementation   --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash

# 2) rebuild J5 overlay
cd ~/j5/ros_ws
rm -rf build install log
colcon build --symlink-install   --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash

# 3) verify ROS CLI and run launcher
command -v ros2
./scripts/run_racemanager.sh --mode ros2 --host 0.0.0.0 --pi-ip <pi-lan-ip>
```
