# J5 — Open Robotics Platform

**Safety first:** See `docs/SAFETY.md`. Recording/touch requires explicit consent; face blurring is enabled by default. QuirkPolicy only affects expressivity—never motion/force loops.

## Why Now (Public Momentum)
- Breakthroughs in generalist robot control
- Open humanoid & mobile base hardware
- Edge compute with NVIDIA Thor / Orin
- Hugging Face open datasets (LeRobot, Open-X-Embodiment)
- Simulation at scale (Isaac Sim, Gym)

## Architecture
- **ROS 2 Kilted Kaiju** baseline (source-build workflow)
- NVIDIA Isaac Sim ≥ 2023.x
- OAK-D depth vision, MoveIt 2 (Kilted)
- FaceRig + QuirkPolicy
- Safety systems in hardware + software

## Supported bodies
| Platform | Locomotion Adapter | Arms/Manip | Status     |
|----------|--------------------|------------|------------|
| LT2      | lt2_base            | None       | Alpha      |
| HD2      | hd2_base            | None       | Planning   |


## Quick start
**Codespaces is CPU-only**; for docs/CI. Use local GPU or GPU VM for Isaac Sim.

Install ROS 2 Kilted Kaiju from source and source your workspace overlay:
```bash
# after building your ROS 2 + j5 workspace from source
source ~/j5/ros_ws/install/setup.bash

```

Install Python dependencies:
```bash
pip install -r requirements.txt
pre-commit install
```

Launch the robot:
```bash
ros2 launch j5_bringup bringup.launch.py
```
If `ros2` or bringup packages are not found, verify:
- You're running a ROS 2 Kilted-supported Linux distro.
- All packages are built for ROS 2 (mixing ROS 1 can break discovery).
- Your environment includes the workspace:
  ```bash
  echo $AMENT_PREFIX_PATH
  echo $CMAKE_PREFIX_PATH
  echo $COLCON_CURRENT_PREFIX
  ```
Re-run the `source` commands above after any changes.

If `ros2` is still missing, your source build likely did not finish. Rebuild the workspace (`colcon build`) and source `.../ros_ws/install/setup.bash` in every shell before launching.

#### Source-build troubleshooting (`catkin_pkg`, `ament_cmake`, wrong Python)
If your `colcon build` fails with errors like:
- `ModuleNotFoundError: No module named 'catkin_pkg'`
- `Could not find a package configuration file provided by "ament_cmake"`
- `execute_process(.../venv/bin/python3 ... package_xml_2_cmake.py ...)`

then one (or more) of these is happening:
1. ROS is building with an app virtualenv Python instead of system Python.
2. `catkin_pkg` is missing from system Python.
3. Your J5 workspace is building without the ROS 2 underlay sourced (so `ament_cmake` is unknown).

Recommended fix sequence:

Known-good command for both underlay and overlay builds:
```bash
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

If you occasionally see:
```
CMake Warning: Manually-specified variables were not used by the project:
  Python3_EXECUTABLE
```
that warning can be benign for packages that do not query Python at configure-time. Treat it as actionable only when a traceback still shows the wrong interpreter (for example `.../venv/bin/python3`).

```bash
# 1) use a fresh shell, and ensure no app venv is active
deactivate 2>/dev/null || true
unset VIRTUAL_ENV
unset PYTHONPATH

# 2) clear stale overlay/underlay prefixes from previous failed builds
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_CURRENT_PREFIX

# 3) force system python for colcon/cmake python hooks
export COLCON_PYTHON_EXECUTABLE=/usr/bin/python3
export Python3_EXECUTABLE=/usr/bin/python3

# 4) install required ROS python dependency
sudo apt update
sudo apt install -y python3-catkin-pkg

# 5) source your ROS 2 core underlay (built from source)
# example path - adjust if your ros2 source workspace is elsewhere
source ~/ros2_kilted/install/setup.bash

# 6) rebuild j5 workspace as overlay
cd ~/j5/ros_ws
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

If you are testing Clang, keep in mind this only changes compilers; it does not fix missing underlay packages:
```bash
export CC=clang
export CXX=clang++
colcon build --cmake-force-configure
```


Based on your diagnostics, Python is now correct (`/usr/bin/python3`) and `catkin_pkg` imports successfully, so the remaining blocker is likely an incomplete ROS 2 underlay (no `ament_cmake` in the sourced prefixes).

Also, warnings about missing paths in `AMENT_PREFIX_PATH`/`CMAKE_PREFIX_PATH` mean stale environment values from previous overlays are still set; clear them before building the underlay.

Use this underlay bootstrap flow in `~/ros2_kilted`:
```bash
cd ~/ros2_kilted
# ensure command spelling is symlink, not simlink
colcon build --symlink-install --packages-up-to ament_cmake
source install/setup.bash

# then finish the full underlay build
colcon build --symlink-install
source install/setup.bash

# verify ament_cmake is now discoverable
echo $CMAKE_PREFIX_PATH | tr ':' '\n' | rg ament_cmake || true
```

Then rebuild the J5 overlay:
```bash
cd ~/j5/ros_ws
rm -rf build install log
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```


Because CMake caches the interpreter path, if you ever configured in a venv you must clean the ROS underlay build cache before retrying:
```bash
cd ~/ros2_kilted
rm -rf build install log
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

If it still fails, capture these diagnostics from the same shell:
```bash
which python3
python3 -c "import sys; print(sys.executable)"
python3 -c "import catkin_pkg; print(catkin_pkg.__file__)"
echo $COLCON_PYTHON_EXECUTABLE
echo $PYTHONPATH
echo $AMENT_PREFIX_PATH
echo $CMAKE_PREFIX_PATH
echo $COLCON_CURRENT_PREFIX
```

#### Build failure: `rmw_zenoh_cpp` (zenoh + `std::optional` template error)
If your build fails in `rmw_zenoh_cpp` with errors like:
- `std::optional<zenoh::CancellationToken>`
- `explicitly-defaulted copy constructor ... requires it to be non-const`

this is usually a **toolchain/vendor compatibility issue** in the zenoh RMW path, not a core J5 package issue.

For now, unless you specifically need zenoh middleware, continue with one of these:

```bash
# Option A: skip zenoh RMW package while building ROS 2 source workspace
colcon build --symlink-install --packages-skip rmw_zenoh_cpp

# Option B: build only the RMW you intend to run (example: Fast DDS)
colcon build --symlink-install --packages-select rmw_fastrtps_cpp
```

After build, pin middleware explicitly in runtime shells:
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# or: export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```


If sourcing fails with a message like:
```
not found: ".../install/rmw_zenoh_cpp/share/rmw_zenoh_cpp/local_setup.bash"
```
it means your `install/` tree still contains stale environment hooks from a prior failed build.

Use this recovery flow in the ROS 2 underlay workspace:
```bash
cd ~/ros2_kilted
rm -rf build install log
colcon build --symlink-install \
  --packages-skip rmw_zenoh_cpp \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

# use setup.bash for normal shell setup
source install/setup.bash
command -v ros2
```

Use `install/setup.bash` for regular use. `install/local_setup.bash` is intended for layering on top of an existing environment and is less forgiving when the install tree is inconsistent.
After underlay builds, source `install/setup.bash` before checking `ros2`; using only `local_setup.bash` may not populate your PATH as expected in broken/partial environments.

If you do need zenoh, treat it as a separate follow-up: test compiler/version combinations and/or newer zenoh vendor revisions in a clean workspace.

#### Interpreting `colcon` summary with many stderr lines
If you see a summary like:
- `1 package failed: rmw_zenoh_cpp`
- several packages `aborted`
- many packages with `stderr output`

this is usually not “many independent failures.” In your case:
- `rmw_zenoh_cpp` is the root failure,
- downstream packages (for example `fastdds`, `sensor_msgs`, `tf2_msgs`) may abort because the build graph was interrupted,
- many `ament_*` packages emit non-fatal warnings to stderr during checks.


If `rmw_implementation` fails while skipping zenoh with errors about missing `rmw_zenoh_cpp/package.sh`, use dependency-based skipping instead of skipping only one package:
```bash
cd ~/ros2_kilted
rm -rf build install log
colcon build --symlink-install \
  --packages-skip-by-dep rmw_zenoh_cpp \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
command -v ros2
```

`--packages-skip-by-dep` excludes packages that depend on zenoh (including `rmw_implementation` in this failure mode), which prevents dangling references in the generated setup files.

To continue without zenoh and unblock the rest of the workspace:
```bash
cd ~/ros2_kilted
colcon build --symlink-install   --packages-skip rmw_zenoh_cpp rmw_test_fixture_implementation   --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

Then rebuild your J5 overlay:
```bash
cd ~/j5/ros_ws
colcon build --symlink-install   --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

Optional (to see all failures in one pass):
```bash
colcon build --symlink-install --continue-on-error
```


If `rmw_zenoh_cpp` fails with a C++ error around:
- `std::optional<zenoh::CancellationToken>`
- GCC 13 `optional` copy-constructor notes

this matches the known zenoh/rmw compatibility issue on some toolchains (especially aarch64 + newer libstdc++). Prefer skipping zenoh for now:

To avoid spending ~40+ minutes building zenoh vendor only to fail later in `rmw_zenoh_cpp`, apply the skip flag on the **first** full build attempt:
```bash
cd ~/ros2_kilted
rm -rf build install log
colcon build --symlink-install \
  --packages-skip rmw_zenoh_cpp \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

After build, set runtime middleware explicitly away from zenoh:
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# or: export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

```bash
cd ~/ros2_kilted
rm -rf build install log
colcon build --symlink-install \
  --packages-skip rmw_zenoh_cpp \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

`vcs` / `rosdep` `pkg_resources is deprecated` warnings are informational; they are not the root failure in this trace.

If core interface packages (e.g. `geometry_msgs`, `example_interfaces`) abort after `rmw_zenoh_cpp` fails, treat this as fallout from the first failure and rebuild underlay clean with zenoh skipped-by-dependency:
```bash
cd ~/ros2_kilted
rm -rf build install log
colcon build --symlink-install \
  --packages-skip rmw_zenoh_cpp \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

If many packages remain "not processed", re-run once after cleanup (same command) to complete the graph.

Even if `zenoh_cpp_vendor` finished successfully, a subsequent `rmw_zenoh_cpp` compile failure with the GCC13 `std::optional<zenoh::CancellationToken>` signature is still the primary blocker.

When summary looks like:
- `1 package failed: rmw_zenoh_cpp`
- `fastdds`/`geometry_msgs` aborted
- only a few packages not processed

use this exact recovery loop:
```bash
cd ~/ros2_kilted
rm -rf build install log
colcon build --symlink-install \
  --packages-skip rmw_zenoh_cpp \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

# run once more to process packages that were previously not processed
colcon build --symlink-install \
  --packages-skip rmw_zenoh_cpp \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

source install/setup.bash
```


#### Interpreting warnings after a successful J5 overlay build
If your build ends with `Summary: 9 packages finished` and only `stderr output` warnings (no failed packages), this is generally healthy.

From your reported output:
- Missing-path warnings in `AMENT_PREFIX_PATH` / `CMAKE_PREFIX_PATH` at build start are typically stale values from prior shells.
- `CATKIN_INSTALL_INTO_PREFIX_ROOT` / `CATKIN_SYMLINK_INSTALL` "not used" warnings are usually benign when packages are pure CMake/ament and do not consume catkin-specific vars.
- `setuptools ... Unbuilt egg for pytest-repeat` is a common distro Python warning and not a hard build failure.

Recommended cleanup before builds:
```bash
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_CURRENT_PREFIX PYTHONPATH VIRTUAL_ENV
```

Quick success criteria:
- No `Failed <<< ...` lines
- `Summary:` reports all targeted packages finished
- `source ~/j5/ros_ws/install/setup.bash && command -v ros2` works


If `command -v ros2` is still empty after `source ~/j5/ros_ws/install/setup.bash`, your overlay was sourced without a ROS 2 underlay in that shell.
Source both, in order:
```bash
source ~/ros2_kilted/install/setup.bash
source ~/j5/ros_ws/install/setup.bash
command -v ros2
```

If `command -v ros2` is still empty even after sourcing underlay + overlay, the ROS underlay likely does not contain the `ros2` CLI yet (for example, `ros2cli` package not built/installed), or your install tree is using isolated layout where `install/bin/` may not exist.

Check directly:
```bash
find ~/ros2_kilted/install -type f -path "*/bin/ros2" | head -n 1
```

If this command returns nothing, your underlay did not build/install `ros2cli`.
Confirm the package exists in source and rebuild CLI targets:
```bash
cd ~/ros2_kilted
colcon list | rg '^ros2cli\b'

rm -rf build install log
colcon build --symlink-install \
  --packages-up-to ros2cli ros2run ros2topic demo_nodes_cpp \
  --packages-skip rmw_zenoh_cpp \
  --packages-skip-by-dep rmw_zenoh_cpp \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
command -v ros2
```

If `colcon list | rg '^ros2cli\b'` prints nothing, your source checkout is incomplete; re-import the official ROS 2 kilted `.repos` file and rebuild.
If that command *does* print `ros2cli` but `find ... '*/bin/ros2'` is still empty, then ros2cli exists in source but was not installed successfully.
Using `--packages-select` for `ros2cli` can fail in partially-built workspaces because required dependencies are not built.
Build with `--packages-up-to` from a clean workspace instead:
```bash
cd ~/ros2_kilted
rm -rf build install log
colcon build --symlink-install --packages-up-to ros2cli ros2run ros2topic demo_nodes_cpp \
  --packages-skip rmw_zenoh_cpp \
  --packages-skip-by-dep rmw_zenoh_cpp \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
find ~/ros2_kilted/install -type f -path '*/ros2cli*/bin/ros2' -o -path '*/bin/ros2' | head -n 5
source install/setup.bash
command -v ros2
```

Then re-source your overlay:
```bash
source ~/j5/ros_ws/install/setup.bash
command -v ros2
```

If a `ros2` binary exists in the install tree but `command -v ros2` is still empty, inspect and repair PATH in that shell:
```bash
echo "$PATH"
ROS2_BIN="$(find ~/ros2_kilted/install -type f -path '*/bin/ros2' | head -n 1)"
export PATH="$(dirname "$ROS2_BIN"):$PATH"
command -v ros2
```

Also verify setup scripts are not exiting early:
```bash
set -x
source ~/ros2_kilted/install/setup.bash
set +x
```

#### rosdep: `Cannot locate rosdep definition for [ament_python]`
If `rosdep install` reports unresolved `ament_python` for J5 overlay packages:

```text
j5_voice: Cannot locate rosdep definition for [ament_python]
j5_bringup: Cannot locate rosdep definition for [ament_python]
```

use this sequence (note `--from-paths`, not `--from`):
```bash
sudo rosdep init 2>/dev/null || true
rosdep update

# source underlay first so core ROS packages are present in environment
source ~/ros2_kilted/install/setup.bash

cd ~/j5/ros_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro kilted --skip-keys "ament_python"
```

Why this works:
- `ament_python` is provided by ROS underlay packages, not a system apt key in many source-build setups.
- `--skip-keys "ament_python"` prevents rosdep from trying to resolve it as an OS package.

The `pkg_resources is deprecated` warning from `/usr/bin/rosdep` is informational and not the failure cause.

#### `AMENT_PREFIX_PATH` / `CMAKE_PREFIX_PATH` are empty or unset
This is not automatically a failure. In a fresh shell before sourcing ROS setup files, these variables are often empty.

Use this minimal sanity flow:
```bash
# fresh shell
echo "$AMENT_PREFIX_PATH"
echo "$CMAKE_PREFIX_PATH"

# source underlay first, then overlay
source ~/ros2_kilted/install/setup.bash
source ~/j5/ros_ws/install/setup.bash

# now re-check
command -v ros2
echo "$AMENT_PREFIX_PATH"
echo "$CMAKE_PREFIX_PATH"
```

If they are still empty after sourcing:
1. Underlay `install/` may be incomplete.
2. `setup.bash` may not exist at the path you sourced.
3. Shell may be running in a restricted/non-bash context.

For reliable source builds on constrained SBCs (Pi), use this pattern:
```bash
cd ~/ros2_kilted
rm -rf build install log
colcon build --symlink-install   --packages-skip rmw_zenoh_cpp rmw_test_fixture_implementation   --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

### Development
Run `make lint` to format and `make test` to run tests.



### Working over SSH (multiple shells)
Yes — the simplest approach is to open multiple SSH sessions (multiple terminal windows/tabs) to the same host.

Recommended split:
- Shell 1: backend (`uvicorn` or launcher script).
- Shell 2: frontend (`npm run dev`) if running manually.
- Shell 3: ROS 2 bridge/tests (`bridge.py`, `ros2 topic pub`, logs).

If you prefer one SSH login, use `tmux` and split panes/windows.


### SSH-only network changes (finding Pi IP on a new network)
If your Pi is headless and only reachable by SSH, avoid relying on password-login as your recovery plan.
Keep key-based SSH enabled and use one or more of these approaches:

1. **Use a stable hostname + mDNS**
   - Set hostname once (example): `sudo hostnamectl set-hostname racetime-pi`
   - Connect with: `ssh <user>@racetime-pi.local` (works on most home LANs with mDNS).

2. **Reserve DHCP lease in router**
   - In your router UI, bind the Pi MAC address to a fixed IP.
   - Then SSH always uses that fixed IP.

3. **Check IP from active SSH session before moving networks**
   - `hostname -I`
   - `ip -4 addr show`

4. **Reconfigure Wi-Fi/network from SSH with NetworkManager (`nmcli`)**
   ```bash
   nmcli dev wifi list
   nmcli dev wifi connect "<SSID>" password "<PASSWORD>"
   nmcli con show --active
   ip -4 addr show
   ```

5. **Last-resort emergency access**
   - Keep a serial console/HDMI fallback if possible for first-boot or bad network configs.
   - Prefer this over permanently enabling password SSH auth.

If you must temporarily enable password auth for recovery, revert it immediately after access is restored:
- Set `PasswordAuthentication no` in `/etc/ssh/sshd_config`, then `sudo systemctl reload ssh`.

### Headless Raspberry Pi launch (backend + frontend + bridge)
Use the helper script from repo root:

```bash
# standalone mode (no ROS 2 on host yet)
./scripts/run_racemanager.sh --mode standalone --host 0.0.0.0 --pi-ip <pi-lan-ip>

# ROS 2 mode (expects a completed source build + install/setup.bash)
./scripts/run_racemanager.sh --mode ros2 --host 0.0.0.0 --pi-ip <pi-lan-ip>
```

From another device on your network, open `http://<pi-lan-ip>:3000`.


## Roadmap
1. Hardware bringup
2. Perception + manipulation
3. Simulation integration
4. Safety + HRI polish

## Race Management + Lap Counting (ROS 2 + USB camera)

This section describes a **minimum viable hardware + software setup** for running race management, lap counting, red-flag pause/resume, and live race telemetry on simple ROS 2-capable hardware.

### 1) Shopping list (simplest practical build)

#### Core compute + IO
- Mini PC with Ubuntu 24.04 or newer (Intel N100/N200 class or better, 8 GB RAM, 256 GB SSD).
- 12 V / 5 A power supply for the mini PC (or OEM PSU).
- USB 3.0 hub (powered) for stable camera + peripheral connectivity.

#### Vision + sensing
- 1080p USB webcam (UVC compliant, 60 FPS preferred; 30 FPS minimum).
- Webcam tripod mount or fixed overhead mount.
- Optional LED work light(s) for consistent illumination.

#### Track markers + mounting ("Lego-like" assembly)
- Foam board or corrugated plastic sheets for track perimeter.
- 3D-printed or block-style clip mounts (zip-tie compatible).
- Reusable cable ties + adhesive cable clips.
- Start/finish gate marker (high-contrast striping; black/white checker or red line).
- Checkpoint markers (numbered visual markers, high contrast).
- Gaffer tape (matte) for lane boundaries and marker placement.

#### Operator station
- 24" monitor (or larger) for race GUI + dashboard.
- USB keyboard/mouse.

#### Optional reliability upgrades
- Small UPS for clean shutdowns and power blips.
- External USB SSD for race archive export/backups.

### 2) Physical assembly guide (modular / Lego-style)

1. **Build track perimeter modules**
   - Assemble perimeter panels from foam board/plastic in repeated segments.
   - Join segments with clip mounts or corner braces so modules can be swapped/re-sized.
2. **Install start/finish line**
   - Place the start/finish marker on a straight section with good camera visibility.
   - Keep at least one marker-width of clean space before/after the line to reduce ambiguity.
3. **Install checkpoints**
   - Place checkpoints in race order (`CP1`, `CP2`, ...).
   - Use unique visual markers per checkpoint to avoid false positives.
4. **Mount camera overhead**
   - Position the webcam high enough to capture the full track or key sectors.
   - Lock focus/exposure where supported; avoid auto-exposure flicker if possible.
5. **Cable and power routing**
   - Route USB/power along the outside perimeter with adhesive clips.
   - Use strain relief near camera and mini PC ports.
6. **Validation pass**
   - Verify each checkpoint and the start/finish line are visible under race lighting.
   - Capture a 30-second test stream and confirm marker readability.

### 3) Software behavior to implement/verify

#### Camera feed → race event pipeline
1. Camera node publishes frames (`/camera/image_raw`).
2. Perception node detects cars and track markers.
3. Event node emits structured events:
   - `cross_start_finish`
   - `cross_checkpoint`
   - `lap_completed`
   - `invalid_order_event`
4. Race state machine consumes events and updates race status.

#### Key race data components
- Drivers/vehicles registry.
- Ordered checkpoint definitions.
- Session state (`green`, `yellow`, `red_flag`, `finished`).
- Lap index + lap time history.
- Sector/checkpoint split times.
- Penalties/invalid crossings.

#### Red flag pause/resume
- On `red_flag`:
  - Freeze lap progression.
  - Keep ingesting raw detections (for audit), but do not commit lap advances.
  - Publish state to GUI so controls show paused context.
- On `resume_green`:
  - Re-enable lap progression from the latest valid checkpoint sequence.
  - Annotate timeline with interruption metadata.

### 4) Local database + real-time GUI requirements

#### Local DB
- Use a local persistent DB (SQLite for single-host simplicity, PostgreSQL optional).
- Store:
  - sessions
  - participants
  - checkpoints
  - events (append-only)
  - lap summaries/materialized views

#### Real-time update path
- Event writer commits to DB.
- Aggregator computes standings and live deltas.
- GUI subscribes to live topic/stream (ROS topic or websocket bridge) and refreshes:
  - track map position overlays
  - lap table and deltas
  - race control state

#### Graphical track representation
- Show track polyline with checkpoint overlays.
- Highlight active leaders + lap count in real time.
- Provide replay scrubber for event timeline and lap review.

### 5) Drag-and-drop dashboard builder

Required capabilities:
- Drag/drop widget canvas (timing tower, lap chart, event log, camera tile, track map).
- Per-widget settings (data source, filter, size, refresh cadence).
- Save/load named layouts to local DB/files.
- Export/import layout JSON for portability.
- Role-based presets (Race Director, Marshal, Spectator display).

### 6) Suggested milestone plan

1. **MVP**: start/finish counting + manual participants.
2. **Checkpoint order validation**: enforce lap integrity.
3. **Red flag control**: pause/resume state machine.
4. **Live GUI**: track map + timing tower.
5. **Dashboard builder**: save/load custom layouts.
6. **Replay + analytics**: post-race review tools.

### 7) Notes on private starter repository

- A private repository can be a useful seed for this roadmap.
- If you want direct code-level incorporation from `casualbrooks/racetrack-master-pro`, grant access and we can map its components into this workspace.

## License
See [LICENSE](LICENSE).
