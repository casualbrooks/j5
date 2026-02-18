# J5 — Open Robotics Platform

**Safety first:** See `docs/SAFETY.md`. Recording/touch requires explicit consent; face blurring is enabled by default. QuirkPolicy only affects expressivity—never motion/force loops.

## Why Now (Public Momentum)
- Breakthroughs in generalist robot control
- Open humanoid & mobile base hardware
- Edge compute with NVIDIA Thor / Orin
- Hugging Face open datasets (LeRobot, Open-X-Embodiment)
- Simulation at scale (Isaac Sim, Gym)

## Architecture
- **ROS 2 Iron** baseline
- NVIDIA Isaac Sim ≥ 2023.x
- OAK-D depth vision, MoveIt 2 (Iron)
- FaceRig + QuirkPolicy
- Safety systems in hardware + software

## Supported bodies
| Platform | Locomotion Adapter | Arms/Manip | Status     |
|----------|--------------------|------------|------------|
| LT2      | lt2_base            | None       | Alpha      |
| HD2      | hd2_base            | None       | Planning   |


## Quick start
**Codespaces is CPU-only**; for docs/CI. Use local GPU or GPU VM for Isaac Sim.

Install ROS 2 Iron and source the environment:
```bash
# follow https://docs.ros.org/en/iron/Installation.html
source /opt/ros/iron/setup.bash
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
- You're running **Ubuntu 22.04** (required for ROS 2 Iron).
- All packages are built for ROS 2 (mixing ROS 1 can break discovery).
- Your environment includes the workspace:
  ```bash
  echo $AMENT_PREFIX_PATH
  echo $CMAKE_PREFIX_PATH
  echo $COLCON_CURRENT_PREFIX
  ```
Re-run the `source` commands above after any changes.

### Development
Run `make lint` to format and `make test` to run tests.


## Roadmap
1. Hardware bringup
2. Perception + manipulation
3. Simulation integration
4. Safety + HRI polish

## Race Management + Lap Counting (ROS 2 + USB camera)

This section describes a **minimum viable hardware + software setup** for running race management, lap counting, red-flag pause/resume, and live race telemetry on simple ROS 2-capable hardware.

### 1) Shopping list (simplest practical build)

#### Core compute + IO
- Mini PC with Ubuntu 22.04 (Intel N100/N200 class or better, 8 GB RAM, 256 GB SSD).
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
