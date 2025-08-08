
# Project J5 — Lovable Tracked Robot (2025)

**Goal:** A 2025‑capable, lovable *tracked* robot with expressive head/eyes, safe manipulation, and on‑device reasoning. Think “Johnny 5 energy,” modern hardware.

## Why now (Public momentum)
- **Edge AI compute:** Jetson Thor / AGX Orin-class modules can run multi‑modal models, VIO, planning, and speech on‑bot.
- **Generalist robot models:** NVIDIA Isaac GR00T (N1 lineage), plus community generalists, make imitation‑ and RL‑style skills practical.
- **Open hardware:** Affordable tracked bases, open manipulation (xArm/UR/Franka), and open humanoids (K-Scale, Reachy) enable shared “brain” across bodies.
- **Open datasets & tooling:** Hugging Face (LeRobot, Open‑X‑Embodiment), ROS 2, MoveIt 2, Nav2, and Isaac Sim lower the barrier to real capability.
- **UX expectations:** Expressivity is now table stakes. We design character and “lovable flaws” as first‑class features (see **QuirkPolicy** below).

## Architecture (high-level)
- **OS/Orchestration:** Ubuntu 22.04 + ROS 2 (Iron/Jazzy).
- **Compute:** Jetson Thor (preferred) or AGX Orin (drop‑in fallback).
- **Perception:** 2× Luxonis OAK‑D Pro (stereo + depth), 2D LiDAR (RPLIDAR S2/S3), IMU, mic array.
- **Navigation:** Isaac Perceptor/cuVSLAM + Nav2.
- **Manipulation:** MoveIt 2 + Isaac Manipulator; start with xArm‑class 6/7‑DOF + soft gripper.
- **Simulation:** Isaac Sim / Omniverse for data + rehearsal.
- **LLM/VLM:** On‑device distilled controller with optional edge/cloud assist for long‑context tasks.
- **Memory:** Lightweight episodic store (SQLite + Faiss) for people/places/jokes.
- **Character:** FaceRig topics + **QuirkPolicy** behavior tree nodes for micro‑pauses, gaze rules, self‑talk, and running gags (never in safety loops).

## Repo layout
See directories in this repo. Key packages live under `ros_ws/src/`:
- `j5_bringup` — launch files, params, diagnostics
- `j5_navigation` — Nav2 stack, maps, controllers
- `j5_perception` — OAK‑D, LiDAR, IMU, fusion
- `j5_manipulation` — MoveIt 2 configs, planners, grippers
- `j5_facerig` — eyes/eyelids/brows, gaze, expressions
- `j5_character` — QuirkPolicy BehaviorTree.CPP nodes
- `j5_memory` — vector DB, episodic recall
- `j5_voice` — ASR/TTS nodes, dialog bridge
- `j5_sim` — Isaac Sim scenes, teleop, DR presets

## Quick start (Codespaces or local)
### Option A: GitHub Codespaces (CPU)
1. Open in Codespaces. The `.devcontainer/` will install ROS 2, colcon, and deps.
2. Build: `colcon build --symlink-install` inside `j5/ros_ws`
3. Source: `source install/setup.bash`
4. Run bringup: `ros2 launch j5_bringup j5_minimal.launch.py`
> Codespaces has no GPU; use it for CI, docs, and CPU‑only unit tests.

### Option B: Local dev (GPU recommended)
- **Workstation or laptop** with NVIDIA GPU + Docker.
- Install NVIDIA Container Toolkit. Then: `devcontainer open` (VS Code) or `docker compose up` using provided images.
- For Jetson targets, build on‑device or use cross‑compile toolchains.

### Simulation
- Install Isaac Sim (local GPU) or run an NGC container on a cloud GPU VM.
- Open `j5/sim/isaac/J5_Apartment.usd` (placeholder) and run the teleop demo.

## Contributing
- Code: Apache‑2.0 • Hardware: CERN‑OHL‑S • Docs: CC‑BY 4.0
- See `docs/CONTRIBUTING.md` and `docs/SAFETY.md`.

## Roadmap
1) Bring‑up (teleop, estop, bumpers) → 2) Mapping & Nav2 → 3) FaceRig + Voice → 4) Manipulation MVP → 5) GR00T‑skills pass → 6) Public demo etiquette.

---
