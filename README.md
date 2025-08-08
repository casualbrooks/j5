# J5 Project Workspace

This repository hosts mechanical, electronics, firmware, ROS 2, simulation, and documentation for the J5 robot platform.

## Project vision (2025)
Goal: A 2025‑capable, lovable tracked robot with expressive head/eyes, safe manipulation, and on‑device reasoning. “Johnny 5 energy,” modern hardware.

### Why now (public momentum)
- Edge AI compute: Jetson AGX Orin/Thor can run multi‑modal models, VIO, planning, and speech on‑bot.
- Generalist robot models: NVIDIA Isaac GR00T lineage + community models enable imitation/RL‑style skills.
- Open hardware: Affordable tracked bases and open arms enable a shared “brain” across bodies.
- Open datasets & tooling: HF datasets, ROS 2, MoveIt 2, Nav2, Isaac Sim lower the barrier to capability.
- UX expectations: Expressivity is table stakes; we design character via “QuirkPolicy” BT nodes.

### Architecture (high‑level)
- OS/Orchestration: Ubuntu 22.04 + ROS 2 (Iron/Jazzy)
- Compute: Jetson Thor (pref) or AGX Orin
- Perception: OAK‑D stereo + depth, 2D LiDAR, IMU, mic array
- Navigation: Isaac Perceptor/cuVSLAM + Nav2
- Manipulation: MoveIt 2 + Isaac Manipulator; xArm‑class 6/7‑DOF + soft gripper
- Simulation: Isaac Sim / Omniverse for data + rehearsal
- LLM/VLM: On‑device distilled controller with optional edge/cloud assist
- Memory: Lightweight episodic store (SQLite + Faiss)
- Character: FaceRig topics + QuirkPolicy BehaviorTree.CPP nodes

## Folder layout
- mechanical/: CAD and mechanical assemblies
- electronics/: Power, MCU, and wiring artifacts
- firmware/: Embedded code for MCUs
- ros_ws/: ROS 2 colcon workspace with J5 packages
- sim/: Simulation assets (Isaac Sim)
- data/: Dataset pointers/manifests
- docs/: Project documentation

## Quick start

### Option A: Windows (PowerShell)
- Install ROS 2 (Humble/Iron) for Windows and Visual Studio Build Tools.
- Ensure a ROS 2 Developer PowerShell is used (colcon/ros2 on PATH).
- Build: cd ros_ws; colcon build --symlink-install
- Source: .\\install\\local_setup.ps1
- Launch: ros2 launch j5_bringup bringup.launch.py

### Option B: Dev Container / Codespaces (CPU)
- Open in VS Code and “Reopen in Container”, or use GitHub Codespaces.
- Build: cd ros_ws; colcon build --symlink-install
- Source (bash): source install/setup.bash
- Launch: ros2 launch j5_bringup bringup.launch.py

### Option C: Local Linux (GPU)
- Install NVIDIA drivers + CUDA, Docker + NVIDIA Container Toolkit (optional).
- Build: cd ros_ws; colcon build --symlink-install
- Source (bash): source install/setup.bash
- Launch: ros2 launch j5_bringup bringup.launch.py

## Simulation
- Install NVIDIA Isaac Sim and open scenes under sim/isaac/ (placeholder assets).
- Use for rehearsal, data generation, and CI sanity checks.

## Build ROS workspace
- Windows: Open a ROS 2 Developer PowerShell, cd ros_ws, run colcon build --symlink-install, then .\\install\\local_setup.ps1.
- Linux: cd ros_ws, colcon build --symlink-install, then source install/setup.bash.

## Roadmap
1) Bring‑up (teleop, estop, bumpers)
2) Mapping & Nav2
3) FaceRig + Voice
4) Manipulation MVP
5) GR00T‑skills pass
6) Public demo etiquette

## Safety
- Read docs/SAFETY.md before operating hardware. Always use an E‑stop and current‑limited bench supply on first power‑up.

## Contributing
- See docs/CONTRIBUTING.md.

## Licensing
- Software: Apache‑2.0 (see LICENSE)
- Third‑party notices: docs/LICENSES.md
