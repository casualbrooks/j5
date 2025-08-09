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

```
ros2 launch j5_bringup bringup.launch.py
```

## Roadmap
1. Hardware bringup
2. Perception + manipulation
3. Simulation integration
4. Safety + HRI polish

## License
See [LICENSE](LICENSE).
