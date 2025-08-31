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

## License
See [LICENSE](LICENSE).
