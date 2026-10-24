# Contributing

## Day 1 Setup
1. Install Python 3 and ROS 2 Kilted, then source the environments:
   ```bash
   source /opt/ros/kilted/setup.bash
   rosdep install --rosdistro kilted --from-paths ros_ws/src --ignore-src -r -y
   make build
   source ros_ws/install/local_setup.bash
   ```
   Verify with `printenv ROS_DISTRO` and `ros2 --help`.
2. Install dependencies:
   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   pip install -r requirements.txt
   pre-commit install
   ```
3. Run the bringup:
   ```bash
   ros2 launch j5_bringup bringup.launch.py
   ```
   If packages are not discovered, ensure you're on Ubuntu 24.04 and check:
   ```bash
   echo $AMENT_PREFIX_PATH
   echo $CMAKE_PREFIX_PATH
   echo $COLCON_CURRENT_PREFIX
   ```

## Development Workflow
- Use `make lint` before committing.
- Run tests with `make test`.
- Submit changes via pull request.
