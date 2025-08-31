# Contributing

## Day 1 Setup
1. Install Python 3 and ROS 2 Iron, then source the environments:
   ```bash
   source /opt/ros/iron/setup.bash
   source ~/alive/j5/ros_ws/install/setup.bash
   ```
   Verify with `ros2 --version`.
2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   pre-commit install
   ```
3. Run the bringup:
   ```bash
   ros2 launch j5_bringup bringup.launch.py
   ```
   If packages are not discovered, ensure you're on Ubuntu 22.04 and check:
   ```bash
   echo $AMENT_PREFIX_PATH
   echo $CMAKE_PREFIX_PATH
   echo $COLCON_CURRENT_PREFIX
   ```

## Development Workflow
- Use `make lint` before committing.
- Run tests with `make test`.
- Submit changes via pull request.
