# Agent Instructions

- Run `pre-commit run --files <paths>` for changed files.
- Execute `make test` after code changes and capture output.
- `make lint` is available for full linting.
- Ensure the `ros2` CLI is on PATH; if not, source `/opt/ros/iron/setup.bash` and `~/alive/j5/ros_ws/install/setup.bash`.
- If packages are missing, inspect:
  ```bash
  echo $AMENT_PREFIX_PATH
  echo $CMAKE_PREFIX_PATH
  echo $COLCON_CURRENT_PREFIX
  ```
