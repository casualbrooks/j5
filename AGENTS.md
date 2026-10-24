# Agent Instructions

- Run `pre-commit run --files <paths>` for changed files.
- Execute `make test` after code changes and capture output.
- `make lint` is available for full linting.
- Ensure the `ros2` CLI is on PATH; if not, source `/opt/ros/kilted/setup.bash` and `ros_ws/install/local_setup.bash`.
- If packages are missing, inspect:
  ```bash
  echo $AMENT_PREFIX_PATH
  echo $CMAKE_PREFIX_PATH
  echo $COLCON_CURRENT_PREFIX
  ```
