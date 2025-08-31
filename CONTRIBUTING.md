# Contributing

## Day 1 Setup
1. Install Python 3 and ROS 2 Iron.
2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   pre-commit install
   ```
3. Run the bringup:
   ```bash
   ros2 launch j5_bringup bringup.launch.py
   ```

## Development Workflow
- Use `make lint` before committing.
- Run tests with `make test`.
- Submit changes via pull request.
