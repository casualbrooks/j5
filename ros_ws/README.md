# J5 ROS 2 workspace

The baseline is ROS 2 Kilted on Ubuntu 24.04 (amd64 and arm64, including
Raspberry Pi 5). Kilted support ends in November 2026; validate a subsequent
supported ROS upgrade separately before long-term deployment.

From the repository root, in a fresh Bash shell:

```bash
source /opt/ros/kilted/setup.bash
rosdep install --rosdistro kilted --from-paths ros_ws/src --ignore-src -r -y
make build
source ros_ws/install/local_setup.bash
make test
ros2 launch j5_bringup bringup.launch.py start_voice:=true
```

For a source-built Kilted installation, substitute
`source ~/ros2_kilted/install/setup.bash` for the `/opt/ros` line. Select one
underlay in a fresh terminal; do not layer Iron, binary Kilted, and source
Kilted installations together. Use `local_setup.bash` for the J5 overlay so
its build-time underlay is not silently reintroduced.

When changing ROS versions, build into new directories or a new worktree:

```bash
cd ros_ws
colcon --log-base log-kilted build --build-base build-kilted --install-base install-kilted
source install-kilted/local_setup.bash
colcon --log-base log-kilted test --build-base build-kilted --install-base install-kilted --return-code-on-test-failure
colcon test-result --test-result-base build-kilted --verbose
```

Check `printenv ROS_DISTRO AMENT_PREFIX_PATH CMAKE_PREFIX_PATH` and
`ros2 pkg prefix rclpy` when diagnosing mixed installations. Do not add
vendored ROS middleware to this workspace or override Fast DDS packages just
to silence a colcon warning; match them to the selected underlay first.

The placeholder voice node is opt-in. The shared baseline does not launch
Race Manager, camera services, or application databases.

If `ros2 launch` is missing, verify `ros2 -h` lists `launch` before sourcing
this overlay. A source-built Kilted underlay needs `ros2launch` and
`launch_ros`; building J5 packages cannot add a missing underlay CLI plugin.

For the Raspberry Pi browser race workflow and durable championship data, see
[`docs/RACE_WEB.md`](../docs/RACE_WEB.md). The existing multi-process dashboard
remains documented in [`apps/racemanager/README.md`](../apps/racemanager/README.md).
