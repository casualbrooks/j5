
#!/usr/bin/env bash
set -euo pipefail
sudo apt-get update
sudo apt-get install -y curl git python3-colcon-common-extensions python3-rosdep python3-vcstool build-essential cmake
# ROS 2 Iron (CPU) minimal install (for Codespaces)
sudo apt-get install -y ros-iron-desktop
sudo rosdep init || true
rosdep update
echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
