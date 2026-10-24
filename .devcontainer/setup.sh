#!/usr/bin/env bash
set -euo pipefail

# Idempotent post-create setup for devcontainer
# - Ensure rosdep is initialized and updated
# - Optionally install dependencies for /workspaces/j5/ros_ws if present

export DEBIAN_FRONTEND=noninteractive

if command -v rosdep >/dev/null 2>&1; then
  if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
  fi
  rosdep update
fi

# Source ROS env in interactive shells
if ! grep -q "/opt/ros/${ROS_DISTRO}/setup.bash" "$HOME/.bashrc" 2>/dev/null; then
  echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> "$HOME/.bashrc"
fi

# Workspace deps
if [ -d "/workspaces/j5/ros_ws/src" ]; then
  set +u
  . /opt/ros/${ROS_DISTRO}/setup.bash
  set -u
  rosdep install --rosdistro "$ROS_DISTRO" --from-paths /workspaces/j5/ros_ws/src --ignore-src -r -y
fi

echo "Devcontainer post-create setup complete."
