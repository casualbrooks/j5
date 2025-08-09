# Devcontainer build for J5
# Select base via build arg VARIANT=cpu|gpu (default cpu)
ARG VARIANT=cpu
ARG ROS_DISTRO=iron

# --- Base stage ---
FROM ubuntu:22.04 AS cpu
FROM nvidia/cuda:12.2.0-runtime-ubuntu22.04 AS gpu

# Final selection
FROM ${VARIANT} AS base

# Re-declare args for use within this stage
ARG ROS_DISTRO=iron
ARG VARIANT=cpu

ENV DEBIAN_FRONTEND=noninteractive

# Enable 'universe' and install core tools first
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common ca-certificates curl gnupg lsb-release sudo locales \
 && add-apt-repository -y universe \
 && apt-get update

# Add ROS 2 apt repository with keyring (Jammy + Iron)
RUN mkdir -p /etc/apt/keyrings \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg \
 && chmod a+r /etc/apt/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Locale, dev tools, and Python, then pip install ros tooling
RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion build-essential git \
    python3-pip python3-venv \
 && python3 -m pip install --no-cache-dir --upgrade pip \
 && python3 -m pip install --no-cache-dir rosdep vcstool colcon-common-extensions \
 && locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
 && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8

# Install ROS desktop
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop \
    && rm -rf /var/lib/apt/lists/*

# Create non-root user (vscode)
ARG USER=vscode
ARG UID=1000
ARG GID=1000
RUN groupadd --gid ${GID} ${USER} && useradd -s /bin/bash --uid ${UID} --gid ${GID} -m ${USER} \
    && echo "${USER} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/90-${USER}
USER ${USER}
WORKDIR /workspaces/j5

# rosdep init/update (allow re-run)
RUN sudo rosdep init 2>/dev/null || true && rosdep update

# Shell setup
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USER}/.bashrc \
 && echo "export ROS_DISTRO=${ROS_DISTRO}" >> /home/${USER}/.bashrc

# Optional GPU runtime note
LABEL org.opencontainers.image.description="J5 devcontainer (ROS 2 ${ROS_DISTRO}) variant=${VARIANT}"

# Final command
CMD ["bash"]
