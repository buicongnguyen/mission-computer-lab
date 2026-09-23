#!/usr/bin/env bash
# Run as root inside Ubuntu 22.04 WSL; uses official ROS and Gazebo repositories.
set -euo pipefail
[[ $(id -u) == 0 ]] || { echo 'Run with sudo inside WSL'; exit 1; }
. /etc/os-release
[[ $VERSION_ID == 22.04 ]] || { echo 'This installer targets Ubuntu 22.04'; exit 1; }
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq
apt-get install -y --no-remove --no-install-recommends curl ca-certificates gnupg lsb-release
if ! dpkg-query -W ros2-apt-source >/dev/null 2>&1; then
  ros_source_version=$(curl -fLsS https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest |
    python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')
  curl -fLsS "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_source_version}/ros2-apt-source_${ros_source_version}.jammy_all.deb" -o /tmp/mission-ros2-apt-source.deb
  dpkg -i /tmp/mission-ros2-apt-source.deb
fi
curl -fLsS https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/mission-osrf-archive-keyring.gpg
printf '%s\n' "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/mission-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" > /etc/apt/sources.list.d/mission-gazebo.list
apt-get update -qq
apt-get install -y --no-remove --no-install-recommends \
  ros-humble-ros-base ros-humble-sensor-msgs ros-humble-nav-msgs \
  python3-colcon-common-extensions python3-rosdep python3-empy python3-cairo \
  build-essential ninja-build cmake ccache git python3-dev python3-venv \
  libeigen3-dev libxml2-dev libxml2-utils libssl-dev libasio-dev \
  libtinyxml2-dev protobuf-compiler pkg-config libopencv-dev bc \
  gz-harmonic ros-humble-ros-gzharmonic-bridge
echo 'ROS 2 Humble and Gazebo Harmonic installation complete.'
