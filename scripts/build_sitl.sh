#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
SITL_WORKSPACE=${SITL_WORKSPACE:-$HOME/work/mission-computer-lab}
mkdir -p "$SITL_WORKSPACE" "$SITL_WORKSPACE/logs" "$SITL_WORKSPACE/ros_ws/src"
ln -sfn "$ROOT/ros2/mission_interfaces" "$SITL_WORKSPACE/ros_ws/src/mission_interfaces"
cd "$SITL_WORKSPACE"
if [[ ! -d PX4-Autopilot/.git ]]; then
  git clone --depth 1 --branch v1.16.0 https://github.com/PX4/PX4-Autopilot.git
fi
git -C PX4-Autopilot submodule update --init --recursive --depth 1
# PX4's version generator enumerates NuttX tags even for SITL builds.
git -C PX4-Autopilot/platforms/nuttx/NuttX/nuttx fetch --depth 1 origin tag nuttx-12.12.0
if [[ ! -d Micro-XRCE-DDS-Agent/.git ]]; then
  git clone --depth 1 --branch v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
fi
if [[ ! -d ros_ws/src/px4_msgs/.git ]]; then
  git clone --depth 1 --branch release/1.16 https://github.com/PX4/px4_msgs.git ros_ws/src/px4_msgs
  git -C ros_ws/src/px4_msgs fetch --depth 1 origin 392e831c1f659429ca83902e66820d7094591410
  git -C ros_ws/src/px4_msgs checkout --detach 392e831c1f659429ca83902e66820d7094591410
fi
[[ $(git -C PX4-Autopilot rev-parse HEAD) == 6ea3539157ca358c70a515878b77077af7d4611d ]] || { echo 'Unexpected PX4 revision; use a fresh SITL_WORKSPACE.'; exit 1; }
[[ $(git -C Micro-XRCE-DDS-Agent rev-parse HEAD) == 73622810d984349b80bbac0ef55fc0b694d62222 ]] || { echo 'Unexpected Agent revision; use a fresh SITL_WORKSPACE.'; exit 1; }
[[ $(git -C ros_ws/src/px4_msgs rev-parse HEAD) == 392e831c1f659429ca83902e66820d7094591410 ]] || { echo 'Unexpected px4_msgs revision; use a fresh SITL_WORKSPACE.'; exit 1; }
python3 -m venv --system-site-packages "$SITL_WORKSPACE/venv"
"$SITL_WORKSPACE/venv/bin/python" -m pip install -r "$ROOT/requirements-sitl.lock.txt"
export PATH="$SITL_WORKSPACE/venv/bin:/usr/bin:$PATH"
cmake -S Micro-XRCE-DDS-Agent -B Micro-XRCE-DDS-Agent/build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$SITL_WORKSPACE/agent-install" \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build Micro-XRCE-DDS-Agent/build --parallel 3
cmake --install Micro-XRCE-DDS-Agent/build
# ROS setup files reference unset variables, so relax nounset while sourcing.
set +u
source /opt/ros/humble/setup.bash
set -u
cd "$SITL_WORKSPACE/ros_ws"
colcon build --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
cd "$SITL_WORKSPACE/PX4-Autopilot"
make px4_sitl_default -j3
git rev-parse HEAD > "$SITL_WORKSPACE/logs/px4-commit.txt"
git -C "$SITL_WORKSPACE/ros_ws/src/px4_msgs" rev-parse HEAD > "$SITL_WORKSPACE/logs/px4-msgs-commit.txt"
git -C "$SITL_WORKSPACE/Micro-XRCE-DDS-Agent" rev-parse HEAD > "$SITL_WORKSPACE/logs/agent-commit.txt"
echo 'SITL dependencies built.'
