#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
SITL_WORKSPACE=${SITL_WORKSPACE:-$HOME/work/mission-computer-lab}
set +u
source /opt/ros/humble/setup.bash
source "$SITL_WORKSPACE/ros_ws/install/setup.bash"
set -u
export PATH="$SITL_WORKSPACE/venv/bin:/usr/bin:$PATH"
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0
# XRCE Agent's PX4-created participant does not inherit ROS_LOCALHOST_ONLY.
# Use a dedicated DDS domain; do not connect this lab to flight hardware.
unset FASTRTPS_DEFAULT_PROFILES_FILE
exec "$SITL_WORKSPACE/venv/bin/python" "$ROOT/integration/run_sitl.py" --workspace "$SITL_WORKSPACE" "$@"
