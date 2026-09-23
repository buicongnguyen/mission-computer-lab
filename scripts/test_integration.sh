#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
SITL_WORKSPACE=${SITL_WORKSPACE:-$HOME/work/mission-computer-lab}
set +u
source /opt/ros/humble/setup.bash
source "$SITL_WORKSPACE/ros_ws/install/setup.bash"
set -u
cd "$ROOT"
exec "$SITL_WORKSPACE/venv/bin/python" -m unittest discover -s integration -p 'test_*.py' -v
