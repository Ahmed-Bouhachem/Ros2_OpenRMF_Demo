#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-$HOME/rmf_ws}"
LOG_DIR="${ROS_LOG_DIR:-/tmp/rmf_demo_logs}"
VENV="${VENV:-$WORKSPACE/.venv}"

cd "$WORKSPACE"

if [ -f "$VENV/bin/activate" ]; then
  set +u
  source "$VENV/bin/activate"
  set -u
else
  echo "Warning: virtual environment not found at $VENV"
fi

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u
export ROS_LOG_DIR="$LOG_DIR"

ros2 run fleet_adapter_template simple_task_client menu "$@"
