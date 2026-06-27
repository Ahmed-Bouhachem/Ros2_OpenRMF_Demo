#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-$HOME/rmf_ws}"
LOG_DIR="${ROS_LOG_DIR:-/tmp/rmf_demo_logs}"
VENV="${VENV:-$WORKSPACE/.venv}"
START_RVIZ=1

if [ "${1:-}" = "--headless" ] || [ "${1:-}" = "--no-rviz" ]; then
  START_RVIZ=0
  shift
fi

cd "$WORKSPACE"

if [ -f "$VENV/bin/activate" ]; then
  # Keep this because this workspace uses a local Python environment.
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

mkdir -p "$LOG_DIR"

pids=()

start_node() {
  local name="$1"
  shift
  echo "Starting $name"
  "$@" >"$LOG_DIR/$name.log" 2>&1 &
  pids+=("$!")
}

cleanup() {
  echo
  echo "Stopping RMF demo..."
  for pid in "${pids[@]}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill "$pid" >/dev/null 2>&1 || true
    fi
  done
  wait >/dev/null 2>&1 || true
}

trap cleanup EXIT INT TERM

start_node schedule ros2 run rmf_traffic_ros2 rmf_traffic_schedule
sleep 1

start_node dispatcher ros2 run rmf_task_ros2 rmf_task_dispatcher
sleep 1

start_node fleet_adapter ros2 run fleet_adapter_template fleet_adapter \
  -c src/fleet_adapter_template/fleet_adapter_template/config.yaml \
  -n src/fleet_adapter_template/fleet_adapter_template/nav_graph.yaml
sleep 4

if [ "$START_RVIZ" -eq 1 ]; then
  start_node rviz ros2 launch fleet_adapter_template two_cars_rviz.launch.py
else
  start_node markers ros2 run fleet_adapter_template simple_car_markers \
    --ros-args \
    -p nav_graph_file:=src/fleet_adapter_template/fleet_adapter_template/nav_graph.yaml
fi

echo
echo "RMF demo is running."
echo "Logs: $LOG_DIR"
if [ "$START_RVIZ" -eq 0 ]; then
  echo "RViz is disabled for this run; AGV markers are still being published."
fi
echo
echo "In another terminal, run:"
echo "  scripts/rmf_task.sh showcase"
echo "  scripts/rmf_task.sh status"
echo
echo "Press Ctrl-C here to stop the demo."

while true; do
  sleep 1
done
