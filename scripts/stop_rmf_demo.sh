#!/usr/bin/env bash
set -euo pipefail

patterns=(
  "scripts/start_rmf_demo.sh"
  "rmf_task_ros2/rmf_task_dispatcher"
  "rmf_traffic_ros2/rmf_traffic_schedule"
  "fleet_adapter_template/lib/fleet_adapter_template/fleet_adapter"
  "ros2 launch fleet_adapter_template two_cars_rviz.launch.py"
  "fleet_adapter_template/lib/fleet_adapter_template/simple_car_markers"
  "/opt/ros/jazzy/lib/rviz2/rviz2 -d .*two_cars.rviz"
  "fleet_adapter_template/lib/fleet_adapter_template/simple_task_client menu"
)

for pattern in "${patterns[@]}"; do
  pkill -f "$pattern" >/dev/null 2>&1 || true
done

sleep 1
ros2 daemon stop >/dev/null 2>&1 || true

echo "Stopped RMF demo processes and refreshed the ROS 2 daemon."
