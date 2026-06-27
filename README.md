# ROS 2 Open-RMF Demo

This repository contains a ROS 2 workspace for experimenting with an Open-RMF
fleet adapter demo.

The main package and detailed run instructions live in:

```text
src/fleet_adapter_template/README.md
```

Common workflow:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select fleet_adapter_template
source install/setup.bash
scripts/start_rmf_demo.sh
```
