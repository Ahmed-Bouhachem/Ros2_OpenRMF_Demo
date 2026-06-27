# fleet_adapter_template

> Note: If you are using Open-RMF binaries from ROS 2 Humble or an older distribution, switch to the [humble](https://github.com/open-rmf/fleet_adapter_template/tree/humble) branch.

The objective of this package is to serve as a reference or template for writing a python based `full_control` RMF fleet adapter.

> Note: The implementation in this package is not the only way to write a `full_control` fleet adapter. It is only one such example that may be helpful for users to quickly integrate their fleets with RMF.

## Step 1: Fill up missing code
Simply fill up certain blocks of code which make API calls to your mobile robotic fleet.
These blocks are highlighted as seen below and are found in `RobotClientAPI.py` and `fleet_adapter.py` respectively.
```
# IMPLEMENT YOUR CODE HERE #
```

The bulk of the work is in populating the `RobotClientAPI.py` file which defines a wrapper for communicating with the fleet of interest.
For example, if your fleet offers a `REST API` with a `GET` method to obtain the position of the robot, then the `RobotAPI::position()` function may be implemented as below

```python
def position(self):
    url = self.prefix + "/data/position" # example endpoint
    try:
        response = requests.get(url)
        response.raise_for_status()
        data = response.json()
        x = data["x"]
        y = data["y"]
        angle = data["angle"]
        return [x, y, angle]
    except HTTPError as http_err:
        print(f"HTTP error: {http_err}")
    except Exception as err:
        print(f"Other error: {err}")
    return None

```

Alternatively, if your robotic fleet offers a websocket port for communication or allows for messages to be exchanged over ROS1/2, then these functions can be implemented using those protocols respectively.

## Step 2: Update config.yaml
The `config.yaml` file contains important parameters for setting up the fleet adapter. There are three broad sections to this file:

1. **rmf_fleet** : containing parameters that describe the robots in this fleet
2. **fleet_manager** : containing configurations to connect to the robot's API in order to retrieve robot status and send commands from RMF
3. **reference_coordinates**: containing two sets of [x,y] coordinates that correspond to the same locations but recorded in RMF (`traffic_editor`) and robot specific coordinates frames respectively. These are required to estimate coordinate transformations from one frame to another. A minimum of 4 matching waypoints is recommended.

> Note: This fleet adapter uses the `nudged` python library to compute transformations from RMF to Robot frame and vice versa. If the user is aware of the `scale`, `rotation` and `translation` values for each transform, they may modify the code in `fleet_adapter.py` to directly create the `nudged` transform objects from these values.

## Step 3: Run the fleet adapter:

Run the command below while passing the paths to the configuration file and navigation graph that this fleet operates on.

The websocket server URI should also be passed as a parameter in this command inorder to publish task statuses to the rest of the RMF entities.

```bash
#minimal required parameters
ros2 run fleet_adapter_template fleet_adapter -c CONFIG_FILE -n NAV_GRAPH

#Usage with the websocket uri
ros2 run fleet_adapter_template fleet_adapter -c CONFIG_FILE -n NAV_GRAPH -s SERVER_URI

#e.g.
ros2 run fleet_adapter_template fleet_adapter -c CONFIG_FILE -n NAV_GRAPH -s ws://localhost:7878
```

## Simple four-AGV visualization demo

This workspace also contains a small learning visualizer that draws RMF robots
as simple AGV-shaped objects in RViz.

The demo world is a tiny virtual warehouse. It is not a SLAM map yet; it is an
RMF navigation graph. RMF uses this graph to understand named places and lanes.

Current waypoints:

- `AGV1_charger`: charger/start point for `AGV1`
- `AGV2_charger`: charger/start point for `AGV2`
- `AGV3_charger`: charger/start point for `AGV3`
- `AGV4_charger`: charger/start point for `AGV4`
- `waiting_area`: shared waiting point before the main lane
- `intersection`: shared crossing point
- `pickup`: first pickup point
- `dropoff`: first dropoff point
- `pickup_B`: second pickup point
- `dropoff_B`: second dropoff point
- `inspection`: extra upper route point
- `storage`: extra upper route point
- `north_aisle`: upper travel aisle
- `south_aisle`: lower travel aisle
- `receiving`: inbound goods area
- `packing`: packing station
- `shipping`: outbound goods area
- `quality_check`: quality inspection station
- `staging`: staging area
- `returns`: returns area
- `pallet_rack_A`: lower rack location
- `pallet_rack_B`: upper rack location
- `maintenance`: maintenance bay
- `battery_swap`: battery swap point

The lanes in `nav_graph.yaml` are written as directed connections. For a
two-way hallway, write both directions, for example `[0, 2, {}]` and
`[2, 0, {}]`. The RViz visualizer only draws that physical lane once.

To inspect the learning map in RMF Traffic Editor:

```bash
cd ~/rmf_ws
traffic-editor src/fleet_adapter_template/fleet_adapter_template/maps/agv_demo_editor/agv_demo.project.yaml
```

That editor project is generated from this demo graph so you can see the
waypoints and lane network. The runtime adapter still uses
`fleet_adapter_template/nav_graph.yaml`.

Build and source the workspace:

```bash
cd ~/rmf_ws
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
colcon build --packages-select fleet_adapter_template
source install/setup.bash
```

Quick start with fewer terminals:

```bash
cd ~/rmf_ws
scripts/stop_rmf_demo.sh
scripts/start_rmf_demo.sh
```

This starts the schedule, dispatcher, fleet adapter, and RViz in one terminal.
It activates `~/rmf_ws/.venv` if it exists, sources ROS Jazzy, and
uses the workspace install.

To start only the RMF backend without RViz:

```bash
cd ~/rmf_ws
scripts/start_rmf_demo.sh --headless
```

In a second terminal, send tasks or read status:

```bash
cd ~/rmf_ws
scripts/rmf_menu.sh
```

The menu lets you choose routes A-H, the two-task traffic conflict demo, the
large-map decision demo, status, a custom loop, list dispatch IDs, or cancel a
task.

You can also run direct commands:

```bash
cd ~/rmf_ws
scripts/rmf_task.sh showcase
scripts/rmf_task.sh all
scripts/rmf_task.sh conflict
scripts/rmf_task.sh both
scripts/rmf_task.sh route_c
scripts/rmf_task.sh route_d
scripts/rmf_task.sh route_e
scripts/rmf_task.sh route_f
scripts/rmf_task.sh route_g
scripts/rmf_task.sh route_h
scripts/rmf_task.sh route_i
scripts/rmf_task.sh route_j
scripts/rmf_task.sh route_k
scripts/rmf_task.sh route_l
scripts/rmf_task.sh status
scripts/rmf_task.sh dispatches
scripts/rmf_task.sh api_cancel patrol.dispatch-0
scripts/rmf_task.sh cancel patrol.dispatch-0
scripts/rmf_task.sh pause_all
scripts/rmf_task.sh resume_all
scripts/rmf_task.sh reset_all
```

Press Ctrl-C in the `scripts/start_rmf_demo.sh` terminal to stop the demo.

Manual startup is still useful when learning each RMF piece separately.

Start the RMF traffic schedule:

```bash
cd ~/rmf_ws
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run rmf_traffic_ros2 rmf_traffic_schedule
```

In another terminal, start the RMF task dispatcher:

```bash
cd ~/rmf_ws
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run rmf_task_ros2 rmf_task_dispatcher
```

In another terminal, start the RMF fleet adapter.

For this fake in-memory robot demo, do not use `-sim` unless Gazebo is also
running and publishing `/clock`. Without an advancing simulation clock, the
adapter may accept a task but appear frozen.

```bash
cd ~/rmf_ws
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run fleet_adapter_template fleet_adapter \
  -c src/fleet_adapter_template/fleet_adapter_template/config.yaml \
  -n src/fleet_adapter_template/fleet_adapter_template/nav_graph.yaml
```

In another terminal:

```bash
cd ~/rmf_ws
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch fleet_adapter_template two_cars_rviz.launch.py
```

The visualizer subscribes to:

- `/fleet_states`: RMF robot names, positions, headings, and battery state

It publishes:

- `/simple_agvs/markers`: the AGV bodies, bumpers, safety strips, lidar
  pucks, beacons, and labels
- RViz markers for the RMF waypoints and lane lines from `nav_graph.yaml`
- A highlighted shared traffic zone between `waiting_area` and `intersection`
- Active task route markers and destination markers for tasks sent by
  `simple_task_client`
- Robot labels with name, mode, task ID, and battery percentage
- `/tf`: the `map -> AGV1`, `map -> AGV2`, `map -> AGV3`, and
  `map -> AGV4` frames

The RViz scene also includes an `Open-RMF decision board`. It shows the latest
dispatch IDs, which fleet/robot RMF selected, and the dispatch status. This is
the easiest view to record when explaining how RMF makes a fleet-level decision.

### Recording the Open-RMF demo

Use this flow when recording a short video:

```bash
cd ~/rmf_ws
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
colcon build --packages-select fleet_adapter_template
source install/setup.bash
scripts/stop_rmf_demo.sh
scripts/start_rmf_demo.sh
```

In a second terminal:

```bash
cd ~/rmf_ws
scripts/rmf_task.sh showcase
```

What to point out in the video:

- The four AGVs start from different charger waypoints.
- `scripts/rmf_task.sh showcase` submits twelve tasks for four AGVs.
- RMF chooses which AGV should receive each dispatch and queues extra work.
- The yellow highlighted lane is the shared traffic zone where RMF coordinates
  robot movement.
- The RViz decision board shows the active dispatch IDs and assignments.

Optional commands while recording:

```bash
scripts/rmf_task.sh pause_all
scripts/rmf_task.sh resume_all
scripts/rmf_task.sh dispatches
scripts/rmf_task.sh status
```

### Submit a loop task

Once the schedule, dispatcher, fleet adapter, and RViz are running, use the
small task client to submit loop tasks with one command word:

```bash
ros2 run fleet_adapter_template simple_task_client route_a
```

This sends `pickup -> dropoff`.

Try the second route:

```bash
ros2 run fleet_adapter_template simple_task_client route_b
```

This sends `pickup_B -> dropoff_B`.

Try the upper/storage routes:

```bash
ros2 run fleet_adapter_template simple_task_client route_c
ros2 run fleet_adapter_template simple_task_client route_d
```

This sends `inspection -> storage` and `storage -> dropoff_B`.

Submit every route so RMF chooses across all four AGVs:

```bash
ros2 run fleet_adapter_template simple_task_client all
```

The full route set is A-L:

- `route_e`: `receiving -> shipping`
- `route_f`: `maintenance -> quality_check`
- `route_g`: `pallet_rack_A -> staging`
- `route_h`: `returns -> packing`
- `route_i`: `battery_swap -> pallet_rack_B`
- `route_j`: `receiving -> returns`
- `route_k`: `pickup_B -> battery_swap`
- `route_l`: `pallet_rack_B -> dropoff`

Submit the original two shared-zone routes:

```bash
ros2 run fleet_adapter_template simple_task_client both
```

Run the traffic conflict lesson:

```bash
ros2 run fleet_adapter_template simple_task_client conflict
```

This submits route A and route B together so two AGVs need the shared
traffic zone. In RViz, watch the highlighted lane between `waiting_area` and
`intersection`. One AGV may pause or wait while RMF coordinates traffic.

Cancel an active task:

```bash
ros2 run fleet_adapter_template simple_task_client dispatches
ros2 run fleet_adapter_template simple_task_client api_cancel patrol.dispatch-0
```

Use `dispatches` to ask the RMF dispatcher for the real active task IDs before
canceling. The dispatcher generates IDs like `patrol.dispatch-0`,
`patrol.dispatch-1`, and so on.

There are now two cancel commands for learning:

- `api_cancel TASK_ID` publishes a modern RMF task API request on
  `/task_api_requests`. Use this first for tasks that have already been handed
  to the fleet adapter.
- `cancel TASK_ID` calls the older dispatcher service `/cancel_task`. This is
  useful to compare behavior, but it may reject tasks that have already been
  handed off to the fleet adapter.

If cancel is rejected, that task is usually already finished, already canceled,
not known by the current dispatcher, or not cancelable by that RMF path.
If `dispatches` or `cancel` says the service is unavailable, restart the demo
with `scripts/start_rmf_demo.sh` because the task dispatcher is not running.

Pause and resume the fake robot API:

```bash
ros2 run fleet_adapter_template simple_task_client pause_all
ros2 run fleet_adapter_template simple_task_client resume_all
ros2 run fleet_adapter_template simple_task_client reset_all
```

These commands call tutorial services on the fleet adapter:

- `/simple_pause_robots`: holds the fake robots in place without forgetting
  their current target.
- `/simple_resume_robots`: releases the hold so the fake robots continue.
- `/simple_reset_robots`: returns the fake robots to their charger poses.

This is not Open-RMF task cancellation. It is a learning tool that lets you see
the difference between "RMF task state" and "robot API motion control".

You can also send a custom loop between any two waypoint names:

```bash
ros2 run fleet_adapter_template simple_task_client loop pickup dropoff
```

Print the latest robot state from `/fleet_states`:

```bash
ros2 run fleet_adapter_template simple_task_client status
```

Command requirements:

- `status` needs the fleet adapter running because it reads `/fleet_states`.
- `route_a` through `route_l`, `both`, `all`, `showcase`, and `loop` need the
  task dispatcher because they call `/submit_task`.
- `dispatches` and `cancel` need the task dispatcher because they call
  `/get_dispatches` and `/cancel_task`.
- `api_cancel` needs the fleet adapter because it publishes to
  `/task_api_requests` and waits for `/task_api_responses`.
- `pause_all` and `resume_all` need the fleet adapter because they call
  `/simple_pause_robots` and `/simple_resume_robots`.
- RViz is only needed if you want to watch the AGVs move.
- The traffic schedule should be running when the fleet adapter starts.

The task client is only a shortcut. Internally, it sends the same kind of RMF
service request as this long command:

```bash
ros2 service call /submit_task rmf_task_msgs/srv/SubmitTask "{requester: tutorial, description: {start_time: {sec: 0, nanosec: 0}, priority: {value: 0}, task_type: {type: 1}, loop: {task_id: loop_test_001, robot_type: '', num_loops: 1, start_name: pickup, finish_name: dropoff}}}"
```

And this is the long form for the second route:

```bash
ros2 service call /submit_task rmf_task_msgs/srv/SubmitTask "{requester: tutorial, description: {start_time: {sec: 0, nanosec: 0}, priority: {value: 0}, task_type: {type: 1}, loop: {task_id: loop_test_002, robot_type: '', num_loops: 1, start_name: pickup_B, finish_name: dropoff_B}}}"
```

Expected result:

- The service response should contain `success=True`.
- The dispatcher will generate its own task ID, often like `patrol.dispatch-0`.
- The fleet adapter should log navigation commands for one robot.
- In RViz, one AGV should move from its charger to `pickup`, then to `dropoff`,
  then usually return to its charger.

Where the command comes from:

```bash
ros2 service list -t | grep submit_task
ros2 interface show rmf_task_msgs/srv/SubmitTask
ros2 interface show rmf_task_msgs/msg/TaskDescription
ros2 interface show rmf_task_msgs/msg/TaskType
ros2 interface show rmf_task_msgs/msg/Loop
```

The important fields are:

- `/submit_task` uses the service type `rmf_task_msgs/srv/SubmitTask`.
- `requester: tutorial` is just a name for whoever is asking RMF for the task.
- `task_type: {type: 1}` means `TYPE_LOOP`.
- `loop.task_id` is the caller's task label.
- `robot_type: ''` leaves robot selection open to RMF.
- `start_name: pickup` and `finish_name: dropoff` must match waypoint names in
  `nav_graph.yaml`.

Open the Gazebo demo:

```bash
ros2 launch fleet_adapter_template two_cars_gazebo.launch.py
```

The Gazebo world is intentionally simple and uses only primitive shapes, so it
is a good first place to learn how objects appear in simulation before adding a
real AGV model.
