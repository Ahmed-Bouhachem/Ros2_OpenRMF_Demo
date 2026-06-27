# Copyright 2026 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from collections import deque
import json
import math

from geometry_msgs.msg import Point, TransformStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rmf_fleet_msgs.msg import FleetState
from rmf_task_msgs.msg import DispatchStates
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
import yaml


def quaternion_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class SimpleAgvMarkers(Node):
    def __init__(self):
        super().__init__('simple_agv_markers')
        self.marker_pub = self.create_publisher(
            MarkerArray, '/simple_agvs/markers', 10
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.robots = {}
        self.robot_marker_bases = {}
        self.clear_markers_pending = True
        self.task_intents = {}
        self.dispatch_states = None
        self.nav_graph_file = self.declare_parameter(
            'nav_graph_file',
            '',
        ).get_parameter_value().string_value
        (
            self.map_vertices,
            self.map_lanes,
            self.route_lanes,
        ) = self.load_nav_graph(self.nav_graph_file)
        self.waypoint_indices = {
            waypoint['name']: waypoint['index']
            for waypoint in self.map_vertices
        }
        self.agv_colors = [
            (0.92, 0.72, 0.16, 1.0),
            (0.0, 0.45, 1.0, 1.0),
            (0.16, 0.78, 0.38, 1.0),
            (1.0, 0.28, 0.12, 1.0),
        ]
        self.create_subscription(
            FleetState,
            '/fleet_states',
            self.handle_fleet_state,
            10,
        )
        self.create_subscription(
            String,
            '/simple_task_client/task_intents',
            self.handle_task_intent,
            10,
        )
        self.create_subscription(
            DispatchStates,
            '/dispatch_states',
            self.handle_dispatch_states,
            10,
        )
        self.create_timer(0.1, self.publish_agvs)

        self.get_logger().info(
            'Visualizing RMF robots from /fleet_states on '
            '/simple_agvs/markers'
        )
        if self.map_vertices:
            self.get_logger().info(
                f'Loaded {len(self.map_vertices)} waypoints and '
                f'{len(self.map_lanes)} lanes from {self.nav_graph_file}'
            )
        else:
            self.get_logger().warn(
                'No nav graph loaded; only robot markers will be shown'
            )

    def load_nav_graph(self, nav_graph_file):
        if not nav_graph_file:
            return [], [], []

        try:
            with open(nav_graph_file, 'r', encoding='utf-8') as graph_file:
                graph = yaml.safe_load(graph_file) or {}
        except OSError as err:
            self.get_logger().warn(
                f'Could not read nav graph [{nav_graph_file}]: {err}'
            )
            return [], [], []

        levels = graph.get('levels', {})
        if not levels:
            return [], [], []

        level_name = next(iter(levels))
        level = levels[level_name]
        vertices = []
        for index, vertex in enumerate(level.get('vertices', [])):
            params = vertex[2] if len(vertex) > 2 else {}
            vertices.append({
                'index': index,
                'x': float(vertex[0]),
                'y': float(vertex[1]),
                'name': params.get('name', f'waypoint_{index}'),
                'params': params,
            })

        lanes = []
        route_lanes = []
        seen_lane_segments = set()
        for lane in level.get('lanes', []):
            start_index = int(lane[0])
            end_index = int(lane[1])
            if start_index < len(vertices) and end_index < len(vertices):
                route_lanes.append((start_index, end_index))
                lane_segment = tuple(sorted((start_index, end_index)))
                if lane_segment not in seen_lane_segments:
                    lanes.append((start_index, end_index))
                    seen_lane_segments.add(lane_segment)

        return vertices, lanes, route_lanes

    def handle_task_intent(self, msg):
        try:
            intent = json.loads(msg.data)
        except json.JSONDecodeError as err:
            self.get_logger().warn(f'Ignoring bad task intent: {err}')
            return

        rmf_task_id = intent.get('rmf_task_id')
        if not rmf_task_id:
            return

        self.task_intents[rmf_task_id] = intent

    def handle_dispatch_states(self, msg):
        self.dispatch_states = msg

    def handle_fleet_state(self, msg):
        for robot in msg.robots:
            self.robots[robot.name] = {
                'name': robot.name,
                'x': robot.location.x,
                'y': robot.location.y,
                'yaw': robot.location.yaw,
                'level_name': robot.location.level_name,
                'battery_percent': robot.battery_percent,
                'task_id': robot.task_id,
                'mode': robot.mode.mode,
                'path': [
                    {'x': location.x, 'y': location.y}
                    for location in robot.path
                ],
                'color': self.color_for_robot(robot.name),
            }

    def publish_agvs(self):
        marker_array = MarkerArray()
        if self.clear_markers_pending:
            marker_array.markers.append(self.make_clear_marker())
            self.clear_markers_pending = False

        marker_array.markers.extend(self.make_map_markers())
        marker_array.markers.extend(self.make_decision_panel_markers())
        agvs = [self.robots[name] for name in sorted(self.robots)]

        for agv in agvs:
            x = agv['x']
            y = agv['y']
            yaw = agv['yaw']

            self.publish_tf(agv['name'], x, y, yaw)
            marker_array.markers.extend(
                self.make_route_markers(
                    self.marker_base_for_robot(agv['name']) + 20,
                    agv,
                )
            )
            marker_array.markers.extend(
                self.make_agv_markers(
                    self.marker_base_for_robot(agv['name']),
                    agv,
                    x,
                    y,
                    yaw,
                )
            )

        self.marker_pub.publish(marker_array)

    def marker_base_for_robot(self, robot_name):
        if robot_name not in self.robot_marker_bases:
            self.robot_marker_bases[robot_name] = (
                3000 + len(self.robot_marker_bases) * 100
            )
        return self.robot_marker_bases[robot_name]

    def make_clear_marker(self):
        marker = Marker()
        marker.action = Marker.DELETEALL
        return marker

    def make_map_markers(self):
        markers = []

        lane_marker = self.make_lane_marker(0)
        if lane_marker is not None:
            markers.append(lane_marker)

        markers.extend(self.make_conflict_zone_markers())

        for waypoint in self.map_vertices:
            markers.append(self.make_waypoint_marker(waypoint))
            markers.append(self.make_waypoint_label(waypoint))

        return markers

    def make_lane_marker(self, marker_id):
        if not self.map_lanes:
            return None

        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'rmf_lanes'
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.045
        self.set_color(marker, (0.30, 0.72, 0.95, 0.85))

        for start_index, end_index in self.map_lanes:
            start = self.map_vertices[start_index]
            end = self.map_vertices[end_index]
            marker.points.append(Point(x=start['x'], y=start['y'], z=0.03))
            marker.points.append(Point(x=end['x'], y=end['y'], z=0.03))

        return marker

    def make_waypoint_marker(self, waypoint):
        marker_id = 1000 + waypoint['index']
        marker = self.make_base_marker(
            marker_id,
            'rmf_waypoints',
            Marker.CYLINDER,
            waypoint['x'],
            waypoint['y'],
            0.04,
            0.0,
        )
        marker.scale.x = 0.26
        marker.scale.y = 0.26
        marker.scale.z = 0.08
        self.set_color(marker, self.color_for_waypoint(waypoint))
        return marker

    def make_waypoint_label(self, waypoint):
        marker = self.make_label(
            2000 + waypoint['index'],
            'rmf_waypoint_labels',
            waypoint['x'],
            waypoint['y'],
            0.30,
            waypoint['name'],
        )
        marker.scale.z = 0.14
        self.set_color(marker, (0.92, 0.94, 0.96, 1.0))
        return marker

    def make_conflict_zone_markers(self):
        if 'waiting_area' not in self.waypoint_indices:
            return []
        if 'intersection' not in self.waypoint_indices:
            return []

        waiting_area = self.map_vertices[self.waypoint_indices['waiting_area']]
        intersection = self.map_vertices[
            self.waypoint_indices['intersection']
        ]
        points = [
            (waiting_area['x'], waiting_area['y']),
            (intersection['x'], intersection['y']),
        ]

        zone_line = self.make_line_strip(
            500,
            'rmf_conflict_zone',
            points,
            (1.0, 0.72, 0.12, 0.92),
            0.12,
            0.10,
        )
        zone_center = self.make_base_marker(
            501,
            'rmf_conflict_zone',
            Marker.CYLINDER,
            intersection['x'],
            intersection['y'],
            0.02,
            0.0,
        )
        zone_center.scale.x = 0.85
        zone_center.scale.y = 0.85
        zone_center.scale.z = 0.03
        self.set_color(zone_center, (1.0, 0.72, 0.12, 0.22))
        zone_label = self.make_label(
            502,
            'rmf_conflict_zone',
            (waiting_area['x'] + intersection['x']) / 2.0,
            (waiting_area['y'] + intersection['y']) / 2.0,
            0.42,
            'shared traffic zone',
        )
        zone_label.scale.z = 0.13
        self.set_color(zone_label, (1.0, 0.82, 0.32, 1.0))

        return [zone_line, zone_center, zone_label]

    def make_decision_panel_markers(self):
        board = self.make_base_marker(
            700,
            'rmf_decision_panel',
            Marker.CUBE,
            4.0,
            -2.8,
            0.85,
            0.0,
        )
        board.scale.x = 6.8
        board.scale.y = 0.06
        board.scale.z = 2.6
        self.set_color(board, (0.04, 0.05, 0.06, 0.68))

        text = self.make_label(
            701,
            'rmf_decision_panel',
            4.0,
            -2.87,
            1.55,
            self.decision_panel_text(),
        )
        text.scale.z = 0.11
        self.set_color(text, (0.95, 0.98, 1.0, 1.0))
        return [board, text]

    def decision_panel_text(self):
        lines = [
            'Open-RMF decision board',
            'Task dispatcher chooses fleet + AGV',
        ]

        active = []
        finished = []
        if self.dispatch_states is not None:
            active = list(self.dispatch_states.active)
            finished = list(self.dispatch_states.finished)

        visible_dispatches = active or finished[-2:]
        if not visible_dispatches:
            lines.append('Waiting for a submitted task...')
            lines.append('Run: scripts/rmf_task.sh showcase')
        else:
            lines.append('Latest dispatch decisions:')
            for dispatch in visible_dispatches[-10:]:
                lines.append(self.dispatch_line(dispatch))

        lines.append('Shared traffic zone is highlighted in yellow')
        return '\n'.join(lines)

    def dispatch_line(self, dispatch):
        assignment = dispatch.assignment
        if assignment.is_assigned:
            assignee = (
                f'{assignment.fleet_name}/{assignment.expected_robot_name}'
            )
        else:
            assignee = 'not assigned yet'

        return (
            f'{self.shorten_task_id(dispatch.task_id)} -> {assignee} '
            f'({self.dispatch_status_name(dispatch.status)})'
        )

    @staticmethod
    def shorten_task_id(task_id):
        if len(task_id) <= 24:
            return task_id
        return f'...{task_id[-21:]}'

    @staticmethod
    def dispatch_status_name(status):
        return {
            0: 'uninitialized',
            1: 'queued',
            2: 'selected',
            3: 'dispatched',
            4: 'failed',
            5: 'canceling',
        }.get(status, str(status))

    def make_route_markers(self, marker_id, car):
        markers = []

        path_points = self.points_from_fleet_state_path(car)
        if path_points:
            markers.append(
                self.make_line_strip(
                    marker_id,
                    f"{car['name']}_rmf_path",
                    path_points,
                    (0.72, 0.95, 1.0, 0.95),
                    0.07,
                    0.12,
                )
            )

        task_points = self.points_from_task_intent(car)
        if task_points:
            markers.append(
                self.make_line_strip(
                    marker_id + 1,
                    f"{car['name']}_task_route",
                    task_points,
                    (1.0, 1.0, 1.0, 0.72),
                    0.04,
                    0.12,
                )
            )
            markers.append(
                self.make_destination_marker(
                    marker_id + 2,
                    f"{car['name']}_task_goal",
                    task_points[-1],
                    car['color'],
                )
            )

        return markers

    def points_from_fleet_state_path(self, car):
        if not car['path']:
            return []

        points = [(car['x'], car['y'])]
        points.extend((pose['x'], pose['y']) for pose in car['path'])
        return points

    def points_from_task_intent(self, car):
        task_id = car['task_id']
        if not task_id:
            return []

        intent = self.task_intents.get(task_id)
        if not intent:
            return []

        start_name = intent.get('start_name')
        finish_name = intent.get('finish_name')
        if start_name not in self.waypoint_indices:
            return []
        if finish_name not in self.waypoint_indices:
            return []

        start_index = self.waypoint_indices[start_name]
        finish_index = self.waypoint_indices[finish_name]
        indices = self.shortest_path(start_index, finish_index)
        if not indices:
            return []

        return [
            (self.map_vertices[index]['x'], self.map_vertices[index]['y'])
            for index in indices
        ]

    def shortest_path(self, start_index, finish_index):
        adjacency = {}
        for start, finish in self.route_lanes:
            adjacency.setdefault(start, []).append(finish)

        queue = deque([(start_index, [start_index])])
        visited = {start_index}
        while queue:
            current, path = queue.popleft()
            if current == finish_index:
                return path

            for neighbor in adjacency.get(current, []):
                if neighbor in visited:
                    continue
                visited.add(neighbor)
                queue.append((neighbor, [*path, neighbor]))

        return []

    def make_line_strip(self, marker_id, ns, points, color, width, z):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = width
        marker.lifetime.sec = 1
        self.set_color(marker, color)
        for x, y in points:
            marker.points.append(Point(x=x, y=y, z=z))
        return marker

    def make_destination_marker(self, marker_id, ns, point, color):
        marker = self.make_base_marker(
            marker_id,
            ns,
            Marker.SPHERE,
            point[0],
            point[1],
            0.34,
            0.0,
        )
        marker.scale.x = 0.34
        marker.scale.y = 0.34
        marker.scale.z = 0.12
        marker.lifetime.sec = 1
        self.set_color(marker, color)
        return marker

    @staticmethod
    def color_for_waypoint(waypoint):
        name = waypoint['name']
        params = waypoint['params']
        if params.get('is_charger'):
            return (0.16, 0.78, 0.38, 1.0)
        if name.startswith('pickup') or name == 'receiving':
            return (0.0, 0.45, 1.0, 1.0)
        if name.startswith('dropoff') or name in ('shipping', 'packing'):
            return (1.0, 0.55, 0.18, 1.0)
        if name == 'intersection':
            return (0.85, 0.45, 1.0, 1.0)
        if name == 'waiting_area':
            return (0.95, 0.76, 0.18, 1.0)
        if name in ('inspection', 'quality_check', 'maintenance'):
            return (0.85, 0.45, 1.0, 1.0)
        if name in ('storage', 'staging', 'returns'):
            return (0.16, 0.78, 0.38, 1.0)
        if name.startswith('pallet_rack') or name == 'battery_swap':
            return (0.95, 0.76, 0.18, 1.0)
        if name.endswith('_aisle'):
            return (0.62, 0.66, 0.70, 1.0)
        return (0.62, 0.66, 0.70, 1.0)

    def color_for_robot(self, robot_name):
        color_index = sum(ord(char) for char in robot_name)
        return self.agv_colors[color_index % len(self.agv_colors)]

    def publish_tf(self, child_frame, x, y, yaw):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = child_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

    def make_agv_markers(self, marker_id, agv, x, y, yaw):
        color = agv['color']
        base = self.make_box(
            marker_id,
            f"{agv['name']}_base",
            x,
            y,
            0.13,
            yaw,
            (0.82, 0.50, 0.16),
            color,
        )
        deck = self.make_box(
            marker_id + 1,
            f"{agv['name']}_deck",
            x,
            y,
            0.30,
            yaw,
            (0.46, 0.32, 0.14),
            (0.12, 0.14, 0.16, 1.0),
        )
        front_x, front_y = self.offset_pose(x, y, yaw, 0.44, 0.0)
        front_bumper = self.make_box(
            marker_id + 2,
            f"{agv['name']}_front_bumper",
            front_x,
            front_y,
            0.20,
            yaw,
            (0.07, 0.56, 0.13),
            (0.03, 0.03, 0.035, 1.0),
        )
        rear_x, rear_y = self.offset_pose(x, y, yaw, -0.44, 0.0)
        rear_bumper = self.make_box(
            marker_id + 3,
            f"{agv['name']}_rear_bumper",
            rear_x,
            rear_y,
            0.20,
            yaw,
            (0.07, 0.56, 0.13),
            (0.03, 0.03, 0.035, 1.0),
        )
        left_x, left_y = self.offset_pose(x, y, yaw, 0.0, 0.29)
        left_safety_strip = self.make_box(
            marker_id + 4,
            f"{agv['name']}_left_safety_strip",
            left_x,
            left_y,
            0.25,
            yaw,
            (0.58, 0.05, 0.06),
            (1.0, 0.86, 0.10, 1.0),
        )
        right_x, right_y = self.offset_pose(x, y, yaw, 0.0, -0.29)
        right_safety_strip = self.make_box(
            marker_id + 5,
            f"{agv['name']}_right_safety_strip",
            right_x,
            right_y,
            0.25,
            yaw,
            (0.58, 0.05, 0.06),
            (1.0, 0.86, 0.10, 1.0),
        )
        lidar = self.make_cylinder(
            marker_id + 6,
            f"{agv['name']}_lidar",
            x,
            y,
            0.44,
            yaw,
            0.16,
            0.10,
            (0.02, 0.025, 0.03, 1.0),
        )
        beacon = self.make_sphere(
            marker_id + 7,
            f"{agv['name']}_beacon",
            x,
            y,
            0.56,
            0.14,
            (1.0, 0.62, 0.02, 1.0),
        )

        label = self.make_label(
            marker_id + 8,
            agv['name'],
            x,
            y,
            0.88,
            self.status_label_for_agv(agv),
        )
        label.scale.z = 0.13
        markers = [
            base,
            deck,
            front_bumper,
            rear_bumper,
            left_safety_strip,
            right_safety_strip,
            lidar,
            beacon,
            label,
        ]

        if agv['mode'] == 4:
            markers.append(
                self.make_waiting_marker(
                    marker_id + 9,
                    f"{agv['name']}_waiting",
                    x,
                    y,
                )
            )

        return markers

    def make_waiting_marker(self, marker_id, ns, x, y):
        marker = self.make_base_marker(
            marker_id,
            ns,
            Marker.CYLINDER,
            x,
            y,
            0.05,
            0.0,
        )
        marker.scale.x = 1.05
        marker.scale.y = 1.05
        marker.scale.z = 0.03
        marker.lifetime.sec = 1
        self.set_color(marker, (1.0, 0.72, 0.12, 0.42))
        return marker

    def status_label_for_agv(self, agv):
        task_id = agv['task_id'] or '-'
        if len(task_id) > 18:
            task_id = f'...{task_id[-15:]}'
        return (
            f"{agv['name']}\n"
            f"mode: {self.mode_name(agv['mode'])}\n"
            f"task: {task_id}\n"
            f"bat: {agv['battery_percent']:.0f}%"
        )

    @staticmethod
    def mode_name(mode):
        return {
            0: 'idle',
            1: 'charging',
            2: 'moving',
            3: 'paused',
            4: 'waiting',
            5: 'emergency',
            6: 'going_home',
            7: 'docking',
            8: 'adapter_error',
            9: 'cleaning',
        }.get(mode, str(mode))

    def make_box(self, marker_id, ns, x, y, z, yaw, scale, color):
        marker = self.make_base_marker(marker_id, ns, Marker.CUBE, x, y, z, yaw)
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        self.set_color(marker, color)
        return marker

    def make_cylinder(
        self, marker_id, ns, x, y, z, yaw, radius, length, color
    ):
        marker = self.make_base_marker(
            marker_id,
            ns,
            Marker.CYLINDER,
            x,
            y,
            z,
            yaw + math.pi / 2.0,
        )
        marker.scale.x = radius
        marker.scale.y = radius
        marker.scale.z = length
        self.set_color(marker, color)
        return marker

    def make_sphere(self, marker_id, ns, x, y, z, diameter, color):
        marker = self.make_base_marker(
            marker_id,
            ns,
            Marker.SPHERE,
            x,
            y,
            z,
            0.0,
        )
        marker.scale.x = diameter
        marker.scale.y = diameter
        marker.scale.z = diameter
        self.set_color(marker, color)
        return marker

    def make_label(self, marker_id, ns, x, y, z, text):
        marker = self.make_base_marker(marker_id, ns, Marker.TEXT_VIEW_FACING,
                                       x, y, z, 0.0)
        marker.text = text
        marker.scale.z = 0.18
        self.set_color(marker, (1.0, 1.0, 1.0, 1.0))
        return marker

    def make_base_marker(self, marker_id, ns, marker_type, x, y, z, yaw):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        return marker

    @staticmethod
    def set_color(marker, color):
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

    @staticmethod
    def offset_pose(x, y, yaw, forward, left):
        return (
            x + forward * math.cos(yaw) - left * math.sin(yaw),
            y + forward * math.sin(yaw) + left * math.cos(yaw),
        )


def main(args=None):
    rclpy.init(args=args)
    node = SimpleAgvMarkers()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
