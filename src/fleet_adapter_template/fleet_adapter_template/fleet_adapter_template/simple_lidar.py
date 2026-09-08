# Copyright 2026 Open-RMF demo contributors
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

"""Simulate planar LiDAR from the same poses used by the RViz AGV markers."""

import math
import os

from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import Point, TransformStamped

from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan

from visualization_msgs.msg import Marker, MarkerArray

import yaml

from .lidar_geometry import box_segments, scan_ranges


class SimpleLidar:
    """Publish independent scans and a shared scene for all observed cars."""

    HEIGHT = 0.44
    PERIOD = 0.1

    def __init__(self, node):
        """Load the shared obstacle scene and configure scan publishers."""
        self.node = node
        self.samples = node.declare_parameter('lidar_samples', 360).value
        self.range_max = node.declare_parameter('lidar_range', 8.0).value
        self.range_min = 0.08
        if self.samples < 4 or self.range_max <= self.range_min:
            raise ValueError('LiDAR needs >= 4 samples and range > 0.08 m')
        default_scene = os.path.join(
            get_package_share_directory('fleet_adapter_template'),
            'lidar_scene.yaml',
        )
        scene_file = node.declare_parameter(
            'lidar_scene_file', default_scene,
        ).value
        with open(scene_file, encoding='utf-8') as stream:
            self.obstacles = yaml.safe_load(stream)['obstacles']
        self.segments = []
        for obstacle in self.obstacles:
            if obstacle['height'] >= self.HEIGHT:
                self.segments.extend(box_segments(
                    obstacle['x'], obstacle['y'],
                    obstacle['length'], obstacle['width'],
                ))
        self.publishers = {}
        self.marker_pub = node.create_publisher(
            MarkerArray, '/simple_agvs/lidar_markers', 10,
        )

    @staticmethod
    def marker(namespace, marker_id, marker_type, stamp):
        """Create a map marker that expires when updates stop."""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime.sec = 1
        return marker

    def scene_markers(self, stamp):
        """Render exactly the boxes used for static scan intersections."""
        markers = []
        for index, obstacle in enumerate(self.obstacles):
            marker = self.marker('lidar_obstacles', index, Marker.CUBE, stamp)
            marker.pose.position.x = float(obstacle['x'])
            marker.pose.position.y = float(obstacle['y'])
            marker.pose.position.z = obstacle['height'] / 2
            marker.scale.x = float(obstacle['length'])
            marker.scale.y = float(obstacle['width'])
            marker.scale.z = float(obstacle['height'])
            wall = 'wall' in obstacle['name']
            marker.color.r, marker.color.g, marker.color.b = (
                (0.32, 0.40, 0.48) if wall else (0.65, 0.43, 0.24)
            )
            marker.color.a = 0.32 if wall else 0.85
            markers.append(marker)
        return markers

    def publish(self, agvs, stamp):
        """Publish scans, sensor transforms, and sparse illustrative beams."""
        markers = self.scene_markers(stamp)
        for index, agv in enumerate(agvs):
            name = agv['name']
            if name not in self.publishers:
                self.publishers[name] = self.node.create_publisher(
                    LaserScan, f'/{name}/scan', qos_profile_sensor_data,
                )
                self.node.get_logger().info(f'Simulated LiDAR: /{name}/scan')

            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = name
            transform.child_frame_id = f'{name}/lidar'
            transform.transform.translation.z = self.HEIGHT
            transform.transform.rotation.w = 1.0
            self.node.tf_broadcaster.sendTransform(transform)

            segments = list(self.segments)
            for other in agvs:
                if (other['name'] != name
                        and other['level_name'] == agv['level_name']):
                    # Planar footprint approximation of each other car.
                    segments.extend(box_segments(
                        other['x'], other['y'], 0.95, 0.62, other['yaw'],
                    ))
            scan = LaserScan()
            scan.header.stamp = stamp
            scan.header.frame_id = transform.child_frame_id
            scan.angle_min = -math.pi
            scan.angle_increment = math.tau / self.samples
            scan.angle_max = (
                scan.angle_min + (self.samples - 1) * scan.angle_increment
            )
            scan.range_min = self.range_min
            scan.range_max = self.range_max
            scan.scan_time = self.PERIOD
            # All rays use one pose snapshot, without rolling-scan distortion.
            scan.time_increment = 0.0
            scan.ranges = scan_ranges(
                agv['x'], agv['y'], agv['yaw'], segments,
                self.samples, self.range_min, self.range_max,
            )
            self.publishers[name].publish(scan)

            rays = self.marker(
                f'{name}_lidar_rays', index, Marker.LINE_LIST, stamp,
            )
            rays.scale.x = 0.008
            rays.color.r, rays.color.g, rays.color.b = agv['color'][:3]
            rays.color.a = 0.16
            # Sparse beams keep routes readable; LaserScan shows all hits.
            for ray in range(0, self.samples, 12):
                distance = scan.ranges[ray]
                if not math.isfinite(distance):
                    continue
                angle = (
                    agv['yaw'] + scan.angle_min + ray * scan.angle_increment
                )
                rays.points.extend([
                    Point(x=agv['x'], y=agv['y'], z=self.HEIGHT),
                    Point(x=agv['x'] + distance * math.cos(angle),
                          y=agv['y'] + distance * math.sin(angle),
                          z=self.HEIGHT),
                ])
            markers.append(rays)
        self.marker_pub.publish(MarkerArray(markers=markers))
