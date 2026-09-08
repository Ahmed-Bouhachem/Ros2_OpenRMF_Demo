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

"""Check scan geometry, occlusion, blind zones, and sensor orientation."""

import math

from fleet_adapter_template.lidar_geometry import (
    box_segments, cast_ray, scan_ranges,
)

import pytest


def test_nearest_surface_occludes_far_wall():
    """A nearby car hides a wall regardless of segment ordering."""
    near = box_segments(2.0, 0.0, 1.0, 1.0)
    far = box_segments(5.0, 0.0, 1.0, 1.0)
    assert cast_ray(0, 0, 1, 0, far + near, 0.08, 8) == pytest.approx(1.5)


def test_no_return_and_out_of_range():
    """Surfaces behind the sensor or beyond its range produce no return."""
    wall = box_segments(5.0, 0.0, 1.0, 1.0)
    assert math.isinf(cast_ray(0, 0, -1, 0, wall, 0.08, 8))
    assert math.isinf(cast_ray(0, 0, 1, 0, wall, 0.08, 4))


def test_blind_zone_does_not_see_through_obstacle():
    """A hit inside the blind zone invalidates the ray."""
    segments = [((0.04, -1), (0.04, 1)), ((2, -1), (2, 1))]
    assert math.isnan(cast_ray(0, 0, 1, 0, segments, 0.08, 8))


def test_rotated_car_footprint():
    """Car heading rotates the reflecting footprint."""
    car = box_segments(3, 0, 2, 1, math.pi / 2)
    assert cast_ray(0, 0, 1, 0, car, 0.08, 8) == pytest.approx(2.5)


def test_scan_tracks_sensor_position_and_heading():
    """Scan angles are relative to the sensor's forward axis."""
    room = box_segments(0, 0, 10, 6)
    ranges = scan_ranges(1, 0, 0, room, 4, 0.08, 8)
    assert ranges == pytest.approx([6, 3, 4, 3])
    rotated = scan_ranges(1, 0, math.pi / 2, room, 4, 0.08, 8)
    assert rotated == pytest.approx([3, 4, 3, 6])


def test_moving_target_changes_return():
    """Moving a target away increases the measured distance."""
    first = box_segments(2, 0, 0.95, 0.62)
    second = box_segments(3, 0, 0.95, 0.62)
    r1 = cast_ray(0, 0, 1, 0, first, 0.08, 8)
    r2 = cast_ray(0, 0, 1, 0, second, 0.08, 8)
    assert r2 - r1 == pytest.approx(1)
