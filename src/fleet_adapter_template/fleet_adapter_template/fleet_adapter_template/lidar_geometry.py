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

"""Planar ray casting for the teaching demo (no physics engine required)."""

import math


def box_segments(x, y, length, width, yaw=0.0):
    """Return the four edges of a box, optionally rotated around its center."""
    c, s = math.cos(yaw), math.sin(yaw)
    corners = [
        (x + c * dx - s * dy, y + s * dx + c * dy)
        for dx, dy in [(-length / 2, -width / 2),
                       (length / 2, -width / 2),
                       (length / 2, width / 2),
                       (-length / 2, width / 2)]
    ]
    return list(zip(corners, corners[1:] + corners[:1]))


def cast_ray(x, y, dx, dy, segments, range_min, range_max):
    """Return the nearest hit along a unit ray, or infinity for no return."""
    nearest = math.inf
    for (ax, ay), (bx, by) in segments:
        sx, sy = bx - ax, by - ay
        denominator = dx * sy - dy * sx
        if abs(denominator) < 1e-10:
            continue
        qx, qy = ax - x, ay - y
        distance = (qx * sy - qy * sx) / denominator
        fraction = (qx * dy - qy * dx) / denominator
        if 0.0 <= distance <= range_max and 0.0 <= fraction <= 1.0:
            nearest = min(nearest, distance)
    # A surface inside the blind zone still occludes surfaces behind it.
    return nearest if nearest >= range_min else math.nan


def scan_ranges(x, y, yaw, segments, samples, range_min, range_max):
    """Sample a full revolution without duplicating the first ray."""
    return [
        cast_ray(x, y, math.cos(yaw - math.pi + i * math.tau / samples),
                 math.sin(yaw - math.pi + i * math.tau / samples),
                 segments, range_min, range_max)
        for i in range(samples)
    ]
