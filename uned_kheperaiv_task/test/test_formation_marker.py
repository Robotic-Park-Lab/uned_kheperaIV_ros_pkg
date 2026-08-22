# Copyright 2026 Robotic Park Lab
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Robotic Park Lab nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Tests for formation_marker.build_distance_marker.

Extracted from two near-identical Agent.gtpose_callback copies in
distance_based_formation_control.py and gazebo_driver.py. Verifies the
extraction reproduces the original color-threshold logic exactly (0.05 m
red, 0.025 m orange, else green) and the line geometry, without needing to
spin up either ROS node.
"""

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point

from uned_kheperaiv_task.formation_marker import build_distance_marker


def _point(x, y, z):
    p = Point()
    p.x = x
    p.y = y
    p.z = z
    return p


def test_marker_is_red_when_error_exceeds_5cm():
    own = _point(0.0, 0.0, 0.0)
    neighbor = _point(1.2, 0.0, 0.0)  # actual distance 1.2, target 1.0 -> error 0.2
    marker = build_distance_marker(own, neighbor, 1.0, stamp=Time())
    assert marker.color.r == 1.0
    assert marker.color.g == 0.0


def test_marker_is_orange_when_error_between_2_5cm_and_5cm():
    own = _point(0.0, 0.0, 0.0)
    neighbor = _point(1.03, 0.0, 0.0)  # error 0.03, between 0.025 and 0.05
    marker = build_distance_marker(own, neighbor, 1.0, stamp=Time())
    assert marker.color.r == 1.0
    assert marker.color.g == 0.5


def test_marker_is_green_when_within_2_5cm():
    own = _point(0.0, 0.0, 0.0)
    neighbor = _point(1.01, 0.0, 0.0)  # error 0.01
    marker = build_distance_marker(own, neighbor, 1.0, stamp=Time())
    assert marker.color.r == 0.0
    assert marker.color.g == 1.0


def test_marker_geometry_matches_the_two_points():
    own = _point(1.0, 2.0, 3.0)
    neighbor = _point(4.0, 5.0, 6.0)
    marker = build_distance_marker(own, neighbor, 1.0, stamp=Time())
    # points.append(p1) then points.append(p0), same order as both originals
    assert marker.points[0].x == 4.0 and marker.points[0].y == 5.0
    assert marker.points[1].x == 1.0 and marker.points[1].y == 2.0
    assert marker.type == 5
    assert marker.action == 0
    assert marker.scale.x == 0.01
