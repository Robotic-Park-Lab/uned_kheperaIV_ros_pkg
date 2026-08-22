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
Shared distance-formation marker builder.

Extracted from two near-identical copies of this exact block: the
``gtpose_callback`` bodies of ``Agent`` in ``distance_based_formation_control.py``
and in ``gazebo_driver.py``. Both built a colour-coded line ``Marker``
between a robot's own ground-truth position and a neighbour's position,
red/orange/green depending on how far the measured distance is from the
formation's target distance for that pair. The math and thresholds (0.05 m
red, 0.025 m orange, else green) were identical in both copies -- only the
attribute names differed (``self.distance``/``self.groundtruth`` vs
``self.d``/``self.gt_pose``).

Pure function, no ROS node/rclpy dependency, so it can be unit tested
without spinning up a node.
"""

from math import sqrt

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


def build_distance_marker(own_position, neighbor_position, target_distance, stamp):
    """
    Build the colour-coded distance line Marker between two agents.

    own_position/neighbor_position are geometry_msgs/Point-like (i.e. with
    .x/.y/.z). target_distance is the distance the formation wants between
    them. stamp is the ROS time to stamp the marker with (e.g.
    node.get_clock().now().to_msg()).
    """
    p0 = Point()
    p0.x = own_position.x
    p0.y = own_position.y
    p0.z = own_position.z

    p1 = Point()
    p1.x = neighbor_position.x
    p1.y = neighbor_position.y
    p1.z = neighbor_position.z

    distance = sqrt(pow(p0.x - p1.x, 2) + pow(p0.y - p1.y, 2) + pow(p0.z - p1.z, 2))

    line = Marker()
    line.header.frame_id = 'map'
    line.header.stamp = stamp
    line.id = 1
    line.type = 5
    line.action = 0
    line.scale.x = 0.01
    line.scale.y = 0.01
    line.scale.z = 0.01

    error = abs(distance - target_distance)
    if error > 0.05:
        line.color.r = 1.0
    elif error > 0.025:
        line.color.r = 1.0
        line.color.g = 0.5
    else:
        line.color.g = 1.0
    line.color.a = 1.0
    line.points.append(p1)
    line.points.append(p0)

    return line
