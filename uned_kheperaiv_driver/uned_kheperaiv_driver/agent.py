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
Agent used by khepera_driver.py (uned_kheperaiv_webots) for neighbour
tracking in formation control: distance-based, fixed-point, or geometry
(sphere/cone/ellipsoid, via multi_agent_pkg) targets, with RViz marker
publishing.

This is a Khepera-owned copy, previously imported from
uned_crazyflie_driver.crazyflie_agent (a real cross-repo dependency that
made uned_kheperaiv_webots require the Crazyflie repo cloned and built in
the same workspace just for formation control -- confirmed unnecessary and
removed).

Adapted from the Crazyflie version, not copied verbatim: the branches that
sent commands through Crazyflie's `high_level_commander` (`scf.cf.*`) are
gone -- there is no such concept for a Webots-simulated Khepera IV, and
`KheperaWebotsDriver` never sets `self.physical = True` in practice (that
flag is for the physical/socket driver in uned_kheperaiv_driver, a
different class entirely -- see kheperaIV_client_driver.py's own, simpler,
socket-based Agent class, kept separate on purpose: it is not the same
protocol and merging the two would change real hardware-facing behaviour
without a way to verify it here).
"""

from math import sqrt

from geometry_msgs.msg import Pose, Point, PoseStamped
from std_msgs.msg import Float64, String
from visualization_msgs.msg import Marker


class Agent():
    def __init__(self, parent, node, id, x=None, y=None, z=None, d=None,
                 k=None, point=None, vector=None, a=None, b=None, c=None):
        self.id = id
        self.idn = float(len(parent.agent_list))
        self.distance = False
        self.parent = parent
        self.node = node
        self.disconnect = False
        self.last_error = 0.0
        self.last_iae = 0.0
        self.k = 1.0
        self.pose = Pose()

        if not id.find("line") == -1:
            self.distance_bool = True
            self.d = 0
            self.point = point
            self.vector = vector
            self.mod = pow(vector.x, 2) + pow(vector.y, 2) + pow(vector.z, 2)
            self.k = self.k * 4.0
        else:
            if d is None:
                self.x = x
                self.y = y
                self.z = z
                self.node.get_logger().info('Agent: %s' % self.str_())
            else:
                self.d = d
                self.distance = True
                self.node.get_logger().info('Agent: %d %s' % (self.idn, self.str_distance_()))
            if self.id in ('origin', 'sphere', 'cone', 'ellipsoid'):
                self.pose.position = point
                self.k = self.k
            self.sub_pose_ = self.node.create_subscription(
                PoseStamped, '/' + self.id + '/local_pose', self.gtpose_callback, 10)
        if not self.parent.digital_twin:
            self.sub_d_ = self.node.create_subscription(
                Float64, self.parent.id + '/' + self.id + '/d', self.d_callback, 10)
            self.publisher_data_ = self.node.create_publisher(
                Float64, self.parent.id + '/' + self.id + '/data', 10)
            self.publisher_order_ = self.node.create_publisher(
                String, '/' + self.id + '/order', 10)
            self.publisher_error_ = self.node.create_publisher(
                Float64, self.parent.id + '/' + self.id + '/error', 10)
            self.publisher_marker_ = self.node.create_publisher(
                Marker, self.parent.id + '/' + self.id + '/marker', 10)

    def str_(self):
        return ('ID: ' + str(self.id) + ' X: ' + str(self.x) +
                ' Y: ' + str(self.y) + ' Z: ' + str(self.z))

    def str_distance_(self):
        return ('ID: ' + str(self.id) + ' Distance: ' + str(self.d))

    def d_callback(self, msg):
        self.d = msg.data
        self.node.get_logger().info('Agent: %s: new d: %.2f' % (self.id, self.d))

    def gtpose_callback(self, msg):
        self.pose = msg.pose
        if not self.disconnect and not self.parent.digital_twin:
            self.node.get_logger().debug(
                'Agent: X: %.2f Y: %.2f Z: %.2f' %
                (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))

            line = Marker()
            p0 = Point()
            p0.x = self.parent.pose.position.x
            p0.y = self.parent.pose.position.y
            p0.z = self.parent.pose.position.z

            p1 = Point()
            p1.x = self.pose.position.x
            p1.y = self.pose.position.y
            p1.z = self.pose.position.z

            line.header.frame_id = 'map'
            line.header.stamp = self.node.get_clock().now().to_msg()
            line.id = 1
            line.type = 5
            line.action = 0
            line.scale.x = 0.01
            line.scale.y = 0.01
            line.scale.z = 0.01

            if self.distance:
                distance = sqrt(pow(p0.x - p1.x, 2) + pow(p0.y - p1.y, 2) + pow(p0.z - p1.z, 2))
                msg_data = Float64()
                msg_data.data = distance
                self.publisher_data_.publish(msg_data)
                if abs(distance - self.d) > 0.05:
                    line.color.r = 1.0
                else:
                    if abs(distance - self.d) > 0.025:
                        line.color.r = 1.0
                        line.color.g = 0.5
                    else:
                        line.color.g = 1.0
            else:
                dx = p0.x - p1.x
                dy = p0.y - p1.y
                dz = p0.z - p1.z
                if abs(dx) > 0.05 or abs(dy) > 0.05 or abs(dz) > 0.05:
                    line.color.r = 1.0
                else:
                    if abs(dx) > 0.025 or abs(dy) > 0.025 or abs(dz) > 0.025:
                        line.color.r = 1.0
                        line.color.g = 0.5
                    else:
                        line.color.g = 1.0
            line.color.a = 1.0
            line.points.append(p1)
            line.points.append(p0)

            self.publisher_marker_.publish(line)
