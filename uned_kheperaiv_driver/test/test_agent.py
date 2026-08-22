# Copyright 2015 Open Source Robotics Foundation, Inc.
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

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Pose, Vector3

from uned_kheperaiv_driver.agent import Agent


class FakeLogger:
    def info(self, *a, **k):
        pass

    def debug(self, *a, **k):
        pass


class FakeClock:
    def now(self):
        return self

    def to_msg(self):
        return Time()


class FakePublisher:
    def publish(self, msg):
        pass


class FakeNode:
    def __init__(self):
        self.subscriptions = []
        self.publishers = []

    def get_logger(self):
        return FakeLogger()

    def get_clock(self):
        return FakeClock()

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subscriptions.append(topic)
        return object()

    def create_publisher(self, msg_type, topic, qos):
        self.publishers.append(topic)
        return FakePublisher()


class FakeParent:
    def __init__(self, digital_twin=False):
        self.agent_list = []
        self.id = 'khepera01'
        self.digital_twin = digital_twin
        self.pose = Pose()


def test_distance_agent_subscribes_to_local_pose_and_tracks_distance():
    node = FakeNode()
    parent = FakeParent()
    agent = Agent(parent, node, 'khepera02', d=1.5)

    assert agent.distance is True
    assert agent.d == 1.5
    assert '/khepera02/local_pose' in node.subscriptions


def test_str_distance_formatting():
    node = FakeNode()
    parent = FakeParent()
    agent = Agent(parent, node, 'khepera02', d=2.0)
    assert agent.str_distance_() == 'ID: khepera02 Distance: 2.0'


def test_line_agent_computes_vector_modulus():
    node = FakeNode()
    parent = FakeParent()
    point = Point(x=0.0, y=0.0, z=0.0)
    vector = Vector3(x=3.0, y=4.0, z=0.0)
    agent = Agent(parent, node, 'line01', point=point, vector=vector)

    assert agent.mod == 25.0  # 3^2 + 4^2
    assert agent.k == 4.0


def test_digital_twin_parent_skips_marker_publishers():
    node = FakeNode()
    parent = FakeParent(digital_twin=True)
    Agent(parent, node, 'khepera02', d=1.0)

    assert node.publishers == []


def test_gtpose_callback_updates_pose_without_crashing():
    node = FakeNode()
    parent = FakeParent()
    agent = Agent(parent, node, 'khepera02', d=1.0)

    msg = type('Msg', (), {'pose': Pose()})()
    msg.pose.position.x = 1.0
    msg.pose.position.y = 2.0
    msg.pose.position.z = 0.0
    agent.gtpose_callback(msg)

    assert agent.pose.position.x == 1.0
