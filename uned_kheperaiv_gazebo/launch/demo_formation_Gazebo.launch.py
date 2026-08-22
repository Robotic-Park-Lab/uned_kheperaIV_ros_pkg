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

import os
import yaml
from yaml.loader import SafeLoader
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess


def generate_launch_description():
    general_config_dir = get_package_share_directory('uned_kheperaiv_config')
    # Same real bugs fixed here as in multiple_robot_Gazebo.launch.py (see
    # that file's comment for the detail): wrong yaml/rviz filenames, and
    # an individual per-robot config_path that doesn't exist as a yaml key
    # in the experience file -- gazebo_driver.py needs its own small
    # defaults file (khepera_gazebo_default.yaml, new).
    config_path = os.path.join(general_config_dir, 'resources', 'Demo_formation_webots.yaml')
    individual_config_path = os.path.join(
        general_config_dir,
        'resources',
        'khepera_gazebo_default.yaml')
    rviz_config_path = os.path.join(general_config_dir, 'rviz', 'default.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    world_path = os.path.join(general_config_dir, 'worlds', 'UNED_RoboticParkLab_invert.world')
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            world_path,
            '-s',
            'libgazebo_ros_init.so',
            '-s',
            'libgazebo_ros_factory.so',
            '--ros-args',
        ],
        output='screen')

    robot_node_list = []

    with open(config_path) as f:
        data = yaml.load(f, Loader=SafeLoader)
        for key, robot in data['Robots'].items():
            print("###  " + robot['name'] + "  ###")

            urdf_path = os.path.join(general_config_dir, 'urdf', robot['name'] + '.urdf')
            pose = robot['pose'].split()
            robot_node_list.append(
                Node(
                    package='uned_kheperaiv_gazebo',
                    executable='inject_entity.py',
                    output='screen',
                    arguments=[
                        urdf_path,
                        pose[0],
                        pose[1],
                        '0.05',
                        pose[2]]),
            )
            robot_node_list.append(Node(package='uned_kheperaiv_task',
                                        executable='gazebo_driver',
                                        name='driver',
                                        namespace=robot['name'],
                                        output='screen',
                                        parameters=[{'use_sim_time': use_sim_time,
                                                     'config_file': individual_config_path,
                                                     'robot': robot['name'],
                                                     'type': 'virtual'},
                                                    ]),
                                   )

    rqt_node = Node(
        package='rqt_gui',
        executable='rqt_gui',
        name='interface',
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
        arguments=['-d', rviz_config_path],
    )

    ld = LaunchDescription()
    ld.add_action(gazebo)
    ld.add_action(rqt_node)
    ld.add_action(rviz_node)
    for robot in robot_node_list:
        ld.add_action(robot)

    return ld
