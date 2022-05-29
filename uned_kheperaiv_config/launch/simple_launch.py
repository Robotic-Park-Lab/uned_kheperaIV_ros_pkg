"""
Demo for spawn_entity.
Launches Gazebo and spawns a model
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():

    world_path = os.path.join(get_package_share_directory('uned_kheperaiv_config'), 'worlds', 'simple.world')
    return LaunchDescription([
        ExecuteProcess(cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',  # Publish /clock
            '-s', 'libgazebo_ros_factory.so',  # Provide gazebo_ros::Node
            world_path
        ], output='screen'),

        # GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
        Node(package='gazebo_ros', executable='spawn_entity.py',
                            arguments=['-entity', 'khepera', '-database', 'khepera_IV','-robot_namespace', 'khepera01','-x', '1'],
                            output='screen')
    ])
