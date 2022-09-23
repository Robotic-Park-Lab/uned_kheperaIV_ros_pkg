import launch
import launch_ros
import os

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory('uned_kheperaiv_webots')

    simulation_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'robot_launch.py')
            )
        )

    simulation_node

    ld = LaunchDescription()
    ld.add_action(simulation_node)

    return ld