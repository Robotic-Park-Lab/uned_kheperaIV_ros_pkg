"""
Demo for spawn_entity.
Launches Gazebo and spawns a model
"""
from http.server import executable
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():

    world_path = os.path.join(get_package_share_directory('uned_kheperaiv_config'), 'worlds', 'RoboticParkLab.world')
    # urdf_path = os.path.join(get_package_share_directory('tello_description'), 'urdf', 'tello_1.urdf')
    # urdf_path2 = os.path.join(get_package_share_directory('tello_description'), 'urdf', 'tello_2.urdf')

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
                            arguments=['-entity', 'khepera01', '-database', 'khepera_IV','-robot_namespace', 'khepera01','-x', '1'],
                            output='screen'),

        Node(package='gazebo_ros', executable='spawn_entity.py',
                            arguments=['-entity', 'khepera02', '-database', 'khepera_IV','-robot_namespace', 'khepera02','-x', '0.5','-y', '0.5'],
                            output='screen'),

        Node(package='uned_kheperaiv_controllers', executable='periodic_pid_position_controller',
                name='position_controller',
                namespace='khepera01',
                output='screen',
                shell=True,
                emulate_tty=True),
        
        Node(package='uned_kheperaiv_controllers', executable='periodic_pid_position_controller',
                name='position_controller',
                namespace='khepera02',
                output='screen',
                shell=True,
                emulate_tty=True)

    ])
