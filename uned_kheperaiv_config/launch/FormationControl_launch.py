from http.server import executable
import os
import random
import numpy as np
import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution,ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_path = os.path.join(get_package_share_directory('uned_kheperaiv_config'), 'worlds', 'UNED_RoboticParkLab_invert.world')
    khepera01_x = str(round(random.uniform(-1.0, 1.0), 2))
    khepera01_y = str(round(random.uniform(-1.0, 1.0), 2))
    khepera02_x = str(round(random.uniform(-1.0, 1.0), 2))
    khepera02_y = str(round(random.uniform(-1.0, 1.0), 2))
    khepera03_x = str(round(random.uniform(-1.0, 1.0), 2))
    khepera03_y = str(round(random.uniform(-1.0, 1.0), 2))

    L = np.matrix(' 0.0,  0.0,  0.5,  0.5,  0.2,  0.0; \
                    0.0,  0.0,  0.5,  0.5,  0.0,  0.2; \
                   -0.5, -0.5, -0.6,  0.0,  0.5, -0.5; \
                   -0.5, -0.5,  0.0, -0.6,  0.5, -0.5; \
                    0.0,  0.0, -0.5,  0.5,  0.0,  0.0; \
                    0.0,  0.0, -0.5,  0.5,  0.0,  0.0')

    e = datetime.datetime.now()
    #  '-u',
    return LaunchDescription([
        ExecuteProcess(cmd=[
            'gazebo',
            '-s', 'libgazebo_ros_init.so',  # Publish /clock
            '-s', 'libgazebo_ros_factory.so',  # Provide gazebo_ros::Node
            world_path
        ], output='screen'),

        # ExecuteProcess(cmd=['ros2', 'bag', 'record', '-a', '-o', e.strftime("%Y-%m-%d-%H-%M"), ], output='screen'),

        # GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
        Node(package='gazebo_ros', executable='spawn_entity.py',
                            arguments=['-entity', 'khepera01', '-database', 'khepera_IV','-robot_namespace', 'khepera01','-x', khepera01_x, '-y', khepera01_y],
                            output='screen'),

        Node(package='uned_kheperaiv_controllers', executable='periodic_pid_position_controller',
                name='position_controller',
                namespace='khepera01',
                output='screen',
                shell=True,
                emulate_tty=True,
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {"LKp": 1.0, "LKi": 0.0, "LKd": 0.0, "LTd": 0.0},
                    {"WKp": 1.0, "WKi": 0.0, "WKd": 0.0, "WTd": 0.0},
                    {"Lco": 0.01, "Lai": 0.015},
                    {"Wco": 0.01, "Wai": 0.015},
                    {"Relative_pose": True},
                ]),
        
        Node(package='uned_kheperaiv_task', executable='shape_based_formation_control',
                name='formation_control',
                namespace='khepera01',
                output='screen',
                shell=True,
                emulate_tty=True,
                remappings=[
                    ('/khepera01/khepera02/pose', '/khepera02/pose'),
                    ('/khepera01/dron01/pose', '/dron01/pose'),
                    ('/khepera01/swarm/status', '/swarm/status')],
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {"config_file": 'path'},
                    {"agents": 'khepera02, dron01'},
                    {"agent_x": ', '.join([str(a) for a in np.squeeze(np.asarray(L[0,[2,4]]))])},
                    {"agent_y": ', '.join([str(a) for a in np.squeeze(np.asarray(L[1,[3,5]]))])},
                ]),

        Node(package='gazebo_ros', executable='spawn_entity.py',
                            arguments=['-entity', 'khepera02', '-database', 'khepera_IV','-robot_namespace', 'khepera02','-x', khepera02_x,'-y', khepera02_y],
                            output='screen'),

        Node(package='uned_kheperaiv_controllers', executable='periodic_pid_position_controller',
                name='position_controller',
                namespace='khepera02',
                output='screen',
                shell=True,
                emulate_tty=True,
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {"LKp": 1.0, "LKi": 0.0, "LKd": 0.0, "LTd": 0.0},
                    {"WKp": 1.0, "WKi": 0.0, "WKd": 0.0, "WTd": 0.0},
                    {"Lco": 0.01, "Lai": 0.015},
                    {"Wco": 0.01, "Wai": 0.015},
                    {"Relative_pose": True},
                ]),

        Node(package='uned_kheperaiv_task', executable='shape_based_formation_control',
                name='formation_control',
                namespace='khepera02',
                output='screen',
                shell=True,
                emulate_tty=True,
                remappings=[
                    ('/khepera02/khepera01/pose', '/khepera01/pose'),
                    ('/khepera02/khepera03/pose', '/khepera03/pose'),
                    ('/khepera02/dron02/pose', '/dron02/pose'),
                    ('/khepera02/swarm/status', '/swarm/status')],
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {"config_file": 'path'},
                    {"agents": 'khepera01, dron02, khepera03'},
                    {"agent_x": ', '.join([str(a) for a in np.squeeze(np.asarray(L[2,[0,2,4]]))])},
                    {"agent_y": ', '.join([str(a) for a in np.squeeze(np.asarray(L[3,[1,3,5]]))])},
                ]),

        Node(package='gazebo_ros', executable='spawn_entity.py',
                            arguments=['-entity', 'khepera03', '-database', 'khepera_IV','-robot_namespace', 'khepera03','-x', khepera03_x,'-y', khepera03_y],
                            output='screen'),

        Node(package='uned_kheperaiv_controllers', executable='periodic_pid_position_controller',
                name='position_controller',
                namespace='khepera03',
                output='screen',
                shell=True,
                emulate_tty=True,
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {"LKp": 1.0, "LKi": 0.0, "LKd": 0.0, "LTd": 0.0},
                    {"WKp": 1.0, "WKi": 0.0, "WKd": 0.0, "WTd": 0.0},
                    {"Lco": 0.01, "Lai": 0.015},
                    {"Wco": 0.01, "Wai": 0.015},
                    {"Relative_pose": True},
                ]),
        
        Node(package='uned_kheperaiv_task', executable='shape_based_formation_control',
                name='formation_control',
                namespace='khepera03',
                output='screen',
                shell=True,
                emulate_tty=True,
                remappings=[
                    ('/khepera03/khepera02/pose', '/khepera02/pose'),
                    ('/khepera03/swarm/status', '/swarm/status')],
                #arguments=['--ros-args', '--log-level', 'info'],
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {"config_file": 'path'},
                    {"agents": 'khepera02'},
                    {"agent_x": ', '.join([str(np.squeeze(np.asarray(L[4,[2]])))])},
                    {"agent_y": ', '.join([str(np.squeeze(np.asarray(L[5,[3]])))])},
                ]),
        Node(package='rqt_gui', executable='rqt_gui',name='interface', parameters=[{'use_sim_time': use_sim_time}]),

        Node(package='uned_vicon_gazebo', executable='vicon_gazebo',
                name='vicon_gazebo',
                output='screen',
                shell=True,
                emulate_tty=True,
                #arguments=['--ros-args', '--log-level', 'info'],
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {"agents": 'khepera01, khepera02, khepera03'},
                ])
    ])
