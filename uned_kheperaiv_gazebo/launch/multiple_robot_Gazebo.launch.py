import os
import pathlib
import launch
import yaml
from yaml.loader import SafeLoader
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess


def generate_launch_description():
    general_config_dir = get_package_share_directory('uned_kheperaiv_config')
    # Real bugs fixed here (found by actually trying to launch this file):
    # - config_path pointed at 'demo_teleop.yaml', which doesn't exist --
    #   the real file is 'Demo_teleop_webots.yaml', and it uses the repo's
    #   current nested Operation/Robots schema (same as experience.launch.py),
    #   not the old flat {robot_id: {...}} shape this file used to assume.
    # - rviz_config_path pointed at 'test.rviz', which doesn't exist either
    #   -- using 'default.rviz' instead.
    # - individual per-robot config_path came from a 'config_path' yaml key
    #   that this experience file doesn't have; gazebo_driver.py actually
    #   needs a file shaped as {robot_id: {task: {...}, communication: {...}}}
    #   -- new resources/khepera_gazebo_default.yaml provides that.
    config_path = os.path.join(general_config_dir, 'resources', 'Demo_teleop_webots.yaml')
    individual_config_path = os.path.join(general_config_dir, 'resources', 'khepera_gazebo_default.yaml')
    rviz_config_path = os.path.join(general_config_dir, 'rviz', 'default.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    world_path = os.path.join(general_config_dir, 'worlds', 'UNED_RoboticParkLab_invert.world')
    gazebo = ExecuteProcess(cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', '--ros-args',
        ], output='screen'
    )

    robot_node_list = []

    with open(config_path) as f:
        data = yaml.load(f, Loader=SafeLoader)
        for key, robot in data['Robots'].items():
            print("###  "+robot['name']+"  ###")

            urdf_path = os.path.join(general_config_dir, 'urdf', robot['name']+'.urdf')
            pose = robot['pose'].split()
            robot_node_list.append(Node(package='uned_kheperaiv_gazebo', executable='inject_entity.py', output='screen',
                                            arguments=[urdf_path, pose[0], pose[1], '0.05', pose[2]]),
            )
            robot_node_list.append(Node(package='uned_kheperaiv_task', 
                                        executable='gazebo_driver',
                                        name='driver',
                                        namespace=robot['name'],
                                        output='screen',
                                        parameters=[{'use_sim_time' : use_sim_time,
                                                     'config_file' : individual_config_path,
                                                     'robot' : robot['name'],
                                                     'type' : 'virtual'},
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
        