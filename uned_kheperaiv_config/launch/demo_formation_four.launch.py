import os
import yaml
from yaml.loader import SafeLoader
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config_package_dir = get_package_share_directory('uned_kheperaiv_config')
    config_path = os.path.join(config_package_dir, 'resources', 'demo_formation.yaml')
    rviz_config_path = os.path.join(config_package_dir, 'rviz', 'test.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    robot_node_list = []

    with open(config_path) as f:
        data = yaml.load(f, Loader=SafeLoader)
        for key, robot in data.items():
            print("###  "+robot['name']+"  ###")
            individual_config_path = os.path.join(config_package_dir, 'resources', robot['config_path'])
            robot_node_list.append(Node(package='uned_kheperaiv_driver',
                                        executable='kheperaIV_client_driver',
                                        name='driver',
                                        namespace=robot['name'],
                                        output='screen',
                                        shell=True,
                                        emulate_tty=True,
                                        parameters=[{'id': robot['name']},
                                                    {'config': individual_config_path}
                                        ]
                                    )
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
    ld.add_action(rqt_node)
    ld.add_action(rviz_node)
    for robot in robot_node_list:
        ld.add_action(robot)

    return ld 