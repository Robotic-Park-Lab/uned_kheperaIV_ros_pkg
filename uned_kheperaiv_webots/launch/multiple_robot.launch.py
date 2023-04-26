import os
import pathlib
import launch
import yaml
from yaml.loader import SafeLoader
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.utils import controller_url_prefix


def generate_launch_description():
    package_dir = get_package_share_directory('uned_kheperaiv_webots')
    general_config_dir = get_package_share_directory('uned_kheperaiv_config')
    config_path = os.path.join(general_config_dir, 'resources', 'demo_teleop.yaml')
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'kheperaiv.urdf')).read_text()
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'RoboticPark_4kh.wbt')
    )

    robot_node_list = []

    with open(config_path) as f:
        data = yaml.load(f, Loader=SafeLoader)
        for key, robot in data.items():
            print("###  "+robot['name']+"  ###")
            individual_config_path = os.path.join(general_config_dir, 'resources', robot['config_path'])

            robot_node_list.append(Node(package='webots_ros2_driver', 
                                            executable='driver', 
                                            output='screen',
                                            name=robot['name'],
                                            additional_env={'WEBOTS_ROBOT_NAME': robot['name'],
                                                            'WEBOTS_CONTROLLER_URL': controller_url_prefix() + robot['name'],
                                                            'WEBOTS_ROBOT_CONFIG_FILE': individual_config_path,
                                                            'WEBOTS_ROBOT_ROLE': robot['type']},
                                            parameters=[{   'robot_description': robot_description,
                                                            'use_sim_time': use_sim_time,
                                                            'set_robot_state_publisher': True},
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

    ros2_close = launch.actions.RegisterEventHandler(
                    event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                    )
                )
    
    ld = LaunchDescription()
    ld.add_action(webots)
    ld.add_action(rqt_node)
    for robot in robot_node_list:
        ld.add_action(robot)
        
    ld.add_action(ros2_close)

    return ld 