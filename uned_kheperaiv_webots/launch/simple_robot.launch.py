import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.utils import controller_url_prefix


def generate_launch_description():
    package_dir = get_package_share_directory('uned_kheperaiv_webots')
    general_config_dir = get_package_share_directory('uned_kheperaiv_config')
    config_path = os.path.join(general_config_dir, 'resources', 'demo_teleop_khepera01.yaml')
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'kheperaiv.urdf')).read_text()
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'RoboticPark_1kh.wbt')
    )
    
    robot01_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='khepera01',
        additional_env={'WEBOTS_ROBOT_NAME': 'khepera01',
                        'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'khepera01',
                        'WEBOTS_ROBOT_CONFIG_FILE': config_path,
                        'WEBOTS_ROBOT_ROLE': 'virtual'},
        parameters=[
            {'robot_description': robot_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    rqt_node = Node(
        package='rqt_gui',
        executable='rqt_gui',
        name='interface',
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        webots,
        robot01_driver,
        robot_state_publisher,
        rqt_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])