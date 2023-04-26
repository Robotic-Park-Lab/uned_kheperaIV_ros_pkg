import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_package_dir = get_package_share_directory('uned_kheperaiv_config')
    config_path = os.path.join(config_package_dir, 'resources', 'demo_teleop.yaml')
    rviz_config_path = os.path.join(config_package_dir, 'rviz', 'test.rviz')

    robot_node = Node(
        package='uned_kheperaiv_driver',
        executable='kheperaIV_client_driver',
        name='driver',
        namespace='khepera04',
        output='screen',
        shell=True,
        emulate_tty=True,
        parameters=[
            {'id': 'khepera04'},
            {'config': config_path}
        ]
    )

    rqt_node = Node(
        package='rqt_gui',
        executable='rqt_gui',
        name='interface'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],

    )

    return LaunchDescription([
        robot_node,
        rqt_node,
        rviz_node
    ])