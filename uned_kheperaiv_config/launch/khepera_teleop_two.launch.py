import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_package_dir = get_package_share_directory('uned_kheperaiv_config')
    config_path = os.path.join(config_package_dir, 'resource', 'khepera_ros2_teleop_default.yaml')
    rviz_config_path = os.path.join(config_package_dir, 'rviz', 'test.rviz')

    robot01_node = Node(
        package='uned_kheperaiv_driver',
        executable='kheperaIV_client_driver',
        name='driver',
        namespace='khepera01',
        output='screen',
        shell=True,
        emulate_tty=True,
        parameters=[
            {'agent_ip': '192.168.0.21'},
            {'port_number': 50000},
            {'id': 'khepera01'},
            {'config': config_path}
        ])

    robot02_node = Node(
        package='uned_kheperaiv_driver',
        executable='kheperaIV_client_driver',
        namespace='khepera02',
        name='driver',
        output='screen',
        shell=True,
        emulate_tty=True,
        parameters=[
            {'agent_ip': '192.168.0.22'},
            {'port_number': 50000},
            {'id': 'khepera02'},
            {'config': config_path}
        ])
    
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
        robot01_node,
        robot02_node,
        rqt_node,
        rviz_node
    ])