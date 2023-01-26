import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_package_dir = get_package_share_directory('uned_kheperaiv_config')
    config_path = os.path.join(config_package_dir, 'resource', 'khepera_ros2_teleop_default.yaml')
    rviz_config_path = os.path.join(config_package_dir, 'rviz', 'test.rviz')

    hostname = '10.196.92.136'
    buffer_size = 200
    topic_namespace = 'vicon'

    robot01_node = Node(
        package='uned_kheperaiv_driver',
        executable='kheperaIV_client_driver',
        name='driver',
        namespace='khepera01',
        output='screen',
        shell=True,
        emulate_tty=True,
        parameters=[
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

    vicon_node = Node(
        package='vicon_receiver',
        executable='vicon_client',
        name='vicon_node',
        parameters=[
            {'hostname': hostname, 'buffer_size': buffer_size, 'namespace': topic_namespace}
        ])

    return LaunchDescription([
        robot01_node,
        robot02_node,
        rqt_node,
        rviz_node,
        vicon_node
    ])