import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_package_dir = get_package_share_directory('uned_kheperaiv_config')
    config_path = os.path.join(config_package_dir, 'resource', 'khepera_ros2_teleop_lighthouse.yaml')
    rviz_config_path = os.path.join(config_package_dir, 'rviz', 'test.rviz')

    swarm_node = Node(
        package='uned_crazyflie_driver',
        executable='swarm_driver',
        name='swarm',
        output='screen',
        shell=True,
        emulate_tty=True,
        parameters=[
            {'first_uri': 'radio://0/80/2M/E7E7E7E706'},
            {'n': 1},
            {'config': config_path}
        ]
    )

    robot_node = Node(
        package='uned_kheperaiv_driver',
        executable='kheperaIV_client_driver',
        name='driver',
        namespace='khepera02',
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

    return LaunchDescription([
        swarm_node,
        robot_node,
        rqt_node,
        rviz_node
    ])