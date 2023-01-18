import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_package_dir = get_package_share_directory('uned_kheperaiv_config')
    config_path = os.path.join(config_package_dir, 'resource', 'khepera_ros2_teleop_default.yaml')
    rviz_config_path = os.path.join(config_package_dir, 'rviz', 'test.rviz')

    swarm_config_package_dir = get_package_share_directory('uned_swarm_config')
    swarm_config_path = os.path.join(swarm_config_package_dir, 'resources', 'AA00_distance_formation_configuration.yaml')
    
    hostname = '10.196.92.136'
    buffer_size = 200
    topic_namespace = 'vicon'

    swarm_node = Node(
        package='uned_crazyflie_driver',
        executable='swarm_driver',
        name='swarm',
        output='screen',
        shell=True,
        emulate_tty=True,
        parameters=[
            {'first_uri': 'radio://0/80/2M/E7E7E7E705'},
            {'n': 1},
            {'config': swarm_config_path}
        ]
    )

    robot_node = Node(
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
            {'init_theta': 1.5707},
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
        ]
    )

    return LaunchDescription([
        # swarm_node,
        vicon_node,
        robot_node,
        rqt_node,
        rviz_node
    ])