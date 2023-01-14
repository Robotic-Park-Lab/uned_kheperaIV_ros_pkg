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
            {'agent_ip': '192.168.0.21'},
            {'port_number': 50000},
            {'id': 'khepera01'},
            {'config': config_path}
        ])

    robot01_task = Node(
        package='uned_kheperaiv_task',
        executable='distance_based_formation_control',
        output='screen',
        name='formation_control',
        namespace='khepera01',
        remappings=[
            ('/khepera01/khepera02/local_pose', '/khepera02/local_pose'),
            ('/khepera01/khepera03/local_pose', '/khepera03/local_pose'),
            ('/khepera01/khepera04/local_pose', '/khepera04/local_pose'),
            ('/khepera01/turtlebot01/local_pose', '/turtlebot01/local_pose'),
            ('/khepera01/swarm/status', '/swarm/status'),
            ('/khepera01/swarm/order', '/swarm/order')],
        parameters=[
            {"config_file": 'path'},
            {"robot": 'khepera01'},
            {"agents": 'khepera02, khepera03, khepera04'},
            {"distance": '0.6, 0.8485, 0.6'},
        ]
    )

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

    robot02_task = Node(
        package='uned_kheperaiv_task',
        executable='distance_based_formation_control',
        output='screen',
        name='formation_control',
        namespace='khepera02',
        remappings=[
            ('/khepera02/khepera01/local_pose', '/khepera01/local_pose'),
            ('/khepera02/khepera03/local_pose', '/khepera03/local_pose'),
            ('/khepera02/khepera04/local_pose', '/khepera04/local_pose'),
            ('/khepera02/turtlebot01/local_pose', '/turtlebot01/local_pose'),
            ('/khepera02/swarm/status', '/swarm/status'),
            ('/khepera02/swarm/order', '/swarm/order')],
        parameters=[
            {"config_file": 'path'},
            {"robot": 'khepera02'},
            {"agents": 'khepera01, khepera03, khepera04'},
            {"distance": '0.6, 0.6, 0.8485'},
        ]
    )

    robot03_node = Node(
        package='uned_kheperaiv_driver',
        executable='kheperaIV_client_driver',
        name='driver',
        namespace='khepera03',
        output='screen',
        shell=True,
        emulate_tty=True,
        parameters=[
            {'agent_ip': '192.168.0.19'},
            {'port_number': 50000},
            {'id': 'khepera03'},
            {'config': config_path}
        ])

    robot03_task = Node(
        package='uned_kheperaiv_task',
        executable='distance_based_formation_control',
        output='screen',
        name='formation_control',
        namespace='khepera03',
        remappings=[
            ('/khepera03/khepera02/local_pose', '/khepera02/local_pose'),
            ('/khepera03/khepera01/local_pose', '/khepera01/local_pose'),
            ('/khepera03/khepera04/local_pose', '/khepera04/local_pose'),
            ('/khepera03/turtlebot01/local_pose', '/turtlebot01/local_pose'),
            ('/khepera03/swarm/status', '/swarm/status'),
            ('/khepera03/swarm/order', '/swarm/order')],
        parameters=[
            {"config_file": 'path'},
            {"robot": 'khepera03'},
            {"agents": 'khepera02, khepera01, khepera04'},
            {"distance": '0.6, 0.8485, 0.6'},
        ]
    )

    robot04_node = Node(
        package='uned_kheperaiv_driver',
        executable='kheperaIV_client_driver',
        name='driver',
        namespace='khepera04',
        output='screen',
        shell=True,
        emulate_tty=True,
        parameters=[
            {'agent_ip': '192.168.0.20'},
            {'port_number': 50000},
            {'id': 'khepera04'},
            {'config': config_path}
        ])

    robot04_task = Node(
        package='uned_kheperaiv_task',
        executable='distance_based_formation_control',
        output='screen',
        name='formation_control',
        namespace='khepera04',
        remappings=[
            ('/khepera04/khepera01/local_pose', '/khepera01/local_pose'),
            ('/khepera04/khepera02/local_pose', '/khepera02/local_pose'),
            ('/khepera04/khepera03/local_pose', '/khepera03/local_pose'),
            ('/khepera04/turtlebot01/local_pose', '/turtlebot01/local_pose'),
            ('/khepera04/swarm/status', '/swarm/status'),
            ('/khepera04/swarm/order', '/swarm/order')],
        parameters=[
            {"config_file": 'path'},
            {"robot": 'khepera04'},
            {"agents": 'khepera01, khepera02, khepera03'},
            {"distance": '0.6, 0.8485, 0.6'},
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

    vicon_node = Node(
        package='vicon_receiver',
        executable='vicon_client',
        name='vicon_node',
        parameters=[
            {'hostname': hostname, 'buffer_size': buffer_size, 'namespace': topic_namespace}
        ])

    return LaunchDescription([
        robot01_node,
        robot01_task,
        robot02_node,
        robot02_task,
        robot03_node,
        robot03_task,
        robot04_node,
        robot04_task,
        rqt_node,
        rviz_node,
        vicon_node
    ])