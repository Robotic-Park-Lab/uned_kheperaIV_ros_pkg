import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_package_dir = get_package_share_directory('uned_kheperaiv_config')
    config_path = os.path.join(config_package_dir, 'resource', 'khepera_ros2_formation_distance_four_lighthouse.yaml')
    rviz_config_path = os.path.join(config_package_dir, 'rviz', 'test.rviz')
    
    swarm_node = Node(
        package='uned_crazyflie_driver',
        executable='swarm_driver',
        name='swarm',
        output='screen',
        shell=True,
        emulate_tty=True,
        parameters=[
            {'first_uri': 'radio://0/80/2M/E7E7E7E705'},
            {'n': 4},
            {'config': config_path}
        ]
    )

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
        ]
    )

    robot01_task = Node(
        package='uned_kheperaiv_task',
        executable='distance_based_formation_control',
        output='screen',
        name='formation_control',
        namespace='khepera01',
        parameters=[
            {'config': config_path},
            {"robot": 'khepera01'},
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
            {'id': 'khepera02'},
            {'config': config_path}
        ]
    )

    robot02_task = Node(
        package='uned_kheperaiv_task',
        executable='distance_based_formation_control',
        output='screen',
        name='formation_control',
        namespace='khepera02',
        parameters=[
            {'config': config_path},
            {"robot": 'khepera02'},
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
            {'id': 'khepera03'},
            {'config': config_path}
        ]
    )

    robot03_task = Node(
        package='uned_kheperaiv_task',
        executable='distance_based_formation_control',
        output='screen',
        name='formation_control',
        namespace='khepera03',
        parameters=[
            {'config': config_path},
            {"robot": 'khepera03'},
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
            {'id': 'khepera04'},
            {'config': config_path}
        ]
    )

    robot04_task = Node(
        package='uned_kheperaiv_task',
        executable='distance_based_formation_control',
        output='screen',
        name='formation_control',
        namespace='khepera04',
        parameters=[
            {'config': config_path},
            {"robot": 'khepera04'},
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
        swarm_node
    ])