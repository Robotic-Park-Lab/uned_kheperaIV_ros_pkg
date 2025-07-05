import os
import pathlib
import launch
import yaml
import datetime
import shutil
from yaml.loader import SafeLoader
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def get_ros2_nodes(context, *args):
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    distributed_architecture = False
    # ExecuteProcess(cmd=['ros2', 'bag', 'record', '-a', '-o', e.strftime("%Y-%m-%d-%H-%M"), ], output='screen'),
    node_list = []

    #-------------------#
    #     Load File     #
    #-------------------#
    file = LaunchConfiguration('config_file')
    file_name = file.perform(context)

    general_package_dir = get_package_share_directory('uned_kheperaiv_config')
    config_path = os.path.join(general_package_dir, 'resources', file_name)
    with open(config_path, 'r') as file:
            documents = yaml.safe_load(file)
    
    #------------------------#
    #     Operation mode     #
    #------------------------#
    if not documents['Operation']['mode'] == 'physical':
        use_sim_time = True
        if documents['Operation']['tool'] == 'Webots':
            webots = WebotsLauncher(
                world=PathJoinSubstitution([general_package_dir, 'worlds', documents['Operation']['world']]),
                mode='realtime',
                ros2_supervisor=False
            )
            node_list.append(webots)
            # node_list.append(webots._supervisor)

            kill_ros2 = launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[
                        launch.actions.EmitEvent(event=launch.events.Shutdown())
                    ],
                )
            )
            node_list.append(kill_ros2)
            '''
            reset_handler = launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots._supervisor,
                    on_exit=get_ros2_nodes,
                )
            )
            node_list.append(reset_handler)
            '''
        elif documents['Operation']['tool'] == 'Gazebo':
            world_path = os.path.join(general_package_dir, 'worlds', documents['Operation']['world'])
            gazebo = ExecuteProcess(cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', '--ros-args',
                ], output='screen'
            )

            kill_ros2 = launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=gazebo,
                    on_exit=[
                        launch.actions.EmitEvent(event=launch.events.Shutdown())
                    ],
                )
            )
            node_list.append(gazebo)
            node_list.append(kill_ros2)

    #----------------------#
    #     Architecture     #
    #----------------------#
    if documents['Architecture']['mode'] == 'centralized':
        controller_config_path = os.path.join(general_package_dir, 'resources', documents['Architecture']['node']['file'])
        node_list.append(Node(
            package=documents['Architecture']['node']['pkg'],
            executable=documents['Architecture']['node']['executable'],
            name=documents['Architecture']['node']['name'],
            output='screen',
            parameters=[
                {'config_file': controller_config_path},
                # {'use_sim_time': use_sim_time},
            ]
        ))
    elif documents['Architecture']['mode'] == 'distributed_ros2':
        print('TO-DO: Distributed control in nodes')
        distributed_architecture = True

    #----------------#
    #     Robots     #
    #----------------#
    physical_crazyflie_list = ''
    physical_khepera_list = ''
    for robot in documents['Robots']:
        if 'khepera' in documents['Robots'][robot]['name']:
            robot_description = os.path.join(general_package_dir, 'resources', 'kheperaiv.urdf')
            if not documents['Robots'][robot]['type'] == 'physical':
                with open(robot_description, 'r') as infp:
                    robot_desc = infp.read()
                aux = robot_desc.replace("khepera00", documents['Robots'][robot]['name'])
                aux = aux.replace("name_id_value", documents['Robots'][robot]['name'])
                aux = aux.replace("config_file_path", config_path)
                enable = documents['Robots'][robot]['camera']
                aux = aux.replace("config_cam", enable)
                robot_controller = WebotsController(
                            robot_name=documents['Robots'][robot]['name'],
                            parameters=[
                                {'robot_description': aux,
                                'use_sim_time': use_sim_time,
                                'set_robot_state_publisher': True},
                            ],
                            respawn=True
                        )
                node_list.append(robot_controller)

            if not documents['Robots'][robot]['type'] == 'virtual':
                if physical_khepera_list == '':
                    physical_khepera_list += documents['Robots'][robot]['name']
                else:
                    physical_khepera_list += ', '+documents['Robots'][robot]['name']
    
    print("###  Physical Robots  ###")
    print(physical_khepera_list)
    if not physical_khepera_list == '':
        for robot_id in physical_khepera_list:
            node_list.append(Node(
                    package='uned_kheperaiv_driver',
                    executable='kheperaIV_client_driver',
                    name='driver',
                    output='screen',
                    namespace=robot_id,
                    shell=True,
                    emulate_tty=True,
                    parameters=[
                        {'config': config_path},
                        # {'use_sim_time': use_sim_time},
                        {'id': robot_id}
                    ]
                )
            )

    #------------------------#
    #     CPU Monitoring     #
    #------------------------#
    if documents['CPU_Monitoring']['enable']:
        node_list.append(Node(
            package=documents['CPU_Monitoring']['node']['pkg'],
            executable=documents['CPU_Monitoring']['node']['executable'],
            name=documents['CPU_Monitoring']['node']['name'],
            output='screen',
            parameters=[{
                'process_name' : documents['CPU_Monitoring']['processes'],
                'process_period' : 0.5},
            ],
        ))
    
    #--------------------#
    #     Interfaces     #
    #--------------------#
    if documents['Interface']['enable']:
        if documents['Interface']['rqt']['enable']:
            rqt_config_path = os.path.join(general_package_dir, 'rqt', documents['Interface']['rqt']['file'])
            node_list.append(Node(
                package=documents['Interface']['rqt']['node']['pkg'],
                executable=documents['Interface']['rqt']['node']['executable'],
                name=documents['Interface']['rqt']['node']['name'],
                parameters=[
                    {'use_sim_time': use_sim_time},
                ],
                arguments=['--perspective-file', rqt_config_path],
            ))
        if documents['Interface']['rviz2']['enable']:
            rviz_config_path = os.path.join(general_package_dir, 'rviz', documents['Interface']['rviz2']['file'])
            node_list.append(Node(
                package=documents['Interface']['rviz2']['node']['pkg'],
                executable=documents['Interface']['rviz2']['node']['executable'],
                name=documents['Interface']['rviz2']['node']['name'],
                parameters=[
                    {'use_sim_time': use_sim_time},
                ],
                arguments=['-d', rviz_config_path],
            ))
            node_list.append(Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                output='screen',
                name='RoboticPark',
                arguments=['--yaw', '3.1415', '--frame-id', 'RoboticPark/base_link', '--child-frame-id', 'map'],
            ))
        if documents['Interface']['own']['enable']:
            own_config_path = os.path.join(general_package_dir, 'resources', documents['Interface']['own']['file'])
            node_list.append(Node(
                package=documents['Interface']['own']['node']['pkg'],
                executable=documents['Interface']['own']['node']['executable'],
                name=documents['Interface']['own']['node']['name'],
                parameters=[
                    {'use_sim_time': use_sim_time},
                ],
                arguments=['-d', own_config_path],
            ))
    
    #----------------------#
    #     Data Logging     #
    #----------------------#
    if documents['Data_Logging']['enable']:
        e = datetime.datetime.now()
        if documents['Data_Logging']['all']:
            if documents['Data_Logging']['name'] == 'date':
                node_list.append(ExecuteProcess(
                    cmd=['ros2', 'bag', 'record', '-a', '-o', e.strftime("%Y-%m-%d-%H-%M")], output='screen', shell=True
                ))
            else:
                node_list.append(ExecuteProcess(
                    cmd=['ros2', 'bag', 'record', '-a', '-o', documents['Data_Logging']['name']], output='screen', shell=True
                ))
        else:
            if documents['Data_Logging']['name'] == 'date':
                node_list.append(ExecuteProcess(
                    cmd=['ros2', 'bag', 'record', '-o', e.strftime("%Y-%m-%d-%H-%M"), documents['Data_Logging']['topics']], output='screen', shell=True
                ))
            else:
                node_list.append(ExecuteProcess(
                    cmd=['ros2', 'bag', 'record', '-o', documents['Data_Logging']['name'], documents['Data_Logging']['topics']], output='screen', shell=True
                ))
    
    
    #--------------------#
    #     Supervisor     #
    #--------------------#
    if documents['Supervisor']['enable']:
        topic_config_path = os.path.join(general_package_dir, 'resources', documents['Supervisor']['node']['file'])
        supervisor = Node(
            package='mars_supervisor_pkg',
            executable='supervisor_node',
            name='supervisor',
            parameters=[
                # {'use_sim_time': use_sim_time},
                {'file': topic_config_path},
                {'config': config_path},
            ],
        )
        
        node_list.append(supervisor)
        '''
        kill_ros2_supervisor = launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=supervisor,
                    on_exit=[
                        launch.actions.EmitEvent(event=launch.events.Shutdown())
                    ],
                )
            )
        
        node_list.append(kill_ros2_supervisor)
        '''
    #---------------#
    #     Other     #
    #---------------#    
    print('TO-DO: Physical nodes: Positioning System')
    
    for agent in documents['Other']:
        if documents['Other'][agent]['enable']:
            config_node_path = os.path.join(general_package_dir, 'resources', documents['Other'][agent]['file'])
            node = Node(
                package=documents['Other'][agent]['pkg'],
                executable=documents['Other'][agent]['executable'],
                name=documents['Other'][agent]['name'],
                parameters=[
                    # {'use_sim_time': use_sim_time},
                    {'file': config_node_path},
                ],
            )
            node_list.append(node)

    return node_list

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value='demo_benchmark_webots.yaml',
            description='path config file'
        ),
        launch.actions.OpaqueFunction(function=get_ros2_nodes),
        # ExecuteProcess(cmd=[file], output='screen'),
    ])