#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    
    pkg_share = get_package_share_directory('warehouse_pkg')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    urdf_file = os.path.join(pkg_share, 'urdf', 'warehouse_bot.urdf.xacro')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    map_file = os.path.join(pkg_share, 'maps', 'my_warehouse_map.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'nav2.rviz')
    world_file = os.path.join(pkg_share, 'worlds', 'custom_world.world')
    
    print(f"\n{'='*70}")
    print(f"Package share directory: {pkg_share}")
    print(f"Map file path: {map_file}")
    print(f"Map file exists: {os.path.exists(map_file)}")
    print(f"Nav2 params: {nav2_params_file}")
    print(f"Nav2 params exists: {os.path.exists(nav2_params_file)}")
    print(f"{'='*70}\n")
    
    if not os.path.exists(map_file):
        print(f"ERROR: Map file not found at {map_file}")
        print(f"Please create the map first using SLAM!")
        print(f"Run: ros2 run nav2_map_server map_saver_cli -f {os.path.join(pkg_share, 'maps', 'my_warehouse_map')}\n")
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    map_yaml_file = LaunchConfiguration('map', default=map_file)

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )
    
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )
    
    declare_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )
    
    declare_map_yaml = DeclareLaunchArgument(
        'maps',
        default_value=map_file,
        description='Full path to map yaml file'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', urdf_file])
        }]
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 
                        'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': world_file}.items()
    )
    
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-entity', 'warehouse_bot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ]
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ]),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': autostart
        }.items()
    )
    
    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_autostart)
    ld.add_action(declare_rviz)
    ld.add_action(declare_map_yaml)
    
    ld.add_action(robot_state_publisher)
    ld.add_action(gazebo)
    ld.add_action(spawn_robot)
    ld.add_action(nav2_bringup)
    ld.add_action(rviz)
    
    return ld
