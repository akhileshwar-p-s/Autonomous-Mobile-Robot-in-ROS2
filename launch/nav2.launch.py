import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('warehouse_pkg')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_share, 'maps', 'my_warehouse_map.yaml'),
        description='Full path to the occupancy grid map yaml file'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        description='Full path to the Nav2 parameters file'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation time'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gazebo.launch.py')
        )
    )

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time, 'yaml_filename': map_file}],
        remappings=remappings,
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=remappings,
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=remappings,
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=remappings,
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=remappings,
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=remappings,
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=remappings + [
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel'),
        ],
    )

    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
            ],
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            os.path.join(nav2_bringup_share, 'rviz', 'nav2_default_view.rviz')
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_map,
        declare_params_file,
        declare_use_sim_time,
        gazebo,
        TimerAction(
            period=5.0,
            actions=[
                map_server,
                amcl,
                controller_server,
                smoother_server,
                planner_server,
                behavior_server,
                bt_navigator,
                waypoint_follower,
                velocity_smoother,
                lifecycle_manager_localization,
                lifecycle_manager_navigation,
            ]
        ),
        TimerAction(
            period=7.0,
            actions=[rviz]
        ),
    ])
