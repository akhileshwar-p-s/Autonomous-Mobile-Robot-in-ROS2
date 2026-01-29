import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_share = FindPackageShare(package='warehouse_pkg').find('warehouse_pkg')
    default_urdf_model_path = os.path.join(pkg_share, 'urdf/warehouse_bot.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/slam.rviz')
    world_file_name = 'custom_world.world'
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)
    slam_params_file = os.path.join(pkg_share, 'config/mapper_params_online_async.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    
    declare_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RViz')

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', default_urdf_model_path])
        }]
    )

    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': world_path}.items()
    )

    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'warehouse_bot',
                   '-topic', 'robot_description',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.0',
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', '0.0'],
        output='screen'
    )

    start_slam_toolbox_cmd = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_rviz_cmd)

    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(start_slam_toolbox_cmd)
    ld.add_action(start_rviz_cmd)

    return ld
