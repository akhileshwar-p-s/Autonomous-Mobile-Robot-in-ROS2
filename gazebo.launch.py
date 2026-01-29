import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    
    pkg_robot_project = get_package_share_directory('warehouse_pkg')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

 
    world_file = PathJoinSubstitution([pkg_robot_project, 'worlds', 'empty.world'])
    
    
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([pkg_robot_project, 'urdf', 'warehouse_bot.urdf.xacro'])
    ])
    
    robot_description = {'robot_description': robot_description_content}

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'warehouse_bot',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.2'],
        output='screen'
    )

    return LaunchDescription([
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity
    ])
