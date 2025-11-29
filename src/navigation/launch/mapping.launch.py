import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_navigation = get_package_share_directory('navigation')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time',
        choices=['true', 'false']
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz',
        choices=['true', 'false']
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='mapping.rviz',
        description='RViz config file'
    )

    slam_toolbox_launch_path = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )

    slam_toolbox_params_path = os.path.join(
        pkg_navigation,
        'config',
        'slam_toolbox_mapping.yaml'
    )

    rviz_config_path = PathJoinSubstitution([
        pkg_navigation, 'rviz', LaunchConfiguration('rviz_config')
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_path),
        launch_arguments=[
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('slam_params_file', slam_toolbox_params_path),
        ]
    )

    ld = LaunchDescription()

    ld.add_action(use_sim_time_arg)
    ld.add_action(rviz_arg)
    ld.add_action(rviz_config_arg)

    ld.add_action(LogInfo(msg=['use_sim_time:', LaunchConfiguration('use_sim_time')]))
    ld.add_action(LogInfo(msg=['rviz:', LaunchConfiguration('rviz')]))
    ld.add_action(LogInfo(msg=['rviz_config:', LaunchConfiguration('rviz_config')]))

#    ld.add_action(rviz_node)
    ld.add_action(slam_toolbox_launch)

    return ld
