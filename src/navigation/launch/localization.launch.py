import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_name ='navigation'

    # Declare launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Whether to launch RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='localization.rviz',
        description='RViz config file name'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time'
    )

    # Paths to external launch file and params
    nav2_localization_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'localization_launch.py'
    )

    localization_params_path = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'amcl_localization.yaml'
    )

    map_file_path = os.path.join(
        get_package_share_directory(pkg_name),
        'maps',
        'my_map2.yaml'
    )

    # RViz node, only launched if rviz=true
    # RViz config path using PathJoinSubstitution
    rviz_config_path = PathJoinSubstitution([
        get_package_share_directory(pkg_name),
        'rviz',
        LaunchConfiguration('rviz_config')
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    # Include the nav2 localization launch file
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_localization_launch_path),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': localization_params_path,
            'map': map_file_path,
        }.items()
    )

    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(rviz_arg)
    ld.add_action(rviz_config_arg)
    ld.add_action(use_sim_time_arg)

    # Add nodes/launch includes
    ld.add_action(rviz_node)
    ld.add_action(localization_launch)

    return ld
