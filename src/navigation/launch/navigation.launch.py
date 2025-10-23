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

    # Launch arguments to enable/disable features
    enable_localization_arg = DeclareLaunchArgument(
        'enable_localization', default_value='true',
        description='Enable localization (AMCL + Nav2 localization)'
    )
    enable_mapping_arg = DeclareLaunchArgument(
        'enable_mapping', default_value='true',
        description='Enable mapping (SLAM Toolbox)'
    )
    enable_navigation_arg = DeclareLaunchArgument(
        'enable_navigation', default_value='true',
        description='Enable full Nav2 navigation stack (planner, controller, etc.)'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz'
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='navigation.rviz',
        description='RViz config file'
    )

    # Paths to existing launch files
    localization_launch_path = os.path.join(pkg_navigation, 'launch', 'localization.launch.py')
    mapping_launch_path = os.path.join(pkg_navigation, 'launch', 'mapping.launch.py')
    nav2_navigation_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py'
    )

    # Localization include (conditional)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch_path),
        launch_arguments={
            'rviz': 'false',  # Avoid multiple RViz instances
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_localization'))
    )

    # Mapping include (conditional)
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mapping_launch_path),
        launch_arguments={
            'rviz': 'false',  # Avoid multiple RViz instances
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_mapping'))
    )

    # Nav2 full navigation launch (conditional)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch_path),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': os.path.join(pkg_navigation, 'config', 'navigation.yaml'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_navigation'))
    )

    # RViz node (only started if rviz=true and main navigation is enabled)
    rviz_config_path = PathJoinSubstitution([pkg_navigation, 'rviz', LaunchConfiguration('rviz_config')])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(enable_localization_arg)
    ld.add_action(enable_mapping_arg)
    ld.add_action(enable_navigation_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(rviz_arg)
    ld.add_action(rviz_config_arg)

    # Add conditional launch includes and node
    ld.add_action(LogInfo(msg=['enable_localization:', LaunchConfiguration('enable_localization')]))
    ld.add_action(LogInfo(msg=['enable_mapping:', LaunchConfiguration('enable_mapping')]))
    ld.add_action(LogInfo(msg=['enable_navigation:', LaunchConfiguration('enable_navigation')]))
    ld.add_action(LogInfo(msg=['use_sim_time:', LaunchConfiguration('use_sim_time')]))
    ld.add_action(LogInfo(msg=['rviz:', LaunchConfiguration('rviz')]))

    ld.add_action(localization_launch)
    ld.add_action(mapping_launch)
    ld.add_action(navigation_launch)
    ld.add_action(rviz_node)

    return ld
