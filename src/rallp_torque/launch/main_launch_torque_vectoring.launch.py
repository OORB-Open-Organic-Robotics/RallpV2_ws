#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Launch arguments
    enable_torque_vectoring_arg = DeclareLaunchArgument(
        'enable_torque_vectoring',
        default_value='true',
        description='Enable torque vectoring control system')
    
    use_python_torque_node_arg = DeclareLaunchArgument(
        'use_python_torque_node',
        default_value='false',
        description='Use Python implementation of torque vectoring node')
    
    enable_visualization_arg = DeclareLaunchArgument(
        'enable_visualization',
        default_value='false',
        description='Enable torque vectoring visualization')
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time for all nodes')

    # Get package directory
    pkg_share = FindPackageShare(package='rallp').find('rallp')
    
    # Configuration files
    default_model_path = os.path.join(pkg_share, 'urdf', 'rallp3.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'config', 'config2.rviz')
    bridge_params = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')
    twist_mux_params = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    torque_vectoring_params = os.path.join(pkg_share, 'config', 'torque_vectoring.yaml')

    # Read robot description
    with open(default_model_path, 'r') as f:
        robot_description_content = f.read()

    # Common parameters including sim time and robot description
    shared_params = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    # BlueDot Control Node (no condition for now)
    blue_dot = Node(
        package='rallp',
        executable='blue_dot_control2.py',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[shared_params],
        output='screen'
    )

    # RViz node (no condition for now)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config_path],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # Gazebo process
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-r', os.path.join(pkg_share, 'world', 'warehouse.sdf')],
        output='screen'
    )

    # Spawn robot entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-entity', 'rallp2', '-file', default_model_path, '-z', '1'],
        output='screen'
    )

    # Gazebo Bridge
    gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_params}],
        output='screen'
    )

    # Twist Mux Node
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[
            twist_mux_params,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'topics.nav_vel.priority': 200,
                'topics.blue_dot_vel.priority': 100
            }
        ],
        remappings=[
            ('/cmd_vel_out', '/cmd_vel_raw'),
            ('/cmd_vel_nav', '/cmd_vel_nav')
        ],
        output='screen'
    )

    # Torque Vectoring Nodes
    use_python = LaunchConfiguration('use_python_torque_node')

    torque_vectoring_cpp_node = Node(
        package='rallp',
        executable='torque_vectoring_node',
        name='torque_vectoring_node',
        parameters=[
            torque_vectoring_params,
            {'enable_torque_vectoring': LaunchConfiguration('enable_torque_vectoring'),
             'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen',
        condition=UnlessCondition(use_python)
    )

    torque_vectoring_python_node = Node(
        package='rallp',
        executable='torque_vectoring_node.py',
        name='torque_vectoring_node_py',
        parameters=[
            torque_vectoring_params,
            {'enable_torque_vectoring': LaunchConfiguration('enable_torque_vectoring'),
             'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen',
        condition=IfCondition(use_python)
    )

    # Torque Vectoring Visualizer
    torque_vectoring_visualizer = Node(
        package='rallp',
        executable='torque_vectoring_visualizer.py',
        name='torque_vectoring_visualizer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_visualization'))
    )

    return LaunchDescription([
        enable_torque_vectoring_arg,
        use_python_torque_node_arg,
        enable_visualization_arg,
        use_sim_time_arg,

        LogInfo(msg="Starting RALLP_V2 with Torque Vectoring System"),
        LogInfo(msg=[('Using Python torque node: '), use_python]),

        # Core nodes
        blue_dot,
        robot_state_publisher_node,
        rviz_node,
        gazebo,
        spawn_entity,
        gazebo_bridge,
        twist_mux,

        # Torque vectoring nodes
        torque_vectoring_cpp_node,
        torque_vectoring_python_node,

        # Visualization
        torque_vectoring_visualizer,

        LogInfo(msg="RALLP_V2 with Torque Vectoring System started"),
    ])
