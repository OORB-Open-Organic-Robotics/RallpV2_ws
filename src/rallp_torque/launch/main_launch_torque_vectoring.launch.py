#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Declare arguments
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

    # Get package directory
    pkg_share = FindPackageShare(package='rallp').find('rallp')
    
    # Configuration files
    default_model_path = os.path.join(pkg_share, 'urdf', 'rallp3.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'config', 'config2.rviz')
    bridge_params = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')
    twist_mux_params = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    torque_vectoring_params = os.path.join(pkg_share, 'config', 'torque_vectoring.yaml')

    # Robot description
    with open(default_model_path, 'r') as f:
        robot_description_content = f.read()

    # Shared parameters
    shared_params = {
        'use_sim_time': True,
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    # BlueDot Control Node
    blue_dot = Node(
        package='rallp',
        executable='blue_dot_control2.py',
        output='screen',
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[shared_params],
        output='screen'
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config_path],
        output='screen'
    )

    # Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-r', os.path.join(pkg_share, 'world', 'warehouse.sdf')], 
        output='screen'
    )

    # Spawn Entity
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

    # Twist Mux (Updated for torque vectoring)
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[
            twist_mux_params,
            {
                'use_sim_time': True,
                'topics.nav_vel.priority': 200,  # Highest priority for Nav2
                'topics.blue_dot_vel.priority': 100  # Lower priority for manual control
            }
        ],
        remappings=[
            ('/cmd_vel_out', '/cmd_vel_raw'),  # Send raw commands to torque vectoring
            ('/cmd_vel_nav', '/cmd_vel_nav')
        ],
        output='screen'
    )

    # Torque Vectoring Node (C++)
    torque_vectoring_cpp_node = Node(
        package='rallp',
        executable='torque_vectoring_node',
        name='torque_vectoring_node',
        parameters=[
            torque_vectoring_params,
            {'enable_torque_vectoring': LaunchConfiguration('enable_torque_vectoring')}
        ],
        output='screen',
        condition=None  # Will be active by default
    )

    # Torque Vectoring Node (Python) - Alternative
    torque_vectoring_python_node = Node(
        package='rallp',
        executable='torque_vectoring_node.py',
        name='torque_vectoring_node_py',
        parameters=[
            torque_vectoring_params,
            {'enable_torque_vectoring': LaunchConfiguration('enable_torque_vectoring')}
        ],
        output='screen',
        condition=None  # Can be enabled instead of C++ version
    )

    # Torque Vectoring Visualizer
    torque_vectoring_visualizer = Node(
        package='rallp',
        executable='torque_vectoring_visualizer.py',
        name='torque_vectoring_visualizer',
        output='screen',
        condition=None  # Can be enabled for debugging
    )

    return LaunchDescription([
        # Arguments
        enable_torque_vectoring_arg,
        use_python_torque_node_arg,
        enable_visualization_arg,
        
        # Launch info
        LogInfo(msg="Starting RALLP_V2 with Torque Vectoring System"),
        
        # Core system nodes
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        blue_dot,
        robot_state_publisher_node,
        rviz_node,
        gazebo,
        spawn_entity,
        gazebo_bridge,
        twist_mux,
        
        # Torque vectoring system
        torque_vectoring_cpp_node,
        # Uncomment for Python version instead:
        # torque_vectoring_python_node,
        
        # Uncomment for visualization:
        # torque_vectoring_visualizer,
        
        LogInfo(msg="RALLP_V2 with Torque Vectoring System started"),
    ])
