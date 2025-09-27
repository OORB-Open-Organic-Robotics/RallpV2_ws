#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    pkg_share = get_package_share_directory('rallp')
    
    # Declare launch arguments
    use_python_node_arg = DeclareLaunchArgument(
        'use_python_node',
        default_value='false',
        description='Use Python implementation instead of C++')
    
    enable_torque_vectoring_arg = DeclareLaunchArgument(
        'enable_torque_vectoring',
        default_value='true',
        description='Enable torque vectoring algorithm')
    
    # Configuration file
    torque_vectoring_config = os.path.join(pkg_share, 'config', 'torque_vectoring.yaml')
    
    # Torque vectoring node (C++ version)
    torque_vectoring_cpp_node = Node(
        package='rallp',
        executable='torque_vectoring_node',
        name='torque_vectoring_node',
        parameters=[
            torque_vectoring_config,
            {'enable_torque_vectoring': LaunchConfiguration('enable_torque_vectoring')}
        ],
        output='screen',
        condition=None  # Will be set based on use_python_node parameter
    )
    
    # Torque vectoring node (Python version)
    torque_vectoring_python_node = Node(
        package='rallp',
        executable='torque_vectoring_node.py',
        name='torque_vectoring_node',
        parameters=[
            torque_vectoring_config,
            {'enable_torque_vectoring': LaunchConfiguration('enable_torque_vectoring')}
        ],
        output='screen',
        condition=None  # Will be set based on use_python_node parameter
    )
    
    # Debug visualization node (optional)
    debug_visualizer_node = Node(
        package='rallp',
        executable='torque_vectoring_visualizer.py',
        name='torque_vectoring_visualizer',
        output='screen',
        condition=None  # Can be enabled separately
    )
    
    return LaunchDescription([
        use_python_node_arg,
        enable_torque_vectoring_arg,
        
        LogInfo(msg="Starting Torque Vectoring System"),
        
        # Note: In a real implementation, you'd use conditional logic here
        # For now, both nodes are included - comment out the one you don't want
        torque_vectoring_cpp_node,
        # torque_vectoring_python_node,
        
        LogInfo(msg="Torque Vectoring System started"),
    ])
