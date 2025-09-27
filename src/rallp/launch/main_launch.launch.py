from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Blue Dot Node
    blue_dot = Node(
        package='rallp',
        executable='blue_dot_control2.py',
        output='screen',
    )

    # Paths to URDF and configurations
    pkg_share = FindPackageShare(package='rallp').find('rallp')
    default_model_path = os.path.join(pkg_share, 'urdf', 'rallp3.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'config', 'config2.rviz')
    bridge_params = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')
    twist_mux_params = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    # Robot description
    with open(default_model_path, 'r') as f:
        robot_description_content = f.read()

    # Shared parameters
    shared_params = {
        'use_sim_time': True,
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

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

    # EKF Node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[os.path.join(get_package_share_directory("robot_localization"), 'config', 'ekf.yaml')],
        output='screen'
    )

    # Twist Mux
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
            ('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped'),
            ('/cmd_vel_nav', '/cmd_vel_nav')
        ],
        output='screen'
    )


    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        blue_dot,
        robot_state_publisher_node,
        rviz_node,
        gazebo,
        spawn_entity,
        gazebo_bridge,
        # ekf_node,
        twist_mux,
    ])