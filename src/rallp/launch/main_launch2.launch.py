from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os
import shutil
import xacro

def generate_launch_description():

    use_ros2_control = LaunchConfiguration('use_ros2_control')
    cmd_vel_target = LaunchConfiguration('cmd_vel_target')
    world = LaunchConfiguration('world')

    # Blue Dot Node
    blue_dot = Node(
        package='rallp',
        executable='blue_dot_control2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('enable_bluedot'))
    )
    
    # Paths to URDF and RViz config
    pkg_share = FindPackageShare(package='rallp').find('rallp')
    default_model_path = os.path.join(pkg_share, 'urdf', 'rallp.urdf.xacro')
    fallback_model_path = os.path.join(pkg_share, 'urdf', 'rallp3.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'config', 'config2.rviz')
    bridge_params = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')
    twist_mux_params = os.path.join(pkg_share,'config','twist_mux.yaml')

    # Generate URDF from XACRO
    try:
        robot_description_content = xacro.process_file(default_model_path).toxml()
    except Exception:
        with open(fallback_model_path, 'r') as urdf_file:
            robot_description_content = urdf_file.read()

    # Robot description for robot_state_publisher
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )
    # Twist Mux node
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",  # Added explicit name
        output='screen',
        parameters=[
            twist_mux_params, 
            {'use_sim_time': True}
        ],
        remappings=[
            ('/cmd_vel_out', cmd_vel_target)
        ]
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': True}],
    )

    # Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-r', world],
        output='screen'
    )

    # Spawn node (delay to ensure Gazebo server is ready)
    spawn_entity = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_entity',
                arguments=[
                    '-entity', 'rallp',
                    '-topic', '/robot_description',
                    '-z', '1'
                ],
                output='screen'
            )
        ]
    )

    # Controller spawners (delay to ensure Gazebo loads first)
    joint_state_broadcaster_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ],
        condition=IfCondition(use_ros2_control)
    )

    diff_drive_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['diff_cont', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ],
        condition=IfCondition(use_ros2_control)
    )

    # ros_gz_bridge node
    gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',  # Added explicit name
        parameters=[{'config_file': bridge_params}],  # Better parameter passing
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot model file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        DeclareLaunchArgument(
            name='enable_bluedot',
            default_value='False',
            description='Enable Blue Dot controller'
        ),
        DeclareLaunchArgument(name='use_ros2_control', default_value='False', description='Load ros2_control components'),
        DeclareLaunchArgument(name='cmd_vel_target', default_value='/cmd_vel_mux', description='Twist mux output topic'),
        DeclareLaunchArgument(
            name='world',
            default_value=os.path.join(pkg_share, 'world', 'warehouse.sdf'),
            description='Absolute path to Gazebo world file'
        ),
        blue_dot,
        twist_mux,
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
        gazebo_bridge,
        joint_state_broadcaster_spawner,
        diff_drive_spawner,
        rviz_node,
    ])
