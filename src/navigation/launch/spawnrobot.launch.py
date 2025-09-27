import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import TextSubstitution
def generate_launch_description():
    # =======================
    # Package paths
    # =======================
    pkg_rallp = get_package_share_directory('rallp')
    pkg_navigation = get_package_share_directory('navigation')

    # =======================
    # Launch arguments
    # =======================
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Launch RViz'
    )

    world_arg = DeclareLaunchArgument(
        'world', default_value='warehouse.sdf',
        description='World file to load in Gazebo'
    )

    model_arg = DeclareLaunchArgument(
        'model', default_value='rallp3.urdf',
        description='URDF robot description file'
    )

    # =======================
    # File paths with substitutions
    # =======================
    urdf_file_path = PathJoinSubstitution([
        pkg_rallp, 'urdf', LaunchConfiguration('model')
    ])

    rviz_config_path = PathJoinSubstitution([
        pkg_rallp, 'config', 'config2.rviz'
    ])

    bridge_params_path = os.path.join(pkg_rallp, 'config', 'gz_bridge.yaml')
    twist_mux_params_path = os.path.join(pkg_rallp, 'config', 'twist_mux.yaml')
    ekf_params_path = os.path.join(pkg_navigation, 'config', 'ekf.yaml')

    # =======================
    # World launch (Gazebo Sim)
    # =======================
    gz_world = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-r',
             PathJoinSubstitution([pkg_rallp, 'world', LaunchConfiguration('world')])],
        output='screen'
    )

    # =======================
    # Spawn robot
    # =======================
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-entity', 'rallp2',
            '-file', urdf_file_path,
            '-z', '1.0'
        ],
        output='screen'
    )

    # =======================
    # Robot state publisher
    # =======================
    robot_state_publisher_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='screen',
    parameters=[
        {
            'robot_description': Command([
                TextSubstitution(text='xacro '),  # note l'espace obligatoire ici
                urdf_file_path  # doit être un PathJoinSubstitution ou similaire
            ]),
            'use_sim_time': True
        },
    ],
    remappings=[
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static')
    ]
)


    # =======================
    # RViz (optional)
    # =======================
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     arguments=['-d', rviz_config_path],
    #     condition=IfCondition(LaunchConfiguration('rviz')),
    #     parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    #     output='screen'
    # )

    # =======================
    # ROS-GZ Bridge
    # =======================
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_params_path}],
        output='screen'
    )

    # =======================
    # EKF Localization
    # =======================
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_params_path, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # =======================
    # Twist Mux
    # =======================
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params_path, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped'),
            ('/cmd_vel_nav', '/cmd_vel_nav')
        ],
        output='screen'
    )

    # =======================
    # Blue dot control
    # =======================
    blue_dot_node = Node(
        package='rallp',
        executable='blue_dot_control2.py',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # =======================
    # Launch description
    # =======================
    ld = LaunchDescription()

    # Declare args
    ld.add_action(use_sim_time_arg)
    ld.add_action(rviz_arg)
    ld.add_action(world_arg)
    ld.add_action(model_arg)

    # Add processes & nodes
    ld.add_action(gz_world)
    ld.add_action(spawn_robot)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(gz_bridge)
    ld.add_action(ekf_node)
    ld.add_action(twist_mux_node)
    ld.add_action(blue_dot_node)

    return ld
