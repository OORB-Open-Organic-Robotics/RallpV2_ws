# RallP Robot Workspace

This workspace (`~/rallpV2_ws/`) integrates multiple branches for the RallP robot project using ROS2 Jazzy. It combines `sensor_fusion`, `teleop_twist`, `web_dev`, and `torque_vectoring` branches into two packages: `rallp` and `rallp_torque`. Below is a description of each package and component, their functionality, and how to run them.

## Workspace Overview
- **Packages**:
  - `rallp`: Core package for robot control, sensor fusion, and teleoperation.
  - `rallp_torque`: Dedicated package for torque vectoring control and testing.
- **Web Interface**: Browser-based control and visualization using `app.js`, `index.html`, `styles.css`, and `oorb_logo.png`.
- **Environment**: ROS2 Jazzy, built with `colcon`, using a virtual environment (`venv`).

## rallp Package
The `rallp` package integrates `sensor_fusion`, `teleop_twist`, and `web_dev` functionality, providing core robot operations.

- **Functionality**:
  - **Sensor Fusion**: Combines data from LiDAR (`sllidar_client.cpp`, `sllidar_node.cpp`), IMU, and other sensors for navigation and mapping (`nav2_bringup`, `slam_toolbox`).
  - **Teleoperation**: Supports joystick control (`joystick.launch.py`, `twist_mux`) and BlueDot app control (`blue_dot_control2.py`).
  - **Simulation**: Uses Gazebo for simulation (`ros_gz_sim`, `ros_gz_bridge`) with URDF (`rallp.urdf.xacro`) and SDF (`rallp2.sdf`) models.
  - **Configuration**: Includes `config/controllers.yaml`, `config/ekf.yaml`, `config/twist_mux.yaml`, etc.
  - **Launch Files**:
    - `main_launch2.launch.py`: Core simulation with `twist_mux` for teleoperation.
    - `joystick.launch.py`: Joystick-based teleoperation.
    - `sllidar_a2m8_launch.py`: LiDAR driver.

## rallp_torque Package
The `rallp_torque` package implements torque vectoring control from the `torque_vectoring` branch, enhancing traction and stability.

- **Functionality**:
  - **Torque Vectoring**: Adjusts wheel torques dynamically (`torque_vectoring_node.py`, `torque_vectoring_tester.py`, `torque_vectoring_visualizer.py`).
  - **Testing**: Includes test suites (`simple_torque_test.py`, `standalone_torque_test.py`, `torque_redistribution_test_suite.py`, etc.) for validation.
  - **Configuration**: Uses `torque_vectoring.yaml`, `torque_vectoring_controllers.yaml`, and `torque_vectoring_test_config.json`.
  - **Launch Files**:
    - `main_launch_torque_vectoring.launch.py`: Main simulation with torque vectoring.
    - `torque_vectoring_launch.py`: Torque-specific launch.

## Web Interface
The `web_dev` branch adds a browser-based interface for controlling and monitoring the RallP robot.

- **Functionality**:
  - Uses `app.js`, `index.html`, `styles.css`, and `oorb_logo.png` to provide a web dashboard.
  - Communicates with ROS2 via `rosbridge_server` (WebSocket interface).
  - Likely controls the robot via `/cmd_vel` and visualizes data like `/scan` (LiDAR).

## Setup Instructions
