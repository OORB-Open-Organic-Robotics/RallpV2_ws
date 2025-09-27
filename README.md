# RallPV2 Complete Workspace

## Overview
The `RallPV2_Complete` workspace combines packages from multiple branches (`sensor_fusion`, `teleop_twist`, `torque_vectoring`, `web_dev`) to provide a comprehensive ROS2 environment for the RallP robot. This README currently covers the `sensor_fusion` branch, with additional branches to be added.

## Sensor Fusion Branch
The `navigation` package provides core autonomous navigation capabilities, integrating LiDAR, IMU, encoders, and cameras for sensor fusion, localization, mapping, and path planning. The `rallp` package supports robot control and simulation, including configurations for `ros2_control`.

### Features
- **Sensor Fusion**: Combines LiDAR, IMU, and odometry for robust localization.
- **Mapping**: Generates 2D maps using SLAM techniques.
- **Localization**: Estimates robot position and orientation in dynamic environments.
- **Path Planning**: Plans and follows safe paths avoiding obstacles.
- **Launch Files**:
  - `navigation/launch/navigation.launch.py`: Main navigation with mapping, localization, and path planning.
  - `navigation/launch/mapping.launch.py`: SLAM mapping.
  - `navigation/launch/localization.launch.py`: Sensor fusion-based localization.
  - `navigation/launch/spawnrobot.launch.py`: Robot spawning in simulation.
  - `rallp/launch/main_launch2.launch.py`: Main launch for sensor_fusion functionality.
  - `rallp/launch/blue_dot_control.py`: Control functionality (check redundancy).
  - `rallp/launch/blue_dot_control2.py`: Alternative control (check redundancy).
  - `rallp/launch/sllidar_a2m8_launch.py`: LiDAR driver.

## Setup
1. Build the workspace:
   ```bash
   cd ~/rallpV2_ws
   colcon build
   source install/setup.bash



## Teleop Twist Branch
The `teleop_twist` branch adds joystick-based teleoperation to the RallP robot using `joy` and `teleop_twist_joy`. It includes:
- **Joystick Control**: Configured via `config/joystick.yaml` and launched with `launch/joystick.launch.py`.
- **Enhanced Launch**: `main_launch2.launch.py` adds `twist_mux` for combining teleoperation and navigation commands.
- **LiDAR Nodes**: `sllidar_client.cpp` and `sllidar_node.cpp` for LiDAR data processing.
- **Launch Files**:
  - `rallp/launch/joystick.launch.py`: Launches joystick teleoperation.
  - `rallp/launch/main_launch2.launch.py`: Updated for teleoperation support.



  ## Web Dev Branch
The `web_dev` branch adds a web-based interface for controlling and monitoring the RallP robot using `app.js`, `index.html`, `styles.css`, and `oorb_logo.png`. It uses `rosbridge_server` for ROS2 communication.
- **Web Interface**: Run `ros2 launch rosbridge_server rosbridge_websocket_launch.xml` and `node app.js`, then access via browser (e.g., http://localhost:3000).
- **ROS2 Integration**: Retains `sensor_fusion` and `teleop_twist` functionality with `rallp` package.
- **Launch Files**:
  - `rallp/launch/main_launch2.launch.py`: Core simulation with `twist_mux`.
  - `rallp/launch/joystick.launch.py`: Joystick teleoperation.