# RallP Robot Workspace

This workspace (`~/rallpV2_ws/`) integrates multiple branches for the RallP robot project using ROS2 Jazzy. It combines `sensor_fusion`, `teleop_twist`, `web_dev`, and `torque_vectoring` branches into two packages: `rallp` and `rallp_torque`. Below is a description of each package and component, their functionality, and how to run them.

## Workspace Overview
- **Packages**:
  - `rallp`: Core package for robot control, sensor fusion, and teleoperation.
  - `rallp_torque`: Dedicated package for torque vectoring control and testing.
  - `navigation`: Dedicated for sensor fusion algorithms.
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
   
## navigation Package
This package provides autonomous navigation capabilities for the robot. It integrates multiple sensors (LiDAR, IMU, encoders, and cameras) to perform sensor fusion, enabling accurate localization, mapping, and path planning.

  - **Functionality**:
    - **Sensor Fusion**: Combines LiDAR, IMU, and odometry data for robust localization.
    - **Mapping**: Generates 2D maps in real-time using SLAM techniques.
    - **Localization**: Estimates the robot's position and orientation in dynamic environments.
    - **Path Planning**: Plans and follows safe paths avoiding obstacles.
  - **Launch Files**:
      - `navigation.launch.py` – Main launch file for the navigation package (includes mapping, localization, and path planning nodes).
      - `spawn.launch.py` – Spawn the robot in simulation.
      - `mapping.launch.py` – Start mapping nodes for the navigation stack.
      - `localization.launch.py` – Start localization nodes using sensor fusion.


## Web Interface
The `web_dev`-related files add a browser-based interface for controlling and monitoring the RallP robot.

- **Functionality**:
  - Uses `app.js`, `index.html`, `styles.css`, and `oorb_logo.png` to provide a web dashboard.
  - Communicates with ROS2 via `rosbridge_server` (WebSocket interface).
  - Controls the robot via `/cmd_vel` and visualizes data like `/scan` (LiDAR).

## Setup Instructions


### ### Prerequisites

First, ensure you have the following installed on **Ubuntu 24.04 LTS**:

1.  **[ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)**: Follow the official "Desktop Install" instructions.
2.  **[Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu)**
3.  **[Navigation2](https://navigation.ros.org/getting_started/index.html)**: Installation is usually included with the `ros-jazzy-desktop` installation.

### ### Step 1: Create Your Workspace
Create a ROS 2 workspace to house the project.

```bash
mkdir ~/rallp_ws
cd ~/rallp_ws
```


### Step 2: Clone the Repository
Clone this repository directly into your workspace.

```bash
git clone https://github.com/OORB-Open-Organic-Robotics/rallp_v2.git rallp
```


### Step 3: Install All Dependencies
The recommended way to install all required ROS packages is with rosdep.

```bash
cd ~/rallp_ws
sudo apt update
rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
```

Next, install the bluedot Python library for Bluetooth control.

```bash
sudo apt install pipx
pipx ensurepath
pipx install bluedot
```


### Step 4: Configure Your Environment
You need to tell ROS where to find the bluedot library. The following command will automatically add the correct path to your shell configuration file (.bashrc).

```bash
echo 'export PYTHONPATH="$HOME/.local/share/pipx/venvs/bluedot/lib/$(ls $HOME/.local/share/pipx/venvs/bluedot/lib/)/site-packages:$PYTHONPATH"' >> ~/.bashrc
source ~/.bashrc
```


### Step 5: Build the Workspace
Compile the code using colcon.

```bash
cd ~/rallp_ws
colcon build --symlink-install
```

## 🚀 Usage
Before running any ROS 2 commands, you must source your workspace in every new terminal.

```bash
cd ~/rallp_ws
source install/setup.bash
```

### Launching the Simulation
This package includes two main launch files:

1. main_launch.launch.py: Uses a basic rallp.urdf model.

2. main_launch2.launch.py: Uses the more detailed rallp.urdf.xacro model with 3D meshes (recommended).

To launch the recommended simulation:

```bash
ros2 launch rallp main_launch2.launch.py
```


### Controlling the Robot
Once the simulation is running, open a new terminal, source your workspace again, and run the following command to control the robot with your keyboard:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
```

## Web Interface
This package adds a browser-based interface for controlling and monitoring the RallP robot.

- **Functionality**:
  - Uses `app.js`, `index.html`, `styles.css`, and `oorb_logo.png` to provide a web dashboard.
  - Communicates with ROS2 via `rosbridge_server` (WebSocket interface).
  - Likely controls the robot via `/cmd_vel` and visualizes data like `/scan` (LiDAR).

---

### How to Run the Web Interface

1. **Run ROS2 and rosbridge**

   On your robot or laptop with ROS 2 installed, in the workspace directory:

   ```bash
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
   ```

  By default, rosbridge server runs at ws://localhost:9090.

2.  **Serve the Web Interface**

  Navigate in your terminal to the workspace directory containing the web interface files (app.js, index.html, etc.), then run a simple HTTP server:

  ```bash
  python3 -m http.server 8000
  ```

3. **Open the Web Interface**

  In your web browser, go to:

  http://localhost:8000

  ### Usage
  
  - **Control Panel**
    Use the **Control Panel** tab to publish robot movement commands.

    The **Active topic** defaults to `/cmd_vel` but can be changed by typing a new topic and clicking Set Topic.

    Control buttons — Forward, Backward, Left, Right, and Stop — send velocity commands using the chosen topic.

    The status bar shows connection status and the last command sent.
    
  - **Real-Time Monitoring**
    Switch to the Real-Time Monitoring tab to subscribe dynamically to sensor topics.

    Enter one or more topic names separated by commas (e.g., `/imu`, `/joint_states`, `/image_raw`).

    Click Add Topic to start subscription.

    Panels will be created for each topic:

    Numeric topics (e.g., IMU, joint states) show real-time graphs of numeric fields.

    Image topics (e.g., `/image_raw`) render live images.

    The layout adjusts dynamically as you add topics.
