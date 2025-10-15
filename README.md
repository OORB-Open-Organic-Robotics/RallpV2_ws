# RallP Robot Workspace

End-to-end simulation, control, and testing for the RallP platform on ROS 2
Jazzy. This workspace merges functionality from several historical branches
(`sensor_fusion`, `teleop_twist`, `web_dev`, and `torque_vectoring`) into three
ROS packages:

- `rallp` – core robot description, Gazebo simulation, teleoperation, BlueDot
  integration, and the web dashboard assets.
- `navigation` – Nav2-based mapping, localization, and navigation flows.
- `rallp_torque` – torque-vectoring controllers, analysis scripts, and demos.

The repository also ships documentation, test scripts, and static assets used
by the torque-vectoring reports and browser dashboard.

---

## Repository Layout

```
RallpV2_ws/
├── src/
│   ├── rallp/              Core robot package (URDF/Xacro, launch, teleop)
│   ├── navigation/         Nav2 wrappers, maps, configs
│   └── rallp_torque/       Torque vectoring nodes, configs, launch files
├── app.js, index.html, …   Web dashboard assets
├── TORQUE_VECTORING_*.md   Torque analysis documentation
├── simple_torque_test.py   Stand-alone torque scripts
└── README.md               This guide
```

Each package contains its own `package.xml`, `setup.py`, and launch/config
directories and can be built with `colcon`.

---

## Quick Start

```bash
# 1) Create a workspace and clone the repo
mkdir -p ~/rallp_ws/src
cd ~/rallp_ws/src
git clone https://github.com/OORB-Open-Organic-Robotics/rallp_v2.git RallpV2_ws

# 2) Install dependencies (rosdep + extra apt packages)
cd ~/rallp_ws
sudo apt update
rosdep init   # skip if already run on your machine
rosdep update
rosdep install --from-paths src -y --ignore-src
sudo apt install ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
                 ros-jazzy-rosbridge-server ros-jazzy-xacro

# 3) Optional: BlueDot support for Bluetooth teleop
sudo apt install pipx
pipx ensurepath
pipx install bluedot
echo 'export PYTHONPATH="$HOME/.local/share/pipx/venvs/bluedot/lib/$(ls $HOME/.local/share/pipx/venvs/bluedot/lib/)/site-packages:$PYTHONPATH"' >> ~/.bashrc
source ~/.bashrc

# 4) Build
cd ~/rallp_ws
colcon build --symlink-install

# 5) Source before every ROS command
source ~/rallp_ws/install/setup.bash
```

> **Tip:** keep separate terminals for Gazebo, teleop, nav2, and web-serving.
> Run `source install/setup.bash` in each new shell.

---

## Simulation & Control (package `rallp`)

### Base Gazebo launch

```bash
ros2 launch rallp main_launch2.launch.py
```

`main_launch2.launch.py` loads the URDF/Xacro robot, spawns it in Gazebo
Harmonic, starts the twist mux, robot_state_publisher, ros_gz_bridge, RViz, and
the Gazebo GUI. Key launch arguments:

| Argument             | Default            | Description                                   |
|----------------------|--------------------|-----------------------------------------------|
| `enable_bluedot`     | `False`            | Start the BlueDot teleop node                 |
| `use_ros2_control`   | `False`            | Spawn `joint_state_broadcaster`/`diff_cont`   |
| `cmd_vel_target`     | `/cmd_vel_mux`     | Twist mux output topic                        |

### Teleoperation options

- **Keyboard (default):**
  ```bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
  ```
  The node publishes the most recent keypress until another command or `Ctrl+C`
  is received. Use `k` (or space) to stop.

- **BlueDot:** enable via launch argument:
  ```bash
  ros2 launch rallp main_launch2.launch.py enable_bluedot:=True
  ```
  The BlueDot node publishes to `/cmd_vel` (works out-of-the-box with the
  updated mux). Requires the mobile app pointing at the host.

- **Joystick:** see `ros2 launch rallp joystick.launch.py` along with your
  joystick-specific ROS drivers.

### ros2_control mode

To drive the simulated controller manager instead of the Gazebo diff-drive
plugin:

```bash
ros2 launch rallp main_launch2.launch.py use_ros2_control:=True cmd_vel_target:=/diff_cont/cmd_vel_unstamped
```

Start teleop against the controller topic:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/diff_cont/cmd_vel_unstamped
```

Allow ~5 seconds for the spawners (TimerActions) to bring up
`joint_state_broadcaster` and `diff_cont`. Inspect with
`ros2 control list controllers`.

### Helpful topics & tools

- `ros2 topic echo /cmd_vel` – raw teleop commands.
- `ros2 topic echo /cmd_vel_mux` – twist_mux output (fed to Gazebo or `diff_cont`).
- `ros2 topic hz /odom` – odometry publishing rate.
- `ros2 run tf2_tools view_frames` – TF tree snapshot.

---

## Navigation Package (`navigation`)

This package wraps Nav2 bringup, SLAM Toolbox, and auxiliary launch files.
Ensure the prerequisite debs (`ros-jazzy-nav2-bringup`, `ros-jazzy-slam-toolbox`)
are installed.

Typical workflow:

1. **Spawn the robot in Gazebo (if not already running):**
   ```bash
   ros2 launch rallp main_launch2.launch.py
   ```

2. **Launch mapping (SLAM Toolbox):**
   ```bash
   ros2 launch navigation mapping.launch.py
   ```
   Generates a 2D map on `/map`. Save with `ros2 run nav2_map_server map_saver_cli -f ~/maps/rallp_map`.

3. **Localization-only launch (AMCL + Nav2):**
   ```bash
   ros2 launch navigation localization.launch.py
   ```

4. **Full navigation stack:**
   ```bash
   ros2 launch navigation navigation.launch.py
   ```
   This includes map server, AMCL, Nav2 Planner/Controller/BT Navigator, and RViz
   panels configured via `config/` and `maps/`.

5. **Spawn utility without full stack:**
   ```bash
   ros2 launch navigation spawnrobot.launch.py
   ```
   Requires the `xacro` executable (`ros-jazzy-xacro`).

Refer to `src/navigation/config/` for parameter files and `src/navigation/maps/`
for sample maps.

---

## Torque Vectoring Package (`rallp_torque`)

Focused on traction control research. Key entry points:

- `ros2 launch rallp_torque main_launch_torque_vectoring.launch.py` – Gazebo
  simulation plus torque controller nodes.
- `ros2 launch rallp_torque torque_vectoring_launch.py` – brings up only the
  torque logic alongside minimal simulation.
- Python scripts at the workspace root (`simple_torque_test.py`,
  `standalone_torque_test.py`) exercise algorithms without ROS.

Configuration lives under `src/rallp_torque/config/` and documentation in
`TORQUE_VECTORING_README.md`, `TORQUE_VECTORING_TEST_STRATEGY.md`, and related
reports.

---

## Web Dashboard

Files (`index.html`, `app.js`, `styles.css`, `oorb_logo.png`) provide a
browser-based controller and monitoring panel powered by rosbridge/roslibjs.
The UI is split into **Control Panel** (publishes geometry commands) and
**Real-Time Monitoring** (dynamic subscriptions with Chart.js visualization).

### Run the dashboard

```bash
# Terminal 1 – rosbridge websocket server
source ~/rallp_ws/install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# Terminal 2 – serve static files
cd ~/rallp_ws/src/RallpV2_ws
python3 -m http.server 8000
```

Open `http://localhost:8000/` in a modern browser. Ensure the machine has
internet access for the hosted `roslibjs`, Chart.js, and Google Fonts assets.

---

## Testing & Utilities

- `ros2 test` targets are sparse; most validation lives in Python scripts under
  the repo root and in `src/rallp_torque/test/`.
- Use `colcon test` to execute available unit tests.
- Additional guides such as `TESTING_GUIDE.md`, `VALIDATION_REPORT.md`, and
  torque documentation describe manual procedures.

---

## Troubleshooting

- **`Package 'nav2_bringup' not found`** – install `ros-jazzy-nav2-bringup`.
- **`Package 'slam_toolbox' not found`** – install `ros-jazzy-slam-toolbox`.
- **`file not found: 'xacro'`** – install `ros-jazzy-xacro`.
- **Gazebo reports `cmd_vel` but robot does not move** – Ensure teleop publishes
  to `/cmd_vel` (default). Use `ros2 topic echo /cmd_vel_mux` to confirm mux
  output.
- **BlueDot not connecting** – verify the Bluetooth device is paired and the
  BlueDot app points at the host IP.
- **Web dashboard cannot connect** – confirm rosbridge is running on port 9090
  and that the browser console shows a successful websocket connection.

---

## Contributing

1. Create a feature branch.
2. Run `colcon build` and relevant launch files to validate changes.
3. Update documentation/tests when behavior changes.
4. Submit a pull request describing the scenario and verification steps.

---

## License

See `LICENSE` for the full Apache 2.0 license text.

