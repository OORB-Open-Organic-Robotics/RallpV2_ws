## Cloning the Workspace

Clone the project repository into your ROS 2 workspace directory:
```bash
cd <ws_name>
git clone -b LidarOnly --single-branch https://github.com/OORB-Open-Organic-Robotics/RallpV2_ws.git
```

## Building the Workspace

Build the workspace with colcon:
```bash
cd <ws_name>
colcon build --symlink-install
source install/setup.bash
```

## Running the RPLIDAR Node and Visualization

Launch the RPLIDAR node and RViz for visualization:
```bash
ros2 launch rplidar_ros view_rplidar.launch.py
```

## RViz Configuration

Ensure:
- The fixed frame in RViz is set to `laser`.
- LaserScan topic `/scan` is visualized as red points.


## Note:
`fake_odom_pub`package is incomplete, but since the visualization is based only on `rplidar_ros2` package, it will be executed without issues.


