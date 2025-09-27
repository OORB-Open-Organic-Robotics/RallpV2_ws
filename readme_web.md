# Oorb Rall-p ROS2 Web Interface

A professional web-based interface for controlling your Rall-p robot using ROS 2 Humble and monitoring real-time sensor data. Features include:

- **Control Panel** for publishing commands to `/cmd_vel` or any custom ROS2 topic.
- **Real-Time Monitoring** with dynamic subscription to sensor topics (`/imu`, `/joint_states`, `/image_raw` etc), displaying graphs and images.
- Modular, responsive UI with professional styling.
- Uses `rosbridge_server` and `roslibjs` for ROS2 communication.
- Graph rendering via Chart.js.
- Easy web navigation between control and monitoring panels.

***

## Repository Contents

| File          | Description                                    |
|---------------|------------------------------------------------|
| `index.html`  | Main HTML page for the interface.              |
| `styles.css`  | CSS styling for layout, colors, fonts, and responsiveness. |
| `app.js`      | Modular JavaScript handling UI interactions, ROS connection, topics, and live data display. |
| `oorb_logo.png` | Your robot branding logo (optional, place in same folder). |

***

## Prerequisites

- **ROS 2 Humble** environment with `rosbridge_server` installed and running.
- Modern web browser (Chrome, Firefox, Edge, Safari).
- Python 3 (for serving files via HTTP).
- Internet connection for loading external libraries (`roslibjs`, `Chart.js`, and Google Fonts).

***

## Installation & Setup

1. **Clone or Download Files**

   Download or clone this repository and place all files (`index.html`, `styles.css`, `app.js`, and optionally `oorb-logo.png`) in the same directory.

2. **Run ROS2 and rosbridge**

   On your robot or laptop with ROS 2 installed:

ros2 launch rosbridge_server rosbridge_websocket_launch.xml


By default, rosbridge server runs at `ws://localhost:9090`.

3. **Serve the Web Interface**

Navigate in your terminal to the folder containing the files, then run a simple HTTP server:

python3 -m http.server 8000


4. **Open the Web Interface**

In your web browser, go to:

http://localhost:8000



***

## Usage

### Control Panel

- Use the **Control Panel** tab to publish robot movement commands.
- The **Active topic** defaults to `/cmd_vel` but can be changed by typing a new topic and clicking **Set Topic**.
- Control buttons: Forward, Backward, Left, Right, and Stop send velocity commands using the chosen topic.
- Status bar shows connection status and last command sent.

### Real-Time Monitoring

- Switch to **Real-Time Monitoring** tab to subscribe dynamically to sensor topics.
- Enter one or more topic names separated by commas (e.g., `/imu, /joint_states, /image_raw`).
- Click **Add Topic** to start subscription.
- Panels will be created for each topic:
- Numeric topics (e.g., IMU, joint states) show real-time graphs of numeric fields.
- Image topics (e.g., `/image_raw`) render live images.
- The layout adjusts dynamically as you add topics.

***
