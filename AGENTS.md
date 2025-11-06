# Repository Guidelines

## Project Structure & Module Organization
This workspace follows the standard ROS 2 overlay layout. Core robot assets live in `src/rallp` (URDF/Xacro, launches, teleop nodes) while navigation flows sit in `src/navigation`. Torque-vectoring controllers, configs, and analysis scripts are under `src/rallp_torque`. Browser dashboard files (`app.js`, `index.html`, `styles.css`) and static imagery live at the repository root. Generated `build/`, `install/`, and `log/` directories are created by `colcon`; keep them out of commits. Technical reports and playbooks are provided as `TORQUE_VECTORING_*.md` and `VALIDATION_REPORT.md` for quick reference.

## Build, Test, and Development Commands
Run ROS dependency resolution from the workspace root:  
```bash
rosdep install --from-paths src -y --ignore-src
```
Build the overlay with symlinks to speed iteration:  
```bash
colcon build --symlink-install
```
Source before launching any ROS node:  
```bash
source install/setup.bash
```
Typical runtime entry points include `ros2 launch rallp main_launch2.launch.py` for the Gazebo stack and `ros2 launch navigation mapping.launch.py` for Nav2. Use `ros2 run teleop_twist_keyboard ...` to publish `cmd_vel` during manual tests.

## Coding Style & Naming Conventions
Follow ROS 2 conventions: C++ sources use 2-space indentation and snake_case file names; Python nodes should satisfy PEP 8 and keep module names lowercase with underscores. Launch files favor descriptive CamelCase class names but snake_case filenames (e.g., `main_launch2.launch.py`). YAML parameters under `config/` use lowercase keys and 2-space indents. When editing web assets, keep JS formatted with semicolons and prefer const/let over var. Run `colcon test --packages-select <pkg>` to lint via the package’s declared `ament_*` linters where available.

## Testing Guidelines
Leverage `colcon test` for integrated package tests; inspect results under `log/latest_test`. Torque-vectoring logic can be validated quickly with `python3 simple_torque_test.py` or by launching `ros2 run rallp torque_vectoring_tester.py`. During simulation, monitor `/cmd_vel_mux`, `/wheel_torques`, and `/diagnostics` topics with `ros2 topic echo` and verify control timing using `ros2 topic hz /odom`.

## Commit & Pull Request Guidelines
Commit messages in this project use short, imperative summaries (e.g., `Fix cmd_vel routing`). Keep related changes squashed and avoid committing generated `build/`, `install/`, or `log/` artifacts. Pull requests should link relevant issues, explain testing performed, and attach screenshots or terminal transcripts when altering the dashboard or launch behavior. Confirm `colcon build` and critical launch files succeed before requesting review.
