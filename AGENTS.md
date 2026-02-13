# Repository Guidelines

## Project Structure & Module Organization
Workspaces are split by role. `silver2_ws/` hosts the Stonefish simulator: its ROS 2 package lives in `silver2_stonefish/src/stonefish_silver/` with subfolders such as `launch/` (entry points), `scripts/` (Python control nodes), `scenarios/` (Stonefish `.scn` files), and `data/` (meshes and textures). `stonefish_ros2/` provides the C++ bridge to ROS topics. Visual-inertial estimation lives in `okvis2_ws/src/okvis2/`, where `launch/` contains ready-made launch files and `config/` stores YAML calibration. Logs, build artifacts, and bags stay outside `src/` to keep commits clean.

## Build, Test, and Development Commands
- `cd silver2_ws && colcon build --packages-select stonefish_ros2 stonefish_silver` — build simulator nodes.
- `cd okvis2_ws && colcon build --packages-select okvis --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo` — build OKVIS with the recommended optimization level.
- `source /opt/ros/jazzy/setup.bash && source silver2_ws/install/setup.bash` — load ROS 2 plus simulator overlays before running tools.
- `ros2 launch stonefish_silver silver_simulation.launch.py` — start Stonefish with the default scenario.
- `ros2 launch okvis okvis_aquatic_silver2_stonefish.launch.xml` — launch OKVIS against the Stonefish stereo feeds.

## Coding Style & Naming Conventions
C++ sources follow ROS 2/ament conventions: clang-format (Google style) with 2-space indents, CamelCase class names, and snake_case files (e.g., `ROS2SimulationManager.cpp`). Python scripts under `scripts/` use PEP 8 (4 spaces, lowercase_with_underscores). Keep launch files declarative, and prefer explicit topic names such as `/silver2/stereo_left` to simplify remaps. Update parameter files in YAML with lowerCamelCase keys, mirroring existing entries like `camera_rate`.

## Testing Guidelines
Use `colcon test --packages-select stonefish_silver stonefish_ros2` for simulator checks and `colcon test --packages-select okvis` for estimator unit tests. When adding nodes, create lightweight system checks that verify topic health (`ros2 topic hz /okvis_odometry`). Name new tests `<package>_test.cpp` or `<node>_test.py` and keep them under `src/<pkg>/test/`. Ensure stereo/IMU synchrony is validated before recording bags.

## Commit & Pull Request Guidelines
Recent history uses short descriptive subjects (e.g., `12/17`). Prefer a clearer format: `<area>: <action>` such as `stonefish: convert cameras to mono8`. Limit the subject to 50 characters, wrap body at 72, and reference issues with `Fixes #123` when applicable. Pull requests should describe the scenario touched, include `colcon build`/`colcon test` output, mention relevant launch files, and attach screenshots or topic dumps when UI or visualization changes are involved.

## Security & Configuration Tips
Never commit `build/`, `install/`, or recorded bags. Store credentials (e.g., for remote registries) via environment variables, not YAML. Always run `source <workspace>/install/setup.bash` in each terminal to avoid mixing message definitions, and pin Stonefish/OKVIS versions in `requirements.txt` or `package.xml` when upgrading dependencies.