# 🤖 Autonomous Mobile Robot in ROS 2 Jazzy

A differential-drive Autonomous Mobile Robot (AMR) built entirely in **ROS 2 Jazzy Jalopy** on **Ubuntu 24.04 LTS**, capable of SLAM-based map generation and fully autonomous point-to-point navigation inside a simulated warehouse environment using **Gazebo Harmonic** and the **Nav2** stack.

> **Migration Notice:** This project was originally developed on **ROS 2 Humble / Ubuntu 22.04**.  
> It has since been fully ported to **ROS 2 Jazzy Jalopy**, the current LTS (Long-Term Support) release of ROS 2.  
> The `humble` branch is preserved for historical reference. All active development is on the `jazzy` branch.

---

## 📌 Table of Contents

- [Robot Overview](#robot-overview)
- [System Requirements](#system-requirements)
- [Project Structure](#project-structure)
- [Installation & Setup](#installation--setup)
- [Running the Project](#running-the-project)
  - [Phase 1 — Launch Simulation](#phase-1--launch-simulation)
  - [Phase 2 — SLAM (Map Generation)](#phase-2--slam-map-generation)
  - [Phase 3 — Autonomous Navigation](#phase-3--autonomous-navigation)
- [Challenges Faced & How They Were Solved](#challenges-faced--how-they-were-solved)
- [Branch Structure](#branch-structure)
- [License](#license)

---

## 🏭 Robot Overview

The warehouse robot is a **two-wheeled differential-drive mobile robot** designed to autonomously navigate inside a warehouse-like environment. It uses a 2D LiDAR for sensing, builds a map of its surroundings using SLAM, and then navigates to user-defined goal poses using the Nav2 stack.

### Key Capabilities

| Capability | Tool Used |
|---|---|
| Simulation | Gazebo Harmonic |
| SLAM (Mapping) | SLAM Toolbox |
| Autonomous Navigation | Nav2 (Navigation2) |
| Motion Control | ros2_control + diff_drive_controller |
| Visualization | RViz2 |
| Robot Description | URDF / Xacro |

### Sensors & Actuators

- **2D LiDAR** — mounted on top, publishes to `/scan`
- **Differential drive** — two wheel joints controlled via `ros2_control`
- **Odometry** — computed by the diff drive controller, published to `/odom`

---

## 🖥️ System Requirements

| Component | Version |
|---|---|
| Operating System | Ubuntu 24.04 LTS |
| ROS 2 | Jazzy Jalopy (LTS) |
| Gazebo | Harmonic |
| ros2_control | jazzy |
| Nav2 | jazzy |
| SLAM Toolbox | jazzy |
| Python | 3.12+ |

### ROS 2 Dependencies

```
ros2_control
ros2_controllers
gazebo_ros2_control
nav2_bringup
nav2_bt_navigator
nav2_controller
nav2_planner
nav2_recoveries
slam_toolbox
robot_state_publisher
joint_state_publisher
teleop_twist_keyboard
```

---

## 📁 Project Structure

```
warehouse_pkg/
├── config/
│   ├── controllers.yaml        # ros2_control diff drive controller config
│   ├── slam_config.yaml        # SLAM Toolbox parameters
│   └── nav2_params.yaml        # Nav2 planner, controller, costmap params
├── launch/
│   ├── gazebo.launch.py        # Spawns robot in Gazebo Harmonic world
│   ├── slam.launch.py          # Runs SLAM Toolbox in async mode
│   └── nav2.launch.py          # Loads saved map + starts Nav2 stack
├── maps/
│   └── warehouse_map.yaml      # Saved map (generated from SLAM)
├── urdf/
│   ├── robot.urdf.xacro        # Main robot description
│   └── robot.gazebo.xacro      # Gazebo plugins (LiDAR, ros2_control)
├── worlds/
│   └── warehouse.world         # Gazebo Harmonic world file
├── CMakeLists.txt
├── package.xml
└── README.md
```

<img width="1920" height="1080" alt="warehouse_bot1" src="https://github.com/user-attachments/assets/79696291-7ddb-498c-b93e-ef706017157c" />


<img width="1920" height="1080" alt="warehouse_bot2" src="https://github.com/user-attachments/assets/c230e512-de28-4119-8328-4814d0c5e812" />

---

## 🔧 Installation & Setup

### Step 1 — Clone the Repository

```bash
git clone -b jazzy https://github.com/akhileshwar-p-s/Autonomous-Mobile-Robot-in-ROS2.git
```

### Step 2 — Place it in Your ROS 2 Workspace

```bash
mkdir -p ~/ros2_ws/src
mv Autonomous-Mobile-Robot-in-ROS2 ~/ros2_ws/src/warehouse_pkg
cd ~/ros2_ws
```

### Step 3 — Install ROS 2 Dependencies

Make sure ROS 2 Jazzy is already installed. Then:

```bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

If any Nav2 or SLAM packages are missing, install them manually:

```bash
sudo apt install ros-jazzy-nav2-bringup \
                 ros-jazzy-slam-toolbox \
                 ros-jazzy-ros2-control \
                 ros-jazzy-ros2-controllers \
                 ros-jazzy-gz-ros2-control
```

### Step 4 — Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select warehouse_pkg
source install/setup.bash
```

> 💡 **Tip:** Add `source ~/ros2_ws/install/setup.bash` to your `~/.bashrc` so you don't have to source it every session.

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 🚀 Running the Project

Open **three separate terminals** for the three phases. Make sure each terminal has the workspace sourced.

---

### Phase 1 — Launch Simulation

Spawns the robot inside the warehouse world in Gazebo Harmonic:

```bash
ros2 launch warehouse_pkg gazebo.launch.py
```

You should see the robot appear in Gazebo. RViz2 will also open showing the TF tree and LiDAR scan.

---

### Phase 2 — SLAM (Map Generation)

In a **new terminal**, launch SLAM Toolbox in async mapping mode:

```bash
ros2 launch warehouse_pkg slam.launch.py
```

Then drive the robot around the warehouse to build the map. Use teleop in another terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Drive around slowly and cover all areas until the map looks complete in RViz2.

**Save the map** once satisfied:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/warehouse_pkg/maps/warehouse_map
```

This generates two files:
- `warehouse_map.pgm` — the occupancy grid image
- `warehouse_map.yaml` — metadata (resolution, origin, thresholds)

---

### Phase 3 — Autonomous Navigation

Stop the SLAM launch. Then in a new terminal:

```bash
ros2 launch warehouse_pkg nav2.launch.py map:=$HOME/ros2_ws/src/warehouse_pkg/maps/warehouse_map.yaml
```

In RViz2:
1. Click **"2D Pose Estimate"** and click on the robot's approximate location on the map to initialize localization (AMCL)
2. Wait for the particle cloud to converge around the robot
3. Click **"Nav2 Goal"** (or "2D Nav Goal") and click any point on the map
4. The robot will plan a path and navigate autonomously

---

## 🔥 Challenges Faced & How They Were Solved

This project involved a full migration from ROS 2 Humble to Jazzy, and several significant issues were encountered and resolved. This section documents them so others attempting a similar migration don't hit the same walls.

---

### 1. ❌ Nav2 Could Not Plan a Path — `GoalState: FAILED`

**Symptom:**
After launching Nav2 and setting a goal pose in RViz2, the robot would not move. The Nav2 action server returned `FAILED` and the planner reported it could not find a valid path, even when the goal was clearly reachable.

**Root Cause:**
The global costmap was not receiving the `/scan` data correctly. The LiDAR plugin used in Gazebo Classic (`libgazebo_ros_ray_sensor.so`) was replaced in Gazebo Harmonic with a different plugin, and the topic it published to was remapped differently. The costmap was therefore building on an empty obstacle layer — it had no knowledge of walls or obstacles, so the planner was treating the entire space as free but still failing due to the robot's footprint being marked as an obstacle on an empty map.

Additionally, the `robot_radius` in `nav2_params.yaml` was set too large, causing the inflation layer to mark the robot's start position as an obstacle itself, making every goal invalid from the start.

**Fix:**
- Updated the Gazebo Harmonic LiDAR plugin in the URDF to use `gz-sim-sensors-system` and verified the scan topic was remapping correctly to `/scan`
- Verified with `ros2 topic echo /scan` that data was actually flowing before launching Nav2
- Reduced `robot_radius` in `nav2_params.yaml` from `0.3` to `0.18` to match the actual robot footprint
- Set `inflation_radius` to `0.25` to give a safe but not overly conservative clearance

---

### 2. ❌ Saved Map Not Loading in Nav2 — Blank / Black Map in RViz2

**Symptom:**
After saving the map from SLAM and launching `nav2.launch.py` with the map path, RViz2 showed either a completely black map or no map at all. Nav2 reported `map_server` could not load the map file.

**Root Cause (1) — Wrong file path in launch argument:**
The `map` argument passed to the launch file was an absolute path with `~` which Python's `os.path` does not expand automatically. The map_server node received a literal `~/ros2_ws/...` path that did not resolve on the filesystem.

**Fix:** Expanded the path explicitly in the launch file:

```python
import os
map_file = os.path.expanduser('~/ros2_ws/src/warehouse_pkg/maps/warehouse_map.yaml')
```

Or pass the full absolute path directly on the command line:

```bash
ros2 launch warehouse_pkg nav2.launch.py map:=/home/<your-username>/ros2_ws/src/warehouse_pkg/maps/warehouse_map.yaml
```

**Root Cause (2) — `.pgm` file path in `.yaml` was incorrect:**
The `warehouse_map.yaml` file generated by `map_saver_cli` contained a relative path to the `.pgm` file:

```yaml
image: warehouse_map.pgm
```

When Nav2's `map_server` loaded the yaml from a different working directory, it could not find the `.pgm` file.

**Fix:** Edit `warehouse_map.yaml` and replace the relative path with the absolute path:

```yaml
image: /home/<your-username>/ros2_ws/src/warehouse_pkg/maps/warehouse_map.pgm
```

---

### 3. ❌ Diff Drive and LiDAR Plugins Not Loading in Gazebo Harmonic

**Symptom:**
After migrating from Gazebo Classic to Gazebo Harmonic, the robot would spawn but the wheels would not respond to `/cmd_vel` commands and no `/scan` data was published.

**Root Cause:**
Gazebo Harmonic uses completely different plugin names compared to Classic. The old URDF `<gazebo>` plugin tags referencing `libgazebo_ros_diff_drive.so` and `libgazebo_ros_ray_sensor.so` are not valid in Harmonic.

**Fix:**
- Replaced diff drive control with `gz_ros2_control` / `GazeboSimSystem` plugin paired with `ros2_control` and `diff_drive_controller`
- Updated the LiDAR plugin to use the native Gazebo Harmonic sensor system
- Verified joint names in `controllers.yaml` matched exactly with the URDF joint names

---

### 4. ❌ Package Naming and CMakeLists Install Issues After Migration

**Symptom:**
After cloning and building, `ros2 launch warehouse_pkg ...` returned `Package not found`.

**Root Cause:**
During migration, some `install()` directives in `CMakeLists.txt` were missing for the `launch/`, `config/`, `maps/`, and `worlds/` directories. ROS 2 only makes installed files available to `ros2 launch` — files not listed in CMakeLists install rules are not accessible after a `colcon build`.

**Fix:** Ensured all resource directories were installed in `CMakeLists.txt`:

```cmake
install(DIRECTORY launch config maps worlds urdf
  DESTINATION share/${PROJECT_NAME}
)
```

---

## 🌿 Branch Structure

| Branch | ROS 2 | Ubuntu | Gazebo | Status |
|---|---|---|---|---|
| `jazzy` | Jazzy Jalopy (LTS) | 24.04 | Harmonic | ✅ Active Development |
| `humble` | Humble Hawksbill | 22.04 | Classic | 🗄️ Archived |

---

## 📄 License

MIT License — feel free to use, modify, and distribute with attribution.

---

## 👤 Author

**Akhileshwar Pratap Singh**  
Engineering Student, RGIPT  
GitHub: [@akhileshwar-p-s](https://github.com/akhileshwar-p-s)
