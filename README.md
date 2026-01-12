# Autonomous-Mobile-Robot-in-ROS2

# ROS2 Mobile Robot - SLAM & Navigation

A mobile robotics project implementing SLAM (Simultaneous Localization and Mapping) and autonomous navigation using ROS2, Gazebo simulation, and RViz visualization.

## 🤖 Overview

This project features a differential drive mobile robot equipped with SLAM capabilities for autonomous navigation in unknown environments. The robot uses sensor data to build maps and localize itself while navigating through obstacles.

**Robot Type:** Differential Drive  
**Simulation:** Gazebo  
**Visualization:** RViz2  
**Navigation:** Nav2 Stack

## ✨ Features

### Implemented
- ✅ URDF robot model with differential drive configuration
- ✅ Gazebo simulation environment
- ✅ RViz visualization setup
- ✅ Custom world environments
- ✅ Launch file for system initialization

### In Development
- 🔨 SLAM implementation
- 🔨 Nav2 navigation stack integration
- 🔨 Sensor fusion (LiDAR/Camera)
- 🔨 Path planning algorithms
- 🔨 Obstacle avoidance

## 📦 Prerequisites

- **OS:** Ubuntu 22.04 (Jammy)
- **ROS2 Distribution:** Humble Hawksbill
- **Gazebo:** Gazebo 11
- **Python:** 3.10+

### Required ROS2 Packages
```bash
sudo apt install ros-humble-desktop
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-robot-localization
```

## 🚀 Installation

1. **Clone the repository**
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/yourusername/robot_pkg.git
   ```

2. **Install dependencies**
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the package**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select robot_pkg
   ```

4. **Source the workspace**
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## 🎮 Usage

### Launch the Robot Simulation

To launch the complete system with Gazebo and RViz:

```bash
ros2 launch robot_pkg display.launch.py
```

### Individual Components

**Launch Gazebo only:**
```bash
ros2 launch robot_pkg gazebo.launch.py
```

**Launch RViz only:**
```bash
rviz2 -d ~/ros2_ws/src/robot_pkg/rviz/display.rviz
```

### Teleoperation

Control the robot using keyboard:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 📁 Project Structure

```
robot_pkg/
├── config/              # Configuration files
├── launch/              
│   └── display.launch.py    # Main launch file
├── maps/                # Saved map files
├── rviz/
│   └── display.rviz     # RViz configuration
├── urdf/
│   ├── my_diffbot.gazebo       # Gazebo-specific URDF
│   └── my_diffbot.urdf.xacro   # Robot description (xacro)
├── worlds/              # Gazebo world files
├── CMakeLists.txt
└── package.xml
```

## ⚙️ Configuration

### Robot Parameters

Edit the URDF/xacro files in the `urdf/` directory to modify:
- Robot dimensions
- Wheel parameters
- Sensor configurations
- Physical properties (mass, inertia)

### Navigation Parameters

Navigation parameters will be stored in `config/` directory:
- `nav2_params.yaml` - Nav2 stack parameters
- `slam_params.yaml` - SLAM configuration
- `controller_params.yaml` - Robot controller settings

## 🤝 Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

### Development Guidelines
- Follow ROS2 coding standards
- Test changes in simulation before submitting
- Update documentation for new features
- Add comments to complex code sections

## 📝 Known Issues

- SLAM integration pending
- Navigation stack configuration needed
- Sensor configurations to be finalized

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 👥 Authors

- **Akhileshwar Pratap Singh** 

## 🙏 Acknowledgments

- ROS2 Community
- Nav2 Development Team
- Gazebo Simulation Platform

## 📧 Contact

For questions or suggestions, please open an issue or contact:
- Email: akhileshwarrc@gmail.com

---

**Note:** This project is under active development. Features and documentation are continuously being updated.
