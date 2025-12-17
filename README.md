# MoveIt2 Simple Arm

> **Note:** This repository contains the `src/` folder of a ROS2 workspace. Clone this into your ROS2 workspace's `src/` directory.

A simple 6-axis robotic arm with gripper using ROS2 and MoveIt2 which demonstrates motion planning, trajectory control, and manipulation capabilities with a custom URDF robot model. This project is an implementation of the course [ROS 2 Moveit 2 - Control a Robotic Arm](https://www.udemy.com/course/ros2-moveit2/) by Edouard Renard on Udemy.

## 🤖 Robot Specifications

- **Arm**: 6 revolute joints (6-DOF)
- **End Effector**: Parallel gripper with mimic joint
- **Controller**: Joint trajectory controller via ros2_control
- **Planning Framework**: MoveIt2 with OMPL planners

## 📋 Project Overview

This repository contains three ROS2 packages that work together to simulate and control a 6-axis robotic arm with a parallel gripper:

- **my_robot_description** - Robot model definition (URDF/Xacro files)
- **my_robot_bringup** - Launch files and configurations for robot startup
- **my_robot_moveit_config** - MoveIt2 configuration for motion planning

### 1. my_robot_description
Contains the robot model definitions:
- `urdf/` - URDF and Xacro files defining robot geometry and kinematics
  - `my_robot.urdf.xacro` - Main robot assembly
  - `arm.xacro` - 6-DOF arm definition
  - `gripper.xacro` - Gripper mechanism
  - `my_robot.ros2_control.xacro` - ros2_control hardware interface
  - `common_properties.xacro` - Shared properties (colors, inertias)
- `launch/display.launch.xml` - RViz visualization launch file
- `rviz/urdf_config.rviz` - RViz configuration

### 2. my_robot_bringup
Launch configurations for starting the robot:
- `launch/my_robot.launch.xml` - Main launch file for robot startup
- `config/ros2_controllers.yaml` - Controller configurations
  - Arm controller (6 joints)
  - Gripper controller
  - Joint state broadcaster

### 3. my_robot_moveit_config
MoveIt2 motion planning configuration:
- `config/` - MoveIt configuration files
  - `my_robot.srdf` - Semantic robot description
  - `kinematics.yaml` - Kinematics solver configuration
  - `joint_limits.yaml` - Joint velocity and acceleration limits
  - `moveit_controllers.yaml` - MoveIt controller mappings
  - `pilz_cartesian_limits.yaml` - Cartesian motion limits
- `launch/` - MoveIt launch files
  - `demo.launch.py` - Full MoveIt demo with RViz
  - `move_group.launch.py` - MoveIt move_group node
  - `moveit_rviz.launch.py` - RViz with MoveIt plugin

## 🚀 Getting Started

### Prerequisites

- Ubuntu 24.04
- ROS2 (Jazzy)
- MoveIt2
- ros2_control and ros2_controllers

```bash
sudo apt install ros-$ROS_DISTRO-moveit \
                 ros-$ROS_DISTRO-ros2-control \
                 ros-$ROS_DISTRO-ros2-controllers \
                 ros-$ROS_DISTRO-joint-state-publisher-gui
```

### Installation

1. **Clone this repository into your ROS2 workspace:**
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/jeddiot/moveit2-simple-arm.git .
   ```
   
   > **Important:** Note the `.` at the end - this clones the contents directly into `src/` since this repo contains the src folder structure.

2. **Build the workspace:**
   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

## 🎮 Usage

### Launch Robot with MoveIt2

Start the complete simulation with motion planning:

```bash
ros2 launch my_robot_moveit_config demo.launch.py
```

This will open RViz with the MoveIt Motion Planning plugin where you can:
- Plan and execute trajectories
- Use the interactive marker to set goal poses
- Visualize collision objects
- Test different motion planners

### Launch Robot Description Only

To view the robot model in RViz without motion planning:

```bash
ros2 launch my_robot_description display.launch.xml
```

### Launch Robot with Controllers

To start the robot with ros2_control controllers:

```bash
ros2 launch my_robot_bringup my_robot.launch.xml
```

## 🎯 Features

- ✅ Fully modular Xacro-based robot description
- ✅ ros2_control integration with mock hardware
- ✅ MoveIt2 motion planning configuration
- ✅ Multiple planning groups (arm, gripper)
- ✅ Predefined poses for easy testing
- ✅ RViz configurations for visualization
- ✅ Joint trajectory controllers for smooth motion
- ✅ Parallel gripper with mimic joint

## 📁 Repository Structure

```
src/
├── my_robot_description/
│   ├── urdf/              # Robot URDF/Xacro files
│   ├── launch/            # Visualization launch files
│   └── rviz/              # RViz configurations
├── my_robot_bringup/
│   ├── launch/            # Robot startup launch files
│   └── config/            # Controller configurations
└── my_robot_moveit_config/
    ├── config/            # MoveIt configurations
    └── launch/            # MoveIt launch files
```

## 🛠️ Development

### Joint Configuration

**Arm Joints:**
- `joint1` through `joint6` - 6-DOF revolute joints
- Planning group: `arm`

**Gripper Joints:**
- `gripper_left_finger_joint` - Active joint
- `gripper_right_finger_joint` - Mimic joint
- Planning group: `gripper`

### Controller Configuration

The robot uses `joint_trajectory_controller` for both arm and gripper, configured at 100Hz update rate.

## 📝 License

This project currently has no license and is open source, available for educational purposes.

## 🤝 Contributing

Contributions, issues, and feature requests are welcome!
