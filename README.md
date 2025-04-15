# ARCS Cohort Gazebo Simulation Package

The `arcs_cohort_gazebo_sim` package provides a Gazebo simulation environment for the ARCS Cohort robot, enabling users to simulate the robot's behavior in various world configurations. It integrates with ROS 2 and Gazebo to support robot control, sensor simulation (e.g., Stereolabs Zed camera, LiDAR), and teleoperation using joysticks or keyboards. This package is designed to work with the `arcs_cohort_description` and `arcs_cohort_navigation` packages for a complete simulation and navigation stack.

## Table of Contents
- [Overview](#overview)
- [Folder Structure](#folder-structure)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [Usage](#usage)
  - [Launching the Simulation](#launching-the-simulation)
  - [Launch Arguments](#launch-arguments)
  - [Teleoperation](#teleoperation)
  - [Navigation](#navigation)
- [Configuration Files](#configuration-files)
- [Contributing](#contributing)
- [License](#license)

## Overview
This package sets up a Gazebo simulation for the ARCS Cohort robot, with the following features:
- A customizable robot model loaded via URDF/Xacro from the `arcs_cohort_description` package.
- Simulated sensors, including a Stereolabs Zed camera (with configurable resolution) and an optional LiDAR.
- Support for differential drive and ROS 2 control plugins for robot actuation.
- Teleoperation via joystick (using `teleop_twist_joy`) or keyboard (using `teleop_twist_keyboard`).
- Integration with `twist_mux` for navigation, configured via the `arcs_cohort_navigation` package.
- ROS-Gazebo bridges for sensor data, including left/right camera images and depth images.
- Conditional node launching based on parameters (e.g., enabling/disabling LiDAR, ROS 2 control, or joint state publishers).

The primary launch file, `gazebo_sim.launch.py`, orchestrates the simulation environment by:
- Loading a specified Gazebo world.
- Spawning the robot model in Gazebo.
- Starting ROS 2 nodes for state publishing, control, and teleoperation.
- Establishing bridges for sensor data communication between Gazebo and ROS 2.

## Folder Structure
The package follows a standard ROS 2 package structure, organized as follows:
arcs_cohort_gazebo_sim/
├── config/
│   ├── gazebo_bridge.yaml          # ROS-Gazebo bridge configuration for sensors
│   ├── gazebo_joystick_teleop.yaml # Joystick teleoperation parameters
│   ├── gazebo_ros2_control_sim.yaml # ROS 2 control configuration for Gazebo
│   └── ros2_control_params.yaml    # Additional ROS 2 control parameters
├── control/
│   ├── gazebo_diff_drive_plugin    # Differential drive plugin for Gazebo
│   └── gazebo_ros2_control_plugin  # ROS 2 control plugin for Gazebo
├── launch/
│   └── gazebo_sim.launch.py        # Main launch file for the simulation
├── worlds/
│   └── test_obstacles_world_1.world # Example Gazebo world file
├── package.xml                     # ROS 2 package metadata
├── CMakeLists.txt                  # Build configuration
└── README.md                       # This file

- **config/**: Contains YAML files for configuring sensors, teleoperation, and control systems.
- **control/**: Includes Gazebo plugins for differential drive and ROS 2 control integration.
- **launch/**: Holds the primary launch file to start the simulation environment.
- **worlds/**: Stores Gazebo world files defining the simulation environment (e.g., obstacles, terrain).

## Dependencies
To use this package, ensure the following dependencies are installed:
- **ROS 2** (Jazzy)
- **Gazebo** (Gazebo Harmonic, compatible with ROS 2 Jazzy)
- **ros_gz** (ROS-Gazebo interface packages):
  - `ros_gz_sim`
  - `ros_gz_bridge`
  - `ros_gz_image`
- **Related ARCS Cohort packages**:
  - `arcs_cohort_description` (provides robot model and URDF/Xacro files)
  - `arcs_cohort_navigation` (provides navigation stack, including twist mux configuration)
- **Other ROS 2 packages**:
  - `robot_state_publisher`
  - `joint_state_publisher`
  - `joint_state_publisher_gui`
  - `controller_manager`
  - `twist_mux`
  - `teleop_twist_joy`
  - `teleop_twist_keyboard`
  - `twist_stamper`
  - `joy`
 




Teleoperation

The package supports two teleoperation modes:

    Joystick:
        Enabled with use_joystick:=true.
        Uses joy_node and teleop_twist_joy nodes.
        Configured via config/gazebo_joystick_teleop.yaml (e.g., axis mappings, velocity scaling).
        Publishes velocity commands to /diff_cont/cmd_vel_unstamped (or /diff_cont/cmd_vel if use_ros2_control:=true).
        Requires a compatible joystick device.
    Keyboard:
        Enabled when use_joystick:=false.
        Uses teleop_twist_keyboard node, launched in a new terminal via xterm.
        Publishes velocity commands to /diff_cont/cmd_vel_unstamped (or /diff_cont/cmd_vel if use_ros2_control:=true).
        Control the robot using arrow keys (refer to teleop_twist_keyboard documentation for key bindings).

Navigation

Navigation support is enabled with use_navigation:=true, which launches the twist_mux node:

    Configured via arcs_cohort_navigation/config/twist_mux.yaml.
    Combines velocity commands from multiple sources (e.g., teleoperation, navigation stack).
    Remaps output to:
        /diff_cont/cmd_vel_nav (stamped as /diff_cont/cmd_vel if use_ros2_control:=true).
        /diff_cont/cmd_vel_unstamped (if use_ros2_control:=false).
    Integrates with the arcs_cohort_navigation package for waypoint navigation or autonomous tasks.

Configuration Files

The config folder contains the following YAML files:

    gazebo_bridge.yaml: Defines ROS-Gazebo bridge topics for sensors (e.g., camera images, depth data, LiDAR).
    gazebo_joystick_teleop.yaml: Parameters for joystick teleoperation, including axis mappings and velocity scaling.
    gazebo_ros2_control_sim.yaml: Configuration for ROS 2 control in Gazebo, specifying controller settings.
    ros2_control_params.yaml: Additional parameters for ROS 2 controllers (e.g., diff_cont for differential drive, joint_broad for joint states).

To customize sensor topics, control behavior, or teleoperation settings, modify these files as needed. Ensure topic names align with your robot model and navigation stack.
