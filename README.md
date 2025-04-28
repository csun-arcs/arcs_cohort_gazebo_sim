# ARCS Cohort Gazebo Simulation Package

The `arcs_cohort_gazebo_sim` package provides a Gazebo simulation environment for the ARCS Cohort robot, enabling users to simulate the robot's behavior in various world configurations. It integrates with ROS 2 and Gazebo to support robot control, sensor simulation (e.g., Stereolabs Zed camera, LiDAR), and teleoperation using joysticks or keyboards.

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

## Overview
This package sets up a Gazebo simulation for the ARCS CoHORT rover, with the following features:
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


## Launch File and Arguments

The launch file, located in the `launch/` folder (`gazebo_sim.launch.py`), is used to load the Gazebo simulation environment, spawn the robot model, and start necessary ROS 2 nodes. It includes several launch arguments to configure the launch process. Below are the supported arguments:

- **`world`**: A path to specify the Gazebo world file to load.
  - **Default**: Path to `test_obstacles_world_1.world` in `arcs_cohort_gazebo_sim`
  - **Example**: `world:=/path/to/custom_world.world`

- **`use_sim_time`**: Determines whether to use simulation time from Gazebo.
  - **Default**: `"true"`
  - **Example**: `use_sim_time:=false`

- **`model_package`**: Specifies the ROS 2 package containing the robot model.
  - **Default**: `"arcs_cohort_description"`
  - **Example**: `model_package:=my_custom_description_package`

- **`model_file`**: A relative path to the robot model file within the `model_package`.
  - **Default**: `"description/robot.urdf.xacro"`
  - **Example**: `model_file:=another_folder/another_model.urdf.xacro`

- **`prefix`**: A prefix for the names of joints, links, etc. in the robot model). E.g. 'base_link' will become 'cohort1_base_link' if prefix is set to 'cohort1'.
  - **Default**: `''`
  - **Example**: `prefix:=my_robot`

- **`camera_resolution`**: Sets the resolution profile for the simulated Stereolabs Zed camera. Options: `"HD2K"`, `"HD1080"`, `"HD720"`, `"VGA"`.
  - **Default**: `"VGA"`
  - **Example**: `camera_resolution:=HD1080`

- **`use_rsp`**: Controls whether to launch the `robot_state_publisher`.
  - **Default**: `"true"`
  - **Example**: `use_rsp:=false`

- **`use_jsp`**: Controls whether to launch the `joint_state_publisher`.
  - **Default**: `"false"`
  - **Example**: `use_jsp:=true`

- **`use_jsp_gui`**: Controls whether to launch the `joint_state_publisher_gui`.
  - **Default**: `"false"`
  - **Example**: `use_jsp_gui:=true`

- **`use_lidar`**: Determines whether to include the LiDAR in the robot description.
  - **Default**: `"false"`
  - **Example**: `use_lidar:=true`

- **`lidar_update_rate`**: Sets the update rate of the LiDAR sensor in Hz.
  - **Default**: `"30"`
  - **Example**: `lidar_update_rate:=10`

- **`use_ros2_control`**: Enables ROS 2 Control for the robot.
  - **Default**: `"false"`
  - **Example**: `use_ros2_control:=true`

- **`use_joystick`**: Enables joystick control for the robot.
  - **Default**: `"false"`
  - **Example**: `use_joystick:=true`

- **`use_navigation`**: Enables the twist mux for navigation.
  - **Default**: `"false"`
  - **Example**: `use_navigation:=true`


## Teleoperation

The package supports two teleoperation modes:

- **Joystick**:
  - Enabled with `use_joystick:=true`.
  - Uses `joy_node` and `teleop_twist_joy` nodes.
  - Configured via `config/gazebo_joystick_teleop.yaml` (e.g., axis mappings, velocity scaling).
  - Publishes velocity commands to `diff_cont/cmd_vel_unstamped` (or `diff_cont/cmd_vel` if `use_ros2_control:=true`).
  - Requires a compatible joystick device.

- **Keyboard**:
  - Enabled when `use_joystick:=false`.
  - Uses `teleop_twist_keyboard` node, launched in a new terminal via `xterm`.
  - Publishes velocity commands to `diff_cont/cmd_vel_unstamped` (or `diff_cont/cmd_vel` if `use_ros2_control:=true`).
  - Control the robot using arrow keys (refer to `teleop_twist_keyboard` documentation for key bindings).

---

## Navigation

Navigation support is enabled with `use_navigation:=true`, which launches the `twist_mux` node:

- Configured via `arcs_cohort_navigation/config/twist_mux.yaml`.
- Combines velocity commands from multiple sources (e.g., teleoperation, navigation stack).
- Remaps output to:
  - `diff_cont/cmd_vel_nav` (stamped as `diff_cont/cmd_vel` if `use_ros2_control:=true`).
  - `diff_cont/cmd_vel_unstamped` (if `use_ros2_control:=false`).
- Integrates with the `arcs_cohort_navigation` package for waypoint navigation or autonomous tasks.

---

## Configuration Files

The `config/` folder contains the following YAML files:

- **`gazebo_bridge.yaml`**: Defines ROS-Gazebo bridge topics for sensors (e.g., camera images, depth data, LiDAR).
- **`gazebo_joystick_teleop.yaml`**: Parameters for joystick teleoperation, including axis mappings and velocity scaling.
- **`gazebo_ros2_control_sim.yaml`**: Configuration for ROS 2 control in Gazebo, specifying controller settings.
- **`ros2_control_params.yaml`**: Additional parameters for ROS 2 controllers (e.g., `diff_cont` for differential drive, `joint_broad` for joint states).

To customize sensor topics, control behavior, or teleoperation settings, modify these files as needed. Ensure topic names align with your robot model and navigation stack.

