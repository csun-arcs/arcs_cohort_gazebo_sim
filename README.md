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
