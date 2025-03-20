# ArUco Marker Tracking Robot

Welcome to the **ArUco Marker Tracking Robot** project! This repository contains the code and documentation for an autonomous robot that uses ROS 2 and OpenCV to detect and track an ArUco marker in real time, while navigating a static environment using SLAM and the Nav2 library.

---

## Table of Contents
1. [Overview](#overview)
2. [Features](#features)
3. [Hardware Setup](#hardware-setup)
4. [Software Requirements](#software-requirements)
5. [Installation](#installation)

---

## Overview
This project aims to create an autonomous TurtleBot-based platform capable of:
- Detecting an ArUco marker in a scene.
- Estimating its 3D position (pose) using camera calibration data.
- Sending navigation goals to a ROS 2 Nav2 stack in real time so the robot continually moves to follow the marker.

In short, this robot can serve as a moving camera-person or guided tour device—helpful in virtual lectures, tours, and poster sessions.

For more details on the objectives, background, and results, please refer to the project’s detailed report.

---

## Features
- **ArUco Marker Detection**: Uses OpenCV’s `aruco` module to detect and estimate the marker’s pose.
- **Camera Calibration**: Intrinsic parameters for the USB camera are used to improve pose estimation accuracy.
- **ROS 2 Nav2 Integration**:
  - **SLAM** for generating a map of the environment.
  - **Path Planning** with A\* and path smoothing using the Nav2 Smoother Server.
  - **Behavior Trees** for robust navigation recovery if the robot is stuck.
  - **Dynamic Window Approach (DWB) Controller** to control the robot’s velocity commands based on local costmaps.
- **Modular ROS 2 Node Architecture**:
  - A dedicated node for vision (ArUco detection) and publishing the marker’s pose.
  - A node for converting the ArUco marker’s location into a dynamic navigation goal (`nav2pose`).

---

## Hardware Setup
This project was developed primarily on a **TurtleBot3 Burger** platform with the following hardware:
- **Raspberry Pi 4B** (4 GB recommended for smoother performance)
- **OpenCR 1.0** microcontroller board
- **RPLidar A1** for SLAM
- **Logitech C920 USB Camera** (or similar)
- **LiPo Battery** (supplied with TurtleBot3)
- **Dynamixel XL430-W250** servo motors (for TurtleBot3’s wheels)

> **Note**: A high-performance PC is recommended to offload the ROS 2 Nav2 stack if the Raspberry Pi alone struggles with real-time navigation plus computer vision.

---

## Software Requirements
- **Ubuntu 22.04** (or 20.04) with **ROS 2** installed (Galactic, Humble, Iron, etc.).
- **Nav2** ROS 2 navigation stack.
- **OpenCV (>=4.x)** with the `aruco` module.
- **MATLAB Camera Calibration** (optional if you already have calibration files; you can also use OpenCV or ROS camera calibration tools).
- **Python 3** (for the ArUco detection scripts and custom ROS 2 nodes).

---

## Installation

1. **Clone this repository** into your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/YourUsername/ArUco-Tracking-Robot.git

2. **Install dependencies**:
   - Install required ROS 2 packages (Nav2, SLAM, etc.):
     ```bash
     sudo apt-get update
     sudo apt-get install ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup ros-${ROS_DISTRO}-slam-toolbox
     ```
   - (Optional) Install MATLAB if you plan to redo camera calibration with the Computer Vision Toolbox, or install ROS camera calibration tools:
     ```bash
     sudo apt-get install ros-${ROS_DISTRO}-camera-calibration
     ```
   - Install OpenCV if it’s not already installed:
     ```bash
     sudo apt-get install libopencv-dev python3-opencv
     ```

3. **Build the workspace**:
   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install
   source install/setup.bash