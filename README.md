# ROS2 Drone Simulator (Joystick Controlled)

A simple ROS2-based drone simulation node controlled using a joystick via "joy" messages.
This projectis meants as a **learning-focused simulation**, not a physics accurate drone model.

The goal was to practice:
- ROS2 nodes
- Joystick input handling
- C++ ROS2 structure

---

## Features

- Joystick controlled movement
- Hover Lock toggle
- Return-To-base mode
- Gravity Simulation with Hover is unlocked

---

## Requirements

- **OS:** Ubuntu 22.04 (tested on a VM running on Linux Mint as main OS)
- **ROS2:** Humble (or compatible)
- **Packages:**
  - rclcpp
  - sensor_msgs
  - joy
- **Hardware:** Game controller / joystick (tested and coded based on the Xbox-360-based Flydigi Apex 3 controller) 

