# ROS2 Drone Simulator (Joystick Controlled)

A simple ROS2-based drone simulation node controlled using a joystick via "joy" messages.
This project is meants as a **learning-focused simulation**, not a physics accurate drone model.

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

---

## Workspace Setup

This Project assumes a standard ROS2 workspace layout:
```
ros2_ws/
├── src/
│   └── drone_sim/
│       ├── src/
│       │   └── drone_sim.cpp
│       ├── CMakeLists.txt
│       └── package.xml
```

---

## Build Instructions

```
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## Running the Simulator
1. Start the joystick node:
```
ros2 run joy joy_node
```
   Verify input:
```
ros2 topic echo /joy
```
2. Run the drone simulation
in a separate terminal window:
```
ros2 run drone_sim drone_sim
```
