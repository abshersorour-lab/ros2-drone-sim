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

(please run each of the following commands in a separate terminal window of their own, all in "~/ros2_ws")
1. Start the joystick node:
```
ros2 run joy joy_node
```
2. Verify input
```
ros2 topic echo /joy
```
3. Run the drone simulation
```
ros2 run drone_sim drone_sim
```

---

## Disclaimer
This project is primarily a learning and experimentation sandbox for practicing C++ and ROS 2 Fundamentals.
It intentionally steers away from real-world flight and control accuracy.

The simulation is designed with simplicity in mind to allow for ease of expansion as in the future it may serve as a prototype for new ideas or integrate
more advanced concepts as my understanding deepens.
