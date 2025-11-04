# 🤖 ROBE-313 Homework 3 – Multi-Language ROS 2 Mobile Robot Simulation

**Author:** Colton Gelety  
**Course:** ROBE 313 – Fall 2025  
**Instructors:** Luis Escobar & Dr. Giacomo Marani  
**License:** Apache 2.0  

---

## 🧭 Project Overview
This project implements a **ROS 2 (Humble)** mobile-robot simulation integrating both **C++** and **Python** nodes within a structured, multi-package workspace.  
The system demonstrates real-time odometry integration, TF broadcasting, and closed-loop motion control, visualized entirely in RViz.

---

## 🧩 Package Summary

| Package | Type | Purpose |
|----------|------|----------|
| 🧰 `custom_interfaces` | `ament_cmake` | Defines a custom `ResetPosition.srv` service used to reset the robot’s pose. |
| ⚙️ `robot_simulator_cpp` | `ament_cmake` | Contains the **C++ odometry node** that subscribes to `/cmd_vel`, integrates velocity to pose, and broadcasts the dynamic `odom → base_link` transform. |
| 🐍 `robot_simulator_py` | `ament_python` | Contains the **Python controller node** that listens to `odom → base_link` via tf2 and drives the robot in a 2 m × 2 m square trajectory using a state machine. |
| 🚀 `robot_bringup` | `ament_cmake` | Integration layer that includes the URDF model, RViz configuration, and the master launch file. |

---

## 🧱 Workspace Structure

```
ros2_ws_gelety/
├── src/
│   ├── custom_interfaces/
│   ├── robot_simulator_cpp/
│   ├── robot_simulator_py/
│   └── robot_bringup/
├── README.md
├── README_ANSWERS.md
└── .gitignore
```

---

## 🦴 URDF Model
The robot consists of:
- A `base_link` (rectangular body)
- A `camera_link` fixed 0.25 m above the base  

The resulting TF tree:

```
odom
 └── base_link
      └── camera_link
```

---

## 🧠 Controller Logic
The Python controller implements a simple finite-state machine:
1. **DRIVE** – moves straight until 2 m are traveled  
2. **TURN** – rotates 90° (π/2 radians)  
3. Repeats 4 times → completes a square  
4. **STOP** – halts motion  

Each side is verified through tf2 feedback from `odom → base_link`.

---

## 🖥️ Launching the Simulation

Build and source your workspace:
```bash
cd ~/ros2_ws_gelety
colcon build --symlink-install
source install/setup.bash
```

Then run the master launch file:
```bash
ros2 launch robot_bringup robot_simulation.launch.py
```

RViz will automatically open using the provided configuration (`simulation.rviz`).

---

## 🧭 Visualizing in RViz
- **Fixed Frame:** `odom`  
- **Displays to add:** TF Tree, RobotModel (`robot_description`), Grid  
- The robot’s axes (`base_link` and `camera_link`) will trace a square trajectory.

---

## 🔧 Reset Service Demo
Reset the odometry pose at any time:
```bash
ros2 service call /ResetPosition custom_interfaces/srv/ResetPosition "{pose: {position: {x: 0.0, y: 0.0, z: 0.0},
         orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

RViz will show the TF frames “jump” back to the origin.

---

## 📦 Dependencies
- ROS 2 Humble Hawksbill  
- `rclcpp`, `rclpy`  
- `geometry_msgs`, `tf2_ros`  
- `robot_state_publisher`, `rviz2`  

Build tested on Ubuntu 22.04 with colcon.

---

## 📹 Demonstration Video
The accompanying video presentation shows:
- The node architecture (`rqt_graph`)  
- TF frames moving in a square trajectory  
- The `ResetPosition` service in action  
- RViz visualization of `odom → base_link → camera_link`

---

## 🧾 Assignment Compliance
✅ **Part 1:** Workspace & Package Structure  
✅ **Part 2:** Custom Service Definition  
✅ **Part 3:** C++ Odometry Node  
✅ **Part 4:** Python Controller Node  
✅ **Part 5:** System Integration & Visualization  
✅ **Part 6:** Research Questions (`README_ANSWERS.md`)  

---

## ⚙️ Build & Run Summary

```bash
# Build workspace
colcon build --symlink-install

# Source environment
source install/setup.bash

# Run full integrated simulation
ros2 launch robot_bringup robot_simulation.launch.py
```

---

## 📄 License
This project is licensed under the **Apache 2.0 License**.  
See the [LICENSE](https://www.apache.org/licenses/LICENSE-2.0) file for details.
