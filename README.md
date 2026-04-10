# Cutting Robot ROS2 Package

## Overview
This package generates a motion planning scene in RViz with:
- ABB IRB1200 5/90 robot arm
- Dukane ultrasonic blade (attached to end effector)
- A configurable block as a collision object

> **Note:** The tool mount position on the end effector and the arm motion are still being finalized.

---

## Prerequisites
- ROS2 (Humble or later)
- MoveIt2
- Python 3.12+
- trimesh (`pip install trimesh`)

---

## Installation

```bash
# Clone the repository into your ROS2 workspace
cd ~/ros2_ws/src
git clone https://github.com/egt1688/cutting_robot.git

# Build the package
cd ~/ros2_ws
colcon build --packages-select cutting_robot
source install/setup.bash
```

---

## Configuration

### Change the robot start and end positions
Go to `abb_irb1200_5_90_macro.srdf.xacro` and find:

```xml
<group_state name="all_zero" group="manipulator">
<group_state name="extended" group="manipulator">
```
Change the joint values (in radians) to adjust the start and end positions of the robot.

---

### Change the block position and dimensions
Go to `nodes/moveit_node.py` and find `add_collision_object()`:

```python
object_positions = [
    (0.6096, 0.3048, 0.25),  # x, y, z position of the block in meters
]
object_dimensions = [
    (0.1, 0.6096, 0.3048),   # width, depth, height of the block in meters
]
```

---

### Change the tool position on the end effector
Go to `nodes/moveit_node.py` and find `add_attached_collision_object()`:

```python
pose.position.x = 0.1    # left/right relative to tool0
pose.position.y = -0.05  # up/down relative to tool0
pose.position.z = 0.24   # along the tool axis (forward/back)
pose.orientation.x = 0.0
pose.orientation.y = 0.707
pose.orientation.z = 0.0
pose.orientation.w = 1.0
```
Adjust position values (meters) and orientation (quaternion) to correctly mount the tool on the end effector.

---
## Update Changes to Rviz
Save all changes then follow these commands:
```bash
cd ros2_ws
colcon build
```

## Running the package

```bash
cd ros2_ws
ros2 launch cutting_robot cutting_robot.launch.py
```

---

## Known Issues
- Tool mount position on the end effector is still being finalized
- Arm motion path is still being finalized

---
