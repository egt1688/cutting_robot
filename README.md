# Cutting Robot ROS2 Package

## Overview
This package generates a motion planning scene in RViz with:
- ABB IRB1200 5/90 robot arm
- UR5E robot arm
- UR10E robot arm
- Dukane ultrasonic blade (attached to end effector)
- A configurable block as a collision object

> **Note:** The tool mount position on the end effector and the arm motion are still being finalized.

---

## Prerequisites
- ROS2 (Humble or later)
- MoveIt2
- Python 3.12+
- trimesh (`pip install trimesh`)
- Universal Robots Package
- ABB Robot Package

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
pose.position.x = 0.11    # left/right relative to tool0
pose.position.y = -0.01  # up/down relative to tool0
pose.position.z = 0.19   # along the tool axis (forward/back)
pose.orientation.x = 0.0
pose.orientation.y = 0.9817477042
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
source install/setup.bash
ros2 launch cutting_robot cutting_robot.launch.py robot:= <robot type>
```

---

## Known Issues
- Duplicate models appearead ir RVIZ
- Arm motion path is still being finalized

---
