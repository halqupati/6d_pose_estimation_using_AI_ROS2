# gripper_description

ROS2 URDF package for the 2-finger parallel jaw gripper.
Built from your 3 real STL files: `body.STL`, `jaw_left.STL`, `jaw_right.STL`.

---

## Package Structure

```
gripper_description/
├── CMakeLists.txt
├── package.xml
├── urdf/
│   └── gripper.urdf         ← Robot description
├── meshes/
│   ├── body.STL             ← Main body (107.5 x 75 x 105.5 mm)
│   ├── jaw_left.STL         ← Left jaw (slides -Y to open)
│   └── jaw_right.STL        ← Right jaw (slides +Y to open)
├── launch/
│   └── display.launch.py    ← Launch RViz2 + jaw slider GUI
└── rviz/
    └── gripper.rviz         ← RViz2 config
```

---

## Measured Geometry (from STL bounding boxes)

| Part | X (mm) | Y (mm) | Z (mm) |
|---|---|---|---|
| body | 107.5 | 75.0 | 105.5 |
| jaw_left | 43.9 | 19.5 | 45.0 |
| jaw_right | 43.9 | 19.5 | 45.0 |

---

## Kinematic Tree

```
base_link  (flange mounting face — connect to JAKA Zu5 tool0)
  └── body  [FIXED, offset z=-13.5mm]
        ├── jaw_left   [PRISMATIC, Y axis, range: -25mm to 0]
        └── jaw_right  [PRISMATIC, Y axis, range:  0 to +25mm]
              └── tcp  [FIXED, tool center point at z=49mm]
```

**Opening the gripper:** jaw_left moves in -Y, jaw_right moves in +Y.
Total max opening = 50mm (25mm per side).

---

## Installation

```bash
# 1. Place in your ROS2 workspace
cp -r gripper_description ~/ros2_ws/src/

# 2. Build
cd ~/ros2_ws
colcon build --packages-select gripper_description
source install/setup.bash

# 3. Visualize
ros2 launch gripper_description display.launch.py
```

You will see RViz2 open with the gripper, and a slider GUI to open/close the jaws.

---

## Attaching to the JAKA Zu5

Add this to your combined robot URDF or launch file:

```xml
<joint name="jaka_to_gripper" type="fixed">
  <parent link="tool0"/>       <!-- JAKA Zu5 end-effector link -->
  <child link="base_link"/>    <!-- Gripper base -->
  <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
</joint>
```

---

## Notes

- **TCP link** is the tool center point — use this as the planning target in MoveIt2.
- **Collision geometry** uses simplified boxes (not mesh) for fast motion planning.
- **Joint limits** set to ±25mm — adjust if your actual gripper travel is different.
- **Jaw mass** set to 50g estimate — update with real mass if available for accurate dynamics.
