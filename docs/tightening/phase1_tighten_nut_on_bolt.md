# Nut tightening in simulation — Phase 1: tighten a nut that is already on the bolt

Phase 0 ([phase0_screw_thread.md](phase0_screw_thread.md)) proved that a Gazebo screw joint
behaves like an M20 thread. Phase 1 makes the JAKA S12 use it: grip the nut, turn it down the
bolt and stop when it is snug.

Branch: `bolt_tightening/jaka_s12`.

## 1. Plan

1. Put a bolt fixture with a threaded nut into the main world.
2. Give the robot a way to *feel* the tightening torque.
3. Add a `tighten` task to the bin-picking node: grip, turn a stroke, release, turn back, repeat,
   and stop at the snug torque.
4. Measure what torque the gripper can actually transmit, then choose the snug threshold.
5. Add the task to the launcher.

## 2. The bolt fixture

`jaka_ros_ws/src/jaka_description/worlds/bolt_m20/model.sdf`, included in `worlds/world.sdf` as
`bolt_fixture` at world (-0.30, 0.75) on `table_2`. That is 6 cm from the ChArUco board and 26 cm
from the nearest pick nut.

| Part | Geometry | Why |
|---|---|---|
| Fixture plate | 100 × 100 × 20 mm, top at 20 mm = **the seat** | The nut stops on it |
| M20 shank | Ø20 mm, top at 40 mm | Only 4 mm above the seated nut. A longer bolt would hit the gripper's jaw rails, which sit about 25 mm above the fingertips. |
| Nut | Three-box hexagon (same as the pick nut), starts **3 turns** (7.5 mm) above the seat | The gripper grips it exactly like a pick-and-place nut |
| `thread` joint | The Phase 0 screw joint: `thread_pitch = -2513.2741`, lower limit −6π rad = seat, friction 0.05 N·m | Right-hand M20, self-locking |

In the robot base frame (1.018 m below the world), the seat is at z = 0.017 and the nut bottom
starts at z = 0.0245. These are the node's default `tighten_*` parameters.

## 3. Feeling the torque: a wrist force-torque sensor

**First attempt: joint effort.** An `effort` state interface was added to the arm joints
(`jaka_s12.ros2_control.xacro`, `ros2_controllers.yaml`). Result: `/joint_states` effort is
**NaN** for every joint, because gz_ros2_control (Humble) does not report joint torque.

**Working solution: a wrist F/T sensor**, like a real tightening cell would use.

- `custom_gripper_macro.urdf.xacro`: a `force_torque` sensor on `gripper_base_to_body_joint`
  (flange ↔ gripper body), measured in the gripper body frame. Its **torque about z is the
  torque on the nut**, because body z is the tool axis. `preserveFixedJoint` stops the URDF→SDF
  conversion from merging that fixed joint away; without a joint there is nothing to measure.
- `worlds/world.sdf`: the `ForceTorque` system as a **world** plugin. Loaded from the robot model
  it never created the sensor (`No publishers on topic [/wrist_ft]`): the robot is spawned into a
  running world and the system missed the sensor. Declaring any world plugin turns off Gazebo's
  default systems, so Physics, UserCommands and SceneBroadcaster are listed explicitly.
- The sim launch bridges `/wrist_ft` to ROS as `geometry_msgs/WrenchStamped`. It also bridges the
  thread angle to `/bolt_fixture/joint_states`, which is simulation ground truth used for tests
  and the launcher only; the robot never uses it.

At rest the sensor reads force z ≈ 5.9 N (the gripper's weight) and torque z ≈ 0.0002 N·m.

## 4. The `tighten` task in the node

`bin_picking_node_v5_to_control_jaka_s12.cpp`, started with `task_mode:=tighten`
(simulation only: the node refuses it with `robot_mode=real`).

```
approach:  ready pose -> above the nut, jaws at nut_yaw + 30 deg (on two flats)
repeat:
  1. open, move the fingertips to 3 mm above the nut's bottom face
  2. close on two flats
  3. turn 120 deg clockwise while moving down 2.5 mm per turn (follow the thread),
     with the torque monitor armed
  4. if the monitor saw the snug torque -> done
     else open, turn the open jaws back 120 deg (no height change) and repeat
finish:    open, lift, ready pose, report turns / strokes / peak torque / time
```

Key pieces, in reading order:

- **`onWristWrench`** runs on its own callback group, in parallel with the blocked main loop.
  While a stroke is armed it low-pass filters |torque_z − baseline| (about 25 ms time constant).
  At the threshold it calls `arm_group_->stop()`, which cancels the running trajectory.
- **`armTorqueMonitor`** takes the baseline just before each stroke, so any static torque from
  the grip is removed.
- **`helixWaypoints`** builds the stroke as 30° Cartesian steps around the bolt axis, with z
  following `thread_pitch × angle / 2π`.
- **Why 120°?** The stroke must be a multiple of the hexagon's 60° symmetry, so the open jaws meet
  two flats again after turning back. It must also be at most 180°, so the direction of rotation
  is unambiguous. The node rejects other values.
- **Safety:** the fingertips are never commanded below 1 mm above the plate. There is a
  `tighten_max_turns` stop, and the gripper always opens on failure.

Launch arguments (`run_bin_picking_nodes_to_control_jaka_s12.launch.py`): `task_mode`,
`tighten_snug_torque`, `tighten_stroke_deg`, `tighten_max_turns`, `tighten_velocity_scaling`.

## 5. Experiment: how much torque can the gripper transmit?

Trial T1: snug threshold set to 50 N·m (unreachable on purpose) and max 4 turns.

| Phase | Filtered peak torque per stroke |
|---|---|
| Strokes 1–9, threading the free-running nut | **0.01 N·m** |
| Strokes 10–13, against the seated nut | **2.55–2.80 N·m**, then the jaws slip around the nut |

Ground truth: the thread angle reached −18.8495 rad (exactly 3 turns, the seat), and the nut
stopped at z = 20.00 mm.

**Conclusion:** with 50 N of jaw force on 30 mm flats, this parallel gripper slips at about
**2.6 N·m**, so the first target of 5 N·m is not reachable. The snug threshold is **1.5 N·m**:
about 150× the running torque, so it never triggers early, and about 40% below the slip torque,
so the nut is still held when it triggers. A real cell would reach specification torque with a
nut runner or socket tool.

## 6. Result

Trial T2, with the defaults `tighten_snug_torque:=1.5` and `tighten_velocity_scaling:=0.5`:

```
Stroke 1: turned 120 deg, total 0.33 turns, nut bottom ~0.0237 m, peak torque 0.06 N*m
...
Stroke 9: turned 120 deg, total 3.00 turns, nut bottom ~0.0170 m, peak torque 0.02 N*m
Snug torque reached: 1.50 N*m (threshold 1.50 N*m). Stopping the stroke.
Stroke 10: turned 0 deg, total 3.00 turns, nut bottom ~0.0170 m, peak torque 1.61 N*m -> SNUG
Nut is snug: 3.00 turns in 10 strokes, peak torque 1.61 N*m, 366 s
```

Ground truth after the run: thread angle −18.8496 rad (the seat) and nut bottom at 20.00 mm.
The 366 s includes CPU contention from a second simulation running at the same time
(about 37 s per stroke).

## 7. Running it

From the launcher: choose **Task: Tighten (bolt)** in the header, then **Start All** (or start
Gazebo and Bin Picking Nodes). The **Bolt Thread** tile shows the turns and the gap to the seat
from simulation ground truth.

Manually:

```bash
ros2 launch bin_packing_pkg jaka_s12_with_two_camera_simulation_with_pcl_and_custom_gripper.launch.py
ros2 launch bin_packing_pkg run_bin_picking_nodes_to_control_jaka_s12.launch.py \
    robot_mode:=sim use_sim_time:=true task_mode:=tighten
```

Watch the torque with `ros2 topic echo /wrist_ft` and the thread with
`ros2 topic echo /bolt_fixture/joint_states`.

## 8. Limits and next steps

- **Bolt pose is known:** it comes from parameters. Phase 3 can detect it with the camera.
- **Speed:** each stroke spends most of its time opening and closing the gripper. Larger strokes
  (180°) cut the number of regrasps from 9 to 6.
- **Hardware:** a real wrist F/T sensor or joint-torque sensing, a measured grip-torque limit and
  low-speed tests are needed before tightening runs on the physical robot.
- **Phase 2:** pick a free nut, place it on the bolt tip, swap it for the threaded nut, then
  tighten.
