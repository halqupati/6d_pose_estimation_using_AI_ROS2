# Nut tightening in simulation — Phase 0: modelling the M20 thread

Goal of the tightening work: the JAKA S12 picks an M20 nut, threads it onto a bolt and runs
it down until it is snug, all in Gazebo (Ignition Fortress).

Phase 0 answers one question before any robot code is written: **can Gazebo simulate the
thread?**

## 1. The problem

Gazebo cannot simulate real thread geometry (helical flanks touching each other). Even if it
could, it would be far too slow and unstable for a robot task.

## 2. The idea: model the thread as a joint

A **screw joint** couples two motions of the nut relative to the bolt:

    travel along the bolt axis = pitch x (rotation angle / 2*pi)

For an M20 coarse thread the pitch is **2.5 mm per revolution**. The bolt is fixed; the nut is
joined to it by the screw joint, so turning the nut automatically moves it along the bolt.

Two more properties come from the joint settings:

| Real thread behaviour | How it is modelled |
|---|---|
| A nut does not turn by itself (self-locking) | Joint `<friction>` = 0.05 N·m |
| Running torque while turning | The same friction, plus `<damping>` |
| The nut stops when it reaches the part (the seat) | Joint lower limit at the seat |

## 3. Experiment 1: what unit does `thread_pitch` use?

The SDF 1.9 specification does not say, and Gazebo versions differ. So the test world first
contained two identical nut-on-bolt models that only differed in the pitch value:

| `thread_pitch` | Interpretation tested | Result |
|---|---|---|
| `0.0025` | metres per revolution | The nut fell about 400 m per radian. **Wrong.** |
| `2513.2741` | radians per metre (2π / 0.0025) | 48.84 mm of travel over 122.76 rad = **2.500 mm/rev** |

**Result:** in Fortress/DART, `thread_pitch` is in **radians per metre**:

    thread_pitch = 2*pi / pitch = 2*pi / 0.0025 = 2513.2741

The same run showed the direction: a **positive** value makes a positive (counter-clockwise)
rotation move the nut **down**. That is a left-hand thread. Real nuts are right-hand, so the
final world uses **`-2513.2741`**:

- clockwise seen from above (negative angle) → nut moves down → **tightening**
- counter-clockwise (positive angle) → nut moves up → **loosening**

A side lesson: a one-shot `ign topic -p` often loses its message before the connection is
set up, which made the first torque tests misleading. The test script therefore sends the
torque through `ros_gz_bridge`, re-published at 50 Hz.

## 4. The test world

`jaka_ros_ws/src/jaka_description/worlds/tests/screw_thread_test.sdf`

- **Bolt link** (welded to the world): a 100 × 100 × 20 mm fixture plate (top face = the seat at
  z = 20 mm) and an M20 shank (Ø20 mm, 60 mm long).
- **Nut link**: the same three-box M20 hexagon collision used by the pick-and-place nut, starting
  3 turns (7.5 mm) above the seat, at z = 27.5 mm.
- **`thread` joint** (type `screw`): `thread_pitch = -2513.2741`, axis +Z, lower limit
  −6π rad (= −3 turns = the seat), friction 0.05 N·m, damping 0.01.
- **Test I/O plugins**: `ApplyJointForce` (torque command on the thread) and
  `JointStatePublisher` (thread angle and velocity).

The bolt and nut never collide with each other: they are in the same model and self-collision
is off, so the joint alone defines how they move.

## 5. The automated check

`tools/screw_thread_test.py` starts the world headless (private Gazebo partition and ROS domain,
so it never touches a running simulation), applies torques and checks:

1. **Start pose**: the nut is at 27.5 mm.
2. **Self-locking**: no rotation with a torque of 0.6 × the thread friction.
3. **Direction**: a clockwise torque moves the nut down.
4. **Pitch**: 2.5 mm per revolution.
5. **Seat**: with the torque still applied, the nut stops at the plate (the joint limit).
6. **Loosening**: a counter-clockwise torque moves the nut up.

Run it:

```bash
source /opt/ros/humble/setup.bash
python3 tools/screw_thread_test.py
```

Result (repeated twice, identical):

```
Screw-thread check (M20, Gazebo Fortress / DART)
  [PASS] start pose                                    z = 27.50 mm (expect 27.5)
  [PASS] self-locking (torque 0.6 x friction)          moved -0.00 deg
  [PASS] clockwise torque tightens (nut moves down)    -0.41 turns, dz = -1.02 mm
  [PASS] pitch 2.5 mm/rev                              2.499 mm/rev
  [PASS] stops at seat under torque                    angle = -18.850 rad (limit -18.850), z = 20.00 mm, vel = -0.000 rad/s
  [PASS] counter-clockwise loosens (nut moves up)      +0.38 turns, dz = +0.96 mm

Result: ALL PASSED
```

## 6. What this means for the next phases

- The screw joint works in Fortress, so no custom thread plugin is needed.
- The seat is a hard joint limit. When the robot turns a seated nut, the nut stops and the
  torque on the robot's wrist (`joint_6`) rises. Phase 1 uses that torque rise to detect "snug"
  and must stop at a software torque limit; the URDF's own `joint_6` effort limit (1000 N·m)
  is far too high to act as a stop.
- In Phase 1 the robot turns the nut by **friction between the finger pads and the nut flats**,
  not by a joint torque command. That is the next thing to verify.
- A screw joint cannot be created during a simulation run. Phase 2 (pick a free nut and thread
  it on) will need to swap the free nut for a threaded one when it lands on the bolt tip.
