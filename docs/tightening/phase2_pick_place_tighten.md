# Nut tightening in simulation — Phase 2: pick a nut, put it on the bolt, tighten it

Phase 1 ([phase1_tighten_nut_on_bolt.md](phase1_tighten_nut_on_bolt.md)) tightened a nut that
was already on the bolt. Phase 2 does the whole job: pick a free nut from the table, set it on
the bolt tip, let the thread catch, and run it down to snug.

Branch: `bolt_tightening/jaka_s12`.

## 1. The problem: a thread cannot "catch" in Gazebo

The thread is a screw joint (Phase 0). A joint must exist when the world loads; Gazebo cannot
attach a screw joint to a free nut during a run. A free nut set on a smooth cylinder would just
sit there.

## 2. The idea: swap the models at the moment the thread would catch

When the nut rests on the bolt tip, a simulation helper replaces two models with one:

| Before | After |
|---|---|
| Empty bolt fixture + free nut lying on its tip | Identical fixture with a nut on a screw joint, at the same position and angle |

For the robot nothing changes: the nut is where it put it. The robot still works only from its
own knowledge (where it placed the nut, at what jaw angle) and its wrist torque. It never reads
the simulator's ground truth.

## 3. The world: an empty bolt

`jaka_description/worlds/bolt_m20/model.sdf` now contains only the fixture plate (seat at 20 mm)
and the shank (tip at 40 mm). In the robot base frame the seat is at z = 0.017 and the tip at
z = 0.037.

## 4. The helper: `bolt_fixture_manager.py`

`bin_packing_pkg/scripts/bolt_fixture_manager.py`, started by the simulation launch file. It
offers two `std_srvs/Trigger` services:

| Service | What it does | Used by |
|---|---|---|
| `/bolt_fixture/engage_nut` | Finds a free nut whose bottom face rests on the bolt tip within 6 mm of the axis, removes it and the empty fixture, and spawns the threaded fixture with the nut 1 mm below the tip and at the same yaw | `task_mode:=pick_tighten` |
| `/bolt_fixture/preload_nut` | Spawns the threaded fixture with a nut 3 turns above the seat | `task_mode:=tighten` (Phase 1 task) |

How it works:

- It talks to Gazebo through the `ign` command line: `ign model --list`, `ign model -m <name> -p`
  for poses, and the world's `remove` and `create` services.
- `threaded_fixture_sdf()` generates the threaded fixture as SDF text: the same plate and shank,
  the three-box hexagon nut, and the screw joint with `thread_pitch = -2π / 0.0025`. The joint
  limits are computed from the nut's start height: lower limit = the seat, upper limit = just off
  the tip.
- The spawned model keeps the name `bolt_fixture`, so `/bolt_fixture/joint_states` and the
  launcher tile keep working.

Tested on its own:

| Check | Result |
|---|---|
| Empty fixture at start | no nut ✓ |
| `preload_nut` | nut at 27.5 mm (3 turns above the seat) ✓ |
| `preload_nut` again | refused: "the bolt already has a threaded nut" ✓ |
| `engage_nut` with nothing on the tip | refused with the reason ✓ |
| `engage_nut` with nut1 on the tip (2.2 mm off-axis, 30° rotated) | nut1 removed, threaded nut at 39.0 mm = 7.60 turns ✓ |

## 5. The robot: `task_mode:=pick_tighten`

`bin_picking_node_v5_to_control_jaka_s12.cpp`:

1. **Pick:** the normal pick sequence 0–4 (detect, hover, descend, grip, lift) is unchanged.
2. **Sequence 5** branches to `placeNutOnBoltAndTighten()`:
   - **Where is the nut in the gripper?** At the grasp the TCP was at `validated_pick_pose_` and
     the nut bottom at detected top − 16 mm, so `tcp_above_nut_bottom` is known.
   - Move above the bolt, keeping the jaw angle the nut was picked with.
   - Lower until the nut bottom is 1 mm above the tip, open, and move up 3 cm.
   - Call `/bolt_fixture/engage_nut`, the simulated thread catch.
   - **The robot's own estimate:** the nut bottom is just below the tip, and the nut's corner
     axis is the jaw angle at release − 30°.
   - Run the Phase 1 tightening loop from there. `runTighteningCycle(nut_bottom_z, nut_yaw,
     max_turns)` now takes its start as arguments; max turns = turns to the seat + 1.
3. One nut per run, since there is one bolt.

`task_mode:=tighten` now first calls `/bolt_fixture/preload_nut`, then tightens as in Phase 1.

## 6. Result

Trial P1 (nut2 replayed from a recorded YOLO detection, 180° strokes):

```
Sequence 0..4: pick and lift nut2
Place on bolt: tcp is 0.0280 m above the nut bottom
Bolt fixture: nut2 engaged: off-axis 1.3 mm, nut bottom 39.0 mm above the table, 7.60 turns above the seat
Stroke 1: turned 180 deg, total 0.50 turns, nut bottom ~0.0348 m, peak torque 0.02 N*m
...
Stroke 15: turned 180 deg, total 7.50 turns, nut bottom ~0.0173 m, peak torque 0.02 N*m
Snug torque reached: 1.54 N*m (threshold 1.50 N*m). Stopping the stroke.
Stroke 16: turned 38 deg, total 7.60 turns, nut bottom ~0.0170 m, peak torque 1.75 N*m -> SNUG
Nut is snug: 7.60 turns in 16 strokes, peak torque 1.75 N*m, 288 s
```

Ground truth: thread angle −47.75 rad (7.60 turns, which matches the robot's own count) and the
nut bottom on the seat (20.00 mm). About 5 minutes from detection to snug.

## 7. A launcher bug found along the way

The C++ build was killed twice ("Terminated", then "Killed"). The launcher's Stop/Start cleanup
stopped *any* process whose command line contained a pattern such as
`bin_picking_node_v5_to_control_jaka_s12`, and the compiler building
`bin_picking_node_v5_to_control_jaka_s12.cpp` matched. `find_process_targets` now skips shells,
compilers, build tools, editors and common command-line tools (`NON_TARGET_EXECUTABLES`).

## 8. Running it

Launcher: **Task: Pick + tighten**, then **Start All**. YOLO and AAE pick a nut; the **Bolt
Thread** tile shows "No nut", then the turns and the gap to the seat.

Manually (with the perception nodes running):

```bash
ros2 launch bin_packing_pkg jaka_s12_with_two_camera_simulation_with_pcl_and_custom_gripper.launch.py
ros2 launch bin_packing_pkg run_bin_picking_nodes_to_control_jaka_s12.launch.py \
    robot_mode:=sim use_sim_time:=true task_mode:=pick_tighten tighten_stroke_deg:=180.0
```

## 9. Limits

- The thread catch is a model swap: placement accuracy matters (6 mm tolerance), but alignment of
  real thread starts is not modelled.
- The bolt pose comes from parameters, not from the camera.
- Simulation only: real hardware needs a real wrist torque signal and a measured grip-torque limit.
