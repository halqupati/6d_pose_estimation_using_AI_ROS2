# Nut tightening in simulation — Phase 3: find the bolt with the camera

Until Phase 2 the robot read the bolt position from parameters. Phase 3 makes it find the bolt
itself with the overhead depth camera, the same camera YOLO uses for the nuts.

Branch: `bolt_tightening/jaka_s12`.

## 1. Why not YOLO?

The YOLO model is trained on one class, `nut`. Training a bolt class would need a labelled
dataset. The bolt is easy to find geometrically instead, because of its height:

| Object | Height above the table | Size seen from above |
|---|---|---|
| Nut | 16 mm | 30–35 mm |
| ChArUco / checkerboard | ~1 mm | large |
| **Bolt tip** | **40 mm** | **Ø20 mm disc** |
| Fixture plate (the seat) | 20 mm | 100 mm square |

## 2. The detector: `bolt_detector.py`

`jaka_ros_ws/src/bin_packing_pkg/scripts/bolt_detector.py`, started by the simulation launch. It
processes the depth image once per second:

1. **Back-project** the depth image (every 2nd pixel) to 3-D points in `base_link`, using TF
   `base_link ← camera_depth_optical_frame`.
2. **Table height** = the most common height (2 mm histogram bins): the largest flat surface.
3. **Tip candidates:** points 28–60 mm above the table, grouped into clusters on a 4 mm XY grid
   (`cv2.connectedComponents`). A cluster counts only if it is 10–28 mm wide, which matches a bare
   bolt tip. A nut on the bolt makes it about 30 mm wide, so the detector ignores it.
4. The **centre** comes from the flat top face only (points within 3 mm of the tip height). Seen
   at an angle, the shank's side also falls in the band and pulled the centre about 2 mm sideways.
5. **Seat:** the median height of points in a ring 25–45 mm from the axis, outside a nut's
   17.3 mm corners, between the table and the tip. That is the fixture plate.
6. If exactly one bolt is found, it publishes `/bolt_detector/tip` and `/bolt_detector/seat`
   (`geometry_msgs/PointStamped`, `base_link`).

### Surprise: Gazebo's `camera_info` is wrong

The first run found the bolt at the right height but at (1.05, 0.60) instead of (−0.30, 0.75).
`/rgbd_camera/camera_info` reports `fx = 277, cx = 160, cy = 120`, which are defaults for a
320×240 image, while the image is 1280×720. That is why the YOLO node hard-codes its intrinsics.
The detector now checks `camera_info`: if the principal point is not near the image centre, it
computes the intrinsics from the sensor's horizontal field of view (1.50098 rad in
`_d455.urdf.xacro`):

    fx = fy = (width / 2) / tan(hfov / 2) = 686.8,   cx = 639.5,   cy = 359.5

This is exactly the YOLO node's value.

### Accuracy

| Bolt position | Detected | Error |
|---|---|---|
| (−0.300, 0.750) | (−0.3004, 0.7495), tip 0.0370, seat 0.0170 | 0.5 mm |
| (−0.150, 0.720), moved and rotated 0.4 rad | (−0.1506, 0.7193), tip 0.0370, seat 0.0170 | 0.9 mm |

## 3. The robot uses the detection

`bin_picking_node_v5_to_control_jaka_s12.cpp`:

- New parameter `bolt_pose_source`: `camera` (default) or `parameters`. It is also a launch
  argument.
- `resolveBoltPose()` waits up to 20 s for a detection, checks that the tip is 10–60 mm above the
  seat, then sets the bolt x/y, tip and seat used by everything that follows. It runs before a
  nut covers the tip: at the start of `tighten`, before the preload, and before placing the nut in
  `pick_tighten`.
- `tighten`: the preloaded nut height is now computed as seat + `tighten_preload_turns` × pitch,
  so it follows the detected seat.

**Test with the bolt moved:** the fixture was moved to (−0.15, 0.72) and rotated 0.4 rad in a
copy of the world, with no parameter changes. The robot detected the bolt, placed nut2 0.5 mm off
the axis, and ran it down 7.60 turns to the seat (ground truth).

## 4. Data changed the snug threshold

The same test showed a weakness. The nut seated during stroke 16, but the torque only reached
1.34 N·m before the jaws slipped around it, below the 1.5 N·m threshold. The robot counted 8.00
turns while the nut had turned 7.60. Across all trials so far:

| | Range seen |
|---|---|
| Running torque (nut turning freely) | 0.01–0.21 N·m |
| Slip torque (jaws slip on a seated nut) | 1.3–2.8 N·m, depending on how squarely the jaws sit on the flats |

**The default `tighten_snug_torque` is now 1.0 N·m**: about 5× the running torque and below every
slip torque seen.

## 5. Running it

The detector starts with the simulation launch. Check it with:

```bash
ros2 topic echo /bolt_detector/tip
```

To fall back to fixed parameters:

```bash
ros2 launch bin_packing_pkg run_bin_picking_nodes_to_control_jaka_s12.launch.py \
    robot_mode:=sim use_sim_time:=true task_mode:=pick_tighten bolt_pose_source:=parameters
```
