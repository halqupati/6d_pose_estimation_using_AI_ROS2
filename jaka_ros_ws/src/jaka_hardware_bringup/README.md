# jaka_hardware_bringup

Bring-up package for running JAKA S12 + custom gripper + RealSense on real hardware.

For the recommended calibration entry points, use the dedicated wrapper packages instead of driving calibration directly from this package:

- eye-in-hand unified launch:
  [calibration.launch.py](/home/ubuntu/6d_pose_estimation/6d_pose_estimation_using_AI_ROS2/jaka_ros_ws/src/jaka_eye_in_hand_calibration/launch/calibration.launch.py)
- eye-to-hand unified launch:
  [calibration.launch.py](/home/ubuntu/6d_pose_estimation/6d_pose_estimation_using_AI_ROS2/jaka_ros_ws/src/jaka_eye_to_hand_calibration/launch/calibration.launch.py)

This package still contains the lower-level robot, camera, and custom ChArUco calibration tooling that those wrappers build on.

## Folder structure

- `launch/robot_hw.launch.py`: S12 driver + MoveIt + optional RViz + optional S12 bin picking launch
- `launch/camera_d435_eye_in_hand.launch.py`: RealSense + eye-in-hand TF + optional point cloud filtering
- `launch/hardware_full.launch.py`: full-stack orchestrator for robot, camera, and planner
- `config/hardware_profile.yaml`: editable template for IP and pipeline toggles
- `config/eye_in_hand_charuco_board.yaml`: ChArUco board template for hand-eye calibration
- `scripts/eye_in_hand_charuco_calibration`: interactive eye-in-hand calibration tool

## Typical commands

Build and source:

```bash
cd /home/ubuntu/6d_pose_estimation/6d_pose_estimation_using_AI_ROS2/jaka_ros_ws
colcon build --packages-select jaka_hardware_bringup
source install/setup.bash
```

Start full hardware stack:

```bash
ros2 launch jaka_hardware_bringup hardware_full.launch.py robot_ip:=192.168.1.100
```

Start full stack without planner:

```bash
ros2 launch jaka_hardware_bringup hardware_full.launch.py launch_planner:=false
```

Start only robot stack:

```bash
ros2 launch jaka_hardware_bringup robot_hw.launch.py robot_ip:=192.168.1.100
```

Start only camera stack:

```bash
ros2 launch jaka_hardware_bringup camera_d435_eye_in_hand.launch.py
```

## Driver integration

`robot_hw.launch.py` is driver-agnostic. Set these arguments for your real JAKA driver package:

- `driver_package` default: `jaka_driver`
- `driver_executable` default: `jaka_driver`
- `driver_namespace` default: empty
- `robot_ip`

Example:

```bash
ros2 launch jaka_hardware_bringup robot_hw.launch.py \
  driver_package:=<your_jaka_driver_pkg> \
  driver_executable:=<your_driver_node> \
  robot_ip:=192.168.1.100
```

## Eye-in-hand calibration

For a wrist-mounted camera, what you need is hand-eye calibration:

- robot base -> wrist link comes from robot TF
- calibration target -> camera comes from image pose estimation
- solver returns wrist link -> camera transform

The board in your photo looks like a **ChArUco board**, not a plain checkerboard. That is fine, but the board geometry must be exact.

### 1. Measure the board

Edit [eye_in_hand_charuco_board.yaml](/home/ubuntu/6d_pose_estimation/6d_pose_estimation_using_AI_ROS2/jaka_ros_ws/src/jaka_hardware_bringup/config/eye_in_hand_charuco_board.yaml:1) and set:

- `squares_x`: number of chessboard squares across
- `squares_y`: number of chessboard squares down
- `square_length_m`: full square edge length in meters
- `marker_length_m`: inner ArUco marker edge length in meters
- `aruco_dictionary`: likely `DICT_4X4_50` for a board like yours, but verify it

From your photo the board appears to be about `4 x 6` squares. Verify by counting and measure it with a ruler or caliper. Do not guess the sizes.

### 2. Start robot state and camera, but disable the unknown wrist-camera TF

Start the robot so TF publishes `base_link -> Link_06`.

For S12:

```bash
ros2 launch jaka_hardware_bringup robot_hw.launch.py \
  robot_ip:=192.168.1.100 \
  use_move_group:=false \
  use_rviz:=false
```

Start the camera without publishing the mount TF:

For S12 / `Link_06`:

```bash
ros2 launch jaka_hardware_bringup camera_d435_eye_in_hand.launch.py \
  parent_frame:=Link_06 \
  publish_mount_tf:=false
```

Important:

- Keep the calibration board fixed on the table for the entire session.
- Move only the robot.
- During calibration you must not publish a guessed `Link_06 -> camera_link` transform.

### 3. Run the interactive calibration tool

For S12 / `Link_06`:

```bash
ros2 run jaka_hardware_bringup eye_in_hand_charuco_calibration \
  --image-topic /camera/camera/color/image_raw \
  --camera-info-topic /camera/camera/color/camera_info \
  --base-frame base_link \
  --effector-frame Link_06 \
  --mount-child-frame camera_link \
  --board-config /home/ubuntu/6d_pose_estimation/6d_pose_estimation_using_AI_ROS2/jaka_ros_ws/src/jaka_hardware_bringup/config/eye_in_hand_charuco_board.yaml \
  --output-dir calibration_runs
```

If you still need a legacy wrist frame, override `parent_frame` or `--effector-frame` explicitly. The bring-up defaults now target S12 / `Link_06`.

Controls:

- `c`: capture one sample
- `s`: solve and save calibration
- `r`: clear captured samples
- `q`: quit

### 4. How to collect good samples

- Capture at least `12` poses. `15` to `25` is better.
- Change height, distance, tilt, and yaw of the wrist.
- Keep the whole board visible in the image.
- Do not capture many nearly identical poses.
- Reduce glare. Your board cover has visible reflections in the photo, which can hurt corner detection.

### 5. Use the result

The tool saves a timestamped folder under `calibration_runs/` containing:

- `eye_in_hand_result.yaml`
- `captured_samples.yaml`
- preview images if you add `--save-preview-images`

The script prints the final transform for:

- parent: `Link_06`
- child: `camera_link`

and also prints a ready launch command like:

```bash
ros2 launch jaka_hardware_bringup camera_d435_eye_in_hand.launch.py \
  parent_frame:=Link_06 \
  camera_frame:=camera_link \
  camera_x:=... camera_y:=... camera_z:=... \
  camera_roll:=... camera_pitch:=... camera_yaw:=...
```

### 6. Why the solver uses the image frame but the final result is for `camera_link`

The board pose is estimated from the image stream, so internally the solver works in the image frame, usually something like:

- `camera_color_optical_frame`

The tool then converts that result back to `camera_link` using the RealSense internal TF chain. That makes the output directly usable by your current launch file.

## Notes

- If your driver already publishes TF and robot state, keep `rsp.launch.py` only if needed.
- Set `use_bin_picking:=true` only after planning and controller topics are verified.
- During calibration, always run the camera launch with `publish_mount_tf:=false`.
