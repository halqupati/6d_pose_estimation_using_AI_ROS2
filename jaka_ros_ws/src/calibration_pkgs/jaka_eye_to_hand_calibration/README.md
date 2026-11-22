# jaka_eye_to_hand_calibration

JAKA S12 eye-to-hand calibration wrapper package.

This package separates the full calibration problem into the same three layers as the eye-in-hand package:

- `camera_calibration` for camera intrinsics
- `moveit_calibration` for RViz-based hand-eye calibration
- `easy_handeye2` for TF-pair based hand-eye calibration

## Best single launch file

If you want one entry point for both real hardware and the currently available simulation support, use:

```bash
ros2 launch jaka_eye_to_hand_calibration calibration.launch.py runtime:=real method:=moveit
```

Supported combinations:

- `runtime:=real method:=intrinsics`
- `runtime:=real method:=moveit`
- `runtime:=real method:=easy_handeye`
- `runtime:=sim method:=intrinsics`
- `runtime:=sim method:=moveit`

Current limitation:

- `runtime:=sim method:=easy_handeye` is not complete yet

Reason:

- this repo now provides a fixed simulated world camera as `camera_link`
- and a simulation-only ChArUco target rigidly attached to `Link_06` for the MoveIt eye-to-hand workflow
- `easy_handeye2` is still not wired for the fixed-camera simulation path in this package

Examples:

```bash
ros2 launch jaka_eye_to_hand_calibration calibration.launch.py runtime:=real method:=moveit robot_ip:=192.168.1.100
ros2 launch jaka_eye_to_hand_calibration calibration.launch.py runtime:=real method:=intrinsics
ros2 launch jaka_eye_to_hand_calibration calibration.launch.py runtime:=sim method:=intrinsics
ros2 launch jaka_eye_to_hand_calibration calibration.launch.py runtime:=sim method:=moveit
```

For the simulated intrinsic command above, you do not need to pass your real checkerboard values unless you replace the Gazebo board model. The wrapper now defaults to the built-in Gazebo checkerboard target:

- `sim_checkerboard_size:=7x7`
- `sim_square_size_m:=0.25`
- `use_gazebo_checkerboard_defaults:=true`

That matches the current `checkerboard_plane` model in the world, which is an 8x8-square checkerboard, so `camera_calibration` must use 7x7 inner corners.

Why this is still the best single launch for eye-to-hand:

- it gives you one consistent interface for real calibration
- it keeps the runtime/method arguments the same as the eye-in-hand package
- it now supports both simulated intrinsics and simulated MoveIt hand-eye on the fixed world camera

Simulation naming:

- fixed world camera base frame: `camera_link`
- fixed world camera optical frame: `camera_color_optical_frame`
- fixed world camera ROS topics: `/rgbd_camera/...`
- wrist eye-in-hand camera base frame: `mounted_camera_link`
- wrist eye-in-hand optical frame: `mounted_camera_color_optical_frame`
- wrist eye-in-hand ROS topics: `/rgbd_mounted_camera/...`

Simulation target note:

- the built-in intrinsic target is a checkerboard model, not an ArUco or ChArUco board
- `camera_calibration` should stay on checkerboard or circle-grid targets
- if you add an ArUco or ChArUco model to Ignition Gazebo, use that for hand-eye target detection, not for `method:=intrinsics`
- you can select a different simulation world with `sim_world:=your_world.sdf`
- for `runtime:=sim method:=moveit`, the launch adds a separate ChArUco target visual to `Link_06`

## Manual robot and camera bringup

If you do not want the unified wrapper launch, you can start the robot side and the camera side manually.

This is useful when:

- you want to debug TF separately
- you want to run RViz by hand
- your fixed external camera has its own vendor launch file
- you want to verify topics before starting calibration

### Robot launch for eye-to-hand

For eye-to-hand calibration, the robot side should publish:

- `base_link -> Link_06`
- joint states
- MoveIt only if the selected calibration method needs it

Robot only, no MoveIt and no RViz:

```bash
ros2 launch jaka_hardware_bringup robot_hw.launch.py \
  robot_ip:=192.168.1.100 \
  use_move_group:=false \
  use_rviz:=false \
  use_bin_picking:=false
```

Robot with MoveIt and RViz for the MoveIt hand-eye plugin:

```bash
ros2 launch jaka_hardware_bringup robot_hw.launch.py \
  robot_ip:=192.168.1.100 \
  use_move_group:=true \
  use_rviz:=true \
  use_bin_picking:=false
```

This is the correct robot launch base for eye-to-hand in this repo:

- [robot_hw.launch.py](/home/ubuntu/6d_pose_estimation/6d_pose_estimation_using_AI_ROS2/jaka_ros_ws/src/jaka_hardware_bringup/launch/robot_hw.launch.py)

### Camera launch for eye-to-hand

For eye-to-hand, the camera is fixed in the workcell, so during calibration you must not publish a guessed:

- `base_link -> camera_link`

If you are using the existing D435 wrapper only as a camera driver, run it with mount TF disabled:

```bash
ros2 launch jaka_hardware_bringup camera_d435_eye_in_hand.launch.py \
  publish_mount_tf:=false \
  enable_filter:=false
```

Important:

- in eye-to-hand calibration, `publish_mount_tf:=false` is mandatory during sampling
- if you publish a guessed `base_link -> camera_link`, the calibration problem is corrupted
- if your external camera has its own ROS 2 launch, use that instead and only keep the same image and camera info topics

The low-level camera wrapper used in this repo is:

- [camera_d435_eye_in_hand.launch.py](/home/ubuntu/6d_pose_estimation/6d_pose_estimation_using_AI_ROS2/jaka_ros_ws/src/jaka_hardware_bringup/launch/camera_d435_eye_in_hand.launch.py)

### Manual combinations by method

For `camera_calibration` intrinsics:

1. Start the camera only.
2. Robot launch is optional.
3. Run `camera_calibration`.

For MoveIt hand-eye:

1. Start `robot_hw.launch.py` with `use_move_group:=true use_rviz:=true`.
2. Start the camera with `publish_mount_tf:=false`.
3. Open the `HandEyeCalibration` panel in RViz.

For `easy_handeye2`:

1. Start `robot_hw.launch.py` with at least TF and joint states.
2. Start the camera without publishing the unknown static transform.
3. Start your detector so it publishes `camera_color_optical_frame -> handeye_target`.
4. Run the `easy_handeye2` calibration launch.

The intended hardware layout is:

- camera fixed in the workcell, not attached to the robot
- calibration target rigidly attached to the robot end effector
- robot base frame: `base_link`
- tool frame: `Link_06`
- camera optical frame: usually `camera_color_optical_frame`

## What changes in eye-to-hand

The unknown transform is different from eye-in-hand.

For eye-to-hand, the solver estimates:

- `base_link -> camera_link`

The target is now the moving object:

- target is attached to `Link_06`
- camera stays fixed in the cell

That is why the sample geometry is inverted compared with a wrist-mounted camera.

## External dependencies

Install the apt packages first:

```bash
sudo apt install python3-vcstool ros-humble-camera-calibration
```

Import the source dependencies:

```bash
cd /home/ubuntu/6d_pose_estimation/6d_pose_estimation_using_AI_ROS2/jaka_ros_ws
vcs import src < src/jaka_eye_to_hand_calibration/config/third_party_calibration.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Notes:

- `moveit_calibration` is imported from the upstream `ros2` branch
- `easy_handeye2_camera_calibration_with_ros2` is imported from the fork you specified
- the unified eye-to-hand launch reuses the eye-in-hand simulation package for simulated intrinsic calibration only
- if your external camera already has its own ROS 2 driver launch, you can keep that and only reuse the frame and topic settings documented here

### Humble build note for `moveit_calibration`

If `colcon build` fails in `moveit_calibration_gui` with errors such as:

```text
fatal error: cv_bridge/cv_bridge.hpp: No such file or directory
fatal error: image_geometry/pinhole_camera_model.hpp: No such file or directory
fatal error: moveit/move_group_interface/move_group_interface.hpp: No such file or directory
```

this is usually a ROS 2 Humble header-name mismatch, not a missing dependency.

On Humble, the installed headers use `.h`, while the cloned `moveit_calibration` source may include `.hpp`.

Patch these three includes inside `moveit_calibration_gui`:

- `cv_bridge/cv_bridge.hpp` -> `cv_bridge/cv_bridge.h`
- `image_geometry/pinhole_camera_model.hpp` -> `image_geometry/pinhole_camera_model.h`
- `moveit/move_group_interface/move_group_interface.hpp` -> `moveit/move_group_interface/move_group_interface.h`

The detailed fix is also documented in:

- [easy_handeye2_camera_calibration_with_ros2/README.md](/home/ubuntu/6d_pose_estimation/6d_pose_estimation_using_AI_ROS2/jaka_ros_ws/src/easy_handeye2_camera_calibration_with_ros2/README.md)

## 1. Intrinsic calibration with `camera_calibration`

Start the camera and the intrinsic calibration GUI:

```bash
ros2 launch jaka_eye_to_hand_calibration camera_intrinsics.launch.py \
  checkerboard_size:=8x6 \
  square_size_m:=0.024
```

Or use the unified launch:

```bash
ros2 launch jaka_eye_to_hand_calibration calibration.launch.py \
  runtime:=real \
  method:=intrinsics
```

For simulation, use:

```bash
ros2 launch jaka_eye_to_hand_calibration calibration.launch.py \
  runtime:=sim \
  method:=intrinsics
```

The simulated command above now uses the Gazebo checkerboard defaults automatically.

If you later replace the simulated checkerboard with one that matches your real 24 mm board, override the defaults:

```bash
ros2 launch jaka_eye_to_hand_calibration calibration.launch.py \
  runtime:=sim \
  method:=intrinsics \
  use_gazebo_checkerboard_defaults:=false \
  checkerboard_size:=8x6 \
  square_size_m:=0.024
```

Defaults:

- image topic: `/camera/camera/color/image_raw`
- camera namespace for `set_camera_info`: `/camera/camera/color`
- no robot-to-camera static TF is published during calibration

Intrinsic calibration is still exactly the same problem as in eye-in-hand. The difference only appears in the extrinsic step.

## 2. Eye-to-hand calibration with MoveIt RViz plugin

Start the robot, external camera, MoveIt, and RViz:

```bash
ros2 launch jaka_eye_to_hand_calibration moveit_handeye.launch.py \
  robot_ip:=192.168.1.100
```

Or use the unified launch:

```bash
ros2 launch jaka_eye_to_hand_calibration calibration.launch.py \
  runtime:=real \
  method:=moveit \
  robot_ip:=192.168.1.100
```

For simulation, use:

```bash
ros2 launch jaka_eye_to_hand_calibration calibration.launch.py \
  runtime:=sim \
  method:=moveit
```

The simulation MoveIt path uses the fixed world camera on `/rgbd_camera/...` and a built-in ChArUco target on the robot. In the HandEyeCalibration panel set:

- Target type: `ChArUco`
- squares X: `5`
- squares Y: `7`
- marker size (px): `200`
- square size (px): `320`
- margin size (px): `8`
- marker border (bits): `1`
- ArUco dictionary: `DICT_5X5_250`
- longest board side (m): `0.1971`
- measured marker size (m): `0.0176`

In RViz:

1. Open `Panels -> Add New Panel`.
2. Add `HandEyeCalibration`.
3. Enter your real printed target dimensions in the `Target` tab.
4. In `Context`, use:

- Sensor configuration: `Eye-to-hand`
- Sensor frame: `camera_color_optical_frame`
- Object frame: `handeye_target`
- End-effector frame: `Link_06`
- Robot base frame: `base_link`

5. For real hardware, attach the target rigidly to the tool. In simulation, the launch already attaches the ChArUco target to `Link_06`.
6. Collect samples with varied wrist orientation.
7. Save the final camera pose.

This path is the most integrated if you want everything inside RViz.

## 3. Eye-to-hand calibration with `easy_handeye2`

Start the easy-handeye wrapper:

```bash
ros2 launch jaka_eye_to_hand_calibration easy_handeye_calibrate.launch.py \
  robot_ip:=192.168.1.100 \
  tracking_base_frame:=camera_color_optical_frame \
  tracking_marker_frame:=handeye_target
```

Or use the unified launch:

```bash
ros2 launch jaka_eye_to_hand_calibration calibration.launch.py \
  runtime:=real \
  method:=easy_handeye \
  robot_ip:=192.168.1.100
```

Important:

- you still need a detector that publishes `camera_color_optical_frame -> handeye_target`
- the target must be rigidly attached to the robot end effector
- the camera must stay fixed in the workcell for the entire session

Publish the saved result later with:

```bash
ros2 launch jaka_eye_to_hand_calibration easy_handeye_publish.launch.py \
  calibration_name:=jaka_s12_eye_to_hand_d435
```

## How the eye-to-hand math works

At each sample, the solver uses:

- robot base to end-effector
- camera to target

Across multiple robot poses, it solves the fixed transform between the stationary camera and the robot base.

For eye-to-hand:

- the target moves with the robot
- the camera does not move

The final result is therefore a workcell camera pose, not a wrist camera pose.

## How to increase calibration efficiency and accuracy

- Run intrinsic calibration before hand-eye calibration.
- Make the target mount rigid. Flex in the bracket or gripper body directly corrupts the result.
- Favor rotation diversity over large translations.
- Use `12` to `20` diverse poses and avoid clusters of nearly identical samples.
- Keep the target large in the image. More pixels usually means better pose estimation.
- Avoid glare and motion blur.
- Keep the camera fixed once sampling starts. Even a small bump invalidates the dataset.
- Make sure the tool marker or board is centered and fully visible at each sample.
- Verify that `base_link`, `Link_06`, `camera_link`, and `camera_color_optical_frame` all exist and are spelled exactly.
- Validate the solution by moving the robot in RViz after calibration. The reconstructed target or camera relationship should remain visually consistent.

## Recommended order

1. Calibrate the camera intrinsics.
2. Choose one extrinsic pipeline:

- MoveIt plugin if you want RViz integration and built-in target detection
- `easy_handeye2` if you already have a target tracker publishing TF

3. Save the resulting `base_link -> camera_link` static transform.
4. Add that transform to your runtime launch for the fixed cell camera.

## Simulation note

For eye-to-hand, simulation now covers the fixed-camera intrinsic flow and the MoveIt hand-eye flow.

What works:

- simulated camera intrinsic calibration through the fixed simulated world camera on `/rgbd_camera/...`
- automatic Gazebo-board defaults for the stock `checkerboard_plane` target:

  - checkerboard size: `7x7` inner corners
  - square size: `0.25` m
  - pattern: `chessboard`
- simulated MoveIt eye-to-hand calibration through the fixed simulated world camera on `/rgbd_camera/...`
- a built-in simulation ChArUco target rigidly attached to `Link_06`

What is missing:

- a full simulated eye-to-hand `easy_handeye2` workflow

Important distinction:

- your real board values such as `checkerboard_size:=8x6` and `square_size_m:=0.024` belong to your physical calibration target
- the stock Gazebo world does not currently contain that same board geometry
- if you add an ArUco or ChArUco model to Gazebo, it is useful for pose-estimation-based hand-eye work, but not for `camera_calibration`

If you want to extend simulation further, the next engineering step is to add a fixed-camera `easy_handeye2` target-detection path that publishes `camera_color_optical_frame -> handeye_target` from the simulated image stream.
