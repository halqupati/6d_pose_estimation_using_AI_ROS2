# jaka_eye_in_hand_calibration

JAKA S12 eye-in-hand calibration wrapper package.

This package gives you three separate pieces of the workflow:

- `camera_calibration` for camera intrinsics
- `moveit_calibration` for RViz-based hand-eye calibration
- `easy_handeye2` for TF-pair based hand-eye calibration

## Best single launch file

If you want one entry point for both real hardware and simulation, use:

```bash
ros2 launch jaka_eye_in_hand_calibration calibration.launch.py runtime:=real method:=moveit
```

Supported combinations:

- `runtime:=real method:=intrinsics`
- `runtime:=real method:=moveit`
- `runtime:=real method:=easy_handeye`
- `runtime:=sim method:=intrinsics`
- `runtime:=sim method:=moveit`
- `runtime:=sim method:=easy_handeye`

Examples:

```bash
ros2 launch jaka_eye_in_hand_calibration calibration.launch.py runtime:=real method:=moveit robot_ip:=192.168.1.100
ros2 launch jaka_eye_in_hand_calibration calibration.launch.py runtime:=real method:=intrinsics
ros2 launch jaka_eye_in_hand_calibration calibration.launch.py runtime:=sim method:=moveit
ros2 launch jaka_eye_in_hand_calibration calibration.launch.py runtime:=sim method:=intrinsics
```

For simulated intrinsic calibration, the wrapper now defaults to the stock Gazebo checkerboard target:

- `sim_checkerboard_size:=7x7`
- `sim_square_size_m:=0.25`
- `use_gazebo_checkerboard_defaults:=true`

That matches the built-in `checkerboard_plane` model in the simulation world. If you replace the simulated board with one that matches your real target, disable those defaults and pass your own values.

Why this is the best launch in this repo:

- it uses the real JAKA + RealSense stack when `runtime:=real`
- it uses the simulated wrist camera when `runtime:=sim`
- it keeps calibration mode separate from bin-picking and planner nodes
- it disables guessed mount TF during real calibration
- it brings up MoveIt only when the chosen method needs it

Simulation naming:

- wrist eye-in-hand camera base frame: `mounted_camera_link`
- wrist eye-in-hand optical frame: `mounted_camera_color_optical_frame`
- wrist eye-in-hand ROS topics: `/rgbd_mounted_camera/...`
- fixed world camera base frame: `camera_link`
- fixed world camera optical frame: `camera_color_optical_frame`
- fixed world camera ROS topics: `/rgbd_camera/...`

Simulation target note:

- the stock Gazebo world uses a checkerboard plane for intrinsic calibration
- `camera_calibration` should stay on checkerboard or circle-grid targets
- if you add an ArUco or ChArUco target model to Gazebo, use it for pose-estimation-based hand-eye work, not for `method:=intrinsics`
- you can pick a different simulation world with `sim_world:=your_world.sdf`

If you instead need a fixed external camera calibration, use the eye-to-hand package:

- [jaka_eye_to_hand_calibration/README.md](/home/ubuntu/6d_pose_estimation/6d_pose_estimation_using_AI_ROS2/jaka_ros_ws/src/jaka_eye_to_hand_calibration/README.md)

The intended hardware layout is:

- camera rigidly mounted to `Link_06`
- calibration target fixed in the robot base frame, usually on the table
- robot base frame: `base_link`
- camera optical frame: usually `camera_color_optical_frame`

## What each tool does

`camera_calibration` only computes camera intrinsics:

- focal length
- principal point
- distortion coefficients

It does not solve the robot-to-camera transform.

`moveit_calibration` solves the eye-in-hand extrinsic transform by combining:

- robot kinematics from TF and joint states
- target pose estimated from the camera image

`easy_handeye2` also solves the hand-eye transform, but it needs a tracking pipeline that already publishes the target pose into TF. In practice that means you must provide a detector that publishes:

- `camera_color_optical_frame -> handeye_target`

The repository you requested, [`halqupati/easy_handeye2_camera_calibration_with_ros2`](https://github.com/halqupati/easy_handeye2_camera_calibration_with_ros2), is a fork of `easy_handeye2`, so the wrapper launch files in this package target that interface.

## External dependencies

`camera_calibration` is available from ROS 2 apt packages. In this environment, `moveit_calibration` is not available as a Humble binary package, so build it from source.

Install what can come from apt:

```bash
sudo apt install python3-vcstool ros-humble-camera-calibration
```

Import the source dependencies for this package:

```bash
cd /home/ubuntu/6d_pose_estimation/6d_pose_estimation_using_AI_ROS2/jaka_ros_ws
vcs import src < src/jaka_eye_in_hand_calibration/config/third_party_calibration.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Notes:

- the included `.repos` file pulls `moveit_calibration` from the upstream `ros2` branch
- it pulls `easy_handeye2_camera_calibration_with_ros2` from the fork you referenced
- if you already cloned either repository elsewhere in `src`, do not import it twice

## 1. Intrinsic calibration with `camera_calibration`

Start the camera and the calibration GUI:

```bash
ros2 launch jaka_eye_in_hand_calibration camera_intrinsics.launch.py \
  checkerboard_size:=8x6 \
  square_size_m:=0.024
```

For the unified simulation path, use:

```bash
ros2 launch jaka_eye_in_hand_calibration calibration.launch.py \
  runtime:=sim \
  method:=intrinsics
```

That simulation command uses the Gazebo checkerboard defaults automatically:

- `7x7` inner corners
- `0.25` m square size
- `chessboard` pattern

If you replace the simulated board with one that matches your real calibration target, run:

```bash
ros2 launch jaka_eye_in_hand_calibration calibration.launch.py \
  runtime:=sim \
  method:=intrinsics \
  use_gazebo_checkerboard_defaults:=false \
  checkerboard_size:=8x6 \
  square_size_m:=0.024
```

Defaults:

- image topic: `/camera/camera/color/image_raw`
- camera namespace for `set_camera_info`: `/camera/camera/color`
- mount TF publication: disabled during calibration

What to do in the GUI:

- move the checkerboard left and right until the X bar fills
- move it up and down until the Y bar fills
- move it closer and farther until the Size bar fills
- tilt it strongly in multiple directions
- click `CALIBRATE`, then `SAVE`

For rolling-shutter or blurry captures, keep `max_chessboard_speed` small. The launch defaults to `0.5` px/frame to reject fast board motion.

## 2. Eye-in-hand calibration with MoveIt RViz plugin

Start the robot, the camera, MoveIt, and RViz:

```bash
ros2 launch jaka_eye_in_hand_calibration moveit_handeye.launch.py \
  robot_ip:=192.168.1.100
```

Then in RViz:

1. Open `Panels -> Add New Panel`.
2. Add `HandEyeCalibration`.
3. In `Target`, create or load your printed target and enter the real metric dimensions.
4. In `Context`, use:

- Sensor configuration: `Eye-in-hand`
- Sensor frame: `camera_color_optical_frame`
- Object frame: `handeye_target`
- End-effector frame: `Link_06`
- Robot base frame: `base_link`

5. In `Calibrate`, collect samples and save the camera pose.

MoveIt calibration internally estimates the target pose from the image, so this path is self-contained once the plugin is installed.

## 3. Eye-in-hand calibration with `easy_handeye2`

Start the wrapper launch:

```bash
ros2 launch jaka_eye_in_hand_calibration easy_handeye_calibrate.launch.py \
  robot_ip:=192.168.1.100 \
  tracking_base_frame:=camera_color_optical_frame \
  tracking_marker_frame:=handeye_target
```

Important:

- this launch does not create the target detector for you
- you must run a tracker that publishes `camera_color_optical_frame -> handeye_target`
- the calibration target must stay fixed relative to `base_link`
- the camera stays fixed relative to `Link_06`

Publish the saved result later with:

```bash
ros2 launch jaka_eye_in_hand_calibration easy_handeye_publish.launch.py \
  calibration_name:=jaka_s12_eye_in_hand_d435
```

## How the eye-in-hand math works

At each sample, the solver needs two poses:

- robot base to end-effector
- camera to calibration target

Across several robot poses, the fixed unknown transform is:

- end-effector to camera

For eye-in-hand:

- the camera is rigidly attached to the wrist
- the target stays fixed in the workcell

The solver eliminates the unknown world placement of the target and keeps the consistent wrist-to-camera transform.

## How to increase calibration efficiency and accuracy

- Calibrate intrinsics first. Bad intrinsics contaminate the extrinsic result.
- Disable any guessed `Link_06 -> camera_link` static TF during calibration. This package’s launches already do that.
- Use strong orientation changes. Rotation diversity matters more than large translations.
- Capture about `12` to `20` samples. Accuracy usually improves a lot after `5`, then plateaus around `12` to `15`.
- Rotate around at least two axes. One-axis motion is poorly conditioned.
- Keep the board close enough to use many pixels, but not so close that corners leave the image.
- Reduce glare on the printed board. Reflections are a common source of bad target fits.
- Lock auto-exposure if the image brightness keeps changing.
- Reject nearly identical poses. Redundant but diverse poses help; repeated poses do not.
- Verify frame names before sampling. For S12 use `Link_06`, not `Link_6`.
- After solving, validate by moving the robot while visualizing the calibrated target in RViz. It should stay stable instead of drifting.

## Recommended order

1. Run intrinsic calibration once for the real camera.
2. Use the MoveIt plugin if you want the most integrated RViz workflow.
3. Use `easy_handeye2` if you already have a reliable tracker publishing target TF.
4. Save the final static transform and place it in your runtime camera launch.
