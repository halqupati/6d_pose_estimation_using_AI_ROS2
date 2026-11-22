# YouTube Tutorial Series Plan

This repo is strong enough for a full tutorial series, not just one video.
The best angle is:

`Build a 6D vision-to-robot pipeline with ROS2, YOLO segmentation, AAE pose estimation, and a JAKA cobot`

## Who the series is for

- ROS2 learners who want a real project instead of toy examples
- robotics students building perception-to-action demos
- engineers interested in simulation first, then real hardware

## Best content strategy

Lead with the result, then unpack the pipeline across short focused episodes.

Recommended structure:

1. show the final demo in the first 20-30 seconds
2. explain one main idea per video
3. keep each video in the 8-15 minute range
4. reuse the same project across the whole playlist
5. finish each episode with the next step in the pipeline

## Series title ideas

- ROS2 6D Pose Estimation to Robot Control
- Build an AI Bin Picking Pipeline with ROS2
- From Camera to Cobot: 6D Pose Estimation in ROS2
- YOLO + AAE + ROS2 for Vision Guided Robotics

## Recommended season outline

### Episode 1: Project overview and final demo

Goal:
Show the complete system first so viewers know what they are building.

Show on screen:

- existing videos in `media_and_files/VID_20260419_013406.mp4`
- existing videos in `media_and_files/VID_20260419_014005.mp4`
- screenshots from `media_and_files/`
- repo overview from `README.md`
- launcher UI from `tools/project_launcher.py`

Talking points:

- what 6D pose estimation means
- why segmentation plus pose estimation is useful for robotics
- difference between simulation and real hardware workflow
- high-level pipeline:
  world camera or wrist camera -> YOLO segmentation -> AAE rotation inference -> pose handoff -> planning -> gripper action

Suggested title:

- ROS2 6D Pose Estimation Pipeline Demo with JAKA Robot

Thumbnail text:

- 6D Pose to Robot

### Episode 2: Workspace setup and dependency install

Goal:
Help viewers reproduce your environment with the least confusion possible.

Show on screen:

- `README.md`
- `requirements.txt`
- `jaka_ros_ws/src/calibration_pkgs/jaka_eye_in_hand_calibration/README.md`
- `jaka_ros_ws/src/calibration_pkgs/jaka_eye_to_hand_calibration/README.md`

Commands to show:

```bash
sudo apt install ros-humble-pcl-ros
sudo apt install ros-humble-moveit-ros-perception
sudo apt install ros-humble-moveit
sudo apt install ros-humble-moveit-msgs
```

If you want to show a full workspace build, use:

```bash
cd /home/ubuntu/6d_pose_estimation/6d_pose_estimation_using_AI_ROS2/jaka_ros_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Important notes to say out loud:

- this project uses ROS2 Humble
- some calibration dependencies are source-based
- the AAE and YOLO parts use separate Python environments
- simulation assets need the Ignition resource path set correctly

Suggested title:

- How to Set Up This ROS2 6D Pose Estimation Project

Thumbnail text:

- Full Setup

### Episode 3: Run the simulation stack

Goal:
Get the viewer from zero to a running Gazebo + RViz + MoveIt demo.

Show on screen:

- `README.md`
- `jaka_ros_ws/src/bin_packing_pkg/launch/jaka_s12_with_two_camera_simulation_with_pcl_and_custom_gripper.launch.py`
- `tools/project_launcher.py`

Commands to show:

```bash
python3 tools/project_launcher.py
```

Manual path:

```bash
ros2 launch bin_packing_pkg jaka_s12_with_two_camera_simulation_with_pcl_and_custom_gripper.launch.py
```

What to explain:

- Gazebo simulation world
- RViz planning view
- MoveIt move group
- camera topics:
  `/rgbd_camera/...`
  `/rgbd_mounted_camera/...`
- why this launch is the best visual starting point for the series

Suggested title:

- Launching the JAKA ROS2 Simulation for 6D Vision

Thumbnail text:

- ROS2 Simulation

### Episode 4: YOLO segmentation and trigger logic

Goal:
Explain how detection enters the pipeline.

Show on screen:

- `yolov12_seg_infer/yolo_ros_infer_sim_v11_to_control_jaka_s12.py`
- the `/trigger`, `/pose_received`, and `/pos_angle` topic flow
- the stable detection window if you record it live

Important code ideas to highlight:

- RGB and depth subscription
- stable detection before publishing
- AAE service request from the YOLO node
- pose acknowledgement handshake with the bin-picking node

Good explanation line:

`YOLO in this repo does more than segmentation; it also orchestrates when the pose estimation cycle is allowed to complete.`

Suggested title:

- YOLO Segmentation in ROS2 for Robot Picking

Thumbnail text:

- YOLO + ROS2

### Episode 5: AAE rotation estimation and 6D pose handoff

Goal:
Show how orientation is solved after segmentation.

Show on screen:

- `aae_infer/autoencoder_ws/aae_ros_infer_sim_v4.py`
- `yolov12_seg_infer/yolo_ros_infer_sim_v11_to_control_jaka_s12.py`
- the `AAE_SERVER_URL` and `/get_rotation` request flow

Important point to explain clearly:

In the current repo, `aae_ros_infer_sim_v4.py` acts as a Flask inference service.
The YOLO-side ROS node crops the object image, sends it to the AAE endpoint, and combines the result into the pose data it publishes onward.

Suggested title:

- How AAE Adds Rotation to 6D Pose Estimation

Thumbnail text:

- Solve Rotation

### Episode 6: Motion planning and pick execution

Goal:
Connect perception outputs to robot action.

Show on screen:

- `jaka_ros_ws/src/bin_packing_pkg/launch/run_bin_picking_nodes_to_control_jaka_s12.launch.py`
- `jaka_ros_ws/src/bin_packing_pkg/src/bin_picking_node_v5_to_control_jaka_s12.cpp`
- custom gripper package files under `jaka_ros_ws/src/custom_gripper_pkg/`

Commands to show:

```bash
ros2 launch bin_packing_pkg run_bin_picking_nodes_to_control_jaka_s12.launch.py
```

Explain:

- MoveIt configuration for the S12
- grasp offsets and safety margins
- collision monitor role
- gripper control logic
- why the robot waits for a valid pose before advancing

Suggested title:

- From 6D Pose to Motion Planning in ROS2

Thumbnail text:

- Vision to Motion

### Episode 7: Optional planner architecture deep dive

Goal:
Show the cleaner modular planner package that documents the intended architecture.

Show on screen:

- `jaka_ros_ws/src/hexnut_6d_planner/README.md`
- `jaka_ros_ws/src/hexnut_6d_planner/launch/planner_pipeline.launch.py`
- `pose_fusion_node.py`
- `grasp_planner_node.py`
- `adaptive_compliance_node.py`

Why this episode is useful:

- it helps viewers understand the system as publish/subscribe blocks
- it is easier to teach from than the larger production launch files
- it gives you a clean architecture episode without editing the main pipeline

Suggested title:

- Clean ROS2 Architecture for 6D Pose and Grasp Planning

Thumbnail text:

- ROS2 Node Graph

### Episode 8: Camera calibration for real deployment

Goal:
Prepare viewers to move from simulation to hardware.

Show on screen:

- `jaka_ros_ws/src/jaka_hardware_bringup/README.md`
- `jaka_ros_ws/src/calibration_pkgs/jaka_eye_in_hand_calibration/README.md`
- `jaka_ros_ws/src/calibration_pkgs/jaka_eye_to_hand_calibration/README.md`

Pick one calibration path per video section:

- eye-in-hand for wrist-mounted camera
- eye-to-hand for fixed workcell camera

Strong teaching point:

Do not try to explain every calibration method in one pass. Pick the method that matches your hardware and keep the video practical.

Suggested title:

- ROS2 Hand-Eye Calibration for a JAKA Robot

Thumbnail text:

- Calibrate Camera

### Episode 9: Real hardware bringup and safety notes

Goal:
Show the transition from simulation to the real cobot with careful framing.

Show on screen:

- `README.md`
- `jaka_ros_ws/src/jaka_hardware_bringup/README.md`
- `jaka_ros_ws/src/jaka_driver/README.md`

Commands to show:

```bash
ros2 launch jaka_driver driver_bringup.launch.py
ros2 launch jaka_hardware_bringup hardware_full.launch.py robot_ip:=192.168.1.100
```

Safety points to say clearly:

- start slow
- verify frames before motion
- test planning in RViz before executing on hardware
- keep emergency stop access clear
- never copy simulation speed assumptions directly to the real robot

Suggested title:

- Running the 6D Pose Pipeline on a Real JAKA Robot

Thumbnail text:

- Real Robot Test

## Recommended publish order

If you want the fastest playlist growth, publish in this order:

1. Episode 1
2. Episode 3
3. Episode 4
4. Episode 5
5. Episode 6
6. Episode 2
7. Episode 8
8. Episode 9
9. Episode 7

Reason:
people usually click on demos and visible results first, then come back for setup and deeper architecture.

## Suggested recording style

- start every video with a 10-20 second preview of the result
- keep your spoken intro under 30 seconds
- show terminal commands slowly and zoom in
- use callout boxes for topic names and node names
- keep one terminal, one RViz view, and one code editor theme throughout the series
- avoid long unedited package install waits; cut those into short time jumps

## Reusable story formula for every episode

Use this structure every time:

1. what the viewer will build in this episode
2. quick demo of the result
3. architecture or concept
4. step-by-step implementation
5. test run
6. one clear next step for the next episode

## Best repo assets to reuse as B-roll

- `media_and_files/VID_20260419_013406.mp4`
- `media_and_files/VID_20260419_014005.mp4`
- `media_and_files/Screencast from 2026-03-12 00-43-34.webm`
- `media_and_files/Screenshot from 2026-03-12 00-17-16.png`
- `media_and_files/Screenshot from 2026-03-12 00-42-18.png`
- `media_and_files/Screenshot from 2026-03-12 00-43-04.png`

## Short trailer script

`In this series, we build a full ROS2 vision-to-action pipeline that detects a hex nut, estimates its 6D pose with AI, and drives a JAKA robot in simulation and on real hardware. We will go from setup, to segmentation, to pose estimation, to planning, to calibration, and finally to robot execution.`

## What to avoid

- do not mix setup, detection, calibration, and hardware in one long first video
- do not hide the final result until the end
- do not spend several minutes reading code without first showing why it matters
- do not promise perfect real-time performance if your pipeline still needs tuning

