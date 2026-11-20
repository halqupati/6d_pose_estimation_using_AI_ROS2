# YouTube Publish Templates

Use these templates as copy-and-edit starting points for each episode.

## Video description template

```text
In this video, I show how to [main result of the episode] using this ROS2 robotics project:
6D pose estimation with YOLO segmentation, AAE-based rotation inference, MoveIt planning, and JAKA robot integration.

Repo:
https://github.com/<your-account>/<your-repo>

Episode focus:
- [point 1]
- [point 2]
- [point 3]

Commands used:
[paste only the commands shown in the video]

Relevant files:
- README.md
- tools/project_launcher.py
- yolov12_seg_infer/yolo_ros_infer_sim_v11_to_control_jaka_s12.py
- aae_infer/autoencoder_ws/aae_ros_infer_sim_v4.py
- jaka_ros_ws/src/bin_packing_pkg/launch/run_bin_picking_nodes_to_control_jaka_s12.launch.py

Chapters:
00:00 Intro
00:20 Demo
01:10 Project overview
03:00 Implementation
08:30 Test run
10:00 Next episode

If this series is useful, subscribe and follow the playlist for the next step in the pipeline.
```

## Playlist description template

```text
This playlist walks through a complete ROS2 6D pose estimation pipeline for robotics.
We use YOLO segmentation, an AAE-based rotation estimator, MoveIt planning, simulation, calibration, and JAKA robot integration.

The playlist is organized to move from visible demo results into setup, perception, planning, and finally real hardware deployment.
```

## Pinned comment template

```text
Next episode: [title]

If you get stuck on one of the setup steps, comment with:
1. your ROS2 distro
2. whether you are using simulation or real hardware
3. the exact command that failed
```

## Thumbnail formula

Use only 2-4 words on the thumbnail.

Good options for this series:

- 6D Pose
- ROS2 Setup
- YOLO + ROS2
- Solve Rotation
- Vision to Motion
- Calibrate Camera
- Real Robot Test

Visual layout:

- left: robot or RViz result
- right: camera view, segmentation mask, or terminal
- one bold arrow from detection to robot action
- one bright accent color only

## Title formula

Keep the title specific and outcome-based.

Good structure:

`[Result] with [tool] in [context]`

Examples:

- Build a ROS2 6D Pose Estimation Pipeline
- YOLO Segmentation for Robot Picking in ROS2
- Hand-Eye Calibration for a JAKA Robot in ROS2
- From Camera Detection to Motion Planning in ROS2

## Recording checklist

Before recording:

- close unrelated windows
- increase terminal font size
- prepare the exact commands in a scratchpad
- make sure RViz opens to a clean view
- confirm sample videos and screenshots are easy to find
- silence desktop notifications

During recording:

- say what the viewer should expect before you type
- pause half a second before switching windows
- keep the mouse still when explaining
- repeat frame names and topic names clearly
- record one clean demo take before the teaching take

After recording:

- trim dead time
- add zoom on code and terminal sections
- add chapter timestamps
- add one end screen pointing to the next episode

## Script card template

Use this mini-outline for every episode:

```text
Hook:
[show final result in one sentence]

Problem:
[what is hard or confusing here]

Plan:
[what you will do in this episode]

Build:
[3-5 concrete steps]

Test:
[what proves it works]

Next:
[bridge to the next episode]
```

## Recommended first three uploads

If you want to start quickly, record these first:

1. project overview and final demo
2. simulation launch and RViz walkthrough
3. YOLO plus AAE perception flow

That gives you a strong opening playlist even before the deeper setup and calibration episodes are finished.

