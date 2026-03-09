# jaka_hardware_bringup

Bring-up package for running JAKA ZU5 + D435 on real hardware.

## Folder structure

- `launch/robot_hw.launch.py`: robot driver + MoveIt + optional RViz + optional bin picking node
- `launch/camera_d435_eye_in_hand.launch.py`: RealSense + eye-in-hand TF + optional point cloud filtering
- `launch/hardware_full.launch.py`: full-stack orchestrator for robot, camera, and planner
- `config/hardware_profile.yaml`: editable template for IP, camera extrinsics, and pipeline toggles

## Typical commands

Build and source:

```bash
cd /home/ubuntu/6d_pose_estimation/nut_pose_pipeline/jaka_ros_ws
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

- `driver_package` (default: `jaka_driver`)
- `driver_executable` (default: `jaka_driver`)
- `driver_namespace` (default: empty)
- `robot_ip`

Example:

```bash
ros2 launch jaka_hardware_bringup robot_hw.launch.py \
  driver_package:=<your_jaka_driver_pkg> \
  driver_executable:=<your_driver_node> \
  robot_ip:=192.168.1.100
```

## Notes

- If your driver already publishes TF and robot state, keep `rsp.launch.py` only if needed.
- Set `use_bin_picking:=true` only after planning and controller topics are verified.
- Update camera static TF (`camera_x/y/z/roll/pitch/yaw`) from your real calibration.
