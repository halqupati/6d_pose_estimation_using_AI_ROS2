# 6d_pose_estimation_using_AI_ROS2
 6D vision-to-control pipeline using YOLOv12-seg and Augmented Autoencoders (AAE) to estimate hex-nut poses and command a Jaka Zu5 cobot via ROS2.


# ROS2 pakages must be install 


```bash
sudo apt install ros-humble-pcl-ros
```
```bash
sudo apt install ros-humble-moveit-ros-perception
```
 # How run simualtion

**1) Run gazebo ign, rviz, moviet, run_move_group_node, ros2_control, ros_gz_bridge and depth_image_converter
```bash
ros2 launch bin_packing_pkg jaka_zu5_with_two_camera_simulation.launch.py
```
**2) Run moveit_config with bin_picking_node as move_group_node2
```bash
ros2 launch bin_packing_pkg run_bin_picking_nodes.launch.py 
```
**3) Activate AAE conda enveroment and run AAE_infer node
```bash
conda activate aae_py37_tf26
python3 aae_ros_infer_sim_v4.py
```
**4) Activate YOLOV12-seg conda enveroment and run Yolo_infer node
```bash
conda activate yolov12_py10
python3 yolo_ros_infer_sim_v11.py
```


```bash

```

# Media

## Demo Video

- [Screencast (WEBM)](media%20and%20files/Screencast%20from%202026-03-12%2000-43-34.webm)

<video src="media%20and%20files/Screencast%20from%202026-03-12%2000-43-34.webm" controls width="900"></video>

## Screenshots

![Simulation screenshot 1](media%20and%20files/Screenshot%20from%202026-03-12%2000-17-16.png)
![Simulation screenshot 2](media%20and%20files/Screenshot%20from%202026-03-12%2000-42-18.png)
![Simulation screenshot 3](media%20and%20files/Screenshot%20from%202026-03-12%2000-43-04.png)
