# 6d_pose_estimation_using_AI_ROS2
 6D vision-to-control pipeline using YOLOv12-seg and Augmented Autoencoders (AAE) to estimate hex-nut poses and command a Jaka Zu5 cobot via ROS2.


# ROS2 pakages must be install 


```bash
sudo apt install ros-humble-pcl-ros
```
```bash
sudo apt install ros-humble-moveit-ros-perception
```
 ## How to run simualtion

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

# Media

## Demo Video

- [Screencast (WEBM)](media_and_files/Screencast%20from%202026-03-12%2000-43-34.webm)

<video src="media%20and%20files/Screencast%20from%202026-03-12%2000-43-34.webm" controls width="900"></video>

## Screenshots

![Simulation screenshot 1](media_and_files/Screenshot%20from%202026-03-12%2000-17-16.png)
![Simulation screenshot 2](media_and_files/Screenshot%20from%202026-03-12%2000-42-18.png)
![Simulation screenshot 3](media_and_files/Screenshot%20from%202026-03-12%2000-43-04.png)


### How to run in real world

*A) Test movement using safe methods
(All details found https://docs.mech-mind.net/1.6/en-GB/SoftwareSuite/RobotIntegrations/FullControlProgram/Jaka/Jaka.html)



**1) Connection Setup:
Network: Ensure your device (Tablet/PC) is on the same network as the JAKA controller.

**2) Login: 
Open the App and click the Connect icon in the top right.  
Default Administrator Password: jakazuadmin

**3) Powering & Enabling the Robot:
Once connected, the status will show "Connected," but the robot is still "Off."

3-1 ) Power On: Click the Power button in the App. You will hear the internal fans and electronics start.

3-2 ) Enable: Once powered, click Enable. This engages the motors and brakes. You will hear a distinct "click" from the joints. The status should now say "Enabled" or "Idle."

**4) Manual Control (Jogging & Free-Drive)
Before running a program, test the movement manually:

4-1) Jogging: Use the on-screen directional keys to move the robot in Joint Space (J1-J6) or Cartesian Space (X, Y, Z).

4-2) Safety Tip: Keep the Global Speed slider at 10% or lower for these tests.
Free-Drive (Drag Teaching): Press the physical blue button on the robot's end-effector (or the Free-Drive button in the App). This allows you to physically push and pull the arm to the hex-nut location to verify reachability.

*B) Moveing Robot using ROS2 official package  (Direct service calls)

**1) ros2 launch jaka_driver robot_start.launch.py ip:=192.168.100.100


*C) Moveig Robot to detected pose using ROS2 official package and vision pipeline

**1) Launch the Planner Server (higher-level MoveIt 2 integration)

```bash
ros2 launch jaka_planner moveit_server.launch.py ip:=192.168.100.100 model:=zu5
```
ip: The IP address of your JAKA controller.
model: Set this to zu5 to load the correct kinematic limits and URDF.

**1-1)(Optional) Verify with RViz (Visual Test)
Before moving the real arm, it is best practice to visualize the planning scene. Launch the MoveIt 2 configuration:
```bash
ros2 launch jaka_zu5_moveit_config moveit_rviz.launch.py
```

In RViz, you should see the JAKA Zu5 model. You can drag the "Goal State" interactive marker (the ball at the end-effector) to a position above where your hex-nut would be, then click Plan and Execute.


**2) Controlling via Python (MoveGroupInterface)
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
