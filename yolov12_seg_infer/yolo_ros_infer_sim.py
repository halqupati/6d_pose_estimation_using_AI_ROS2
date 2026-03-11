import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import threading
import cv2
import numpy as np
import requests
import os
from cv_bridge import CvBridge
from ultralytics import YOLO
from scipy.spatial.transform import Rotation as R_tool

# --- Global Variables for Data Synchronization ---
bridge = CvBridge()
rgb_image = np.zeros((480, 640, 3), np.uint8)
depth_image = np.zeros((480, 640), np.float32)
trigger_signal = False

# Test topics of simulation jaka env

class Depth_CameraSubscriber(Node):
    def __init__(self):
        super().__init__('depth_subscriber')
        self.subscription = self.create_subscription(Image, '/rgbd_camera/depth_image', self.callback, 10)
    def callback(self, data):
        global depth_image
        # Notes Realsense in GZ Sim usually sends 32FC1 (float depth in mm or meters)
        depth_image = bridge.imgmsg_to_cv2(data, 'passthrough')

class RGB_CameraSubscriber(Node):
    def __init__(self):
        super().__init__('rgb_subscriber')
        self.subscription = self.create_subscription(Image, '/rgbd_camera/image', self.callback, 10)
    def callback(self, data):
        global rgb_image
        rgb_image = bridge.imgmsg_to_cv2(data, "bgr8")

class TriggerSubscriber(Node):
    def __init__(self):
        super().__init__('trigger_subscriber')
        self.subscription = self.create_subscription(Bool, '/trigger', self.callback, 10)
    def callback(self, data):
        global trigger_signal
        trigger_signal = data.data

# --- Main Estimator Node (The AAE Bridge) --- very improtant to improve

class NutEstimatorAAE(Node):
    def __init__(self):
        super().__init__('aae_estimator_node')
        
        # 1. Load YOLOv12-seg (Ensure the path to your best.pt is correct)
        self.model = YOLO("/home/hussain/6d_pose_estimation_using_AI_ROS2/nut_pose_pipeline/yolov12_seg_infer/model/best.pt")
        
        # 2. Camera Intrinsics (Values from your Gazebo cam_K.txt)
        self.K = np.array([[615.0, 0, 320.0], [0, 615.0, 240.0], [0, 0, 1]])
        
        # 3. ROS Publishers & TF
        self.pub_target_pose = self.create_publisher(PoseStamped, '/target_nut_pose', 10)
        self.pub_vis_img = self.create_publisher(Image, '/aae_result_vis', 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.T_wc = np.eye(4) # World -> Camera transform
        self.get_camera_tf = False
        
        # Timer for the inference loop (10Hz)
        self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("AAE-YOLO Node Initialized")

    def timer_callback(self):
        global rgb_image, depth_image, trigger_signal
        
        if not trigger_signal or np.all(rgb_image == 0):
            return

        # Step 1: Get Camera Pose in World
        if not self.get_camera_tf:
            try:
                # Replace 'Camera' and 'World' with your actual frame names in GZ
                t = self.tf_buffer.lookup_transform('world', 'camera_link_optical', rclpy.time.Time())
                self.T_wc = self.transform_to_matrix(t)
                self.get_camera_tf = True
                self.get_logger().info("Successfully check Tf tree")
            except Exception as e:
                self.get_logger().warn(f"Waiting for TF: {e}")
                return

        # Step 2: YOLOv12-seg Detection
        results = self.model(rgb_image, conf=0.8, verbose=False)
        self.get_logger().info("Successfully start detection model")
        
        for r in results:
            if r.masks is not None:
                # We take the first detection for the Nut
                mask_data = r.masks.data[0].cpu().numpy() # 2D Binary Mask
                x1, y1, x2, y2 = map(int, r.boxes.xyxy[0])
                
                # Step 3: Call AAE Server for Rotation (R)
                # We send the crop to the Py3.7 environment via Flask
                nut_crop = rgb_image[y1:y2, x1:x2]
                R_matrix = self.request_aae_rotation(nut_crop)
                
                if R_matrix is None: continue

                # Step 4: Calculate Translation (T) using Depth
                # Find median depth of the pixels inside the YOLO mask
                z_val = np.median(depth_image[mask_data > 0]) 
                if np.isnan(z_val) or z_val <= 0: continue
                
                z_meters = z_val / 1000.0 if z_val > 10 else z_val 
                cx, cy = r.boxes.xywh[0][0], r.boxes.xywh[0][1]
                tx = (cx - self.K[0, 2]) * z_meters / self.K[0, 0]
                ty = (cy - self.K[1, 2]) * z_meters / self.K[1, 1]
                T_vector = np.array([tx, ty, z_meters])
                T_cam_obj = np.eye(4)
                T_cam_obj[:3, :3] = R_matrix
                T_cam_obj[:3, 3] = T_vector           
                T_world_obj = self.T_wc @ T_cam_obj
                self.publish_pose(T_world_obj)

    def request_aae_rotation(self, crop):
        try:
            _, img_encoded = cv2.imencode('.jpg', crop)
            response = requests.post('http://localhost:5001/get_rotation', 
                                     files={"image": img_encoded.tobytes()}, timeout=0.5)
            return np.array(response.json()['R'])
        except Exception as e:
            self.get_logger().error(f"AAE Server Error: {e}")
            return None

    def transform_to_matrix(self, t):
        m = np.eye(4)
        m[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        q = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
        m[:3, :3] = R_tool.from_quat(q).as_matrix()
        return m

    def publish_pose(self, mat):
        p = PoseStamped()
        p.header.frame_id = "World"
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y, p.pose.position.z = mat[:3, 3]
        q = R_tool.from_matrix(mat[:3, :3]).as_quat()
        p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w = q
        self.pub_target_pose.publish(p)

# --- Execution Logic --- not implement yet

if __name__ == '__main__':
    rclpy.init()
    
    # Create all nodes
    estimator = NutEstimatorAAE()
    rgb_sub = RGB_CameraSubscriber()
    depth_sub = Depth_CameraSubscriber()
    trigger = TriggerSubscriber()

    # Use MultiThreadedExecutor to handle all subscribers simultaneously
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(estimator)
    executor.add_node(rgb_sub)
    executor.add_node(depth_sub)
    executor.add_node(trigger)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        while rclpy.ok():
            pass # Keep alive
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        executor_thread.join()
