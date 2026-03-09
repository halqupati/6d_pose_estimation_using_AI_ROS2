import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
import threading
import queue
import cv2
import numpy as np
import requests
import time
from cv_bridge import CvBridge
from ultralytics import YOLO
from scipy.spatial.transform import Rotation as R_tool

# Globals
bridge = CvBridge()
rgb_image = None
depth_image = None
trigger_signal = False
data_lock = threading.Lock()
display_queue = queue.Queue(maxsize=1)

class CameraSystem(Node):
    def __init__(self):
        super().__init__('camera_system_sub')
        self.create_subscription(Image, '/rgbd_camera/image', self.rgb_cb, 10)
        self.create_subscription(Image, '/rgbd_camera/depth_image', self.depth_cb, 10)
        self.create_subscription(Bool, '/trigger', self.trigger_cb, 10)

    def rgb_cb(self, msg):
        global rgb_image
        with data_lock:
            rgb_image = bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_cb(self, msg):
        global depth_image
        with data_lock:
            depth_image = bridge.imgmsg_to_cv2(msg, 'passthrough')

    def trigger_cb(self, msg):
        global trigger_signal
        trigger_signal = msg.data
        if msg.data:
            self.get_logger().info("\033[94mTrigger Received: Starting stable detection...\033[0m")

class NutEstimatorAAE(Node):
    def __init__(self):
        super().__init__('aae_estimator_node')
        self.model = YOLO("/home/ubuntu/6d_pose_estimation/nut_pose_pipeline/yolov12_seg_infer/model/best.pt")
        self.model.to('cpu')
        self.pos_angle_pub = self.create_publisher(Float32MultiArray, '/pos_angle', 10)

        # Stability logic
        self.pose_buffer = []
        self.buffer_size = 5
        self.stability_threshold = 0.005 # 5mm

        # Calibrated Parameters
        self.K = np.array([
            [686.795514, 0.0, 639.460651],
            [0.0, 686.779086, 359.347469],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.is_tf_ready = False
        self.T_wc = np.eye(4)

        self.create_timer(0.1, self.process_frame)

    def process_frame(self):
        global rgb_image, depth_image, trigger_signal
        
        with data_lock:
            if rgb_image is None or depth_image is None:
                return
            local_rgb = rgb_image.copy()
            local_depth = depth_image.copy()
            active = trigger_signal

        # Update TF once
        if not self.is_tf_ready:
            try:
                t = self.tf_buffer.lookup_transform('Link_0', 'camera_depth_optical_frame', rclpy.time.Time())
                self.T_wc = self.transform_to_matrix(t)
                self.is_tf_ready = True
            except: return

        # Only process inference if trigger is TRUE
        if not active:
            self.pose_buffer = [] # Reset buffer when idle
            try: display_queue.put_nowait(local_rgb)
            except: pass
            return

        results = self.model(local_rgb, conf=0.55, verbose=False, retina_masks=True)
        viz_img = local_rgb.copy()

        for r in results:
            if r.masks is None: continue
            
            # 1. Get Projection
            det_x, det_y = r.boxes.xywh[0][0].item(), r.boxes.xywh[0][1].item()
            raw_mask = r.masks.data[0].cpu().numpy()
            mask = cv2.resize(raw_mask, (local_depth.shape[1], local_depth.shape[0]), interpolation=cv2.INTER_NEAREST) > 0
            
            depths = local_depth[mask]
            if len(depths) == 0: continue
            z_m = np.median(depths[~np.isnan(depths)])
            if z_m <= 0.05: continue

            tx = (det_x - self.K[0, 2]) * z_m / self.K[0, 0]
            ty = (det_y - self.K[1, 2]) * z_m / self.K[1, 1]
            T_vec = np.array([tx, ty, z_m])

            # 2. Get Rotation
            box = r.boxes.xyxy[0].cpu().numpy().astype(int)
            crop = local_rgb[box[1]:box[3], box[0]:box[2]]
            try:
                _, enc = cv2.imencode('.jpg', crop)
                resp = requests.post('http://localhost:5001/get_rotation', files={"image": enc.tobytes()}, timeout=1.0)
                R_mat = np.array(resp.json()['R'])
            except: continue

            # 3. Transform to World
            T_camera_nut = np.eye(4)
            T_camera_nut[:3, :3] = R_mat
            T_camera_nut[:3, 3] = T_vec
            T_world_nut = self.T_wc @ T_camera_nut
            
            # 4. Stability Check
            current_pos = T_world_nut[:3, 3]
            self.pose_buffer.append(current_pos)
            if len(self.pose_buffer) > self.buffer_size: self.pose_buffer.pop(0)

            if len(self.pose_buffer) == self.buffer_size:
                std_dev = np.std(self.pose_buffer, axis=0)
                if np.all(std_dev < self.stability_threshold):
                    # Pose is stable, broadcast and publish
                    self.broadcast_and_publish(T_world_nut)
                    cv2.putText(viz_img, "STABLE", (int(det_x), int(det_y)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                else:
                    self.get_logger().warn("Waiting for Nut to stop moving...")

            # Draw for visualization
            cv2.rectangle(viz_img, (box[0], box[1]), (box[2], box[3]), (255, 0, 0), 2)

        try: display_queue.put_nowait(viz_img)
        except queue.Full: pass

    def transform_to_matrix(self, t):
        m = np.eye(4)
        m[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        q = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
        m[:3, :3] = R_tool.from_quat(q).as_matrix()
        return m

    def broadcast_and_publish(self, mat):
        # Broadcast TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'Link_0'
        t.child_frame_id = 'detected_nut'
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = mat[:3, 3]
        q = R_tool.from_matrix(mat[:3, :3]).as_quat()
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        self.tf_broadcaster.sendTransform(t)
    
        # Publish Topic
        msg = Float32MultiArray()
        # [x, y, z, qx, qy, qz, qw]
        msg.data = [mat[0,3], mat[1,3], mat[2,3], q[0], q[1], q[2], q[3]]
        self.pos_angle_pub.publish(msg)
        self.get_logger().info(f"\033[92mStable Pose Published: {msg.data[:3]}\033[0m")

if __name__ == '__main__':
    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor()
    cam_node = CameraSystem()
    est_node = NutEstimatorAAE()
    executor.add_node(cam_node)
    executor.add_node(est_node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    cv2.namedWindow("Stable Detection", cv2.WINDOW_NORMAL)
    try:
        while rclpy.ok():
            try:
                frame = display_queue.get(timeout=0.1)
                cv2.imshow("Stable Detection", frame)
            except queue.Empty: pass
            
            if cv2.waitKey(1) & 0xFF == 27: break
    except KeyboardInterrupt: pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()
