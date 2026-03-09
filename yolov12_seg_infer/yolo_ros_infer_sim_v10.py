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

class NutEstimatorAAE(Node):
    def __init__(self):
        super().__init__('aae_estimator_node')
        self.model = YOLO("/home/ubuntu/6d_pose_estimation/nut_pose_pipeline/yolov12_seg_infer/model/best.pt")
        self.model.to('cpu')
        self.pos_angle_pub = self.create_publisher(Float32MultiArray, '/pos_angle', 10)

        # === CALIBRATED PARAMETERS (from your terminal output) ===
        self.K = np.array([
            [686.795514,   0.000000, 639.460651],  # fx, 0, cx
            [  0.000000, 686.779086, 359.347469],  # 0, fy, cy
            [  0.000000,   0.000000,   1.000000]
        ], dtype=np.float32)

        self.D = np.array([0.000444, -0.000358, -0.000029, -0.000008, 0.0], dtype=np.float32)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.is_tf_ready = False
        self.T_wc = np.eye(4)

        self.create_timer(0.3, self.process_frame)

    def process_frame(self):
        global rgb_image, depth_image, trigger_signal
        
        with data_lock:
            if rgb_image is None or depth_image is None or not trigger_signal:
                return
            local_rgb = rgb_image.copy()
            local_depth = depth_image.copy()

        # Update Transform from Link_0 to Camera Optical Frame
        if not self.is_tf_ready:
            try:
                # Use camera_color_optical_frame as the reference for K-matrix projection
                t = self.tf_buffer.lookup_transform('Link_0', 'camera_depth_optical_frame', rclpy.time.Time())
                self.T_wc = self.transform_to_matrix(t)
                self.is_tf_ready = True
            except: return

        results = self.model(local_rgb, conf=0.55, verbose=False, retina_masks=True)

        for r in results:
            if r.boxes is None or len(r.boxes) == 0 or r.masks is None: continue

            # Get Detection Center (cx, cy in pixels)
            det_x, det_y = r.boxes.xywh[0][0].item(), r.boxes.xywh[0][1].item()

            # Align Mask to Depth
            raw_mask = r.masks.data[0].cpu().numpy()
            h, w = local_depth.shape
            mask = cv2.resize(raw_mask, (w, h), interpolation=cv2.INTER_NEAREST) > 0
            
            depths = local_depth[mask]
            if len(depths) == 0: continue
            z_m = np.median(depths[~np.isnan(depths)])
            if np.isnan(z_m) or z_m <= 0.05: continue

            # === FIXED PROJECTION LOGIC ===
            # Subtract Principal Point (cx, cy) from Calibration Matrix K
            tx = (det_x - self.K[0, 2]) * z_m / self.K[0, 0]
            ty = (det_y - self.K[1, 2]) * z_m / self.K[1, 1]
            T_vec = np.array([tx, ty, z_m])

            # Get Rotation from AAE
            box = r.boxes.xyxy[0].cpu().numpy().astype(int)
            crop = local_rgb[box[1]:box[3], box[0]:box[2]]
            try:
                _, enc = cv2.imencode('.jpg', crop)
                resp = requests.post('http://localhost:5001/get_rotation', files={"image": enc.tobytes()}, timeout=2.0)
                R_mat = np.array(resp.json()['R'])
            except: continue

            # Final Transformation to World Frame (Link_0)
            T_camera_nut = np.eye(4)
            T_camera_nut[:3, :3] = R_mat
            T_camera_nut[:3, 3] = T_vec
            T_world_nut = self.T_wc @ T_camera_nut
            
            self.broadcast_pose(T_world_nut)

    def transform_to_matrix(self, t):
        m = np.eye(4)
        m[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        q = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
        m[:3, :3] = R_tool.from_quat(q).as_matrix()
        return m

    def broadcast_pose(self, mat):
        # Existing TF broadcasting
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'Link_0'
        t.child_frame_id = 'detected_nut'
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = mat[:3, 3]
        q = R_tool.from_matrix(mat[:3, :3]).as_quat()
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        self.tf_broadcaster.sendTransform(t)
    
        # NEW: Publish to /pos_angle topic
        pos_angle_msg = Float32MultiArray()
    
        # Format: [x, y, z, qx, qy, qz, qw]
        pose_data = [
            mat[0, 3], mat[1, 3], mat[2, 3],  # position
            q[0], q[1], q[2], q[3]            # quaternion
        ]
    
        pos_angle_msg.data = pose_data
        self.pos_angle_pub.publish(pos_angle_msg)
        self.get_logger().info(f'Published to /pos_angle: {pose_data[:3]}')

    def send_to_display(self, img):
        try: display_queue.put_nowait(img)
        except queue.Full: pass

if __name__ == '__main__':
    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor()
    nodes = [CameraSystem(), NutEstimatorAAE()]
    for node in nodes: executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    while rclpy.ok(): time.sleep(1)
