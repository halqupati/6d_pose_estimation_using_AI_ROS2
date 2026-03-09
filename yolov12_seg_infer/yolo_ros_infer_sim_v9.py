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
last_heartbeat = time.time()

class CameraSystem(Node):
    def __init__(self):
        super().__init__('camera_system_sub')
        self.create_subscription(Image, '/rgbd_camera/image', self.rgb_cb, 10)
        self.create_subscription(Image, '/rgbd_camera/depth_image', self.depth_cb, 10)
        self.create_subscription(Bool, '/trigger', self.trigger_cb, 10)

    def rgb_cb(self, msg):
        global rgb_image
        try:
            with data_lock:
                rgb_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"RGB error: {e}")

    def depth_cb(self, msg):
        global depth_image
        try:
            with data_lock:
                depth_image = bridge.imgmsg_to_cv2(msg, 'passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth error: {e}")

    def trigger_cb(self, msg):
        global trigger_signal
        trigger_signal = msg.data

class NutEstimatorAAE(Node):
    def __init__(self):
        super().__init__('aae_estimator_node')
        # Load YOLO Model
        self.model = YOLO("/home/ubuntu/6d_pose_estimation/nut_pose_pipeline/yolov12_seg_infer/model/best.pt")
        self.model.to('cpu')
        self.get_logger().info("YOLO model loaded with Calibration Params")
        
        self.pos_angle_pub = self.create_publisher(Float32MultiArray, '/pos_angle', 10)

        # === UPDATED WITH YOUR CALIBRATION RESULTS ===
        self.K = np.array([
            [686.795514,   0.000000, 639.460651],  # fx, 0, cx
            [  0.000000, 686.779086, 359.347469],  # 0, fy, cy
            [  0.000000,   0.000000,   1.000000]
        ], dtype=np.float32)

        self.D = np.array([0.000444, -0.000358, -0.000029, -0.000008, 0.000000], dtype=np.float32)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.is_tf_ready = False
        self.T_wc = np.eye(4)

        self.create_timer(0.3, self.process_frame)

    def process_frame(self):
        global rgb_image, depth_image, trigger_signal
        
        with data_lock:
            if rgb_image is None or depth_image is None:
                return
            display_img = rgb_image.copy()
            local_depth = depth_image.copy()
            local_trigger = trigger_signal

        if not local_trigger:
            return

        # Lookup Transform - Using color_optical_frame to match RGB calibration
        try:
            t = self.tf_buffer.lookup_transform('Link_0', 'camera_color_optical_frame', rclpy.time.Time())
            self.T_wc = self.transform_to_matrix(t)
            self.is_tf_ready = True
        except Exception:
            return

        # ── YOLO Inference ──────────────────────────────────────
        results = self.model(display_img, conf=0.55, verbose=False, device='cpu', retina_masks=True)

        for r in results:
            if r.boxes is None or len(r.boxes) == 0 or r.masks is None:
                continue

            display_img = r.plot()

            # ── Mask & Depth ────────────────────────────────────
            raw_mask = r.masks.data[0].cpu().numpy()
            h, w = local_depth.shape
            if raw_mask.shape[:2] != (h, w):
                mask = cv2.resize(raw_mask, (w, h), interpolation=cv2.INTER_NEAREST) > 0
            else:
                mask = raw_mask > 0

            depths = local_depth[mask]
            if len(depths) == 0: continue
            z_m = np.median(depths[~np.isnan(depths)])
            if np.isnan(z_m) or z_m <= 0.05: continue

            # ── Translation Logic (Using Principal Point cx, cy) ──
            # cx, cy from YOLO detection
            det_x, det_y = r.boxes.xywh[0][0].item(), r.boxes.xywh[0][1].item()
            
            # Calibration constants
            f_x, f_y = self.K[0, 0], self.K[1, 1]
            c_x, c_y = self.K[0, 2], self.K[1, 2]

            # Project pixels to 3D Camera Space
            tx = (det_x - c_x) * z_m / f_x
            ty = (det_y - c_y) * z_m / f_y
            T_vec = np.array([tx, ty, z_m])

            # ── Rotation via AAE Server ──────────────────────────
            box = r.boxes.xyxy[0].cpu().numpy().astype(int)
            crop = rgb_image[box[1]:box[3], box[0]:box[2]]
            try:
                _, enc = cv2.imencode('.jpg', crop)
                resp = requests.post('http://localhost:5001/get_rotation',
                                     files={"image": enc.tobytes()}, timeout=2.0)
                R_mat = np.array(resp.json()['R'])
            except Exception as e:
                self.get_logger().error(f"Rotation error: {e}")
                continue

            # ── Transform to World (Link_0) ──────────────────────
            display_img = self.draw_3d_axis(display_img, R_mat, T_vec)
            T_camera_nut = self.compose_mat(R_mat, T_vec)
            T_world_nut = self.T_wc @ T_camera_nut
            
            self.broadcast_pose(T_world_nut)

        self.send_to_display(display_img)

    def draw_3d_axis(self, img, R, t):
        axis_len = 0.03
        axis_pts = np.float32([[0,0,0], [axis_len,0,0], [0,axis_len,0], [0,0,axis_len]]).reshape(-1, 3)
        rvec, _ = cv2.Rodrigues(R)
        # Included self.D to account for lens distortion in visualization
        imgpts, _ = cv2.projectPoints(axis_pts, rvec, t.reshape(3,1), self.K, self.D)
        imgpts = imgpts.astype(int)
        origin = tuple(imgpts[0].ravel())
        cv2.line(img, origin, tuple(imgpts[1].ravel()), (0,0,255), 2)
        cv2.line(img, origin, tuple(imgpts[2].ravel()), (0,255,0), 2)
        cv2.line(img, origin, tuple(imgpts[3].ravel()), (255,0,0), 2)
        return img

    def transform_to_matrix(self, t):
        m = np.eye(4)
        m[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        q = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
        m[:3, :3] = R_tool.from_quat(q).as_matrix()
        return m

    def compose_mat(self, R, t):
        m = np.eye(4); m[:3, :3] = R; m[:3, 3] = t
        return m

    def broadcast_pose(self, mat):
        now = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'Link_0'
        t.child_frame_id = 'detected_nut'
        t.transform.translation.x = float(mat[0, 3])
        t.transform.translation.y = float(mat[1, 3])
        t.transform.translation.z = float(mat[2, 3])
        q = R_tool.from_matrix(mat[:3, :3]).as_quat()
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        self.tf_broadcaster.sendTransform(t)

        msg = Float32MultiArray()
        euler = R_tool.from_matrix(mat[:3, :3]).as_euler('xyz')
        msg.data = [float(mat[0,3]), float(mat[1,3]), float(mat[2,3]), float(euler[2])]
        self.pos_angle_pub.publish(msg)

    def send_to_display(self, img):
        try: display_queue.put_nowait(img)
        except queue.Full: pass

if __name__ == '__main__':
    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor()
    nodes = [CameraSystem(), NutEstimatorAAE()]
    for node in nodes: executor.add_node(node)
    
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    cv2.namedWindow("6D Pose Visualization", cv2.WINDOW_NORMAL)
    while rclpy.ok():
        try:
            img = display_queue.get(timeout=0.1)
            cv2.imshow("6D Pose Visualization", img)
        except queue.Empty: pass
        if cv2.waitKey(1) == 27: break
    rclpy.shutdown()
