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
        # Ensure these topic names match your Gazebo plugin exactly
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
        self.get_logger().info(f"Trigger: {trigger_signal}")


class NutEstimatorAAE(Node):
    def __init__(self):
        super().__init__('aae_estimator_node')
        self.model = YOLO("/home/ubuntu/6d_pose_estimation/nut_pose_pipeline/yolov12_seg_infer/model/best.pt")
        self.model.to('cpu')
        self.get_logger().info("YOLO model loaded on CPU")
        
        self.pos_angle_pub = self.create_publisher(Float32MultiArray, '/pos_angle', 10)

        # Replace your old self.K with this:
        self.K = np.array([
        [686.795514,   0.000000, 639.460651],  # fx, 0, cx
        [  0.000000, 686.779086, 359.347469],  # 0, fy, cy
        [  0.000000,   0.000000,   1.000000]
        ], dtype=np.float32)

# Add the distortion coefficients (D)
self.D = np.array([0.000444, -0.000358, -0.000029, -0.000008, 0.000000], dtype=np.float32)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.is_tf_ready = False
        self.T_wc = np.eye(4)

        self.create_timer(0.3, self.process_frame)
        self.get_logger().info("v7 - Fixed Mask Indexing & Depth Scaling")

    def process_frame(self):
        global rgb_image, depth_image, trigger_signal, last_heartbeat
        now = time.time()

        if now - last_heartbeat > 10:
            self.get_logger().info("Heartbeat: still running")
            last_heartbeat = now

        with data_lock:
            if rgb_image is None or depth_image is None:
                return
            display_img = rgb_image.copy()
            local_depth = depth_image.copy()
            local_trigger = trigger_signal

        if not local_trigger:
            cv2.putText(display_img, "IDLE - WAITING FOR TRIGGER", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            self.send_to_display(display_img)
            return

        if not self.is_tf_ready:
            try:
                # IMPORTANT: Use 'camera_depth_optical_frame' if that's what Gazebo uses
                t = self.tf_buffer.lookup_transform('world', 'camera_depth_optical_frame', rclpy.time.Time())
                self.T_wc = self.transform_to_matrix(t)
                self.is_tf_ready = True
                self.get_logger().info("TF ready")
            except Exception as e:
                self.get_logger().debug(f"TF wait: {str(e)[:80]}")
                self.send_to_display(display_img)
                return

        # ── YOLO Inference ──────────────────────────────────────
        t0 = time.time()
        try:
            # retina_masks=True helps keep masks at high resolution
            results = self.model(display_img, conf=0.55, verbose=False, device='cpu', retina_masks=True)
        except Exception as e:
            self.get_logger().error(f"YOLO crashed: {e}")
            return

        t_yolo = time.time() - t0
        detection_found = False

        for r in results:
            if r.boxes is None or len(r.boxes) == 0 or r.masks is None:
                continue

            detection_found = True
            display_img = r.plot()

            # ── FIX: Mask Resizing for Depth Alignment ──────────
            raw_mask = r.masks.data[0].cpu().numpy()
            h, w = local_depth.shape
            
            # If mask size doesn't match depth size, resize it
            if raw_mask.shape[0] != h or raw_mask.shape[1] != w:
                mask = cv2.resize(raw_mask, (w, h), interpolation=cv2.INTER_NEAREST) > 0
            else:
                mask = raw_mask > 0

            depths = local_depth[mask]
            if len(depths) == 0:
                continue

            # Gazebo uses meters, median filter helps remove noise
            z_m = np.median(depths[~np.isnan(depths)])
            
            if np.isnan(z_m) or z_m <= 0.05:
                continue
            # ── Center & Translation (FIXED) ────────────────────────────
            cx, cy = r.boxes.xywh[0][0].item(), r.boxes.xywh[0][1].item()
            # Use the actual focal lengths (K[0,0], K[1,1]) and principal points (K[0,2], K[1,2])
            f_x = self.K[0, 0]
            f_y = self.K[1, 1]
            c_x = self.K[0, 2]
            c_y = self.K[1, 2]

            # Standard Pinhole Camera Inverse Projection
            y_calibration_offset = 0.016  # Adjust this value (meters) to match 0.064 to 0.08

            tx = (cx - c_x) * z_m / f_x
            ty = ((cy - c_y) * z_m / f_y) - y_calibration_offset
            T_vec = np.array([tx, ty, z_m])

            # ── Rotation via External Server ─────────────────────
            box = r.boxes.xyxy[0].cpu().numpy().astype(int)
            crop = rgb_image[box[1]:box[3], box[0]:box[2]]

            try:
                _, enc = cv2.imencode('.jpg', crop)
                resp = requests.post('http://localhost:5001/get_rotation',
                                     files={"image": enc.tobytes()}, timeout=2.0)
                R_mat = np.array(resp.json()['R'])
            except Exception as e:
                self.get_logger().error(f"Rotation service error: {e}")
                continue

            # ── Transform & Broadcast ───────────────────────────
            display_img = self.draw_3d_axis(display_img, R_mat, T_vec)
            T_camera_nut = self.compose_mat(R_mat, T_vec)
            T_world_nut = self.T_wc @ T_camera_nut
            
            self.broadcast_pose(T_world_nut)

        if not detection_found:
            self.get_logger().info("No detections", once=True)

        self.send_to_display(display_img)

    def draw_3d_axis(self, img, R, t):
        axis_len = 0.03
        axis_pts = np.float32([[0,0,0], [axis_len,0,0], [0,axis_len,0], [0,0,axis_len]]).reshape(-1, 3)
        rvec, _ = cv2.Rodrigues(R)
        imgpts, _ = cv2.projectPoints(axis_pts, rvec, t.reshape(3,1), self.K, None)
        imgpts = imgpts.astype(int)
        origin = tuple(imgpts[0].ravel())
        cv2.line(img, origin, tuple(imgpts[1].ravel()), (0,0,255), 2) # X - Red
        cv2.line(img, origin, tuple(imgpts[2].ravel()), (0,255,0), 2) # Y - Green
        cv2.line(img, origin, tuple(imgpts[3].ravel()), (255,0,0), 2) # Z - Blue
        return img

    def transform_to_matrix(self, t):
        m = np.eye(4)
        m[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        q = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
        m[:3, :3] = R_tool.from_quat(q).as_matrix()
        return m

    def compose_mat(self, R, t):
        m = np.eye(4)
        m[:3, :3] = R
        m[:3, 3] = t
        return m

    def broadcast_pose(self, mat):
        now = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'world'
        t.child_frame_id = 'detected_nut'
        t.transform.translation.x = float(mat[0, 3])
        t.transform.translation.y = float(mat[1, 3])
        t.transform.translation.z = float(mat[2, 3])
        q = R_tool.from_matrix(mat[:3, :3]).as_quat()
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        self.tf_broadcaster.sendTransform(t)

        # Publish for pick-and-place node
        msg = Float32MultiArray()
        euler = R_tool.from_matrix(mat[:3, :3]).as_euler('xyz')
        msg.data = [float(mat[0,3]), float(mat[1,3]), float(mat[2,3]), float(euler[2])]
        self.pos_angle_pub.publish(msg)

    def send_to_display(self, img):
        try:
            display_queue.put_nowait(img)
        except queue.Full:
            pass

if __name__ == '__main__':
    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor()
    camera_node = CameraSystem()
    estimator_node = NutEstimatorAAE()
    executor.add_node(camera_node)
    executor.add_node(estimator_node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        cv2.namedWindow("6D Pose Visualization", cv2.WINDOW_NORMAL)
        while rclpy.ok():
            try:
                img = display_queue.get(timeout=0.1)
                cv2.imshow("6D Pose Visualization", img)
            except queue.Empty:
                pass
            if cv2.waitKey(1) == 27:
                break
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

class CameraSystem(Node):
    def __init__(self):
        super().__init__('camera_system_sub')
        # Ensure these topic names match your Gazebo plugin exactly
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
        self.get_logger().info(f"Trigger: {trigger_signal}")


class NutEstimatorAAE(Node):
    def __init__(self):
        super().__init__('aae_estimator_node')
        self.model = YOLO("/home/ubuntu/6d_pose_estimation/nut_pose_pipeline/yolov12_seg_infer/model/best.pt")
        self.model.to('cpu')
        self.get_logger().info("YOLO model loaded on CPU")
        
        self.pos_angle_pub = self.create_publisher(Float32MultiArray, '/pos_angle', 10)

        # D455 Nominal Intrinsics for 1280x720
        self.K = np.array([
            [900.0,   0.0, 640.0],  # fx, 0, cx
            [  0.0, 900.0, 360.0],  # 0, fy, cy
            [  0.0,   0.0,   1.0]
        ], dtype=np.float32)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.is_tf_ready = False
        self.T_wc = np.eye(4)

        self.create_timer(0.3, self.process_frame)
        self.get_logger().info("v7 - Fixed Mask Indexing & Depth Scaling")

    def process_frame(self):
        global rgb_image, depth_image, trigger_signal, last_heartbeat
        now = time.time()

        if now - last_heartbeat > 10:
            self.get_logger().info("Heartbeat: still running")
            last_heartbeat = now

        with data_lock:
            if rgb_image is None or depth_image is None:
                return
            display_img = rgb_image.copy()
            local_depth = depth_image.copy()
            local_trigger = trigger_signal

        if not local_trigger:
            cv2.putText(display_img, "IDLE - WAITING FOR TRIGGER", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            self.send_to_display(display_img)
            return

        if not self.is_tf_ready:
            try:
                # IMPORTANT: Use 'camera_depth_optical_frame' if that's what Gazebo uses
                t = self.tf_buffer.lookup_transform('Link_0', 'camera_color_optical_frame', rclpy.time.Time())
                self.T_wc = self.transform_to_matrix(t)
                self.is_tf_ready = True
                self.get_logger().info("TF ready")
            except Exception as e:
                self.get_logger().debug(f"TF wait: {str(e)[:80]}")
                self.send_to_display(display_img)
                return

        # ── YOLO Inference ──────────────────────────────────────
        t0 = time.time()
        try:
            # retina_masks=True helps keep masks at high resolution
            results = self.model(display_img, conf=0.55, verbose=False, device='cpu', retina_masks=True)
        except Exception as e:
            self.get_logger().error(f"YOLO crashed: {e}")
            return

        t_yolo = time.time() - t0
        detection_found = False

        for r in results:
            if r.boxes is None or len(r.boxes) == 0 or r.masks is None:
                continue

            detection_found = True
            display_img = r.plot()

            # ── FIX: Mask Resizing for Depth Alignment ──────────
            raw_mask = r.masks.data[0].cpu().numpy()
            h, w = local_depth.shape
            
            # If mask size doesn't match depth size, resize it
            if raw_mask.shape[0] != h or raw_mask.shape[1] != w:
                mask = cv2.resize(raw_mask, (w, h), interpolation=cv2.INTER_NEAREST) > 0
            else:
                mask = raw_mask > 0

            depths = local_depth[mask]
            if len(depths) == 0:
                continue

            # Gazebo uses meters, median filter helps remove noise
            z_m = np.median(depths[~np.isnan(depths)])
            
            if np.isnan(z_m) or z_m <= 0.05:
                continue

            # ── Center & Translation ────────────────────────────
            h, w, _ = display_img.shape # Get current image width/height
            center_x = w / 2
            center_y = h / 2

            # Update tx and ty calculation:
            tx = (cx - center_x) * z_m / self.K[0, 0]
            ty = (cy - center_y) * z_m / self.K[1, 1]
            T_vec = np.array([tx, ty, z_m])
            


            # ── Rotation via External Server ─────────────────────
            box = r.boxes.xyxy[0].cpu().numpy().astype(int)
            crop = rgb_image[box[1]:box[3], box[0]:box[2]]

            try:
                _, enc = cv2.imencode('.jpg', crop)
                resp = requests.post('http://localhost:5001/get_rotation',
                                     files={"image": enc.tobytes()}, timeout=2.0)
                R_mat = np.array(resp.json()['R'])
            except Exception as e:
                self.get_logger().error(f"Rotation service error: {e}")
                continue

            # ── Transform & Broadcast ───────────────────────────
            display_img = self.draw_3d_axis(display_img, R_mat, T_vec)
            T_camera_nut = self.compose_mat(R_mat, T_vec)
            T_world_nut = self.T_wc @ T_camera_nut
            
            self.broadcast_pose(T_world_nut)

        if not detection_found:
            self.get_logger().info("No detections", once=True)

        self.send_to_display(display_img)

    def draw_3d_axis(self, img, R, t):
        axis_len = 0.03
        axis_pts = np.float32([[0,0,0], [axis_len,0,0], [0,axis_len,0], [0,0,axis_len]]).reshape(-1, 3)
        rvec, _ = cv2.Rodrigues(R)
        imgpts, _ = cv2.projectPoints(axis_pts, rvec, t.reshape(3,1), self.K, None)
        imgpts = imgpts.astype(int)
        origin = tuple(imgpts[0].ravel())
        cv2.line(img, origin, tuple(imgpts[1].ravel()), (0,0,255), 2) # X - Red
        cv2.line(img, origin, tuple(imgpts[2].ravel()), (0,255,0), 2) # Y - Green
        cv2.line(img, origin, tuple(imgpts[3].ravel()), (255,0,0), 2) # Z - Blue
        return img

    def transform_to_matrix(self, t):
        m = np.eye(4)
        m[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        q = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
        m[:3, :3] = R_tool.from_quat(q).as_matrix()
        return m

    def compose_mat(self, R, t):
        m = np.eye(4)
        m[:3, :3] = R
        m[:3, 3] = t
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

        # Publish for pick-and-place node
        msg = Float32MultiArray()
        euler = R_tool.from_matrix(mat[:3, :3]).as_euler('xyz')
        msg.data = [float(mat[0,3]), float(mat[1,3]), float(mat[2,3]), float(euler[2])]
        self.pos_angle_pub.publish(msg)

    def send_to_display(self, img):
        try:
            display_queue.put_nowait(img)
        except queue.Full:
            pass

if __name__ == '__main__':
    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor()
    camera_node = CameraSystem()
    estimator_node = NutEstimatorAAE()
    executor.add_node(camera_node)
    executor.add_node(estimator_node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        cv2.namedWindow("6D Pose Visualization", cv2.WINDOW_NORMAL)
        while rclpy.ok():
            try:
                img = display_queue.get(timeout=0.1)
                cv2.imshow("6D Pose Visualization", img)
            except queue.Empty:
                pass
            if cv2.waitKey(1) == 27:
                break
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()
