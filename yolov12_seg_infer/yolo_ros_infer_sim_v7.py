import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool , Float32MultiArray
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

# For heartbeat
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
            # self.get_logger().info("RGB received", once=True)
        except Exception as e:
            self.get_logger().error(f"RGB error: {e}")

    def depth_cb(self, msg):
        global depth_image
        try:
            with data_lock:
                depth_image = bridge.imgmsg_to_cv2(msg, 'passthrough')
            # self.get_logger().info("Depth received", once=True)
        except Exception as e:
            self.get_logger().error(f"Depth error: {e}")

    def trigger_cb(self, msg):
        global trigger_signal
        trigger_signal = msg.data
        self.get_logger().info(f"Trigger: {trigger_signal}")


class NutEstimatorAAE(Node):
    def __init__(self):
        super().__init__('aae_estimator_node')
        self.model = YOLO("/home/ubuntu/6d_pose_estimation/6d_pose_estimation_using_AI_ROS2/yolov12_seg_infer/model/best.pt")
        self.model.to('cpu')
        self.get_logger().info("YOLO model loaded on CPU")
        # 1. Create the Publisher for the bin_picking node
        self.pos_angle_pub = self.create_publisher(Float32MultiArray, '/pos_angle', 10)

        self.K = np.array([[615.0, 0, 320.0], [0, 615.0, 240.0], [0, 0, 1]], dtype=np.float32)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.is_tf_ready = False
        self.T_wc = np.eye(4)

        self.create_timer(0.3, self.process_frame)
        self.get_logger().info("v6 - Detailed debug logging enabled")

    def process_frame(self):
        global rgb_image, depth_image, trigger_signal, last_heartbeat

        now = time.time()

        # Heartbeat every ~10 seconds
        if now - last_heartbeat > 10:
            self.get_logger().info("Heartbeat: still running")
            last_heartbeat = now

        with data_lock:
            if rgb_image is None:
                return
            display_img = rgb_image.copy()
            local_depth = depth_image.copy() if depth_image is not None else None
            local_trigger = trigger_signal

        if local_depth is not None:
            med_all = np.median(local_depth)
            valid_count = np.sum((~np.isnan(local_depth)) & (local_depth > 0))
            total_px = local_depth.size
            self.get_logger().debug(f"Depth global: median={med_all:.4f} m, valid px={valid_count}/{total_px} ({valid_count/total_px*100:.1f}%)")

        if not local_trigger:
            cv2.putText(display_img, "IDLE - WAITING FOR TRIGGER", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            self.send_to_display(display_img)
            return

        if local_depth is None:
            cv2.putText(display_img, "MISSING DEPTH", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            self.send_to_display(display_img)
            return

        if not self.is_tf_ready:
            try:
                t = self.tf_buffer.lookup_transform('Link_0', 'camera_color_optical_frame', rclpy.time.Time())
                self.T_wc = self.transform_to_matrix(t)
                self.is_tf_ready = True
                self.get_logger().info("TF ready")
            except Exception as e:
                self.get_logger().debug(f"TF wait: {str(e)[:80]}")
                self.send_to_display(display_img)
                return

        # ── YOLO ────────────────────────────────────────────────
        t0 = time.time()
        try:
            results = self.model(display_img, conf=0.55, verbose=False, device='cpu')
        except Exception as e:
            self.get_logger().error(f"YOLO crashed: {e}")
            self.send_to_display(display_img)
            return
        t_yolo = time.time() - t0

        self.get_logger().info(f"YOLO took {t_yolo:.3f}s → {len(results)} result(s)")

        detection_found = False

        for ri, r in enumerate(results):
            if r.boxes is None or len(r.boxes) == 0:
                self.get_logger().debug(f"Result {ri}: no boxes")
                continue

            confs = r.boxes.conf.cpu().numpy()
            self.get_logger().info(f"Result {ri}: {len(r.boxes)} boxes, confs = {np.round(confs, 2).tolist()}")

            if r.masks is None or len(r.masks) == 0:
                self.get_logger().debug(f"Result {ri}: has boxes but no masks")
                continue

            detection_found = True
            display_img = r.plot()  # draw boxes + masks

            mask = r.masks.data[0].cpu().numpy() > 0
            mask_pixels = np.sum(mask)
            self.get_logger().debug(f"Mask area: {mask_pixels} px")

            # ── Depth under mask ────────────────────────────────
            depths = local_depth[mask]
            if len(depths) == 0:
                self.get_logger().warning("Mask exists but no depth pixels under it")
                continue

            z_m = np.median(depths)
            self.get_logger().info(f"Median depth under mask: {z_m:.4f} m")

            if np.isnan(z_m) or z_m <= 0.001 or z_m > 15.0:
                self.get_logger().warning(f"Invalid depth {z_m:.4f} → skipping pose")
                continue

            if z_m > 10:
                z_m /= 1000.0
                self.get_logger().info(f"Converted depth from mm to m: {z_m:.4f}")

            # ── Center & translation ────────────────────────────
            cx = r.boxes.xywh[0][0].cpu().item()
            cy = r.boxes.xywh[0][1].cpu().item()
            tx = (cx - self.K[0, 2]) * z_m / self.K[0, 0]
            ty = (cy - self.K[1, 2]) * z_m / self.K[1, 1]
            T_vec = np.array([tx, ty, z_m])

            # ── Rotation via HTTP ───────────────────────────────
            crop = display_img[int(r.boxes.xyxy[0][1]):int(r.boxes.xyxy[0][3]),
                               int(r.boxes.xyxy[0][0]):int(r.boxes.xyxy[0][2])]

            t_http0 = time.time()
            try:
                _, enc = cv2.imencode('.jpg', crop)
                resp = requests.post('http://localhost:5001/get_rotation',
                                     files={"image": enc.tobytes()},
                                     timeout=12.0)
                resp.raise_for_status()
                R_mat = np.array(resp.json()['R'])
                t_http = time.time() - t_http0
                self.get_logger().info(f"HTTP rotation OK ({t_http:.2f}s)")
            except Exception as e:
                self.get_logger().error(f"HTTP rotation failed: {str(e)[:120]}")
                continue

            # ── Draw & broadcast ────────────────────────────────
            display_img = self.draw_3d_axis(display_img, R_mat, T_vec)
            T_wn = self.T_wc @ self.compose_mat(R_mat, T_vec)
            self.print_pose_info(T_wn)
            self.broadcast_pose(T_wn)

        if not detection_found:
            self.get_logger().info("No valid detection this frame")

        self.send_to_display(display_img)

    def send_to_display(self, img):
        try:
            display_queue.put_nowait(img)
        except queue.Full:
            pass

    # ──────────────────────────────────────────────────────────────
    # The rest remains the same (draw_3d_axis, transform_to_matrix,
    # print_pose_info, compose_mat, broadcast_pose)
    # ──────────────────────────────────────────────────────────────

    def draw_3d_axis(self, img, R, t):
        axis_len = 0.05
        axis_pts = np.float32([[0,0,0], [axis_len,0,0], [0,axis_len,0], [0,0,axis_len]]).reshape(-1, 3)
        rvec, _ = cv2.Rodrigues(R)
        imgpts, _ = cv2.projectPoints(axis_pts, rvec, t.reshape(3,1), self.K, None)
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

    def print_pose_info(self, mat):
        pos = mat[:3, 3]
        euler = R_tool.from_matrix(mat[:3, :3]).as_euler('xyz', degrees=True)
        print("\n" + "═"*60)
        print("DETECTED NUT POSE (Link_0)")
        print(f"Pos   : {pos[0]:8.3f} {pos[1]:8.3f} {pos[2]:8.3f}")
        print(f"Euler : {euler[0]:8.1f} {euler[1]:8.1f} {euler[2]:8.1f} deg")
        print("═"*60 + "\n")

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
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])
        self.tf_broadcaster.sendTransform(t)
        
        pos = mat[:3, 3] # [x, y, z]
        
        euler = R_tool.from_matrix(mat[:3, :3]).as_euler('xyz', degrees=False)
        yaw_angle = euler[2] # The Z-axis rotation

        msg = Float32MultiArray()
        msg.data = [
            float(pos[0]), # x
            float(pos[1]), # y
            float(pos[2]), # z
            float(yaw_angle) # angle (in radians)
        ]
        self.pos_angle_pub.publish(msg)
        self.get_logger().info(f"Published to /pos_angle: {msg.data}")


if __name__ == '__main__':
    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor()
    camera_node = CameraSystem()
    estimator_node = NutEstimatorAAE()
    executor.add_node(camera_node)
    executor.add_node(estimator_node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print("Main thread - OpenCV display loop started")
    print("Press ESC to quit")

    try:
        cv2.namedWindow("6D Pose Visualization", cv2.WINDOW_NORMAL)
        while rclpy.ok():
            try:
                img = display_queue.get(timeout=0.1)
                cv2.imshow("6D Pose Visualization", img)
            except queue.Empty:
                pass
            if cv2.waitKey(1) == 27:
                print("ESC pressed → shutdown")
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        print("Shutdown complete.")
