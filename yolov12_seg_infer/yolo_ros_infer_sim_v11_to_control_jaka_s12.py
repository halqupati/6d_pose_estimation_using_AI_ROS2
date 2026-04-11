import os
import queue
import threading
import time

import cv2
import numpy as np
import requests
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R_tool
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from ultralytics import YOLO


bridge = CvBridge()
rgb_image = None
depth_image = None
trigger_signal = False
data_lock = threading.Lock()
display_queue = queue.Queue(maxsize=1)

WINDOW_NAME = "Stable Detection"
BUTTON_X1 = 20
BUTTON_Y1 = 20
BUTTON_X2 = 250
BUTTON_Y2 = 80
AAE_SERVER_PORT = int(os.environ.get("AAE_SERVER_PORT", "5001"))
AAE_SERVER_URL = os.environ.get("AAE_SERVER_URL", f"http://localhost:{AAE_SERVER_PORT}")


class CameraSystem(Node):
    def __init__(self):
        super().__init__("camera_system_sub")
        self.create_subscription(Image, "/rgbd_camera/image", self.rgb_cb, 10)
        self.create_subscription(Image, "/rgbd_camera/depth_image", self.depth_cb, 10)
        self.create_subscription(Bool, "/trigger", self.trigger_cb, 10)
        self.trigger_pub = self.create_publisher(Bool, "/trigger", 10)

    def rgb_cb(self, msg):
        global rgb_image
        with data_lock:
            rgb_image = bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_cb(self, msg):
        global depth_image
        with data_lock:
            depth_image = bridge.imgmsg_to_cv2(msg, "passthrough")

    def trigger_cb(self, msg):
        global trigger_signal
        trigger_signal = msg.data
        if msg.data:
            self.get_logger().info("\033[94mTrigger received: starting stable detection.\033[0m")
        else:
            self.get_logger().info("\033[90mTrigger cleared.\033[0m")

    def publish_trigger(self, value: bool):
        msg = Bool()
        msg.data = value
        self.trigger_pub.publish(msg)

    def start_trigger_cycle(self):
        # Force a clean false -> true edge so the estimator starts a fresh cycle.
        self.publish_trigger(False)
        time.sleep(0.05)
        self.publish_trigger(True)
        self.get_logger().info("\033[96mManual trigger cycle published.\033[0m")


class NutEstimatorAAE(Node):
    def __init__(self):
        super().__init__("yolo_estimator_node")
        self.model = YOLO(
            "/home/ubuntu/6d_pose_estimation/6d_pose_estimation_using_AI_ROS2/"
            "yolov12_seg_infer/model/best.pt"
        )
        self.model.to("cpu")
        self.pos_angle_pub = self.create_publisher(Float32MultiArray, "/pos_angle", 10)
        self.create_subscription(Bool, "/pose_received", self.pose_received_cb, 10)

        self.pose_buffer = []
        self.buffer_size = 5
        self.stability_threshold = 0.005

        self.K = np.array(
            [
                [686.795514, 0.0, 639.460651],
                [0.0, 686.779086, 359.347469],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.is_tf_ready = False
        self.T_wc = np.eye(4)

        self.previous_trigger_active = False
        self.waiting_for_pose_ack = False
        self.cycle_completed = False
        self.status_text = "IDLE"
        self.last_stable_pose = None

        self.get_logger().info(f"Using AAE endpoint: {AAE_SERVER_URL}/get_rotation")
        self.create_timer(0.1, self.process_frame)

    def pose_received_cb(self, msg: Bool):
        if msg.data and self.waiting_for_pose_ack:
            self.waiting_for_pose_ack = False
            self.cycle_completed = True
            self.pose_buffer = []
            self.status_text = "POSE SENT"
            self.get_logger().info(
                "\033[92mPose acknowledged by bin_picking_node_v7_to_control_jaka_s12. "
                "Waiting for next trigger.\033[0m"
            )

    def process_frame(self):
        global rgb_image, depth_image, trigger_signal

        with data_lock:
            if rgb_image is None or depth_image is None:
                return
            local_rgb = rgb_image.copy()
            local_depth = depth_image.copy()
            active = trigger_signal

        if active and not self.previous_trigger_active:
            self.pose_buffer = []
            self.waiting_for_pose_ack = False
            self.cycle_completed = False
            self.last_stable_pose = None
            self.status_text = "TRIGGERED"
        self.previous_trigger_active = active

        if not self.is_tf_ready:
            try:
                transform = self.tf_buffer.lookup_transform(
                    "base_link", "camera_depth_optical_frame", rclpy.time.Time()
                )
                self.T_wc = self.transform_to_matrix(transform)
                self.is_tf_ready = True
            except Exception:
                return

        if not active:
            self.pose_buffer = []
            self.waiting_for_pose_ack = False
            self.cycle_completed = False
            self.status_text = "IDLE"
            self.push_display_frame(self.decorate_frame(local_rgb))
            return

        if self.cycle_completed:
            self.status_text = "WAIT NEXT TRIGGER"
            self.push_display_frame(self.decorate_frame(local_rgb))
            return

        if self.waiting_for_pose_ack:
            self.status_text = "WAIT ACK"
            self.push_display_frame(self.decorate_frame(local_rgb))
            return

        results = self.model(local_rgb, conf=0.55, verbose=False, retina_masks=True)
        viz_img = local_rgb.copy()
        found_detection = False

        for result in results:
            if result.masks is None:
                continue

            found_detection = True
            det_x = result.boxes.xywh[0][0].item()
            det_y = result.boxes.xywh[0][1].item()
            raw_mask = result.masks.data[0].cpu().numpy()
            mask = cv2.resize(
                raw_mask,
                (local_depth.shape[1], local_depth.shape[0]),
                interpolation=cv2.INTER_NEAREST,
            ) > 0

            depths = local_depth[mask]
            depths = depths[~np.isnan(depths)]
            if len(depths) == 0:
                continue

            z_m = np.median(depths)
            if z_m <= 0.05:
                continue

            tx = (det_x - self.K[0, 2]) * z_m / self.K[0, 0]
            ty = (det_y - self.K[1, 2]) * z_m / self.K[1, 1]
            t_vec = np.array([tx, ty, z_m])

            box = result.boxes.xyxy[0].cpu().numpy().astype(int)
            crop = local_rgb[box[1]:box[3], box[0]:box[2]]
            try:
                _, encoded = cv2.imencode(".jpg", crop)
                response = requests.post(
                    f"{AAE_SERVER_URL}/get_rotation",
                    files={"image": encoded.tobytes()},
                    timeout=1.0,
                )
                r_mat = np.array(response.json()["R"])
            except Exception:
                continue

            t_camera_nut = np.eye(4)
            t_camera_nut[:3, :3] = r_mat
            t_camera_nut[:3, 3] = t_vec
            t_world_nut = self.T_wc @ t_camera_nut

            current_pos = t_world_nut[:3, 3]
            self.pose_buffer.append(current_pos)
            if len(self.pose_buffer) > self.buffer_size:
                self.pose_buffer.pop(0)

            cv2.rectangle(viz_img, (box[0], box[1]), (box[2], box[3]), (255, 0, 0), 2)

            if len(self.pose_buffer) == self.buffer_size:
                std_dev = np.std(self.pose_buffer, axis=0)
                if np.all(std_dev < self.stability_threshold):
                    self.last_stable_pose = t_world_nut.copy()
                    self.broadcast_and_publish(t_world_nut)
                    self.waiting_for_pose_ack = True
                    self.status_text = "WAIT ACK"
                    cv2.putText(
                        viz_img,
                        "STABLE",
                        (int(det_x), int(det_y)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        (0, 255, 0),
                        2,
                    )
                    break

                self.status_text = "STABILIZING"
                self.get_logger().warn("Waiting for nut to stop moving...")
            else:
                self.status_text = "COLLECTING"

        if not found_detection:
            self.status_text = "NO NUT"

        self.push_display_frame(self.decorate_frame(viz_img))

    def push_display_frame(self, frame):
        try:
            display_queue.put_nowait(frame)
        except queue.Full:
            pass

    def transform_to_matrix(self, transform):
        matrix = np.eye(4)
        matrix[:3, 3] = [
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ]
        quat = [
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ]
        matrix[:3, :3] = R_tool.from_quat(quat).as_matrix()
        return matrix

    def broadcast_and_publish(self, mat):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = "base_link"
        tf_msg.child_frame_id = "detected_nut"
        tf_msg.transform.translation.x = float(mat[0, 3])
        tf_msg.transform.translation.y = float(mat[1, 3])
        tf_msg.transform.translation.z = float(mat[2, 3])
        quat = R_tool.from_matrix(mat[:3, :3]).as_quat()
        tf_msg.transform.rotation.x = float(quat[0])
        tf_msg.transform.rotation.y = float(quat[1])
        tf_msg.transform.rotation.z = float(quat[2])
        tf_msg.transform.rotation.w = float(quat[3])
        self.tf_broadcaster.sendTransform(tf_msg)

        msg = Float32MultiArray()
        msg.data = [
            float(mat[0, 3]),
            float(mat[1, 3]),
            float(mat[2, 3]),
            float(quat[0]),
            float(quat[1]),
            float(quat[2]),
            float(quat[3]),
        ]
        self.pos_angle_pub.publish(msg)
        self.get_logger().info(f"\033[92mStable pose published once: {msg.data[:3]}\033[0m")

    def decorate_frame(self, frame):
        viz = frame.copy()

        button_color = (40, 170, 40) if not trigger_signal else (80, 80, 80)
        cv2.rectangle(viz, (BUTTON_X1, BUTTON_Y1), (BUTTON_X2, BUTTON_Y2), button_color, -1)
        cv2.rectangle(viz, (BUTTON_X1, BUTTON_Y1), (BUTTON_X2, BUTTON_Y2), (255, 255, 255), 2)
        cv2.putText(
            viz,
            "START TRIGGER",
            (BUTTON_X1 + 15, BUTTON_Y1 + 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
        )

        cv2.putText(
            viz,
            f"STATUS: {self.status_text}",
            (20, 115),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
        )

        if self.last_stable_pose is not None:
            pos = self.last_stable_pose[:3, 3]
            cv2.putText(
                viz,
                f"LAST POSE: x={pos[0]:.3f} y={pos[1]:.3f} z={pos[2]:.3f}",
                (20, 150),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 220, 0),
                2,
            )

        return viz


def mouse_callback(event, x, y, flags, camera_node):
    if event != cv2.EVENT_LBUTTONUP:
        return

    if BUTTON_X1 <= x <= BUTTON_X2 and BUTTON_Y1 <= y <= BUTTON_Y2:
        camera_node.start_trigger_cycle()


if __name__ == "__main__":
    rclpy.init()

    executor = rclpy.executors.MultiThreadedExecutor()
    cam_node = CameraSystem()
    est_node = NutEstimatorAAE()
    executor.add_node(cam_node)
    executor.add_node(est_node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(WINDOW_NAME, mouse_callback, cam_node)

    try:
        while rclpy.ok():
            try:
                frame = display_queue.get(timeout=0.1)
                cv2.imshow(WINDOW_NAME, frame)
            except queue.Empty:
                pass

            if cv2.waitKey(1) & 0xFF == 27:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()
