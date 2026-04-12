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
YOLO_CONF_THRESHOLD = float(os.environ.get("YOLO_CONF_THRESHOLD", "0.35"))
YOLO_IMGSZ = int(os.environ.get("YOLO_IMGSZ", "1280"))
YOLO_MAX_DET = int(os.environ.get("YOLO_MAX_DET", "20"))
TRACK_MATCH_DISTANCE_PX = float(os.environ.get("TRACK_MATCH_DISTANCE_PX", "90.0"))
MAX_MISSING_FRAMES = int(os.environ.get("MAX_MISSING_FRAMES", "6"))
MIN_MASK_PIXELS = int(os.environ.get("MIN_MASK_PIXELS", "120"))
MIN_DEPTH_SAMPLES = int(os.environ.get("MIN_DEPTH_SAMPLES", "80"))
AAE_CROP_PADDING_PX = int(os.environ.get("AAE_CROP_PADDING_PX", "12"))


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
        self.visible_candidate_count = 0
        self.tracked_center = None
        self.max_missing_frames = MAX_MISSING_FRAMES
        self.track_match_distance_px = TRACK_MATCH_DISTANCE_PX
        self.min_mask_pixels = MIN_MASK_PIXELS
        self.min_depth_samples = MIN_DEPTH_SAMPLES
        self.aae_crop_padding_px = AAE_CROP_PADDING_PX
        self.missed_frames = 0
        self.yolo_conf_threshold = YOLO_CONF_THRESHOLD
        self.yolo_imgsz = YOLO_IMGSZ
        self.yolo_max_det = YOLO_MAX_DET

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
        self.create_timer(0.1, self.refresh_detected_tf)

    def pose_received_cb(self, msg: Bool):
        if msg.data and self.waiting_for_pose_ack:
            self.waiting_for_pose_ack = False
            self.cycle_completed = True
            self.reset_tracking()
            self.status_text = "POSE SENT"
            self.get_logger().info(
                "\033[92mPose acknowledged by bin_picking node. "
                "Waiting for next trigger.\033[0m"
            )

    def reset_tracking(self):
        self.pose_buffer = []
        self.tracked_center = None
        self.visible_candidate_count = 0
        self.missed_frames = 0

    def extract_candidates(self, result, local_rgb, local_depth):
        candidates = []
        if result.masks is None or result.boxes is None:
            return candidates

        mask_count = len(result.masks.data)
        box_count = len(result.boxes.xyxy)
        if mask_count == 0 or box_count == 0:
            return candidates

        max_x = local_rgb.shape[1] - 1
        max_y = local_rgb.shape[0] - 1

        for idx in range(min(mask_count, box_count)):
            raw_box = result.boxes.xyxy[idx].cpu().numpy().astype(int)
            box = raw_box.copy()
            box[0] = int(np.clip(box[0], 0, max_x))
            box[1] = int(np.clip(box[1], 0, max_y))
            box[2] = int(np.clip(box[2], 0, max_x))
            box[3] = int(np.clip(box[3], 0, max_y))
            if box[2] <= box[0] or box[3] <= box[1]:
                continue

            raw_mask = result.masks.data[idx].cpu().numpy()
            mask = cv2.resize(
                raw_mask,
                (local_depth.shape[1], local_depth.shape[0]),
                interpolation=cv2.INTER_NEAREST,
            ) > 0
            mask_area = int(np.count_nonzero(mask))
            if mask_area < self.min_mask_pixels:
                continue

            depths = local_depth[mask]
            depths = depths[np.isfinite(depths)]
            depths = depths[depths > 0.05]
            if depths.size < self.min_depth_samples:
                continue

            det_x = float(result.boxes.xywh[idx][0].item())
            det_y = float(result.boxes.xywh[idx][1].item())
            z_m = float(np.median(depths))
            tx = (det_x - self.K[0, 2]) * z_m / self.K[0, 0]
            ty = (det_y - self.K[1, 2]) * z_m / self.K[1, 1]

            conf = 0.0
            if result.boxes.conf is not None:
                conf = float(result.boxes.conf[idx].item())

            center = np.array([det_x, det_y], dtype=np.float32)
            track_distance = float("inf")
            if self.tracked_center is not None:
                track_distance = float(np.linalg.norm(center - self.tracked_center))

            candidates.append(
                {
                    "box": box,
                    "center": center,
                    "conf": conf,
                    "mask_area": mask_area,
                    "t_vec": np.array([tx, ty, z_m], dtype=np.float32),
                    "track_distance": track_distance,
                }
            )

        return candidates

    def rank_candidates(self, candidates):
        def score(candidate):
            return candidate["conf"] * 1000.0 + candidate["mask_area"]

        if self.tracked_center is None:
            return sorted(candidates, key=score, reverse=True)

        def sort_key(candidate):
            candidate_score = score(candidate)
            matched = candidate["track_distance"] <= self.track_match_distance_px
            if matched:
                return (0, candidate["track_distance"], -candidate_score)
            return (1, 0.0, -candidate_score)

        return sorted(candidates, key=sort_key)

    def crop_candidate(self, local_rgb, box):
        x1, y1, x2, y2 = box
        x1 = max(0, x1 - self.aae_crop_padding_px)
        y1 = max(0, y1 - self.aae_crop_padding_px)
        x2 = min(local_rgb.shape[1], x2 + self.aae_crop_padding_px)
        y2 = min(local_rgb.shape[0], y2 + self.aae_crop_padding_px)
        if x2 <= x1 or y2 <= y1:
            return None
        return local_rgb[y1:y2, x1:x2]

    def estimate_world_pose(self, local_rgb, candidate):
        crop = self.crop_candidate(local_rgb, candidate["box"])
        if crop is None or crop.size == 0:
            return None

        try:
            encoded_ok, encoded = cv2.imencode(".jpg", crop)
            if not encoded_ok:
                return None

            response = requests.post(
                f"{AAE_SERVER_URL}/get_rotation",
                files={"image": encoded.tobytes()},
                timeout=1.0,
            )
            response.raise_for_status()
            r_mat = np.array(response.json()["R"], dtype=np.float32)
            if r_mat.shape != (3, 3):
                return None
        except Exception:
            return None

        t_camera_nut = np.eye(4, dtype=np.float32)
        t_camera_nut[:3, :3] = r_mat
        t_camera_nut[:3, 3] = candidate["t_vec"]
        return self.T_wc @ t_camera_nut

    def draw_candidates(self, image, candidates, selected_candidate=None):
        selected_box = None
        if selected_candidate is not None:
            selected_box = tuple(int(v) for v in selected_candidate["box"])

        for candidate in candidates:
            box = tuple(int(v) for v in candidate["box"])
            is_selected = box == selected_box
            color = (0, 255, 0) if is_selected else (255, 0, 0)
            thickness = 3 if is_selected else 1
            cv2.rectangle(image, (box[0], box[1]), (box[2], box[3]), color, thickness)
            cv2.putText(
                image,
                f"{candidate['conf']:.2f}",
                (box[0], max(20, box[1] - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
            )

    def refresh_detected_tf(self):
        if self.last_stable_pose is not None:
            self.broadcast_transform(self.last_stable_pose)

    def process_frame(self):
        global rgb_image, depth_image, trigger_signal

        with data_lock:
            if rgb_image is None or depth_image is None:
                return
            local_rgb = rgb_image.copy()
            local_depth = depth_image.copy()
            active = trigger_signal

        if active and not self.previous_trigger_active:
            self.reset_tracking()
            self.waiting_for_pose_ack = False
            self.cycle_completed = False
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
            self.reset_tracking()
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

        results = self.model(
            local_rgb,
            conf=self.yolo_conf_threshold,
            imgsz=self.yolo_imgsz,
            max_det=self.yolo_max_det,
            verbose=False,
            retina_masks=True,
        )
        viz_img = local_rgb.copy()
        candidates = []
        for result in results:
            candidates.extend(self.extract_candidates(result, local_rgb, local_depth))

        self.visible_candidate_count = len(candidates)
        if not candidates:
            self.missed_frames += 1
            if self.missed_frames >= self.max_missing_frames:
                self.pose_buffer = []
                self.tracked_center = None
                self.status_text = "NO NUT"
            else:
                self.status_text = "TRACK LOST"
            self.push_display_frame(self.decorate_frame(viz_img))
            return

        ranked_candidates = self.rank_candidates(candidates)
        selected_candidate = None
        selected_pose = None
        matched_track = False

        for candidate in ranked_candidates:
            pose = self.estimate_world_pose(local_rgb, candidate)
            if pose is None:
                continue

            selected_candidate = candidate
            selected_pose = pose
            matched_track = (
                self.tracked_center is not None
                and candidate["track_distance"] <= self.track_match_distance_px
            )
            break

        self.draw_candidates(viz_img, candidates, selected_candidate)

        if selected_candidate is None:
            self.missed_frames += 1
            self.status_text = "AAE FAIL"
            self.push_display_frame(self.decorate_frame(viz_img))
            return

        if self.tracked_center is None or not matched_track:
            self.pose_buffer = []

        self.tracked_center = selected_candidate["center"]
        self.missed_frames = 0

        current_pos = selected_pose[:3, 3]
        self.pose_buffer.append(current_pos)
        if len(self.pose_buffer) > self.buffer_size:
            self.pose_buffer.pop(0)

        det_x, det_y = selected_candidate["center"]

        if len(self.pose_buffer) == self.buffer_size:
            std_dev = np.std(self.pose_buffer, axis=0)
            if np.all(std_dev < self.stability_threshold):
                self.last_stable_pose = selected_pose.copy()
                self.broadcast_and_publish(selected_pose)
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
            else:
                self.status_text = "STABILIZING"
        else:
            self.status_text = "COLLECTING"

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

    def broadcast_transform(self, mat):
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

        return quat

    def broadcast_and_publish(self, mat):
        quat = self.broadcast_transform(mat)

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
        self.get_logger().info(f"\033[92mStable pose published: {msg.data[:3]}\033[0m")

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

        cv2.putText(
            viz,
            f"CANDIDATES: {self.visible_candidate_count}",
            (20, 185),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 200, 0),
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
