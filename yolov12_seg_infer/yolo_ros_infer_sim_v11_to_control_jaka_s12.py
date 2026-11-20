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
OBJECT_YAW_SYMMETRY_DEG = float(os.environ.get("OBJECT_YAW_SYMMETRY_DEG", "60.0"))
YAW_STABILITY_THRESHOLD_DEG = float(os.environ.get("YAW_STABILITY_THRESHOLD_DEG", "8.0"))
YAW_OFFSET_DEG = float(os.environ.get("YAW_OFFSET_DEG", "0.0"))
CONTOUR_WORLD_EPSILON = float(os.environ.get("CONTOUR_WORLD_EPSILON", "0.0015"))
MIN_WORLD_CONTOUR_POINTS = int(os.environ.get("MIN_WORLD_CONTOUR_POINTS", "30"))

# Explicit nut-model selection; must match the AAE server's AAE_NUT_MODEL.
# No fallback between entries: selecting m6 never loads the legacy model.
YOLO_MODEL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "model")
YOLO_MODEL_REGISTRY = {
    # YOLOv12n-seg, single class 'nut', trained 2025-11-25 (ultralytics 8.3.232) - this
    # predates the M6 work and is the detector the legacy M20 AAE pipeline ran with.
    "m20": "best.pt",
    # Expected M6 detector. NOT present in the repository/machine as of this change;
    # copy the trained file here under this name before selecting m6.
    "m6": "best_m6_yolov26.pt",
}
_NUT_MODEL_ENV = os.environ.get("NUT_MODEL")
NUT_MODEL = (_NUT_MODEL_ENV or "m20").strip().lower()


def resolve_yolo_model_path() -> str:
    if NUT_MODEL not in YOLO_MODEL_REGISTRY:
        raise SystemExit(f"[YOLO] Unknown NUT_MODEL={NUT_MODEL!r}. Valid values: {sorted(YOLO_MODEL_REGISTRY)}")
    path = os.path.join(YOLO_MODEL_DIR, YOLO_MODEL_REGISTRY[NUT_MODEL])
    if not os.path.isfile(path) or os.path.getsize(path) == 0:
        available = sorted(f for f in os.listdir(YOLO_MODEL_DIR) if f.endswith(".pt")) if os.path.isdir(YOLO_MODEL_DIR) else []
        raise SystemExit(
            f"[YOLO] MODEL NOT FOUND for NUT_MODEL={NUT_MODEL}: {path}\n"
            f"       Available .pt files in {YOLO_MODEL_DIR}: {available or 'none'}\n"
            "       Refusing to start (no fallback to another nut model)."
        )
    return path


def check_aae_pairing() -> None:
    """Refuse to run if the AAE server reports a different nut model than YOLO."""
    try:
        response = requests.get(f"{AAE_SERVER_URL}/health", timeout=2.0)
        info = response.json()
    except Exception as exc:  # server not up yet (manual start order) - warn, do not block
        print(f"[YOLO] WARNING: could not query AAE /health at {AAE_SERVER_URL} ({exc}); "
              f"nut-model pairing NOT verified. Ensure AAE_NUT_MODEL={NUT_MODEL}.")
        return
    aae_model = info.get("nut_model")
    if aae_model is None:
        print("[YOLO] WARNING: AAE server does not report nut_model (older server); pairing NOT verified.")
    elif aae_model != NUT_MODEL:
        raise SystemExit(
            f"[YOLO] NUT MODEL MISMATCH: YOLO NUT_MODEL={NUT_MODEL} but AAE server reports "
            f"nut_model={aae_model} ({info.get('experiment')}). Start both with the same model."
        )
    else:
        print(f"[YOLO] Nut-model pairing verified: YOLO={NUT_MODEL}, AAE={aae_model} ({info.get('experiment')}).")


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
        # Model is chosen explicitly via NUT_MODEL (see YOLO_MODEL_REGISTRY); path validated first.
        model_path = resolve_yolo_model_path()
        self.model = YOLO(model_path)
        self.model.to("cpu")
        if _NUT_MODEL_ENV is None:
            self.get_logger().warn("NUT_MODEL not set; using 'm20' (previous default behaviour).")
        train_name = (getattr(self.model, "ckpt", None) or {}).get("train_args", {}).get("name", "unknown")
        self.get_logger().info(
            f"YOLO model: nut_model={NUT_MODEL} path={model_path} task={self.model.task} "
            f"classes={self.model.names} train_run={train_name}"
        )
        if self.model.task != "segment":
            raise SystemExit(f"[YOLO] {model_path} is a '{self.model.task}' model; a segmentation model is required.")
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
        self.min_world_contour_points = MIN_WORLD_CONTOUR_POINTS
        self.aae_crop_padding_px = AAE_CROP_PADDING_PX
        self.object_yaw_symmetry = np.deg2rad(max(1.0, OBJECT_YAW_SYMMETRY_DEG))
        self.yaw_stability_threshold = np.deg2rad(max(0.5, YAW_STABILITY_THRESHOLD_DEG))
        self.yaw_offset = np.deg2rad(YAW_OFFSET_DEG)
        self.contour_world_epsilon = max(1e-4, CONTOUR_WORLD_EPSILON)
        self.missed_frames = 0
        self.yolo_conf_threshold = YOLO_CONF_THRESHOLD
        self.yolo_imgsz = YOLO_IMGSZ
        self.yolo_max_det = YOLO_MAX_DET
        self.yaw_buffer = []
        self.last_stable_yaw = None
        self.last_yaw_source = "N/A"

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
        self.yaw_buffer = []
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
                    "mask": mask,
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

    def normalize_angle(self, angle):
        return (angle + np.pi) % (2.0 * np.pi) - np.pi

    def choose_equivalent_yaw(self, base_yaw, reference_yaw):
        base_yaw = self.normalize_angle(base_yaw)
        if reference_yaw is None:
            return base_yaw

        candidates = [base_yaw + k * self.object_yaw_symmetry for k in range(-6, 7)]
        return min(
            candidates,
            key=lambda yaw: abs(self.normalize_angle(yaw - reference_yaw)),
        )

    def circular_mean_with_period(self, angles, period, weights=None):
        if len(angles) == 0:
            return None

        scaled = np.asarray(angles, dtype=np.float64) * (2.0 * np.pi / period)
        vectors = np.exp(1j * scaled)
        if weights is None:
            mean_vector = np.mean(vectors)
        else:
            weights = np.asarray(weights, dtype=np.float64)
            weight_sum = float(np.sum(weights))
            if weight_sum <= 0.0:
                return None
            mean_vector = np.sum(vectors * weights) / weight_sum

        if abs(mean_vector) < 1e-6:
            return None
        mean_angle = np.angle(mean_vector) * (period / (2.0 * np.pi))
        return self.normalize_angle(mean_angle)

    def pixel_to_world_xy(self, u, v, depth_value):
        cam_x = (u - self.K[0, 2]) * depth_value / self.K[0, 0]
        cam_y = (v - self.K[1, 2]) * depth_value / self.K[1, 1]
        cam_point = np.array([cam_x, cam_y, depth_value, 1.0], dtype=np.float32)
        world_point = self.T_wc @ cam_point
        return world_point[:2]

    def extract_world_yaw(self, rotation_matrix):
        return self.normalize_angle(np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0]))

    def build_tabletop_rotation(self, yaw):
        return R_tool.from_euler("z", yaw).as_matrix()

    def estimate_mask_world_yaw(self, mask, local_depth):
        mask_u8 = (mask.astype(np.uint8)) * 255
        inner_mask = cv2.erode(mask_u8, np.ones((3, 3), dtype=np.uint8), iterations=1)
        contour_source = inner_mask if np.count_nonzero(inner_mask) >= self.min_world_contour_points else mask_u8
        contours, _ = cv2.findContours(contour_source, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if not contours:
            return None

        contour = max(contours, key=cv2.contourArea)
        if contour is None or len(contour) < self.min_world_contour_points:
            return None

        contour_points = contour[:, 0, :]
        sample_step = max(1, len(contour_points) // 180)
        world_points = []

        for u, v in contour_points[::sample_step]:
            if not (0 <= v < local_depth.shape[0] and 0 <= u < local_depth.shape[1]):
                continue

            depth_value = float(local_depth[v, u])
            if not np.isfinite(depth_value) or depth_value <= 0.05:
                continue

            world_points.append(self.pixel_to_world_xy(float(u), float(v), depth_value))

        if len(world_points) < self.min_world_contour_points:
            return None

        world_points = np.asarray(world_points, dtype=np.float32).reshape(-1, 1, 2)
        hull = cv2.convexHull(world_points)
        if hull is None or len(hull) < 3:
            return None

        approx = cv2.approxPolyDP(hull, self.contour_world_epsilon, True)
        polygon = approx if approx is not None and len(approx) >= 3 else hull
        polygon_points = polygon[:, 0, :]
        if len(polygon_points) < 3:
            return None

        edge_vectors = np.roll(polygon_points, -1, axis=0) - polygon_points
        edge_lengths = np.linalg.norm(edge_vectors, axis=1)
        min_edge_length = max(self.contour_world_epsilon * 2.0, 0.002)
        valid_edges = edge_lengths > min_edge_length
        if np.count_nonzero(valid_edges) < 3:
            return None

        edge_angles = np.arctan2(edge_vectors[valid_edges, 1], edge_vectors[valid_edges, 0])
        edge_weights = edge_lengths[valid_edges]
        return self.circular_mean_with_period(
            edge_angles,
            self.object_yaw_symmetry,
            edge_weights,
        )

    def refine_pose_orientation(self, world_pose, candidate, local_depth, matched_track):
        raw_yaw = self.extract_world_yaw(world_pose[:3, :3])
        contour_yaw = self.estimate_mask_world_yaw(candidate["mask"], local_depth)

        base_yaw = contour_yaw if contour_yaw is not None else raw_yaw
        reference_yaw = raw_yaw
        if matched_track and self.yaw_buffer:
            reference_yaw = self.yaw_buffer[-1]

        refined_yaw = self.choose_equivalent_yaw(base_yaw + self.yaw_offset, reference_yaw)
        refined_pose = world_pose.copy()
        refined_pose[:3, :3] = self.build_tabletop_rotation(refined_yaw)
        yaw_source = "MASK" if contour_yaw is not None else "AAE"
        return refined_pose, refined_yaw, yaw_source

    def compute_stable_pose(self):
        mean_position = np.mean(np.asarray(self.pose_buffer, dtype=np.float32), axis=0)
        reference_yaw = self.yaw_buffer[-1] if self.yaw_buffer else 0.0
        normalized_yaws = np.array(
            [self.choose_equivalent_yaw(yaw, reference_yaw) for yaw in self.yaw_buffer],
            dtype=np.float32,
        )
        mean_yaw = float(np.mean(normalized_yaws))

        stable_pose = np.eye(4, dtype=np.float32)
        stable_pose[:3, 3] = mean_position
        stable_pose[:3, :3] = self.build_tabletop_rotation(mean_yaw)
        return stable_pose, mean_yaw

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
                self.yaw_buffer = []
                self.tracked_center = None
                self.status_text = "NO NUT"
            else:
                self.status_text = "TRACK LOST"
            self.push_display_frame(self.decorate_frame(viz_img))
            return

        ranked_candidates = self.rank_candidates(candidates)
        selected_candidate = None
        selected_pose = None
        selected_yaw = None
        matched_track = False

        for candidate in ranked_candidates:
            candidate_matched_track = (
                self.tracked_center is not None
                and candidate["track_distance"] <= self.track_match_distance_px
            )
            pose = self.estimate_world_pose(local_rgb, candidate)
            if pose is None:
                continue

            refined_pose, refined_yaw, yaw_source = self.refine_pose_orientation(
                pose,
                candidate,
                local_depth,
                candidate_matched_track,
            )
            selected_candidate = candidate
            selected_pose = refined_pose
            selected_yaw = refined_yaw
            matched_track = candidate_matched_track
            self.last_yaw_source = yaw_source
            break

        self.draw_candidates(viz_img, candidates, selected_candidate)

        if selected_candidate is None:
            self.missed_frames += 1
            self.status_text = "AAE FAIL"
            self.push_display_frame(self.decorate_frame(viz_img))
            return

        if self.tracked_center is None or not matched_track:
            self.pose_buffer = []
            self.yaw_buffer = []

        self.tracked_center = selected_candidate["center"]
        self.missed_frames = 0

        current_pos = selected_pose[:3, 3]
        self.pose_buffer.append(current_pos)
        self.yaw_buffer.append(selected_yaw)
        if len(self.pose_buffer) > self.buffer_size:
            self.pose_buffer.pop(0)
        if len(self.yaw_buffer) > self.buffer_size:
            self.yaw_buffer.pop(0)

        det_x, det_y = selected_candidate["center"]

        if len(self.pose_buffer) == self.buffer_size and len(self.yaw_buffer) == self.buffer_size:
            pos_std = np.std(self.pose_buffer, axis=0)
            reference_yaw = self.yaw_buffer[-1]
            normalized_yaws = np.array(
                [self.choose_equivalent_yaw(yaw, reference_yaw) for yaw in self.yaw_buffer],
                dtype=np.float32,
            )
            yaw_std = float(np.std(normalized_yaws))
            if np.all(pos_std < self.stability_threshold) and yaw_std < self.yaw_stability_threshold:
                stable_pose, stable_yaw = self.compute_stable_pose()
                self.last_stable_pose = stable_pose
                self.last_stable_yaw = stable_yaw
                self.broadcast_and_publish(stable_pose)
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
        yaw = self.extract_world_yaw(mat[:3, :3])

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
        self.get_logger().info(
            f"\033[92mStable pose published: xyz={msg.data[:3]} yaw={yaw:.3f} source={self.last_yaw_source}\033[0m"
        )

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
            if self.last_stable_yaw is not None:
                cv2.putText(
                    viz,
                    f"LAST YAW: {np.degrees(self.last_stable_yaw):.1f} deg ({self.last_yaw_source})",
                    (20, 175),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 220, 0),
                    2,
                )

        cv2.putText(
            viz,
            f"CANDIDATES: {self.visible_candidate_count}",
            (20, 210),
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
    resolve_yolo_model_path()  # fail before ROS/camera setup if the model file is missing
    check_aae_pairing()
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
