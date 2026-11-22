#!/usr/bin/env python3
"""Find the bolt with the depth camera (no training data needed).

The YOLO model only knows nuts, so the bolt is found geometrically in the depth image of
the fixed overhead camera:

  1. Back-project the depth image into 3-D points in the robot base frame.
  2. The table is the most common height in the scene (largest flat surface).
  3. The bolt TIP is a small disc (M20: ~20 mm across) 28-60 mm above the table. Nothing
     else on the table is that high and that small: nuts are 16 mm tall, the boards are flat.
  4. The SEAT is the fixture plate around the bolt: points in a ring 25-45 mm from the axis
     (outside a nut's 17.3 mm corners), between the table and the tip.

Publishes (geometry_msgs/PointStamped, frame base_link), only for a valid detection:
  /bolt_detector/tip   bolt axis x, y and tip height z
  /bolt_detector/seat  bolt axis x, y and seat (plate top) height z

A nut sitting on the bolt makes the top ~30 mm wide, which is rejected as "not a bare bolt
tip", so the robot should read the detection while the bolt is empty.
"""
import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener


class BoltDetector(Node):
    def __init__(self) -> None:
        super().__init__("bolt_detector")
        self.depth_topic = self.declare_parameter("depth_topic", "/rgbd_camera/depth_image").value
        self.info_topic = self.declare_parameter("camera_info_topic", "/rgbd_camera/camera_info").value
        self.camera_frame = self.declare_parameter("camera_frame", "camera_depth_optical_frame").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.pixel_step = int(self.declare_parameter("pixel_step", 2).value)
        # Geometry windows (metres, relative to the table / bolt axis).
        self.tip_band = self.declare_parameter("tip_band_above_table", [0.028, 0.060]).value
        self.tip_width = self.declare_parameter("tip_width_range", [0.010, 0.028]).value
        self.ring = self.declare_parameter("seat_ring_radius", [0.025, 0.045]).value
        # Gazebo Fortress publishes a wrong camera_info for this camera (fx=277, cx=160, cy=120:
        # defaults for 320x240 while the image is 1280x720). When camera_info does not match the
        # image, the intrinsics are computed from the sensor's horizontal FOV (URDF _d455.urdf.xacro).
        self.horizontal_fov = self.declare_parameter("horizontal_fov", 1.50098).value
        self.warned_camera_info = False

        self.K = None
        self.latest_depth = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(CameraInfo, self.info_topic, self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, self.depth_topic, self.on_depth, qos_profile_sensor_data)
        self.tip_pub = self.create_publisher(PointStamped, "/bolt_detector/tip", 10)
        self.seat_pub = self.create_publisher(PointStamped, "/bolt_detector/seat", 10)
        self.create_timer(1.0, self.detect)
        self.last_report = ""
        self.get_logger().info(f"Bolt detector: depth {self.depth_topic}, frame {self.camera_frame}")

    def on_info(self, msg: CameraInfo) -> None:
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def on_depth(self, msg: Image) -> None:
        if msg.encoding != "32FC1":
            return
        self.latest_depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)

    def intrinsics(self, width: int, height: int) -> np.ndarray:
        """camera_info K if it matches the image (principal point near the centre),
        otherwise a pinhole K computed from the horizontal field of view."""
        if self.K is not None and abs(self.K[0, 2] - width / 2) < 0.1 * width \
                and abs(self.K[1, 2] - height / 2) < 0.1 * height:
            return self.K
        if not self.warned_camera_info:
            self.get_logger().warn(
                f"camera_info does not match the {width}x{height} image; using K from "
                f"horizontal_fov={self.horizontal_fov:.5f} rad")
            self.warned_camera_info = True
        f = (width / 2.0) / np.tan(self.horizontal_fov / 2.0)
        return np.array([[f, 0.0, (width - 1) / 2.0], [0.0, f, (height - 1) / 2.0], [0.0, 0.0, 1.0]])

    # ------------------------------------------------------------------ steps
    def camera_to_base(self) -> np.ndarray | None:
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, self.camera_frame, Time())
        except TransformException:
            return None
        matrix = np.eye(4)
        q = t.transform.rotation
        matrix[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        matrix[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        return matrix

    def points_in_base(self, depth: np.ndarray, T: np.ndarray) -> np.ndarray:
        """Step 1: depth image -> N x 3 points in the base frame (every pixel_step-th pixel)."""
        s = self.pixel_step
        d = depth[::s, ::s]
        v, u = np.mgrid[0:depth.shape[0]:s, 0:depth.shape[1]:s]
        valid = np.isfinite(d) & (d > 0.05)
        d, u, v = d[valid], u[valid], v[valid]
        K = self.intrinsics(depth.shape[1], depth.shape[0])
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        cam = np.stack([(u - cx) * d / fx, (v - cy) * d / fy, d, np.ones_like(d)])
        return (T @ cam)[:3].T

    @staticmethod
    def table_height(points: np.ndarray) -> float:
        """Step 2: the most common height near the robot base (2 mm bins)."""
        z = points[:, 2]
        z = z[(z > -0.10) & (z < 0.10)]
        hist, edges = np.histogram(z, bins=np.arange(-0.10, 0.10, 0.002))
        best = int(np.argmax(hist))
        return float(0.5 * (edges[best] + edges[best + 1]))

    def tip_candidates(self, points: np.ndarray, table_z: float) -> list[np.ndarray]:
        """Step 3: clusters of points in the tip height band, grouped on a 4 mm XY grid."""
        band = points[(points[:, 2] > table_z + self.tip_band[0]) & (points[:, 2] < table_z + self.tip_band[1])]
        if len(band) == 0:
            return []
        cell = 0.004
        ij = np.floor(band[:, :2] / cell).astype(int)
        origin = ij.min(axis=0)
        ij -= origin
        grid = np.zeros(ij.max(axis=0) + 1, dtype=np.uint8)
        grid[ij[:, 0], ij[:, 1]] = 1
        count, labels = cv2.connectedComponents(grid, connectivity=8)
        point_labels = labels[ij[:, 0], ij[:, 1]]
        return [band[point_labels == label] for label in range(1, count)]

    def detect(self) -> None:
        if self.latest_depth is None:
            return
        T = self.camera_to_base()
        if T is None:
            return
        points = self.points_in_base(self.latest_depth, T)
        table_z = self.table_height(points)

        found = []
        for cluster in self.tip_candidates(points, table_z):
            width = float(np.max(np.ptp(cluster[:, :2], axis=0)))
            if len(cluster) < 10 or not (self.tip_width[0] <= width <= self.tip_width[1]):
                continue
            tip_z = float(np.percentile(cluster[:, 2], 90))
            # Centre from the flat top face only: seen at an angle, part of the shank's side
            # also falls in the band and would pull the centre sideways (~2 mm).
            top = cluster[cluster[:, 2] >= tip_z - 0.003]
            x, y = top[:, 0].mean(), top[:, 1].mean()
            # Step 4: the plate around the bolt gives the seat height.
            radius = np.hypot(points[:, 0] - x, points[:, 1] - y)
            ring = points[(radius > self.ring[0]) & (radius < self.ring[1]) &
                          (points[:, 2] > table_z + 0.005) & (points[:, 2] < tip_z - 0.005)]
            if len(ring) < 20:
                continue
            found.append((x, y, tip_z, float(np.median(ring[:, 2])), width))

        if len(found) != 1:
            self.report(f"{len(found)} bolt candidates (need exactly 1); table z={table_z:.4f}")
            return
        x, y, tip_z, seat_z, width = found[0]
        stamp = self.get_clock().now().to_msg()
        for publisher, z in ((self.tip_pub, tip_z), (self.seat_pub, seat_z)):
            msg = PointStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = self.base_frame
            msg.point.x, msg.point.y, msg.point.z = float(x), float(y), float(z)
            publisher.publish(msg)
        self.report(f"bolt at x={x:.4f} y={y:.4f} tip z={tip_z:.4f} seat z={seat_z:.4f} "
                    f"(tip width {width * 1000:.0f} mm, table z={table_z:.4f})")

    def report(self, text: str) -> None:
        """Log only when the result changes (rounded), so the log stays readable."""
        if text != self.last_report:
            self.get_logger().info(text)
            self.last_report = text


def main() -> None:
    rclpy.init()
    node = BoltDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
