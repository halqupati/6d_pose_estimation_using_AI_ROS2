import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
import threading, cv2, numpy as np, requests
from cv_bridge import CvBridge
from ultralytics import YOLO
from scipy.spatial.transform import Rotation as R_tool

# Global Buffers
bridge = CvBridge()
rgb_image = None
depth_image = None
trigger_signal = True

class CameraSystem(Node):
    def __init__(self):
        super().__init__('camera_system_sub')
        self.create_subscription(Image, '/rgbd_camera/image', self.rgb_cb, 10)
        self.create_subscription(Image, '/rgbd_camera/depth_image', self.depth_cb, 10)
        self.create_subscription(Bool, '/trigger', self.trigger_cb, 10)
  
    def rgb_cb(self, msg):
        global rgb_image
        try:
            # Gazebo publishes rgb8, convert to bgr8 for OpenCV
            rgb_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            self.get_logger().info("RGB image received!", once=True)
        except Exception as e:
            self.get_logger().error(f"RGB conversion error: {e}")

    def depth_cb(self, msg):
        global depth_image
        try:
            # GZ Sim often sends 32FC1 (float32)
            depth_image = bridge.imgmsg_to_cv2(msg, 'passthrough')
            self.get_logger().info("Depth image received!", once=True)
        except Exception as e:
            self.get_logger().error(f"Depth conversion error: {e}")

    def trigger_cb(self, msg):
        global trigger_signal
        trigger_signal = msg.data
        self.get_logger().info(f"Trigger State: {trigger_signal}")


class NutEstimatorAAE(Node):
    def __init__(self):
        super().__init__('aae_estimator_node')
        # Load YOLO on CPU to prevent GPU OOM errors seen in your screenshots
        self.model = YOLO("/home/hussain/6d_pose_estimation/nut_pose_pipeline/yolov12_seg_infer/model/best.pt")
        self.model.to('cpu')
        self.get_logger().info("YOLO model loaded on CPU")        
        # Camera Intrinsics
        self.K = np.array([[615.0, 0, 320.0], [0, 615.0, 240.0], [0, 0, 1]], dtype=np.float32)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.create_timer(0.1, self.process_frame)
        self.get_logger().info("6D Pose Estimator Started")

    def transform_to_matrix(self, t):
        m = np.eye(4)
        m[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        q = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
        m[:3, :3] = R_tool.from_quat(q).as_matrix()
        return m

    def compose_mat(self, R, t):
        m = np.eye(4)
        m[:3, :3], m[:3, 3] = R, t
        return m

    def process_frame(self):
        global rgb_image, depth_image, trigger_signal
        if rgb_image is None or depth_image is None or not trigger_signal:
            return
        
        display_img = rgb_image.copy()

        # STEP 1: Update Camera Transform EVERY frame (Crucial for moving robots)
        try:
            t_msg = self.tf_buffer.lookup_transform('world', 'camera_link_optical', rclpy.time.Time())
            T_world_camera = self.transform_to_matrix(t_msg)
        except Exception as e:
            return

        results = self.model(rgb_image, conf=0.6, verbose=False, device='cpu')
        for r in results:
            if r.masks is not None:
                mask = r.masks.data[0].cpu().numpy()
                x1, y1, x2, y2 = map(int, r.boxes.xyxy[0].cpu().numpy())
                crop = rgb_image[y1:y2, x1:x2]
                
                try:
                    # STEP 2: Get Rotation from AAE
                    _, enc = cv2.imencode('.jpg', crop)
                    resp = requests.post('http://localhost:5001/get_rotation', files={"image": enc.tobytes()}, timeout=2.0)
                    R_mat = np.array(resp.json()['R'])
                    
                    # STEP 3: Calculate Position in Camera Frame
                    z_m = np.median(depth_image[mask > 0])
                    if np.isnan(z_m) or z_m <= 0: continue
                    # Gazebo Check: ensure meters
                    if z_m > 50: z_m /= 1000.0 
                    
                    cx, cy = r.boxes.xywh[0][0].cpu().item(), r.boxes.xywh[0][1].cpu().item()
                    tx = (cx - self.K[0, 2]) * z_m / self.K[0, 0]
                    ty = (cy - self.K[1, 2]) * z_m / self.K[1, 1]
                    T_vec = np.array([tx, ty, z_m])

                    # STEP 4: Compose World Matrix
                    T_camera_nut = self.compose_mat(R_mat, T_vec)
                    T_world_nut = T_world_camera @ T_camera_nut

                    # STEP 5: SHOW POSE MATRIX IN TERMINAL
                    self.print_pose_info(T_world_nut)

                    # STEP 6: Broadcast to RViz
                    self.broadcast_pose(T_world_nut)

                except Exception as e:
                    self.get_logger().error(f"Inference Error: {e}")

        cv2.imshow("6D Pose Visualization", display_img)
        cv2.waitKey(1)

    def print_pose_info(self, mat):
        pos = mat[:3, 3]
        euler = R_tool.from_matrix(mat[:3, :3]).as_euler('xyz', degrees=True)
        print("\n" + "="*40)
        print(f"NUT POSE (WORLD FRAME):")
        print(f"Position (XYZ): {pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}")
        print(f"Euler (RPY deg): {euler[0]:.1f}, {euler[1]:.1f}, {euler[2]:.1f}")
        print("-" * 40)
        print("Transformation Matrix:")
        print(mat)
        print("="*40 + "\n")

    def broadcast_pose(self, mat):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'detected_nut'
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = mat[:3, 3]
        q = R_tool.from_matrix(mat[:3, :3]).as_quat()
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        self.tf_broadcaster.send_transform(t)

if __name__ == '__main__':
    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor()
    nodes = [CameraSystem(), NutEstimatorAAE()]
    for n in nodes: executor.add_node(n)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        while rclpy.ok(): pass
    finally: rclpy.shutdown()
