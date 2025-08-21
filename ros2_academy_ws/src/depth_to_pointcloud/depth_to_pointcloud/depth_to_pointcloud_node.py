import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge
import numpy as np
from academy_robot_pointcloud_interfaces.srv import GetPointCloud

class DepthToPointCloudServer(Node):
    def __init__(self):
        super().__init__('depth_to_pointcloud_server')
        self.bridge = CvBridge()
        self.latest_cloud = None
        self.camera_info = None

        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/depth/camera_info')
        self.declare_parameter('output_cloud', '/camera/depth/points')
        self.declare_parameter('stride', 2)          # decimation stride
        self.declare_parameter('cap_points', 80000)  # hard cap

        depth_topic = self.get_parameter('depth_topic').value
        info_topic  = self.get_parameter('camera_info_topic').value
        out_topic   = self.get_parameter('output_cloud').value
        self.stride = int(self.get_parameter('stride').value)
        self.capN   = int(self.get_parameter('cap_points').value)

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.depth_sub = self.create_subscription(Image, depth_topic, self.depth_cb, sensor_qos)
        self.info_sub  = self.create_subscription(CameraInfo, info_topic, self.info_cb, 10)
        self.cloud_pub = self.create_publisher(PointCloud2, out_topic, 10)
        self.srv       = self.create_service(GetPointCloud, '/get_latest_pointcloud', self.handle_get_cloud)

        self.get_logger().info(
            f"Listening:\n  depth: {depth_topic}\n  info:  {info_topic}\nPublishing cloud: {out_topic}\nService: /get_latest_pointcloud"
        )

    def info_cb(self, msg: CameraInfo):
        self.camera_info = msg

    def depth_cb(self, img_msg: Image):
        if self.camera_info is None:
            return

        depth = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
        enc = img_msg.encoding
        if enc in ('16UC1', 'mono16'):
            z = depth.astype(np.float32) * 0.001  # mm -> m
        elif enc in ('32FC1',):
            z = depth.astype(np.float32)          # meters
        else:
            self.get_logger().warn_once(f"Unknown depth encoding '{enc}', assuming meters")
            z = depth.astype(np.float32)

        h, w = z.shape
        fx = float(self.camera_info.k[0]); fy = float(self.camera_info.k[4])
        cx = float(self.camera_info.k[2]); cy = float(self.camera_info.k[5])
        if fx == 0.0 or fy == 0.0:
            self.get_logger().warn_once("Invalid intrinsics (fx/fy == 0). Skipping frame.")
            return

        # Decimate first (cheap)
        s = max(1, self.stride)
        z = z[::s, ::s]
        h2, w2 = z.shape
        ys, xs = np.indices((h2, w2), dtype=np.float32)
        xs = xs * s
        ys = ys * s

        valid = np.isfinite(z) & (z > 0.0) & (z < 10.0)
        if not np.any(valid):
            return

        zv = z[valid]
        xv = (xs[valid] - cx) * zv / fx
        yv = (ys[valid] - cy) * zv / fy
        xyz = np.stack((xv, yv, zv), axis=-1).astype(np.float32)

        # Hard cap
        N = xyz.shape[0]
        cap = max(1000, self.capN)
        if N > cap:
            sel = np.random.choice(N, cap, replace=False)
            xyz = xyz[sel]

        self.latest_cloud = self.xyz_to_pointcloud2(xyz, img_msg.header.frame_id)
        self.cloud_pub.publish(self.latest_cloud)

    def xyz_to_pointcloud2(self, xyz, frame_id):
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud = PointCloud2()
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.header.frame_id = frame_id
        cloud.height = 1
        cloud.width = int(xyz.shape[0])
        cloud.is_bigendian = False
        cloud.is_dense = True
        cloud.fields = fields
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.data = xyz.tobytes(order='C')
        return cloud

    def handle_get_cloud(self, request, response):
        if self.latest_cloud is None:
            self.get_logger().warn("No pointcloud available yet.")
            response.cloud = PointCloud2()
        else:
            pts = self.latest_cloud.width * self.latest_cloud.height
            kb  = len(self.latest_cloud.data) / 1024.0
            self.get_logger().info(f"Serving cloud: {pts} pts, {kb:.1f} KiB")
            response.cloud = self.latest_cloud
        return response

def main():
    rclpy.init()
    node = DepthToPointCloudServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
