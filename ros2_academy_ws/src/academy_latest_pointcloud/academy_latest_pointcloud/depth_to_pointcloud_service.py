import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from builtin_interfaces.msg import Time
from academy_robot_pointcloud_interfaces.srv import GetPointCloud
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np

class DepthToPointCloudService(Node):
    def __init__(self):
        super().__init__('depth_to_pointcloud_service')

        self.declare_parameter('depth_topic', '/front_rgbd_camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/front_rgbd_camera/depth/camera_info')
        self.declare_parameter('frame_id', '')      # '' => keep incoming header.frame_id
        self.declare_parameter('qos_depth', 10)
        self.declare_parameter('z_min', 0.1)
        self.declare_parameter('z_max', 0.0)        # 0 => no max clamp

        depth_topic = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        qos_depth = int(self.get_parameter('qos_depth').value)
        self.override_frame_id: str = self.get_parameter('frame_id').value
        self.z_min = float(self.get_parameter('z_min').value)
        self.z_max = float(self.get_parameter('z_max').value)

        # Sensor-style QoS usually matches camera publishers
        qos = QoSProfile(
            depth=qos_depth,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        self.sub_depth = self.create_subscription(Image, depth_topic, self.on_depth, qos)
        self.sub_info = self.create_subscription(CameraInfo, camera_info_topic, self.on_info, qos)

        self.latest_cloud: PointCloud2 | None = None
        self.latest_stamp: Time = Time()
        self.camera_info: CameraInfo | None = None

        # Precomputed pixel grid
        self._u = None
        self._v = None

        self.srv = self.create_service(GetPointCloud, 'get_latest_pointcloud', self.handle_get_cloud)
        self.get_logger().info('Ready: service /get_latest_pointcloud')

    def on_info(self, msg: CameraInfo):
        self.camera_info = msg
        h, w = msg.height, msg.width
        xs = np.arange(w, dtype=np.float32)
        ys = np.arange(h, dtype=np.float32)
        self._u, self._v = np.meshgrid(xs, ys)

    def on_depth(self, msg: Image):
        if self.camera_info is None or self._u is None:
            return

        if msg.encoding == '32FC1':
            z = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        elif msg.encoding == '16UC1':
            depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            z = depth.astype(np.float32) / 1000.0
        else:
            self.get_logger().warn(f'Unsupported depth encoding: {msg.encoding}')
            return

        valid = np.isfinite(z)
        if self.z_min > 0:
            valid &= (z >= self.z_min)
        if self.z_max > 0:
            valid &= (z <= self.z_max)
        if not np.any(valid):
            return

        K = np.array(self.camera_info.k, dtype=np.float32).reshape(3, 3)
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]

        zv = z[valid]
        xv = (self._u[valid] - cx) * zv / fx
        yv = (self._v[valid] - cy) * zv / fy
        points = np.column_stack((xv, yv, zv)).astype(np.float32)

        header = msg.header  # keep stamp from the depth message
        if self.override_frame_id:
            header.frame_id = self.override_frame_id

        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        cloud = pc2.create_cloud(header, fields, points)
        # Optionally refresh stamp to "now":
        # cloud.header.stamp = self.get_clock().now().to_msg()

        self.latest_stamp = cloud.header.stamp
        self.latest_cloud = cloud

    def handle_get_cloud(self, request, response):
        if self.latest_cloud is None:
            response.success = False
            response.cloud = PointCloud2()
        else:
            response.success = True
            response.cloud = self.latest_cloud
        return response

def main():
    rclpy.init()
    node = DepthToPointCloudService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
