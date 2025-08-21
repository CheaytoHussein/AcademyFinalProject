import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge
import numpy as np
import struct
from academy_robot_pointcloud_interfaces.srv import GetPointCloud


class DepthToPointCloudServer(Node):

    def __init__(self):
        super().__init__('depth_to_pointcloud_server')
        self.bridge = CvBridge()
        self.latest_cloud = None
        self.camera_info = None
        depth_topic = self.declare_parameter(
            'depth_topic',
            '/camera/depth/image_raw'
        )
        info_topic = self.declare_parameter('camera_info_topic', '/camera/depth/camera_info')
        self.depth_sub = self.create_subscription(Image, depth_topic, self.depth_cb, 10)
        self.info_sub = self.create_subscription(CameraInfo, info_topic, self.info_cb, 10)
        self.srv = self.create_service(GetPointCloud, 'get_pointcloud', self.handle_get_cloud)

    def info_cb(self, msg: CameraInfo):
        self.camera_info = msg

    def depth_cb(self, img_msg: Image):
        if self.camera_info is None:
            return
        # Convert depth image to numpy
        depth = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough').astype(np.float32)
        h, w = depth.shape
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]
        # Back-project to 3D (meters). Mask invalid/zero depths.
        ys, xs = np.indices((h, w))
        z = depth
        valid = z > 0.0
        x = (xs - cx) * z / fx
        y = (ys - cy) * z / fy
        xyz = np.stack((x[valid], y[valid], z[valid]), axis=-1)
        # Create PointCloud2
        self.latest_cloud = self.xyz_to_pointcloud2(xyz, img_msg.header.frame_id)

    def xyz_to_pointcloud2(self, xyz, frame_id):
        # Build PointCloud2 (xyz only)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        # Pack data
        buffer = b''.join([struct.pack('fff', *pt) for pt in xyz])
        cloud = PointCloud2()
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.header.frame_id = frame_id
        cloud.height = 1
        cloud.width = xyz.shape[0]
        cloud.is_bigendian = False
        loud.is_dense = True
        cloud.fields = fields
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.data = buffer
        return cloud

    def handle_get_cloud(self, request, response):
        if self.latest_cloud is None:
            self.get_logger().warn("No pointcloud available yet.")
            response.cloud = PointCloud2()
        else:
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
