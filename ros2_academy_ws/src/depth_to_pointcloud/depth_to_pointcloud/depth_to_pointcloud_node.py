#!/usr/bin/env python3

import math
import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo, PointCloud2

from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as pc2


class DepthToPointCloudNode(Node):
    """
    ROS2 node for converting depth images to point clouds.
    """

    def __init__(self):
        super().__init__('depth_to_pointcloud_node')

        self.bridge = CvBridge()

        self.width = None
        self.height = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        
        self.declare_parameter('depth_topic', '/front_rgbd_camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/front_rgbd_camera/depth/camera_info')
        self.declare_parameter('point_cloud_topic', '/point_cloud')

        self.declare_parameter('horizontal_fov_deg', 85.2)
        self.declare_parameter('vertical_fov_deg', 48.0)

        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.point_cloud_topic = self.get_parameter('point_cloud_topic').value

        self.horizontal_fov_deg = self.get_parameter('horizontal_fov_deg').value
        self.vertical_fov_deg = self.get_parameter('vertical_fov_deg').value

        self.depth_sub = self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 10)

        self.pcd_pub = self.create_publisher(PointCloud2, self.point_cloud_topic, 10)
        
        self.get_logger().info('Depth to PointCloud Node initialized')

    def camera_info_callback(self, msg):
        if self.fx is None:
            self.get_logger().info('Camera info received. Calculating intrinsics from parameters.')
            self.width = msg.width
            self.height = msg.height
            H_FOV_rad = math.radians(self.horizontal_fov_deg)
            V_FOV_rad = math.radians(self.vertical_fov_deg)
            self.fx = self.width / (2 * math.tan(H_FOV_rad / 2))
            self.fy = self.height / (2 * math.tan(V_FOV_rad / 2))
            self.cx = self.width / 2
            self.cy = self.height / 2
            self.get_logger().info(f'Using calculated intrinsics: fx={self.fx:.2f}, fy={self.fy:.2f}, cx={self.cx:.2f}, cy={self.cy:.2f}')

    def depth_callback(self, msg):
        if self.fx is None:
            self.get_logger().warn('Camera parameters not yet received, skipping depth processing')
            return

        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            pcd_msg = self.process_depth_image(depth_image, msg.header)
            
            if pcd_msg is not None:
                self.pcd_pub.publish(pcd_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')

    def process_depth_image(self, depth_image, header):
        try:
            # Clean depth image by removing invalid values
            depth_image_clean = np.copy(depth_image)
            depth_image_clean[np.isinf(depth_image_clean)] = np.nan

            valid_mask = ~np.isnan(depth_image_clean)
            if not np.any(valid_mask):
                self.get_logger().warn('No valid depth values found')
                return None

            # Create Open3D point cloud from depth image
            o3d_depth = o3d.geometry.Image(depth_image_clean.astype(np.float32))
            intrinsic = o3d.camera.PinholeCameraIntrinsic(self.width, self.height, self.fx, self.fy, self.cx, self.cy)
            pcd = o3d.geometry.PointCloud.create_from_depth_image(o3d_depth, intrinsic)

            # Convert to ROS PointCloud2 message
            points = np.asarray(pcd.points)
            cloud_header = Header()
            cloud_header.stamp = header.stamp
            cloud_header.frame_id = header.frame_id
            raw_pcd_msg = pc2.create_cloud_xyz32(cloud_header, points)

            return raw_pcd_msg

        except Exception as e:
            self.get_logger().error(f'Error in depth processing: {str(e)}')
            return None


def main(args=None):
    rclpy.init(args=args)
    node = DepthToPointCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()