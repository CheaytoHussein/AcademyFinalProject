#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge
import numpy as np
import math
import open3d as o3d
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2



class PCDProcessorNode(Node):
    """
    ROS2 node for processing depth images and converting them to point clouds using Open3D.
    """
    
    def __init__(self):
        super().__init__('pcd_processor_node')
        
        # Initialize CV bridge for image conversion
        self.bridge = CvBridge()
        
        # Camera parameters (will be updated from camera_info)
        self.width = None
        self.height = None
        self.fx = None
        self.fy = None
        
        # Parameters
        self.declare_parameter('depth_topic', '/front_rgbd_camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/front_rgbd_camera/depth/camera_info')
        self.declare_parameter('point_cloud_topic', '/processed_point_cloud')
        self.declare_parameter('min_depth', 0.1)
        self.declare_parameter('max_depth', 10.0)
        self.declare_parameter('debug_mode', False)
        self.declare_parameter('horizontal_fov_deg', 85.2)
        self.declare_parameter('vertical_fov_deg', 48.0)
        
        # Get parameters
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.point_cloud_topic = self.get_parameter('point_cloud_topic').value
        self.min_depth = self.get_parameter('min_depth').value
        self.max_depth = self.get_parameter('max_depth').value
        self.debug_mode = self.get_parameter('debug_mode').value
        self.horizontal_fov_deg = self.get_parameter('horizontal_fov_deg').value
        self.vertical_fov_deg = self.get_parameter('vertical_fov_deg').value
        
        # Subscribers
        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10
        )
        
        # Publishers
        self.point_cloud_pub = self.create_publisher(
            PointCloud2,
            self.point_cloud_topic,
            10
        )
        
        self.get_logger().info(f'PCD Processor Node initialized')
        self.get_logger().info(f'Subscribing to depth topic: {self.depth_topic}')
        self.get_logger().info(f'Publishing to point cloud topic: {self.point_cloud_topic}')
    
    def camera_info_callback(self, msg):
        """Callback for camera info messages to get camera parameters."""
        if self.fx is None:
            self.get_logger().info('Camera info received. Calculating intrinsics from parameters.')
            self.width = msg.width
            self.height = msg.height

            # Convert FOV from degrees to radians
            H_FOV_rad = math.radians(self.horizontal_fov_deg)
            V_FOV_rad = math.radians(self.vertical_fov_deg)

            # Calculate focal lengths from FOV and image size
            self.fx = self.width / (2 * math.tan(H_FOV_rad / 2))
            self.fy = self.height / (2 * math.tan(V_FOV_rad / 2))
            self.cx = self.width / 2
            self.cy = self.height / 2
            
            self.get_logger().info(f'Using calculated intrinsics: fx={self.fx:.2f}, fy={self.fy:.2f}, cx={self.cx:.2f}, cy={self.cy:.2f}')
    
    def depth_callback(self, msg):
        """Callback for depth image messages."""
        if self.fx is None:
            self.get_logger().warn('Camera parameters not yet received, skipping depth processing')
            return
        
        try:
            # Convert ROS Image to OpenCV format
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # Process depth image and create point cloud
            point_cloud = self.process_depth_image(depth_image, msg.header)
            
            if point_cloud is not None:
                # Publish the processed point cloud
                self.point_cloud_pub.publish(point_cloud)
                
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')
    
    def process_depth_image(self, depth_image, header):
        """
        Process depth image and convert to point cloud using Open3D.
        """
        try:
            # Filter out invalid depth values
            depth_image_clean = depth_image.copy()
            depth_image_clean[np.isinf(depth_image_clean)] = np.nan
            depth_image_clean[depth_image_clean < self.min_depth] = np.nan
            depth_image_clean[depth_image_clean > self.max_depth] = np.nan
            
            # Check if we have any valid depth values
            valid_mask = ~np.isnan(depth_image_clean)
            if not np.any(valid_mask):
                if self.debug_mode:
                    self.get_logger().warn('No valid depth values found after filtering')
                return None
            
            # Create Open3D depth image
            o3d_depth = o3d.geometry.Image(depth_image_clean.astype(np.float32))
            
            # Create the intrinsic matrix for Open3D
            intrinsic = o3d.camera.PinholeCameraIntrinsic()
            intrinsic.set_intrinsics(
                self.width,                   # width
                self.height,                  # height
                self.fx,                      # fx
                self.fy,                      # fy
                self.cx,                      # cx
                self.cy                       # cy
            )
            
            # Convert depth image to point cloud
            pcd = o3d.geometry.PointCloud.create_from_depth_image(o3d_depth, intrinsic)
            points = np.asarray(pcd.points)
            
            if len(points) == 0:
                if self.debug_mode:
                    self.get_logger().warn('No points generated from depth image')
                return None
            
            # Keep point cloud in the same frame as the depth image
            cloud_header = Header()
            cloud_header.stamp = header.stamp
            cloud_header.frame_id = header.frame_id
            cloud_msg = pc2.create_cloud_xyz32(cloud_header, points)
            
            return cloud_msg
                
        except Exception as e:
            self.get_logger().error(f'Error in depth processing: {str(e)}')
            return None


def main(args=None):
    rclpy.init(args=args)
    node = PCDProcessorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
