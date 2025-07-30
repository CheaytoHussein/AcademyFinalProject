#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge
import numpy as np
import open3d as o3d
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros


class PCDProcessorNode(Node):
    """
    ROS2 node for processing depth images and converting them to point clouds using Open3D.
    """
    
    def __init__(self):
        super().__init__('pcd_processor_node')
        
        # Initialize CV bridge for image conversion
        self.bridge = CvBridge()
        
        # Initialize TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Camera parameters (will be updated from camera_info)
        self.camera_matrix = None
        
        # Parameters
        self.declare_parameter('depth_topic', '/robot/front_rgbd_camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/robot/front_rgbd_camera/depth/camera_info')
        self.declare_parameter('point_cloud_topic', '/processed_point_cloud')
        self.declare_parameter('min_depth', 0.1)
        self.declare_parameter('max_depth', 10.0)
        self.declare_parameter('target_frame', 'robot_base_footprint')
        self.declare_parameter('debug_mode', False)
        
        # Get parameters
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.point_cloud_topic = self.get_parameter('point_cloud_topic').value
        self.min_depth = self.get_parameter('min_depth').value
        self.max_depth = self.get_parameter('max_depth').value
        self.target_frame = self.get_parameter('target_frame').value
        self.debug_mode = self.get_parameter('debug_mode').value
        
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
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.get_logger().info('Camera parameters received')
    
    def depth_callback(self, msg):
        """Callback for depth image messages."""
        if self.camera_matrix is None:
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
                if self.debug_mode:
                    self.get_logger().info('Published processed point cloud')
                
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
            
            # Create intrinsic matrix for Open3D
            intrinsic = o3d.camera.PinholeCameraIntrinsic()
            intrinsic.set_intrinsics(
                depth_image.shape[1],  # width
                depth_image.shape[0],  # height
                self.camera_matrix[0, 0],  # fx
                self.camera_matrix[1, 1],  # fy
                self.camera_matrix[0, 2],  # cx
                self.camera_matrix[1, 2]   # cy
            )
            
            # Convert depth image to point cloud
            pcd = o3d.geometry.PointCloud.create_from_depth_image(o3d_depth, intrinsic)
            points = np.asarray(pcd.points)
            
            if len(points) == 0:
                if self.debug_mode:
                    self.get_logger().warn('No points generated from depth image')
                return None
            
            # Determine optical frame ID
            if 'depth_frame' in header.frame_id:
                optical_frame_id = header.frame_id.replace('depth_frame', 'depth_optical_frame')
            elif 'depth_optical_frame' in header.frame_id:
                optical_frame_id = header.frame_id
            else:
                optical_frame_id = header.frame_id
            
            # Transform to target frame if needed
            if optical_frame_id != self.target_frame:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.target_frame,
                        optical_frame_id,
                        header.stamp,
                        timeout=rclpy.duration.Duration(seconds=0.1)
                    )
                    
                    # Apply transformation
                    transformed_points = self.transform_points(points, transform)
                    
                    # Create point cloud message
                    cloud_header = Header()
                    cloud_header.stamp = header.stamp
                    cloud_header.frame_id = self.target_frame
                    cloud_msg = pc2.create_cloud_xyz32(cloud_header, transformed_points)
                    
                    return cloud_msg
                    
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    if self.debug_mode:
                        self.get_logger().warn(f'TF transform failed: {str(e)}')
                    # Fall back to optical frame
                    cloud_header = Header()
                    cloud_header.stamp = header.stamp
                    cloud_header.frame_id = optical_frame_id
                    cloud_msg = pc2.create_cloud_xyz32(cloud_header, points)
                    return cloud_msg
            else:
                # No transformation needed
                cloud_header = Header()
                cloud_header.stamp = header.stamp
                cloud_header.frame_id = optical_frame_id
                cloud_msg = pc2.create_cloud_xyz32(cloud_header, points)
                return cloud_msg
                
        except Exception as e:
            self.get_logger().error(f'Error in depth processing: {str(e)}')
            return None
    
    def transform_points(self, points, transform):
        """Transform points using TF2 transform."""
        # Extract rotation and translation
        rotation = transform.transform.rotation
        translation = transform.transform.translation
        
        # Convert quaternion to rotation matrix
        q = np.array([rotation.x, rotation.y, rotation.z, rotation.w])
        rotation_matrix = self.quaternion_to_rotation_matrix(q)
        
        # Apply transformation to each point
        transformed_points = []
        for point in points:
            rotated_point = rotation_matrix @ point
            transformed_point = rotated_point + np.array([translation.x, translation.y, translation.z])
            transformed_points.append(transformed_point)
        
        return np.array(transformed_points)
    
    def quaternion_to_rotation_matrix(self, q):
        """Convert quaternion to rotation matrix."""
        x, y, z, w = q
        
        # Normalize quaternion
        norm = np.sqrt(x*x + y*y + z*z + w*w)
        x, y, z, w = x/norm, y/norm, z/norm, w/norm
        
        # Convert to rotation matrix
        R = np.array([
            [1 - 2*y*y - 2*z*z, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
            [2*x*y + 2*w*z, 1 - 2*x*x - 2*z*z, 2*y*z - 2*w*x],
            [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x*x - 2*y*y]
        ])
        
        return R


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