#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge
import cv2
import numpy as np
import open3d as o3d
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped


class PCDProcessorNode(Node):
    """
    ROS2 node for processing depth images and converting them to point clouds using Open3D.
    Subscribes to depth images and camera info, publishes processed point clouds.
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
        self.distortion_coeffs = None
        
        # Parameters
        self.declare_parameter('depth_topic', '/robot/front_rgbd_camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/robot/front_rgbd_camera/depth/camera_info')
        self.declare_parameter('point_cloud_topic', '/processed_point_cloud')
        self.declare_parameter('min_depth', 0.1)
        self.declare_parameter('max_depth', 10.0)
        self.declare_parameter('downsample_voxel_size', 0.01)
        self.declare_parameter('remove_outliers', True)
        self.declare_parameter('outlier_nb_points', 20)
        self.declare_parameter('outlier_radius', 0.1)
        self.declare_parameter('target_frame', 'robot_base_footprint')
        
        # Get parameters
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.point_cloud_topic = self.get_parameter('point_cloud_topic').value
        self.min_depth = self.get_parameter('min_depth').value
        self.max_depth = self.get_parameter('max_depth').value
        self.downsample_voxel_size = self.get_parameter('downsample_voxel_size').value
        self.remove_outliers = self.get_parameter('remove_outliers').value
        self.outlier_nb_points = self.get_parameter('outlier_nb_points').value
        self.outlier_radius = self.get_parameter('outlier_radius').value
        self.target_frame = self.get_parameter('target_frame').value
        
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
        self.get_logger().info(f'Subscribing to camera info topic: {self.camera_info_topic}')
        self.get_logger().info(f'Publishing to point cloud topic: {self.point_cloud_topic}')
    
    def camera_info_callback(self, msg):
        """Callback for camera info messages to get camera parameters."""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.distortion_coeffs = np.array(msg.d)
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
                self.get_logger().debug('Published processed point cloud')
                
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')
    
    def process_depth_image(self, depth_image, header):
        """
        Process depth image and convert to point cloud using Open3D.
        
        Args:
            depth_image: OpenCV depth image
            header: ROS message header
            
        Returns:
            PointCloud2 message or None if processing fails
        """
        try:
            # Create Open3D depth image
            o3d_depth = o3d.geometry.Image(depth_image.astype(np.float32))
            
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
            
            # Apply coordinate transformation to match ROS2 camera convention
            # ROS2 cameras use: X forward, Y left, Z up
            # Open3D uses: X right, Y down, Z forward
            # We need to transform: (x, y, z) -> (z, -x, -y)
            points = np.asarray(pcd.points)
            if len(points) > 0:
                # Transform coordinates to match ROS2 convention
                transformed_points = np.column_stack([
                    points[:, 2],   # Z becomes X (forward)
                    -points[:, 0],  # -X becomes Y (left)
                    -points[:, 1]   # -Y becomes Z (up)
                ])
                
                # Apply depth filtering in the transformed coordinate system
                valid_mask = (transformed_points[:, 0] >= self.min_depth) & (transformed_points[:, 0] <= self.max_depth)
                filtered_points = transformed_points[valid_mask]
                
                if len(filtered_points) == 0:
                    self.get_logger().warn('No valid points after depth filtering')
                    return None
                
                # Create new point cloud with filtered points
                filtered_pcd = o3d.geometry.PointCloud()
                filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)
                
                # Downsample point cloud
                if self.downsample_voxel_size > 0:
                    filtered_pcd = filtered_pcd.voxel_down_sample(self.downsample_voxel_size)
                
                # Remove outliers
                if self.remove_outliers:
                    filtered_pcd, _ = filtered_pcd.remove_statistical_outlier(
                        nb_neighbors=self.outlier_nb_points,
                        std_ratio=self.outlier_radius
                    )
                
                # Convert to ROS PointCloud2
                points = np.asarray(filtered_pcd.points)
                
                # Transform point cloud to target frame if different from source frame
                if header.frame_id != self.target_frame:
                    try:
                        # Get transform from source frame to target frame
                        transform = self.tf_buffer.lookup_transform(
                            self.target_frame,
                            header.frame_id,
                            header.stamp,
                            timeout=rclpy.duration.Duration(seconds=0.1)
                        )
                        
                        # Transform points to target frame
                        transformed_points = self.transform_points(points, transform)
                        
                        # Create header with target frame
                        cloud_header = Header()
                        cloud_header.stamp = header.stamp
                        cloud_header.frame_id = self.target_frame
                        
                        # Create point cloud message with transformed points
                        cloud_msg = pc2.create_cloud_xyz32(cloud_header, transformed_points)
                        
                        return cloud_msg
                        
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                        self.get_logger().warn(f'TF transform failed: {str(e)}')
                        # Fall back to original frame
                        cloud_header = Header()
                        cloud_header.stamp = header.stamp
                        cloud_header.frame_id = header.frame_id
                        cloud_msg = pc2.create_cloud_xyz32(cloud_header, points)
                        return cloud_msg
                else:
                    # No transformation needed
                    cloud_header = Header()
                    cloud_header.stamp = header.stamp
                    cloud_header.frame_id = header.frame_id
                    cloud_msg = pc2.create_cloud_xyz32(cloud_header, points)
                    return cloud_msg
            else:
                self.get_logger().warn('No points generated from depth image')
                return None
                
        except Exception as e:
            self.get_logger().error(f'Error in depth processing: {str(e)}')
            return None
    
    def transform_points(self, points, transform):
        """
        Transform points using TF2 transform.
        
        Args:
            points: numpy array of points (N, 3)
            transform: TransformStamped message
            
        Returns:
            Transformed points as numpy array (N, 3)
        """
        # Extract rotation and translation from transform
        rotation = transform.transform.rotation
        translation = transform.transform.translation
        
        # Convert quaternion to rotation matrix
        q = np.array([rotation.x, rotation.y, rotation.z, rotation.w])
        rotation_matrix = self.quaternion_to_rotation_matrix(q)
        
        # Apply transformation to each point
        transformed_points = []
        for point in points:
            # Apply rotation and translation
            rotated_point = rotation_matrix @ point
            transformed_point = rotated_point + np.array([translation.x, translation.y, translation.z])
            transformed_points.append(transformed_point)
        
        return np.array(transformed_points)
    
    def quaternion_to_rotation_matrix(self, q):
        """
        Convert quaternion to rotation matrix.
        
        Args:
            q: numpy array [x, y, z, w]
            
        Returns:
            3x3 rotation matrix
        """
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