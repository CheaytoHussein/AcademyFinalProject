#!/usr/bin/env python3

import os
import math
import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo, PointCloud2

from cv_bridge import CvBridge

import sensor_msgs_py.point_cloud2 as pc2
from scipy.spatial.transform import Rotation as R

from ament_index_python.packages import get_package_share_directory


class PCDProcessorNode(Node):
    """
    ROS2 node for processing depth images, filtering ground and walls, and performing PCD registration.
    """

    def __init__(self):
        super().__init__('pcd_processor_node')

        self.bridge = CvBridge()

        self.width = None
        self.height = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        
        self.declare_parameter('depth_topic', '/front_rgbd_camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/front_rgbd_camera/depth/camera_info')
        self.declare_parameter('object_pose_topic', '/object_pose')
        self.declare_parameter('object_point_cloud_topic', '/object_point_cloud')
        self.declare_parameter('min_depth', 0.1)
        self.declare_parameter('max_depth', 3.0)  # Crop points beyond 3 meters
        self.declare_parameter('debug_mode', False)
        self.declare_parameter('horizontal_fov_deg', 85.2)
        self.declare_parameter('vertical_fov_deg', 48.0)
        self.declare_parameter('mesh_package_name', 'academy_robot_description')
        self.declare_parameter('mesh_file_path', 'meshes/socket_cap_screw.stl')
        # Simple plane removal - remove large planar surfaces only
        self.declare_parameter('plane_distance_threshold', 0.05)
        self.declare_parameter('min_plane_points', 5000)  # Only remove planes with many points
        self.declare_parameter('max_plane_iterations', 3)  # Conservative iteration limit
        
        # Clustering for object isolation
        self.declare_parameter('dbscan_eps', 0.02)
        self.declare_parameter('dbscan_min_points', 10)
        self.declare_parameter('min_object_points', 50)

        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.object_pose_topic = self.get_parameter('object_pose_topic').value
        self.object_point_cloud_topic = self.get_parameter('object_point_cloud_topic').value
        self.min_depth = self.get_parameter('min_depth').value
        self.max_depth = self.get_parameter('max_depth').value
        self.debug_mode = self.get_parameter('debug_mode').value
        self.horizontal_fov_deg = self.get_parameter('horizontal_fov_deg').value
        self.vertical_fov_deg = self.get_parameter('vertical_fov_deg').value
        self.mesh_package_name = self.get_parameter('mesh_package_name').value
        self.mesh_file_path = self.get_parameter('mesh_file_path').value
        self.plane_distance_threshold = self.get_parameter('plane_distance_threshold').value
        self.min_plane_points = self.get_parameter('min_plane_points').value
        self.max_plane_iterations = self.get_parameter('max_plane_iterations').value
        self.dbscan_eps = self.get_parameter('dbscan_eps').value
        self.dbscan_min_points = self.get_parameter('dbscan_min_points').value
        self.min_object_points = self.get_parameter('min_object_points').value

        self.depth_sub = self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 10)

        self.object_pose_pub = self.create_publisher(PoseStamped, self.object_pose_topic, 10)
        self.object_pcd_pub = self.create_publisher(PointCloud2, self.object_point_cloud_topic, 10)
        # Debug publishers for pipeline stages
        self.object_pcd_stage1_pub = self.create_publisher(PointCloud2, '/object_pcd_stage1_raw', 10)
        self.object_pcd_stage2_pub = self.create_publisher(PointCloud2, '/object_pcd_stage2_filtered', 10)
        self.object_pcd_stage3_pub = self.create_publisher(PointCloud2, '/object_pcd_stage3_clustered', 10)

        self.target_pcd = self.load_mesh_as_pcd(self.mesh_package_name, self.mesh_file_path)
        
        self.get_logger().info('PCD Processor Node initialized')

    def load_mesh_as_pcd(self, mesh_package, mesh_file):
        try:
            # Use ament_index_python to resolve the package share directory
            package_share = get_package_share_directory(mesh_package)
            mesh_file_abs = os.path.join(package_share, mesh_file)
            self.get_logger().info(f"Trying to load mesh from: {mesh_file_abs}")
            mesh = o3d.io.read_triangle_mesh(mesh_file_abs)
            if not mesh.has_vertices():
                self.get_logger().error(f"Failed to load mesh file or mesh is empty: {mesh_file_abs}")
                return None
            pcd = mesh.sample_points_uniformly(number_of_points=1000)
            return pcd
        except Exception as e:
            self.get_logger().error(f"Failed to load mesh file: {e}")
            return None

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
            object_pose, object_pcd_msg = self.process_depth_image(depth_image, msg.header)
            
            if object_pose is not None:
                self.object_pose_pub.publish(object_pose)

            if object_pcd_msg is not None:
                self.object_pcd_pub.publish(object_pcd_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')

    def process_depth_image(self, depth_image, header):
        try:
            depth_image_clean = np.copy(depth_image)
            depth_image_clean[np.isinf(depth_image_clean)] = np.nan
            depth_image_clean[depth_image_clean < self.min_depth] = np.nan
            depth_image_clean[depth_image_clean > self.max_depth] = np.nan

            valid_mask = ~np.isnan(depth_image_clean)
            if not np.any(valid_mask):
                if self.debug_mode:
                    self.get_logger().warn('No valid depth values found after filtering')
                return None, None

            o3d_depth = o3d.geometry.Image(depth_image_clean.astype(np.float32))
            intrinsic = o3d.camera.PinholeCameraIntrinsic(self.width, self.height, self.fx, self.fy, self.cx, self.cy)
            pcd = o3d.geometry.PointCloud.create_from_depth_image(o3d_depth, intrinsic)

            # Crop with Open3D: keep only points in a box in front of the camera, above the floor
            bbox = o3d.geometry.AxisAlignedBoundingBox(
                min_bound=[-0.5, -1, 0.5],  # x, y, z
                max_bound=[0.5, 0.2, 2.0]
            )
            pcd = pcd.crop(bbox)
            if len(pcd.points) == 0:
                self.get_logger().warn('No points remain after Open3D cropping')
                return None, None

            # Debug: Publish and log raw point cloud
            if self.debug_mode:
                points_stage1 = np.asarray(pcd.points)
                self.get_logger().info(f'Stage 1 (raw) point count: {len(points_stage1)}')
                cloud_header = Header()
                cloud_header.stamp = header.stamp
                cloud_header.frame_id = header.frame_id
                msg_stage1 = pc2.create_cloud_xyz32(cloud_header, points_stage1)
                self.object_pcd_stage1_pub.publish(msg_stage1)

            # --- REMOVE PLANE FILTERING ---
            # Directly cluster the cropped point cloud
            labels = np.array(pcd.cluster_dbscan(eps=self.dbscan_eps, min_points=self.dbscan_min_points, print_progress=False))
            unique_labels = set(labels)
            if self.debug_mode:
                self.get_logger().info(f'Found {len(unique_labels)-1} clusters after cropping')

            # Keep all clusters above min_object_points, merge them
            merged_indices = []
            for label in unique_labels:
                if label == -1:
                    continue  # Skip noise
                cluster_indices = np.where(labels == label)[0]
                if len(cluster_indices) >= self.min_object_points:
                    merged_indices.extend(cluster_indices)
            if not merged_indices:
                self.get_logger().warn('No clusters above minimum size after clustering')
                return None, None
            merged_pcd = pcd.select_by_index(merged_indices)

            # Debug: Publish and log merged object cloud
            if self.debug_mode:
                points_stage2 = np.asarray(merged_pcd.points)
                self.get_logger().info(f'Stage 2 (merged clusters) point count: {len(points_stage2)}')
                cloud_header = Header()
                cloud_header.stamp = header.stamp
                cloud_header.frame_id = header.frame_id
                msg_stage2 = pc2.create_cloud_xyz32(cloud_header, points_stage2)
                self.object_pcd_stage2_pub.publish(msg_stage2)

            # Two-stage registration for robustness
            source_pcd = merged_pcd.voxel_down_sample(voxel_size=0.01)
            target_pcd = self.target_pcd
            
            if len(source_pcd.points) < 10 or not target_pcd.has_points():
                self.get_logger().warn('Insufficient points for registration')
                return None, None
            
            # Estimate normals for both point clouds
            source_pcd.estimate_normals()
            target_pcd.estimate_normals()
            
            # Step 1: Global Registration (RANSAC with FPFH features)
            if self.debug_mode:
                self.get_logger().info('Starting global registration (RANSAC)...')
            
            source_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
                source_pcd, o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=100))
            target_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
                target_pcd, o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=100))
            
            distance_threshold = 0.1
            result_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
                source_pcd, target_pcd, source_fpfh, target_fpfh, True,
                distance_threshold,
                o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 3,
                [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                 o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)],
                o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
            
            if self.debug_mode:
                self.get_logger().info(f'Global registration fitness: {result_ransac.fitness:.4f}')
            
            # Step 2: Local Registration (ICP) using global result as initial guess
            if self.debug_mode:
                self.get_logger().info('Starting local registration (ICP)...')
            
            result_icp = o3d.pipelines.registration.registration_icp(
                source_pcd, target_pcd, 0.05, result_ransac.transformation,
                o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
            
            best_fitness = result_icp.fitness
            best_transformation = result_icp.transformation
            best_pcd = source_pcd
            
            if self.debug_mode:
                self.get_logger().info(f'Final ICP fitness: {best_fitness:.4f}')
                
            # Check if registration was successful
            if best_fitness < 0.1:  # Low fitness indicates poor alignment
                self.get_logger().warn(f'Poor registration fitness: {best_fitness:.4f}')
                # Still publish the result, but warn user
            pose_msg = self.create_pose_stamped_msg(best_transformation, header)
            points = np.asarray(best_pcd.points)
            cloud_header = Header()
            cloud_header.stamp = header.stamp
            cloud_header.frame_id = header.frame_id
            object_pcd_msg = pc2.create_cloud_xyz32(cloud_header, points)
            if self.debug_mode:
                self.object_pcd_stage3_pub.publish(object_pcd_msg)
            return pose_msg, object_pcd_msg

        except Exception as e:
            self.get_logger().error(f'Error in depth processing: {str(e)}')
            return None, None

    def create_pose_stamped_msg(self, transformation, header):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = header.stamp
        pose_msg.header.frame_id = header.frame_id

        translation = transformation[0:3, 3]
        rotation_matrix = transformation[0:3, 0:3]
        
        r = R.from_matrix(rotation_matrix)
        quat = r.as_quat()

        pose_msg.pose.position.x = translation[0]
        pose_msg.pose.position.y = translation[1]
        pose_msg.pose.position.z = translation[2]
        
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        return pose_msg

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
