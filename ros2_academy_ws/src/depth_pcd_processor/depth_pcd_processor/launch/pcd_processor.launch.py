#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for PCD processor node."""
    
    # Launch arguments
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/robot/front_rgbd_camera/depth/image_raw',
        description='Topic for depth images'
    )
    
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/robot/front_rgbd_camera/depth/camera_info',
        description='Topic for camera info'
    )
    
    point_cloud_topic_arg = DeclareLaunchArgument(
        'point_cloud_topic',
        default_value='/processed_point_cloud',
        description='Topic for publishing processed point clouds'
    )
    
    min_depth_arg = DeclareLaunchArgument(
        'min_depth',
        default_value='0.1',
        description='Minimum depth value in meters'
    )
    
    max_depth_arg = DeclareLaunchArgument(
        'max_depth',
        default_value='10.0',
        description='Maximum depth value in meters'
    )
    
    downsample_voxel_size_arg = DeclareLaunchArgument(
        'downsample_voxel_size',
        default_value='0.01',
        description='Voxel size for downsampling (0 to disable)'
    )
    
    remove_outliers_arg = DeclareLaunchArgument(
        'remove_outliers',
        default_value='true',
        description='Whether to remove statistical outliers'
    )
    
    outlier_nb_points_arg = DeclareLaunchArgument(
        'outlier_nb_points',
        default_value='20',
        description='Number of neighbors for outlier removal'
    )
    
    outlier_radius_arg = DeclareLaunchArgument(
        'outlier_radius',
        default_value='0.1',
        description='Standard deviation ratio for outlier removal'
    )
    
    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='false',
        description='Enable debug logging for point cloud processing'
    )
    
    target_frame_arg = DeclareLaunchArgument(
        'target_frame',
        default_value='robot_base_footprint',
        description='Target frame for point cloud transformation'
    )
    
    transform_mode_arg = DeclareLaunchArgument(
        'transform_mode',
        default_value='1',
        description='Transform mode: 1=standard, 2=no_transform, 3=alternative'
    )
    
    # PCD Processor Node
    pcd_processor_node = Node(
        package='depth_pcd_processor',
        executable='pcd_processor_node',
        name='pcd_processor_node',
        output='screen',
        parameters=[{
            'depth_topic': LaunchConfiguration('depth_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'point_cloud_topic': LaunchConfiguration('point_cloud_topic'),
            'min_depth': LaunchConfiguration('min_depth'),
            'max_depth': LaunchConfiguration('max_depth'),
            'downsample_voxel_size': LaunchConfiguration('downsample_voxel_size'),
            'remove_outliers': LaunchConfiguration('remove_outliers'),
            'outlier_nb_points': LaunchConfiguration('outlier_nb_points'),
            'outlier_radius': LaunchConfiguration('outlier_radius'),
            'target_frame': LaunchConfiguration('target_frame'),
            'debug_mode': LaunchConfiguration('debug_mode'),
            'transform_mode': LaunchConfiguration('transform_mode'),
        }]
    )
    
    return LaunchDescription([
        depth_topic_arg,
        camera_info_topic_arg,
        point_cloud_topic_arg,
        min_depth_arg,
        max_depth_arg,
        downsample_voxel_size_arg,
        remove_outliers_arg,
        outlier_nb_points_arg,
        outlier_radius_arg,
        debug_mode_arg,
        target_frame_arg,
        transform_mode_arg,
        pcd_processor_node,
    ]) 