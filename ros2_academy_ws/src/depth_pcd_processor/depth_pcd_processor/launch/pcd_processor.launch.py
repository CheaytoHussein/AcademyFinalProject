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
        default_value='/front_rgbd_camera/depth/image_raw',
        description='Topic for depth images'
    )
    
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/front_rgbd_camera/depth/camera_info',
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
    
    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='false',
        description='Enable debug logging for point cloud processing'
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
            'debug_mode': LaunchConfiguration('debug_mode'),
        }]
    )
    
    return LaunchDescription([
        depth_topic_arg,
        camera_info_topic_arg,
        point_cloud_topic_arg,
        min_depth_arg,
        max_depth_arg,
        debug_mode_arg,
        pcd_processor_node,
    ]) 