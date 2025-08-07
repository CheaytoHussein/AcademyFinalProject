#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for depth to pointcloud node."""
    
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
        default_value='/point_cloud',
        description='Topic for publishing point clouds'
    )
    

    
    horizontal_fov_deg_arg = DeclareLaunchArgument(
        'horizontal_fov_deg',
        default_value='85.2',
        description='Horizontal field of view in degrees'
    )
    
    vertical_fov_deg_arg = DeclareLaunchArgument(
        'vertical_fov_deg',
        default_value='48.0',
        description='Vertical field of view in degrees'
    )
    
    # Depth to PointCloud Node
    depth_to_pointcloud_node = Node(
        package='depth_to_pointcloud',
        executable='depth_to_pointcloud_node',
        name='depth_to_pointcloud_node',
        output='screen',
        parameters=[{
            'depth_topic': LaunchConfiguration('depth_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'point_cloud_topic': LaunchConfiguration('point_cloud_topic'),
            'horizontal_fov_deg': LaunchConfiguration('horizontal_fov_deg'),
            'vertical_fov_deg': LaunchConfiguration('vertical_fov_deg'),
        }]
    )
    
    return LaunchDescription([
        depth_topic_arg,
        camera_info_topic_arg,
        point_cloud_topic_arg,
        horizontal_fov_deg_arg,
        vertical_fov_deg_arg,
        depth_to_pointcloud_node,
    ])