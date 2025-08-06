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
    
    object_point_cloud_topic_arg = DeclareLaunchArgument(
        'object_point_cloud_topic',
        default_value='/object_point_cloud',
        description='Topic for publishing object point clouds'
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
    
    mesh_package_name_arg = DeclareLaunchArgument(
        'mesh_package_name',
        default_value='academy_robot_description',
        description='Name of the package containing the mesh file'
    )
    mesh_file_path_arg = DeclareLaunchArgument(
        'mesh_file_path',
        default_value='meshes/socket_cap_screw.stl',
        description='Path to the mesh file relative to the package share directory'
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
            'object_point_cloud_topic': LaunchConfiguration('object_point_cloud_topic'),
            'min_depth': LaunchConfiguration('min_depth'),
            'max_depth': LaunchConfiguration('max_depth'),
            'debug_mode': LaunchConfiguration('debug_mode'),
            'mesh_package_name': LaunchConfiguration('mesh_package_name'),
            'mesh_file_path': LaunchConfiguration('mesh_file_path'),
        }]
    )
    
    return LaunchDescription([
        depth_topic_arg,
        camera_info_topic_arg,
        object_point_cloud_topic_arg,
        min_depth_arg,
        max_depth_arg,
        debug_mode_arg,
        mesh_package_name_arg,
        mesh_file_path_arg,
        pcd_processor_node,
    ])
