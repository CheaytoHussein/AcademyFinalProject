#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for complete simulation with PCD processing."""
    
    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='demo.sdf.world',
        description='World file to load'
    )
    
    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='academy_robot',
        description='Robot name'
    )
    
    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='academy_robot_plus',
        description='Robot model'
    )
    
    has_arm_arg = DeclareLaunchArgument(
        'has_arm',
        default_value='true',
        description='Whether the robot has an arm'
    )
    
    # PCD Processor parameters
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
    
    # Include world launch
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('academy_robot_gazebo_ignition'),
                'launch',
                'spawn_world.launch.py'
            ])
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world')
        }.items()
    )
    
    # Include robot launch
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('academy_robot_gazebo_ignition'),
                'launch',
                'spawn_robot.launch.py'
            ])
        ]),
        launch_arguments={
            'robot': LaunchConfiguration('robot'),
            'robot_model': LaunchConfiguration('robot_model'),
            'has_arm': LaunchConfiguration('has_arm')
        }.items()
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
        # Arguments
        world_arg,
        robot_arg,
        robot_model_arg,
        has_arm_arg,
        depth_topic_arg,
        camera_info_topic_arg,
        point_cloud_topic_arg,
        horizontal_fov_deg_arg,
        vertical_fov_deg_arg,
        
        # Launch components
        world_launch,
        robot_launch,
        depth_to_pointcloud_node,
    ])