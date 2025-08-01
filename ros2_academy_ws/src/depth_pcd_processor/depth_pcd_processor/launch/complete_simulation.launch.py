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
        # Arguments
        world_arg,
        robot_arg,
        robot_model_arg,
        has_arm_arg,
        depth_topic_arg,
        camera_info_topic_arg,
        point_cloud_topic_arg,
        min_depth_arg,
        max_depth_arg,
        debug_mode_arg,
        
        # Launch components
        world_launch,
        robot_launch,
        pcd_processor_node,
    ]) 