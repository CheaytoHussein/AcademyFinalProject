from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    exe = LaunchConfiguration('exe')
    pointcloud_service = LaunchConfiguration('pointcloud_service')
    mesh_path = LaunchConfiguration('mesh_path')

    return LaunchDescription([
        DeclareLaunchArgument('exe', default_value='pose_estimator', description='Executable name in academy_robot_pose_estimator'),
        DeclareLaunchArgument('pointcloud_service', default_value='/depth_to_pointcloud/get_latest'),
        DeclareLaunchArgument('mesh_path', default_value='ros2_academy_ws/src/academy_robot_simulation/academy_robot_gazebo_ignition/meshes/socket_cap_screw.stl'),
        Node(
            package='academy_robot_pose_estimator',
            executable=exe,
            name=exe,
            output='screen',
            parameters=[{
                'pointcloud_service': pointcloud_service,
                'mesh_path': mesh_path
            }]
        )
    ])
