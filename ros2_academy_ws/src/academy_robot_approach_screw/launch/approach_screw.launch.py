from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='academy_robot_approach_screw',
            executable='approach_screw_server',
            name='approach_screw_server',
            parameters=[{
                'scan_topic': '/scan',
                'cmd_vel_topic': '/cmd_vel',
                'window_deg': 20.0,
                'stop_distance': 0.35,
                'max_forward_speed': 0.15,
                'speed_kp': 0.8,
            }],
            output='screen'
        )
    ])
