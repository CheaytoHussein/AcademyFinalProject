from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='academy_robot_pick_place',
            executable='pick_place_orchestrator',
            name='pick_place_orchestrator',
            output='screen',
            parameters=[{
                'stop_distance': 0.35,
                'max_forward_speed': 0.15,
                'approach_timeout': 30.0,
                'place_frame': 'base_link',
                'place_x': 0.4,
                'place_y': -0.2,
                'place_z': 0.3,
            }]
        )
    ])
