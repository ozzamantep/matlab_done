from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='underwater_perception',
            executable='sonar_target_tracker',
            name='sonar_target_tracker',
            output='screen',
        ),
        Node(
            package='underwater_perception',
            executable='camera_target_detector.py',
            name='camera_target_detector',
            output='screen',
        ),
        Node(
            package='underwater_navigation',
            executable='auv_controller',
            name='auv_controller',
            output='screen',
        ),
        Node(
            package='underwater_navigation',
            executable='hider_evasion_node',
            name='hider_evasion_node',
            output='screen',
        ),
    ])

