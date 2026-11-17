import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('underwater_gazebo')
    world_path = os.path.join(pkg_dir, 'worlds', 'underwater_hide_and_seek.sdf')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_path],
            output='screen'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
                '/seeker/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            ],
            output='screen'
        ),
    ])
