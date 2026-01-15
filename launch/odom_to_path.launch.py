import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lio_sam',
            executable='lio_sam_odom_to_path',
            name='odom_to_path',
            output='screen',
            parameters=[{
                'odom_topic': '/sensing/gnss/robins/ros/gps_odom',
                'path_topic': '/gps_path',
                'frame_id': 'map',
                'max_path_size': 10000,
                'min_distance': 0.5  # 0.5 metre minimum mesafe
            }]
        )
    ])
