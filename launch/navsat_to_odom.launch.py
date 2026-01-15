import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lio_sam',
            executable='lio_sam_navsat_to_odom',
            name='navsat_to_odom',
            output='screen',
            parameters=[{
                'input_topic': 'sensing/gnss/robins/ros/gps_nav_sat_fix_null',
                'output_topic': 'sensing/gnss/robins/ros/gps_odom',
                'frame_id': 'odom',
                'child_frame_id': 'base_link',
                'use_first_fix_as_origin': True
            }]
        )
    ])
