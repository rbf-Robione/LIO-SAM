import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node


def generate_launch_description():

    share_dir = get_package_share_directory('lio_sam')
    parameter_file = LaunchConfiguration('params_file')
    # Defaults to false (live sensors). When playing back a bag with
    # `ros2 bag play --clock`, set this to true so every LIO-SAM node and
    # rviz follow /clock; otherwise TF will be wall-time-stamped while
    # pointclouds carry bag-recording stamps and rviz's message filter
    # discards everything ("queue is full" / "frame too old").
    use_sim_time = LaunchConfiguration('use_sim_time')
    xacro_path = os.path.join(share_dir, 'config', 'robot.urdf.xacro')
    rviz_config_file = os.path.join(share_dir, 'config', 'rviz2.rviz')

    # Sensor profile selects which <sensor>_params.yaml under config/ to use
    # (OT128 or XT32). The explicit `params_file:=` form still wins because
    # it's checked first in the resolved path below.
    sensor_profile = LaunchConfiguration('sensor')

    sensor_declare = DeclareLaunchArgument(
        'sensor',   
        default_value='OT128',
        description='Sensor profile name. Picks <sensor>_params.yaml from the lio_sam config directory unless params_file is overridden.')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            share_dir, 'config', [sensor_profile, '_params.yaml']
        ]),
        description='Path to the LIO-SAM ROS2 parameters file. Defaults to <sensor>_params.yaml; override with `params_file:=<path>` for a fully custom file.')

    use_sim_time_declare = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use /clock instead of wall time. Set to true when feeding LIO-SAM from `ros2 bag play --clock`.')

    print("urdf_file_name : {}".format(xacro_path))

    sim_time_param = {'use_sim_time': use_sim_time}

    return LaunchDescription([
        sensor_declare,
        params_declare,
        use_sim_time_declare,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments='0.0 0.0 0.0 0.0 0.0 0.0 map odom'.split(' '),
            parameters=[parameter_file, sim_time_param],
            output='screen'
            ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro', ' ', xacro_path])
            }, sim_time_param]
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_imuPreintegration',
            name='lio_sam_imuPreintegration',
            parameters=[parameter_file, sim_time_param],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_imageProjection',
            name='lio_sam_imageProjection',
            parameters=[parameter_file, sim_time_param],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_featureExtraction',
            name='lio_sam_featureExtraction',
            parameters=[parameter_file, sim_time_param],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_mapOptimization',
            name='lio_sam_mapOptimization',
            parameters=[parameter_file, sim_time_param],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[sim_time_param],
            output='screen'
        )
    ])
