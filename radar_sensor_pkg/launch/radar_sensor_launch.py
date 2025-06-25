#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    return LaunchDescription([
        # Static transform publishers for proper frame setup
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                 '0', '0', '0', '0', '0', '0', 'map', 'base_link'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                 '0', '0', '0.5', '0', '0', '0', 'base_link', 'radar_link'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                 '0', '0', '0.3', '0', '0', '0', 'base_link', 'gps_link'],
            output='screen'
        ),
        
        # Radar point cloud publisher
        Node(
            package='radar_sensor_pkg',
            executable='radar_pointcloud_publisher',
            name='radar_pointcloud_publisher',
            output='screen',
            parameters=[
                {'expected_sensor_count': 8},
                {'collection_timeout': 0.5}
            ]
        ),
        
        # Robot telemetry publisher
        Node(
            package='radar_sensor_pkg',
            executable='robot_telemetry_publisher',
            name='robot_telemetry_publisher',
            output='screen'
        ),
    ])
