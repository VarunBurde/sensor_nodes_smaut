#!/bin/bash
source ~/ros2_humble/install/setup.bash 

ros2 run tf2_ros static_transform_publisher 0.0 0.0 0 0 0 0 radar_link radar_frame &
ros2 run tf2_ros static_transform_publisher 0.0 0.0 0 0 0 0 gps_link gps_frame &
ros2 run tf2_ros static_transform_publisher 0.0 0.0 0 0 0 0 wheel_link wheel_frame &

wait