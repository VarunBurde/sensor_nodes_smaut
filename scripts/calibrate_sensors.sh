#!/bin/bash
source ~/ros2_humble/install/setup.bash 


echo "Starting static TF publishers..."

# Run each static_transform_publisher in the background
# The '&' at the end sends the command to the background
ros2 run tf2_ros static_transform_publisher 0.1 0.0 0.5 0.0 0.0 0.0 base_link camera_link &
ros2 run tf2_ros static_transform_publisher 0.2 0.3 0.1 0.0 0.0 1.57 base_link radar_link &
ros2 run tf2_ros static_transform_publisher -0.05 0.0 0.08 0.0 0.0 0.0 base_link unilidar_imu_initial &
ros2 run tf2_ros static_transform_publisher -0.05 0.0 0.08 0.0 0.0 0.0 base_link gps_link &
ros2 run tf2_ros static_transform_publisher -0.05 0.0 0.08 0.0 0.0 0.0 base_link wheel_link &


wait

echo "All static TF publishers are running. Press Ctrl+C to stop the script and potentially the background processes."