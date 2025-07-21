#!/bin/bash

echo "Starting the combined automation script..."

# --- 1. Run SMAUT Robot Vision remotely ---
#echo "Executing remote smaut_robot_vision command..."
#ssh root@192.168.232.50 "cd smaut_robot_vision/smaut_robot_vision && ./target/aarch64-unknown-linux-gnu/release/smaut_robot_vision config.toml" &
#SSH_PID=$! # Store the PID of the SSH process
#echo "Remote smaut_robot_vision process started in the background (PID: $SSH_PID)."
#sleep 5 # Give the remote process a moment to initialize

# --- 2. Run Python Publishers ---
echo "Setting up environment for Python publishers..."
source ~/ros2_humble/install/setup.bash
source /home/ciirc/varun_ws/venv/bin/activate
echo "Running Python publishers..."
python /home/ciirc/varun_ws/src/scripts/rust_publisher/run_all_publishers.py &
PYTHON_PUBLISHERS_PID=$!
echo "Python publishers started in the background (PID: $PYTHON_PUBLISHERS_PID)."

# # --- 3. Run Static Transform Publisher ---
# echo "Setting up environment for static transform publisher..."
# source ~/ros2_humble/install/setup.bash
# echo "Running static transform publisher..."
# ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map radar_frame &
# STATIC_TF_PID=$!
# echo "Static transform publisher started in the background (PID: $STATIC_TF_PID)."

# --- 4. Launch RealSense Camera ---
echo "Setting up environment for RealSense camera..."
source ~/ros2_humble/install/setup.bash
source /home/ciirc/sensor_nodes_ws/install/setup.bash
echo "Launching RealSense camera..."
ros2 launch realsense2_camera rs_launch.py \
    rgb_camera.color_profile:="848x480x30" \
    depth_module.depth_profile:="848x480x30" \
    align_depth.enable:=true &
REALSENSE_PID=$!
echo "RealSense camera launched in the background (PID: $REALSENSE_PID)."

# --- 5. Launch Unitree Lidar ---
echo "Setting up environment for Unitree Lidar..."
source ~/ros2_humble/install/setup.bash
source /home/ciirc/unilidar_sdk2/unitree_lidar_ros2/install/setup.bash
echo "Launching Unitree Lidar..."
ros2 launch unitree_lidar_ros2 launch_without_rviz.py &
LIDAR_PID=$!
echo "Unitree Lidar launched in the background (PID: $LIDAR_PID)."

echo "All components have been initiated in the background."
echo "You can monitor their output in separate terminals or log files."
echo "To terminate all these processes, you may need to kill them manually (e.g., using 'kill <PID>')."
echo "This script will now exit, but the background processes will continue."

# If you want the script to wait until all background processes complete,
# uncomment the following line. However, for long-running ROS nodes,
# this will make the script wait indefinitely.
# wait $SSH_PID $PYTHON_PUBLISHERS_PID $STATIC_TF_PID $REALSENSE_PID $LIDAR_PID