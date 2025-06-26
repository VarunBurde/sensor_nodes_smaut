#!/bin/bash

echo "Starting the combined automation script..."
echo "Console output from background processes is suppressed."
echo "Press Ctrl+C to terminate all launched processes."

# --- Trap to catch Ctrl+C (SIGINT) and terminate all background processes ---
# Initialize an array to store PIDs of all background processes
declare -a ALL_PIDS

# Function to kill all processes in ALL_PIDS
cleanup_on_exit() {
    echo -e "\nCtrl+C detected. Terminating all background processes..."
    for pid in "${ALL_PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then # Check if process exists
            kill "$pid"
            echo "Terminated PID: $pid"
        fi
    done
    # Kill the SSH process explicitly if it's still running
    if [[ -n "$SSH_PID" ]] && kill -0 "$SSH_PID" 2>/dev/null; then
        echo "Terminating SSH process (PID: $SSH_PID)..."
        kill "$SSH_PID"
    fi
    # Also kill any potential lingering `ros2` processes (e.g., launch files)
    # This is a bit more aggressive and might kill other ros2 processes, use with caution if you have others running
    # pkill -f "ros2 launch"
    # pkill -f "ros2 run"
    exit 0
}

# Set the trap: When SIGINT is received, call cleanup_on_exit function
trap cleanup_on_exit SIGINT

# --- 1. Run SMAUT Robot Vision remotely ---
echo "Executing remote smaut_robot_vision command..."
# Redirect remote output to /dev/null
ssh root@192.168.232.50 "cd smaut_robot_vision/smaut_robot_vision && ./target/aarch64-unknown-linux-gnu/release/smaut_robot_vision config.toml > /dev/null 2>&1" &
SSH_PID=$! # Store the PID of the SSH process
ALL_PIDS+=($SSH_PID) # Add to the list of PIDs to manage
echo "Remote smaut_robot_vision process started in the background (PID: $SSH_PID)."
sleep 5 # Give the remote process a moment to initialize

# --- 2. Run Python Publishers ---
echo "Setting up environment for Python publishers..."
source ~/ros2_humble/install/setup.bash
source /home/ciirc/varun_ws/venv/bin/activate
echo "Running Python publishers..."
python /home/ciirc/varun_ws/src/scripts/rust_publisher/run_all_publishers.py > /dev/null 2>&1 &
PYTHON_PUBLISHERS_PID=$!
ALL_PIDS+=($PYTHON_PUBLISHERS_PID)
echo "Python publishers started in the background (PID: $PYTHON_PUBLISHERS_PID)."

# --- 3. Run Static Transform Publishers ---
echo "Setting up environment for static transform publishers..."
source ~/ros2_humble/install/setup.bash

echo "Starting static TF publishers..."

# Define an array to hold PIDs of static transform publishers (these will also be added to ALL_PIDS)
declare -a STATIC_TF_PIDS

# Run each static_transform_publisher in the background, suppressing output
ros2 run tf2_ros static_transform_publisher 0.0 0.0 0 0 0 0 radar_link radar_frame > /dev/null 2>&1 &
STATIC_TF_PIDS+=($!)
ros2 run tf2_ros static_transform_publisher 0.0 0.0 0 0 0 0 gps_link gps_frame > /dev/null 2>&1 &
STATIC_TF_PIDS+=($!)
ros2 run tf2_ros static_transform_publisher 0.0 0.0 0 0 0 0 wheel_link wheel_frame > /dev/null 2>&1 &
STATIC_TF_PIDS+=($!)
ros2 run tf2_ros static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 base_link camera_link > /dev/null 2>&1 &
STATIC_TF_PIDS+=($!)
ros2 run tf2_ros static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 base_link radar_link > /dev/null 2>&1 &
STATIC_TF_PIDS+=($!)
ros2 run tf2_ros static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 base_link unilidar_imu_initial > /dev/null 2>&1 &
STATIC_TF_PIDS+=($!)
ros2 run tf2_ros static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 base_link gps_link > /dev/null 2>&1 &
STATIC_TF_PIDS+=($!)
ros2 run tf2_ros static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 base_link wheel_link > /dev/null 2>&1 &
STATIC_TF_PIDS+=($!)

ALL_PIDS+=("${STATIC_TF_PIDS[@]}") # Add all static TF PIDs to the master list
echo "Static transform publishers started in the background (PIDs: ${STATIC_TF_PIDS[*]})."

# --- 4. Launch RealSense Camera ---
echo "Setting up environment for RealSense camera..."
source ~/ros2_humble/install/setup.bash
source /home/ciirc/sensor_nodes_ws/install/setup.bash
echo "Launching RealSense camera..."
ros2 launch realsense2_camera rs_launch.py \
    rgb_camera.color_profile:="848x480x30" \
    depth_module.depth_profile:="848x480x30" \
    align_depth.enable:=true \
    pointcloud.enable:=true \
    enable_accel:=true \
    enable_gyro:=true \
    unite_imu_method:=1 > /dev/null 2>&1 &
REALSENSE_PID=$!
ALL_PIDS+=($REALSENSE_PID)
echo "RealSense camera launched in the background (PID: $REALSENSE_PID)."

# --- 5. Launch Unitree Lidar ---
echo "Setting up environment for Unitree Lidar..."
source ~/ros2_humble/install/setup.bash
source /home/ciirc/unilidar_sdk2/unitree_lidar_ros2/install/setup.bash
echo "Launching Unitree Lidar..."
ros2 launch unitree_lidar_ros2 launch_without_rviz.py > /dev/null 2>&1 &
LIDAR_PID=$!
ALL_PIDS+=($LIDAR_PID)
echo "Unitree Lidar launched in the background (PID: $LIDAR_PID)."

# # --- 6. Launch point_lio ---
# echo "Setting up environment for point_lio..."
# source ~/ros2_humble/install/setup.bash
# source ~/catkin_point_lio_unilidar/install/setup.bash
# echo "Launching point_lio..."
# ros2 launch point_lio correct_odom_unilidar_l2.launch.py > /dev/null 2>&1 &
# POINT_LIO_PID=$!
# ALL_PIDS+=($POINT_LIO_PID)
# echo "point_lio launched in the background (PID: $POINT_LIO_PID)."

# # --- 7. Launch rtabmap ---
# echo "Setting up environment for rtabmap..."
# source ~/ros2_humble/install/setup.bash
# source ~/anirudh_ws/install/setup.bash
# echo "Launching rtabmap..."
# ros2 launch rtabmap_launch rtabmap.launch.py \
#     args:="--delete_db_on_start" \
#     depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
#     rgb_topic:=/camera/camera/color/image_raw \
#     camera_info_topic:=/camera/camera/color/camera_info \
#     approx_sync:=true \
#     frame_id:=base_link \
#     odom_frame_id:=rtab_odom \
#     publish_tf_odom:=false \
#     localization:=true \
#     rtabmap_viz:=false \
#     odom_topic:=/rtabmap/odometry_raw \
#     map_frame_id:=map_rtab > /dev/null 2>&1 &
# RTABMAP_PID=$!
# ALL_PIDS+=($RTABMAP_PID)
# echo "rtabmap launched in the background (PID: $RTABMAP_PID)."

echo "All components have been initiated in the background."
echo "The script will now keep running to maintain the trap, do NOT close the terminal."
echo "To terminate all processes, press Ctrl+C in this terminal."

# Keep the script running to allow the trap to catch Ctrl+C
# A simple `read` or `wait` without arguments can achieve this.
# Using `tail -f /dev/null` is another common trick to keep the script alive.
wait -n # Wait for any child process to exit. This is better than `read` if you want to react to a child dying.
# Or, if you simply want to keep the script alive indefinitely until Ctrl+C:
while true; do sleep 1; done