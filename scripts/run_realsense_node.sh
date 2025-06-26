source ~/ros2_humble/install/setup.bash 
source /home/ciirc/sensor_nodes_ws/install/setup.bash

ros2 launch realsense2_camera rs_launch.py \
    rgb_camera.color_profile:="848x480x30" \
    depth_module.depth_profile:="848x480x30" \
    align_depth.enable:=true \
    pointcloud.enable:=true \
    enable_accel:=true \
    enable_gyro:=true \
    unite_imu_method:=1