source ~/ros2_humble/install/setup.bash 
source ~/anirudh_ws/install/setup.bash

ros2 launch rtabmap_launch rtabmap.launch.py \
    args:="--delete_db_on_start" \
    depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
    rgb_topic:=/camera/camera/color/image_raw \
    camera_info_topic:=/camera/camera/color/camera_info \
    approx_sync:=true \
    frame_id:=base_link \
    odom_frame_id:=rtab_odom \
    localization:=true \
    rtabmap_viz:=false


