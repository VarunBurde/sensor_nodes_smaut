# Radar Sensor Package

A ROS2 package for publishing radar sensor data and robot telemetry information.

## Features

- **Radar Point Cloud Publisher**: Publishes radar sensor data as PointCloud2 messages using proper `radar_link` frame
- **Robot Telemetry Publisher**: Publishes GPS, pose, diagnostics, and other robot data
- **Proper TF Frame Setup**: Uses ROS-compliant frame naming conventions

## Frame Structure

```
map → base_link → radar_link
             → gps_link
```

## Installation

1. **Install Python dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

2. **Build the package:**
   ```bash
   cd /home/ciirc/varun_ws
   colcon build --packages-select radar_sensor_pkg
   source install/setup.bash
   ```

## Usage

### Launch all nodes with proper transforms:
```bash
ros2 launch radar_sensor_pkg radar_sensor_launch.py
```

### Run individual nodes:

**Radar Point Cloud Publisher:**
```bash
ros2 run radar_sensor_pkg radar_pointcloud_publisher
```

**Robot Telemetry Publisher:**
```bash
ros2 run radar_sensor_pkg robot_telemetry_publisher
```

### Set up transforms manually (if not using launch file):
```bash
# Base frame
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link

# Radar sensor (adjust position as needed)
ros2 run tf2_ros static_transform_publisher 0 0 0.5 0 0 0 base_link radar_link

# GPS sensor (adjust position as needed)  
ros2 run tf2_ros static_transform_publisher 0 0 0.3 0 0 0 base_link gps_link
```

## Topics Published

### Radar Data:
- `/radar/pointcloud` (sensor_msgs/PointCloud2)

### Robot Data:
- `/robot/gps/fix` (sensor_msgs/NavSatFix)
- `/robot/pose` (geometry_msgs/PoseWithCovarianceStamped)
- `/robot/emergency_active` (std_msgs/Bool)
- `/robot/autonomous_state` (std_msgs/Bool)
- `/robot/wheel_angle` (std_msgs/Float32)
- `/robot/gps/rtk_fix` (std_msgs/Int32)
- `/robot/diagnostics` (diagnostic_msgs/DiagnosticArray)

## Configuration

Edit `config/radar_config.yaml` to modify:
- ZMQ endpoints
- Sensor count expectations
- Collection timeouts
- Frame IDs
- Topic names

## Dependencies

- rclpy
- sensor_msgs
- std_msgs
- geometry_msgs
- tf2_ros
- diagnostic_msgs
- pyzmq
- msgpack
- numpy
