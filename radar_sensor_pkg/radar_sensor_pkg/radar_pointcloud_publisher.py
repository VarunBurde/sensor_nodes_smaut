#!/usr/bin/env python3
"""
Radar PointCloud Publisher - Publishes radar sensor data as ROS2 PointCloud2 messages
"""

from dataclasses import dataclass
from typing import List
import zmq
import msgpack
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct
import numpy as np

@dataclass
class SensorDataCart:
    timestamp: float
    id_board: int
    id_sensor: int
    id_deployment: int
    x: float
    y: float
    z: float
    velocity: float
    cov_mat_xx: float
    cov_mat_yy: float
    cov_mat_zz: float
    cov_mat_vv: float
    snr: float


def decode_sensor_data(bundle: bytes) -> List[SensorDataCart]:
    """Decode radar sensor data from MessagePack"""
    raw_list = msgpack.unpackb(bundle, raw=False)
    return [SensorDataCart(**item) for item in raw_list]

class RadarPointCloudPublisher(Node):
    def __init__(self):
        super().__init__('radar_pointcloud_publisher')
        
        # Create publisher for PointCloud2 messages
        self.publisher_ = self.create_publisher(
            PointCloud2,
            '/radar/pointcloud',
            10
        )
        
        # ZMQ setup for radar sensor data
        self.zmq_context = zmq.Context()
        self.radar_socket = self.zmq_context.socket(zmq.SUB)
        self.radar_endpoint = "tcp://192.168.232.50:35001"
        self.radar_socket.connect(self.radar_endpoint)
        self.radar_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.radar_socket.setsockopt(zmq.RCVTIMEO, 50)  # 50ms timeout
        
        # Statistics
        self.radar_message_count = 0
        self.last_stats_time = time.time()
        self.stats_interval = 1.0
        
        # Data collection for aggregating multiple sensor readings
        self.collected_points = []
        self.sensors_seen = set()
        self.expected_sensor_count = 8  # Wait for all 8 sensors
        self.collection_timeout = 0.5  # 500ms timeout for collecting from all sensors
        self.last_collection_time = time.time()
        
        # Create timer for processing messages
        self.timer = self.create_timer(0.01, self.process_zmq_messages)  # 100Hz
        
        self.get_logger().info(f'Radar PointCloud publisher started')
        self.get_logger().info(f'Listening for radar sensor data on {self.radar_endpoint}')
        self.get_logger().info(f'Waiting for data from {self.expected_sensor_count} sensors before publishing')

    def __del__(self):
        """Cleanup ZMQ resources"""
        try:
            if hasattr(self, 'radar_socket'):
                self.radar_socket.close()
            if hasattr(self, 'zmq_context'):
                self.zmq_context.term()
        except Exception as e:
            pass  # Ignore cleanup errors

    def convert_sensor_data_to_radar_points(self, sensor_data_list: List[SensorDataCart]) -> List[SensorDataCart]:
        """Convert sensor data to radar points (passthrough for now)"""
        return sensor_data_list

    def create_pointcloud2_msg(self, radar_points, frame_id="radar_link"):
        """Convert radar points to PointCloud2 message"""
        
        # Extract XYZ coordinates from radar points
        points = [[point.x, point.y, point.z] for point in radar_points]
        xyz_points = np.asarray(points)
        
        # Create array with 6 dimensions for PointCloud2 fields
        num_points = xyz_points.shape[0]
        points_array = np.zeros((num_points, 6), dtype=np.float32)
        
        # Fill in point data
        points_array[:, 0:3] = xyz_points         # XYZ coordinates
        points_array[:, 3] = 100.0                # intensity (default value)
        points_array[:, 4] = 0.0                  # ring (default value)
        points_array[:, 5] = 0.0                  # time (default value)

        # Create PointCloud2 message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        
        # Define point fields
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='time', offset=20, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Create a custom implementation for creating PointCloud2 messages
        pointcloud_msg = self.create_cloud(header, fields, points_array.tolist())
        pointcloud_msg.is_dense = True
        
        return pointcloud_msg
    
    def create_cloud(self, header, fields, points):
        """
        Custom implementation to create a PointCloud2 message
        """
        msg = PointCloud2()
        msg.header = header
        
        # Set basic message properties
        msg.height = 1
        msg.width = len(points)
        msg.fields = fields
        
        # Calculate point_step (size of each point in bytes)
        # Last field offset + size
        last_field = fields[-1]
        point_step = last_field.offset + 4  # Assuming FLOAT32 (4 bytes)
        msg.point_step = point_step
        msg.row_step = point_step * len(points)
        
        # Create binary data from points
        buffer = bytearray()
        for point in points:
            for value in point:
                buffer.extend(struct.pack('f', value))  # Pack as float
        
        msg.data = bytes(buffer)
        msg.is_bigendian = False
        msg.is_dense = True
        
        return msg

    def should_publish_collected_data(self) -> bool:
        """Check if we should publish the collected data based on sensors seen or timeout"""
        current_time = time.time()
        
        # Publish if we have data and either:
        # 1. We've collected from all expected sensors, or
        # 2. Enough time has passed since we started collecting (timeout)
        if len(self.collected_points) > 0:
            time_since_collection_start = current_time - self.last_collection_time
            
            # Publish if we have all sensors or timeout reached
            return len(self.sensors_seen) >= self.expected_sensor_count or time_since_collection_start >= self.collection_timeout
        
        return False
    
    def publish_collected_data(self):
        """Publish the collected radar points and reset collection"""
        if self.collected_points:
            pointcloud_msg = self.create_pointcloud2_msg(self.collected_points)
            self.publisher_.publish(pointcloud_msg)
            
            # Print statistics periodically
            if time.time() - self.last_stats_time > self.stats_interval:
                self.get_logger().info(
                    f'Published PointCloud2 with {len(self.collected_points)} points from {len(self.sensors_seen)}/{self.expected_sensor_count} sensors. '
                    f'Radar msgs: {self.radar_message_count} '
                    f'in the last {self.stats_interval} seconds.'
                )
                self.last_stats_time = time.time()
                self.radar_message_count = 0
        
        # Reset collection
        self.collected_points = []
        self.sensors_seen = set()
        self.last_collection_time = time.time()

    def process_zmq_messages(self):
        """Process incoming ZMQ messages from radar sensor"""
        try:
            # Process radar sensor data (port 35001)
            bundle = self.radar_socket.recv(zmq.NOBLOCK)
            sensor_data_list = decode_sensor_data(bundle)
            self.radar_message_count += 1
            
            # Convert sensor data to radar points and collect them
            if sensor_data_list:
                radar_points = self.convert_sensor_data_to_radar_points(sensor_data_list)
                
                # If this is the first data in a new collection cycle, reset timer
                if len(self.collected_points) == 0:
                    self.last_collection_time = time.time()
                
                # Collect points from this sensor
                self.collected_points.extend(radar_points)
                
                # Track which sensors we've seen (using sensor ID only)
                for point in radar_points:
                    self.sensors_seen.add(point.id_sensor)
                
                # Check if we should publish the collected data
                if self.should_publish_collected_data():
                    self.publish_collected_data()
                
        except zmq.Again:
            # No radar sensor data available - check if we should publish due to timeout
            if self.should_publish_collected_data():
                self.publish_collected_data()
        except Exception as e:
            self.get_logger().error(f"Failed to process radar sensor data: {e}")

    def destroy_node(self):
        """Clean up ZMQ resources"""
        if hasattr(self, 'radar_socket'):
            self.radar_socket.close()
        if hasattr(self, 'zmq_context'):
            self.zmq_context.term()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    radar_publisher = RadarPointCloudPublisher()
    
    try:
        rclpy.spin(radar_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        radar_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
