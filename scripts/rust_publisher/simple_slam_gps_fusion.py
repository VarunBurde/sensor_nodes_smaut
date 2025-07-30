#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import zmq
import struct
import math
import msgpack
from dataclasses import dataclass
from nav_msgs.msg import Odometry

@dataclass
class RobotStatus:
    x_map: float
    y_map: float
    heading: float
    rtk_fix: int

@dataclass
class RobotModel:
    robot_status: RobotStatus

def parse_robot_model(data: dict) -> RobotModel:
    return RobotModel(
        robot_status=RobotStatus(**data["robot_status"])
    )

class SimpleSLAMGPSFusion(Node):
    def __init__(self):
        super().__init__('simple_slam_gps_fusion')
        
        # Core state
        self.robot_model = None
        self.rtk_fix_status = 0
        
        # Coordinate transformation parameters
        self._transform_established = False
        self._translation_x = 0.0
        self._translation_y = 0.0
        self._rotation = 0.0
        
        # Seamless handover state
        self._last_slam_x = 0.0
        self._last_slam_y = 0.0
        self._last_slam_phi = 0.0
        self._last_gps_x = 0.0
        self._last_gps_y = 0.0
        self._last_gps_phi = 0.0
        
        # Output data
        self.output_x = 0.0
        self.output_y = 0.0
        self.output_phi = 0.0
        
        # ZMQ for robot model data
        self.zmq_context = zmq.Context()
        self.robot_socket = self.zmq_context.socket(zmq.SUB)
        self.robot_socket.connect("tcp://192.168.232.50:35000")
        self.robot_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.robot_socket.setsockopt(zmq.RCVTIMEO, 50)
        
        # ZMQ publisher for output
        self.pub_socket = self.zmq_context.socket(zmq.PUB)
        self.pub_socket.bind("tcp://*:27745")
        
        # ROS2 SLAM subscriber
        self.odom_sub = self.create_subscription(
            Odometry, '/rtabmap/odometry_raw', self.slam_callback, 10
        )
        
        # Timers
        self.create_timer(0.1, self.check_robot_model)
        self.create_timer(0.1, self.publish_output)
        self.create_timer(2.0, self.debug_print)
        
        self.get_logger().info("Simple SLAM-GPS Fusion initialized")

    def quaternion_to_yaw(self, quat):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def check_robot_model(self):
        """Get robot model data via ZMQ"""
        try:
            msg = self.robot_socket.recv(zmq.NOBLOCK)
            data = msgpack.unpackb(msg, raw=False)
            self.robot_model = parse_robot_model(data)
            self.rtk_fix_status = self.robot_model.robot_status.rtk_fix
        except zmq.Again:
            pass
        except Exception as e:
            self.get_logger().error(f"Robot model error: {e}")

    def slam_callback(self, msg):
        """Process SLAM data and fuse with GPS"""
        # Extract SLAM data
        slam_x = msg.pose.pose.position.x
        slam_y = msg.pose.pose.position.y
        slam_phi = self.quaternion_to_yaw(msg.pose.pose.orientation)
        
        # Apply 180° rotation for camera-to-mower transformation
        slam_phi = self.normalize_angle(slam_phi + math.pi)
        
        # Fuse with GPS
        if self.has_rtk_fix():
            self.output_x, self.output_y, self.output_phi = self.gps_mode(slam_x, slam_y, slam_phi)
        elif self._transform_established:
            self.output_x, self.output_y, self.output_phi = self.slam_mode(slam_x, slam_y, slam_phi)
        else:
            # Raw SLAM mode - no transformation yet
            self.output_x, self.output_y, self.output_phi = slam_x, slam_y, slam_phi

    def has_rtk_fix(self):
        """Check if RTK fix is available"""
        return self.robot_model is not None and self.rtk_fix_status == 1

    def gps_mode(self, slam_x, slam_y, slam_phi):
        """Use GPS coordinates directly and establish/update transformation"""
        gps_x = self.robot_model.robot_status.x_map
        gps_y = self.robot_model.robot_status.y_map
        gps_phi = self.robot_model.robot_status.heading
        
        # Establish coordinate transformation on first RTK fix
        if not self._transform_established:
            self._translation_x = gps_x - slam_x
            self._translation_y = gps_y - slam_y
            self._rotation = self.normalize_angle(gps_phi - slam_phi)
            self._transform_established = True
            
            self.get_logger().info(f"Transform: dx={self._translation_x:.3f}, dy={self._translation_y:.3f}, dφ={self._rotation:.3f}")
        
        # Store state for seamless handover
        self._last_slam_x = slam_x
        self._last_slam_y = slam_y
        self._last_slam_phi = slam_phi
        self._last_gps_x = gps_x
        self._last_gps_y = gps_y
        self._last_gps_phi = gps_phi
        
        return gps_x, gps_y, gps_phi

    def slam_mode(self, slam_x, slam_y, slam_phi):
        """Use transformed SLAM coordinates with seamless handover"""
        # Calculate movement delta from last RTK position
        delta_x = slam_x - self._last_slam_x
        delta_y = slam_y - self._last_slam_y
        delta_phi = self.normalize_angle(slam_phi - self._last_slam_phi)
        
        # Apply coordinate transformation to deltas
        cos_r = math.cos(self._rotation)
        sin_r = math.sin(self._rotation)
        
        transformed_dx = delta_x * cos_r - delta_y * sin_r
        transformed_dy = delta_x * sin_r + delta_y * cos_r
        
        # Continue from last GPS position
        map_x = self._last_gps_x + transformed_dx
        map_y = self._last_gps_y + transformed_dy
        map_phi = self.normalize_angle(self._last_gps_phi + delta_phi)
        
        return map_x, map_y, map_phi

    def publish_output(self):
        """Publish fused coordinates via ZMQ"""
        try:
            data = struct.pack(">Bddd", 1, self.output_x, self.output_y, self.output_phi)
            self.pub_socket.send(data, zmq.NOBLOCK)
        except zmq.Again:
            pass

    def debug_print(self):
        """Debug output"""
        rtk_status = "RTK_FIX" if self.has_rtk_fix() else f"NO_RTK({self.rtk_fix_status})"
        transform_status = "ESTABLISHED" if self._transform_established else "NOT_SET"
        
        self.get_logger().info(f"Status: {rtk_status}, Transform: {transform_status}")
        self.get_logger().info(f"Output: x={self.output_x:.3f}, y={self.output_y:.3f}, φ={self.output_phi:.3f}")
        
        if self.robot_model and self.has_rtk_fix():
            gps_x = self.robot_model.robot_status.x_map
            gps_y = self.robot_model.robot_status.y_map
            gps_phi = self.robot_model.robot_status.heading
            self.get_logger().info(f"GPS: x={gps_x:.3f}, y={gps_y:.3f}, φ={gps_phi:.3f}")

    def shutdown(self):
        """Clean shutdown"""
        self.pub_socket.close()
        self.robot_socket.close()
        self.zmq_context.term()

def main():
    rclpy.init()
    node = SimpleSLAMGPSFusion()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
