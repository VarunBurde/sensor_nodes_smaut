#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import zmq
import struct
import threading
import time
import math
import msgpack
from dataclasses import dataclass
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

@dataclass
class Info:
    timestamp: float
    id: str
    serial_number: str

@dataclass
class PowerManagement:
    batery_voltage: float

@dataclass
class NavigationGNSS:
    lon: float
    lat: float
    alt: float
    accuracy: float
    num_sats: int
    pdop: float
    hdop: float
    vdop: float
    geoid_height: float

@dataclass
class RobotStatus:
    autonomous_state: bool
    x_map: float
    y_map: float
    speed: float
    heading: float
    wheel_angle: float
    speed_left: float
    speed_right: float
    rtk_fix: int

@dataclass
class RobotDynamics:
    speed_change: float
    heading_change: float
    wheel_angle_change: float

@dataclass
class RobotModel:
    info: Info
    robot_status: RobotStatus
    navigation_gnss: NavigationGNSS
    power_management: PowerManagement
    emergency_active: bool
    dynamics: RobotDynamics

def parse_robot_model(data: dict) -> RobotModel:
    return RobotModel(
        info=Info(**data["info"]),
        robot_status=RobotStatus(**data["robot_status"]),
        navigation_gnss=NavigationGNSS(**data["navigation_gnss"]),
        power_management=PowerManagement(**data["power_management"]),
        emergency_active=data["emergency_active"],
        dynamics=RobotDynamics(**data["dynamics"]),
    )

class SLAMToZMQPublisher(Node):
    def __init__(self):
        super().__init__('slam_to_zmq_publisher')
        
        # Conversion factor from radians to degrees
        self.CONVERT = 57.29577951308232
        
        # SLAM data structure
        self.slam = {
            "validity": False,
            "robot_pose_x": 0.0,
            "robot_pose_y": 0.0,
            "robot_pose_phi": 0.0
        }
        
        # GPS reference point (will be set from ZMQ)
        self.gps_reference = {
            "lat": None,
            "lon": None,
            "alt": None,
            "is_set": False
        }
        
        # Robot model data storage
        self.robot_model = None
        self.rtk_fix_status = 0  # Will be updated from robot model data
        
        # Heading tracking and filtering
        self.previous_heading = None
        self.heading_filter_alpha = 0.8  # Filter coefficient (0.0 = no filtering, 1.0 = no change)
        self.accumulated_heading_change = 0.0  # Track heading changes from SLAM
        self.last_slam_heading = None
        
        # Camera to mower transformation (180 degree rotation in Z-axis)
        self.camera_to_mower_heading_offset = math.pi  # 180 degrees in radians
        
        # ZMQ setup for publishing SLAM data
        self.SLAM_PORT = 27745
        self.zmq_context = zmq.Context()
        self.publisher_socket = self.zmq_context.socket(zmq.PUB)
        # Bind using the specified format
        self.publisher_socket.bind(b"tcp://*:%d" % self.SLAM_PORT)
        self.get_logger().info(f"ZMQ Publisher bound to tcp://*:{self.SLAM_PORT}")
        
        # ZMQ setup for receiving GPS reference data
        self.gps_ref_socket = self.zmq_context.socket(zmq.SUB)
        self.gps_ref_endpoint = "tcp://192.168.232.55:27746"
        self.gps_ref_socket.connect(self.gps_ref_endpoint)
        self.gps_ref_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.gps_ref_socket.setsockopt(zmq.RCVTIMEO, 50)  # 50ms timeout
        self.get_logger().info(f"Connected to GPS reference at: {self.gps_ref_endpoint}")
        
        # ZMQ setup for receiving data (if needed)
        self.socket = self.zmq_context.socket(zmq.SUB)
        self.endpoint = "tcp://192.168.232.50:35000"
        self.socket.connect(self.endpoint)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.socket.setsockopt(zmq.RCVTIMEO, 50)  # 50ms timeout
        
        # ROS2 subscriber for RTAB-Map odometry
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/rtabmap/odometry_raw',
            self.odometry_callback,
            10  # QoS depth
        )
        
        # Timer for GPS reference point reading
        self.gps_ref_timer = self.create_timer(0.1, self.check_gps_reference)  # 10 Hz
        
        # Timer for robot model data reading
        self.robot_model_timer = self.create_timer(0.1, self.check_robot_model)  # 10 Hz
        
        # Publishing timer (ROS2 style)
        self.publishing = True
        self.publish_timer = self.create_timer(0.1, self.publish_slam_data)  # 10 Hz
        
        # Debug timer to check if we're receiving ROS data
        self.debug_timer = self.create_timer(5.0, self.debug_status)  # Every 5 seconds
        self._message_count = 0
        
        self.get_logger().info("SLAM to ZMQ Publisher initialized")
        self.get_logger().info(f"Publishing SLAM data on ZMQ port: {self.SLAM_PORT}")
        self.get_logger().info("Subscribed to /rtabmap/odometry_raw")
        self.get_logger().info("Waiting for GPS reference point and RTK fix status = 1")

    def debug_status(self):
        """Debug function to check if we're receiving ROS messages"""
        self.get_logger().info(f"Status: Received {self._message_count} odometry messages. Current validity: {self.slam['validity']}")
        
        # More detailed GPS status logging
        rtk_status_name = "RTK_FIXED" if self.rtk_fix_status == 1 else f"NO_RTK({self.rtk_fix_status})"
        self.get_logger().info(f"GPS Reference set: {self.gps_reference['is_set']}, RTK Status: {rtk_status_name}")
        
        if hasattr(self, '_slam_reference_x') and hasattr(self, '_gps_reference_x'):
            offset_x = self._gps_reference_x - self._slam_reference_x
            offset_y = self._gps_reference_y - self._slam_reference_y
            offset_phi = self._gps_reference_phi - self._slam_reference_phi if hasattr(self, '_gps_reference_phi') else 0.0
            self.get_logger().info(f"GPS offset established: offset=({offset_x:.3f}, {offset_y:.3f}, {offset_phi:.3f}rad)")
            self.get_logger().info(f"  SLAM ref: ({self._slam_reference_x:.3f}, {self._slam_reference_y:.3f}, {getattr(self, '_slam_reference_phi', 0.0):.3f})")
            self.get_logger().info(f"  GPS ref: ({self._gps_reference_x:.3f}, {self._gps_reference_y:.3f}, {getattr(self, '_gps_reference_phi', 0.0):.3f})")
        else:
            self.get_logger().info("No GPS offset established yet")
        
        if self.robot_model:
            self.get_logger().info(f"Robot model GPS data:")
            self.get_logger().info(f"  GPS: x_map={self.robot_model.robot_status.x_map:.3f}, y_map={self.robot_model.robot_status.y_map:.3f}, heading={self.robot_model.robot_status.heading:.3f}rad")
        
        # Current published robot position
        self.get_logger().info(f"Current published robot pose:")
        self.get_logger().info(f"  Robot: x={self.slam['robot_pose_x']:.3f}, y={self.slam['robot_pose_y']:.3f}, φ={self.slam['robot_pose_phi']:.3f}rad")
        
        # Heading tracking status
        if self.previous_heading is not None:
            self.get_logger().info(f"Heading tracking: prev={self.previous_heading:.3f}rad, accumulated_change={self.accumulated_heading_change:.3f}rad")
        else:
            self.get_logger().info("No heading tracking established yet")
        
        if self._message_count == 0:
            self.get_logger().warn("No odometry messages received! Check if /rtabmap/odometry_raw topic exists and is publishing data.")
        if not self.gps_reference['is_set']:
            self.get_logger().warn("GPS reference point not received yet!")

    def check_robot_model(self):
        """Check for robot model data from ZMQ"""
        try:
            # Try to receive robot model data
            msg = self.socket.recv(zmq.NOBLOCK)
            unpacked = msgpack.unpackb(msg, raw=False)
            self.robot_model = parse_robot_model(unpacked)
            
            # Update RTK fix status from robot model
            self.rtk_fix_status = self.robot_model.robot_status.rtk_fix
            
            if self.rtk_fix_status == 1:
                self.get_logger().debug("RTK fix achieved! Map offset will be applied when GPS reference is available.")
            
        except zmq.Again:
            # No robot model data available yet
            pass
        except Exception as e:
            self.get_logger().error(f"Failed to parse robot model data: {e}")

    def check_gps_reference(self):
        """Check for GPS reference point from ZMQ"""
        try:
            # Try to receive GPS reference data
            data = self.gps_ref_socket.recv(zmq.NOBLOCK)
            lla_ref = struct.unpack('>ddd', data)
            
            # Only set reference if we don't have it yet (first valid reading)
            if not self.gps_reference['is_set']:
                self.gps_reference['lat'] = self.CONVERT * lla_ref[0]  # Convert to degrees
                self.gps_reference['lon'] = self.CONVERT * lla_ref[1]  # Convert to degrees
                self.gps_reference['alt'] = lla_ref[2]
                self.gps_reference['is_set'] = True
                
                self.get_logger().info(f"GPS reference point set: Lat={self.gps_reference['lat']:.8f}°, "
                                     f"Lon={self.gps_reference['lon']:.8f}°, Alt={self.gps_reference['alt']:.2f}m")
                
        except zmq.Again:
            # No GPS reference data available yet
            pass
        except struct.error as e:
            self.get_logger().error(f"Failed to parse GPS reference data: {e}")

    def quaternion_to_yaw(self, quat):
        """Convert quaternion to yaw angle in radians"""
        # Convert quaternion to yaw (euler angle around z-axis)
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def normalize_angle_diff(self, angle_diff):
        """Normalize angle difference to [-pi, pi] range"""
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        return angle_diff

    def filter_heading(self, new_heading, reference_heading):
        """Apply low-pass filter to heading to smooth transitions"""
        if reference_heading is None:
            return new_heading
        
        # Calculate the shortest angular difference
        diff = self.normalize_angle_diff(new_heading - reference_heading)
        
        # Apply filter
        filtered_diff = self.heading_filter_alpha * diff
        filtered_heading = reference_heading + filtered_diff
        
        # Normalize to [-pi, pi] range
        while filtered_heading > math.pi:
            filtered_heading -= 2 * math.pi
        while filtered_heading < -math.pi:
            filtered_heading += 2 * math.pi
            
        return filtered_heading

    def camera_to_mower_heading(self, camera_heading):
        """Convert camera heading to mower heading (180-degree Z-axis transformation)"""
        # Add 180 degrees (π radians) to camera heading to get mower heading
        mower_heading = camera_heading + self.camera_to_mower_heading_offset
        
        # Normalize to [-pi, pi] range
        while mower_heading > math.pi:
            mower_heading -= 2 * math.pi
        while mower_heading < -math.pi:
            mower_heading += 2 * math.pi
            
        return mower_heading

    def should_apply_gps_offset(self):
        """Check if we should apply GPS offset (GPS reference set and RTK fix achieved)"""
        return self.gps_reference['is_set'] and self.rtk_fix_status == 1 and self.robot_model is not None

    def reset_slam_offset(self):
        """Reset the GPS offset tracking (useful if you want to re-establish the GPS reference)"""
        if hasattr(self, '_slam_reference_x'):
            delattr(self, '_slam_reference_x')
        if hasattr(self, '_slam_reference_y'):
            delattr(self, '_slam_reference_y')
        if hasattr(self, '_slam_reference_phi'):
            delattr(self, '_slam_reference_phi')
        if hasattr(self, '_gps_reference_x'):
            delattr(self, '_gps_reference_x')
        if hasattr(self, '_gps_reference_y'):
            delattr(self, '_gps_reference_y')
        if hasattr(self, '_gps_reference_phi'):
            delattr(self, '_gps_reference_phi')
        
        # Reset heading tracking
        self.previous_heading = None
        self.accumulated_heading_change = 0.0
        self.last_slam_heading = None
        
        self.get_logger().info("GPS offset and heading tracking reset - will be re-established on next RTK fix")

    def apply_gps_offset(self, slam_x, slam_y, slam_phi):
        """Apply GPS reference offset to SLAM coordinates with seamless handover and heading filtering"""
        if self.should_apply_gps_offset():
            # When RTK fix is available, use GPS position and heading
            gps_x = self.robot_model.robot_status.x_map
            gps_y = self.robot_model.robot_status.y_map
            gps_phi = self.robot_model.robot_status.heading  # Use GPS heading
            
            # Filter GPS heading to smooth transitions
            if self.previous_heading is not None:
                gps_phi = self.filter_heading(gps_phi, self.previous_heading)
            
            # Update the offset references for seamless handover when RTK is lost
            # This ensures SLAM continues from the current GPS position and heading
            self._slam_reference_x = slam_x
            self._slam_reference_y = slam_y
            self._slam_reference_phi = slam_phi
            self._gps_reference_x = gps_x
            self._gps_reference_y = gps_y
            self._gps_reference_phi = gps_phi
            
            # Update tracking variables
            self.previous_heading = gps_phi
            self.last_slam_heading = slam_phi
            self.accumulated_heading_change = 0.0  # Reset accumulation when using GPS
            
            return gps_x, gps_y, gps_phi
        else:
            # RTK fix not available - use offset if we have established one, otherwise raw SLAM
            if hasattr(self, '_slam_reference_x') and hasattr(self, '_gps_reference_x'):
                # Continue from the last GPS position using SLAM relative movement
                # This ensures seamless continuation from where GPS left off
                offset_x = (slam_x - self._slam_reference_x) + self._gps_reference_x
                offset_y = (slam_y - self._slam_reference_y) + self._gps_reference_y
                
                # For heading, accumulate SLAM changes from the last GPS heading
                if self.last_slam_heading is not None:
                    # Calculate SLAM heading change since last GPS reading
                    slam_heading_change = self.normalize_angle_diff(slam_phi - self.last_slam_heading)
                    self.accumulated_heading_change += slam_heading_change
                    self.last_slam_heading = slam_phi
                    
                    # Apply accumulated change to last GPS heading
                    offset_phi = self._gps_reference_phi + self.accumulated_heading_change
                    
                    # Normalize the result
                    while offset_phi > math.pi:
                        offset_phi -= 2 * math.pi
                    while offset_phi < -math.pi:
                        offset_phi += 2 * math.pi
                else:
                    # Fallback to basic offset calculation
                    offset_phi = (slam_phi - self._slam_reference_phi) + self._gps_reference_phi
                    self.last_slam_heading = slam_phi
                
                # Filter the heading for smoothness
                if self.previous_heading is not None:
                    offset_phi = self.filter_heading(offset_phi, self.previous_heading)
                
                self.previous_heading = offset_phi
                
                return offset_x, offset_y, offset_phi
            else:
                # No offset established yet, use raw SLAM coordinates
                self.previous_heading = slam_phi
                self.last_slam_heading = slam_phi
                return slam_x, slam_y, slam_phi

    def odometry_callback(self, msg):
        """Callback function for ROS odometry messages"""
        self._message_count += 1
        
        try:
            # Validate message structure
            if not hasattr(msg, 'pose') or not hasattr(msg.pose, 'pose'):
                self.get_logger().warn("Invalid odometry message structure")
                self.slam["validity"] = False
                return
                
            # Extract position
            raw_x = msg.pose.pose.position.x
            raw_y = msg.pose.pose.position.y
            z = msg.pose.pose.position.z  # Also available if needed
            
            # Extract orientation and convert quaternion to euler angle (yaw)
            quat = msg.pose.pose.orientation
            raw_phi = self.quaternion_to_yaw(quat)  # yaw angle in radians
            
            # Convert camera heading to mower heading (180-degree transformation)
            mower_phi = self.camera_to_mower_heading(raw_phi)
            
            # Apply GPS offset if conditions are met (now includes heading)
            x, y, phi = self.apply_gps_offset(raw_x, raw_y, mower_phi)
            
            # Update SLAM data
            self.slam["validity"] = True  # Assume valid if we receive data
            self.slam["robot_pose_x"] = x
            self.slam["robot_pose_y"] = y
            self.slam["robot_pose_phi"] = phi

            # Log with GPS offset status
            if hasattr(self, '_slam_reference_x') and hasattr(self, '_gps_reference_x'):
                # We have an established offset
                if self.should_apply_gps_offset():
                    self.get_logger().info(f"SLAM data (GPS mode):")
                    self.get_logger().info(f"  Camera: x={raw_x:.3f}, y={raw_y:.3f}, φ={raw_phi:.3f}")
                    self.get_logger().info(f"  Robot:  x={x:.3f}, y={y:.3f}, φ={phi:.3f} (GPS corrected)")
                else:
                    self.get_logger().info(f"SLAM data (GPS-offset mode):")
                    self.get_logger().info(f"  Camera: x={raw_x:.3f}, y={raw_y:.3f}, φ={raw_phi:.3f}")
                    self.get_logger().info(f"  Robot:  x={x:.3f}, y={y:.3f}, φ={phi:.3f} (offset from GPS)")
            else:
                # No offset established yet
                if self.should_apply_gps_offset():
                    self.get_logger().info(f"SLAM data (establishing GPS offset):")
                    self.get_logger().info(f"  Camera: x={raw_x:.3f}, y={raw_y:.3f}, φ={raw_phi:.3f}")
                    self.get_logger().info(f"  Robot:  x={x:.3f}, y={y:.3f}, φ={phi:.3f} (GPS corrected)")
                else:
                    status_parts = []
                    if not self.gps_reference['is_set']:
                        status_parts.append("waiting GPS ref")
                    if self.rtk_fix_status != 1:
                        status_parts.append(f"waiting RTK fix (current: {self.rtk_fix_status})")
                    if self.robot_model is None:
                        status_parts.append("waiting robot model")
                    status = ", ".join(status_parts) if status_parts else "ready"
                    self.get_logger().info(f"SLAM data (raw - {status}):")
                    self.get_logger().info(f"  Camera: x={raw_x:.3f}, y={raw_y:.3f}, φ={raw_phi:.3f}")
                    self.get_logger().info(f"  Robot:  x={x:.3f}, y={y:.3f}, φ={phi:.3f} (mower frame)")
            
        except Exception as e:
            self.get_logger().error(f"Error processing odometry data: {e}")
            self.slam["validity"] = False

    def handle_slam(self, data):
        """Handle incoming SLAM data (if receiving from ZMQ)"""
        try:
            data = struct.unpack(">Bddd", data)
            self.slam["validity"] = data[0]
            self.slam["robot_pose_x"] = data[1]
            self.slam["robot_pose_y"] = data[2]
            self.slam["robot_pose_phi"] = data[3]
        except Exception as e:
            self.get_logger().error(f"Error unpacking SLAM data: {e}")

    def publish_slam_data(self):
        """Publish SLAM data to ZMQ (called by timer)"""
        try:
            # Pack data in the same format as handle_slam expects
            validity = 1 if self.slam["validity"] else 0
            packed_data = struct.pack(
                ">Bddd", 
                validity,
                self.slam["robot_pose_x"],
                self.slam["robot_pose_y"],
                self.slam["robot_pose_phi"]
            )
            
            # Send via ZMQ
            self.publisher_socket.send(packed_data, zmq.NOBLOCK)
            
            # Debug: Log what we're publishing every 5 seconds
            if hasattr(self, '_last_debug_time'):
                if time.time() - self._last_debug_time > 5.0:
                    self.get_logger().info(f"Publishing ZMQ: validity={validity}, x={self.slam['robot_pose_x']:.3f}, y={self.slam['robot_pose_y']:.3f}, phi={self.slam['robot_pose_phi']:.3f}")
                    self._last_debug_time = time.time()
            else:
                self._last_debug_time = time.time()
            
        except zmq.Again:
            # No subscribers, continue
            pass
        except Exception as e:
            self.get_logger().error(f"Error publishing SLAM data: {e}")

    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down SLAM to ZMQ Publisher")
        self.publishing = False
        
        # Close ZMQ sockets
        self.publisher_socket.close()
        self.socket.close()
        self.gps_ref_socket.close()
        self.zmq_context.term()

def main():
    # Initialize ROS2
    rclpy.init()
    
    publisher = None
    try:
        publisher = SLAMToZMQPublisher()
        
        # Keep the node running
        rclpy.spin(publisher)
        
    except KeyboardInterrupt:
        if publisher:
            publisher.get_logger().info("SLAM to ZMQ Publisher interrupted")
    except Exception as e:
        if publisher:
            publisher.get_logger().error(f"Error in main: {e}")
    finally:
        # Clean shutdown
        if publisher:
            publisher.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
