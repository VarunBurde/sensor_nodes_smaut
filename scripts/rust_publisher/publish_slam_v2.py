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
    x_local: float
    y_local: float
    heading :float
    rel_pos_n :float
    rel_pos_e :float
    rel_pos_d :float
    acc_n :float
    acc_e :float
    acc_d :float

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
        
        # GPS quality and stability parameters
        self.gps_quality_threshold = 0.5  # meters, only use GPS if accuracy is better
        self.max_gps_jump_distance = 2.0  # meters, reject GPS readings that jump more than this
        self.rtk_stability_time = 1.0  # seconds, require RTK to be stable for this long (reduced)
        self.transformation_lock_time = 0.0  # seconds, minimum time before allowing transformation update (disabled for continuous updates)
        self.min_satellites = 6  # minimum number of satellites for reliable GPS (relaxed)
        self.gps_buffer_size = 10  # number of readings to average for smoothing (reduced for faster response)
        
        # GPS tracking variables
        self.previous_gps_position = None  # (x, y, timestamp)
        self.rtk_stable_since = None  # timestamp when RTK became stable
        self.last_transformation_time = None
        self.transformation_quality = None  # GPS accuracy when transformation was established
        self.gps_position_buffer = []  # buffer for GPS smoothing
        
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
        
        if hasattr(self, '_coordinate_transform_established'):
            self.get_logger().info(f"Coordinate transformation established:")
            self.get_logger().info(f"  Translation: dx={self._slam_to_gps_translation_x:.3f}, dy={self._slam_to_gps_translation_y:.3f}")
            self.get_logger().info(f"  Rotation: dφ={self._slam_to_gps_rotation:.3f}rad ({self._slam_to_gps_rotation * 57.3:.1f}°)")
            
            # Show seamless handover state
            if hasattr(self, '_last_rtk_gps_x'):
                self.get_logger().info(f"  Last RTK state for handover:")
                self.get_logger().info(f"    GPS: x={self._last_rtk_gps_x:.3f}, y={self._last_rtk_gps_y:.3f}, φ={self._last_rtk_gps_phi:.3f}")
                self.get_logger().info(f"    SLAM: x={self._last_rtk_slam_x:.3f}, y={self._last_rtk_slam_y:.3f}, φ={self._last_rtk_slam_phi:.3f}")
            else:
                self.get_logger().info(f"  No handover state stored yet")
        else:
            self.get_logger().info("No coordinate transformation established yet")
        
        if self.robot_model:
            self.get_logger().info(f"Robot model GPS data:")
            self.get_logger().info(f"  GNSS Local: x_local={self.robot_model.navigation_gnss.x_local:.3f}, y_local={self.robot_model.navigation_gnss.y_local:.3f}, heading={self.robot_model.navigation_gnss.heading:.3f}rad")
            self.get_logger().info(f"  Robot Map:  x_map={self.robot_model.robot_status.x_map:.3f}, y_map={self.robot_model.robot_status.y_map:.3f}, heading={self.robot_model.robot_status.heading:.3f}rad")
            self.get_logger().info(f"  Using: GNSS Local coordinates (x_local, y_local) as GPS source")
        
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

    def is_gps_quality_acceptable(self):
        """Check if current GPS quality meets our standards"""
        if not self.robot_model or not self.robot_model.navigation_gnss:
            return False
        
        gnss = self.robot_model.navigation_gnss
        
        # Check accuracy
        if gnss.accuracy > self.gps_quality_threshold:
            return False
        
        # Check number of satellites
        if gnss.num_sats < self.min_satellites:
            return False
        
        # Check HDOP (horizontal dilution of precision) - lower is better - relaxed threshold
        if gnss.hdop > 3.0:
            return False
        
        return True

    def detect_gps_jump(self, current_x, current_y):
        """Detect if GPS position represents an unrealistic jump"""
        if self.previous_gps_position is None:
            return False  # No previous position to compare
        
        prev_x, prev_y, prev_time = self.previous_gps_position
        current_time = time.time()
        
        # Calculate distance and time difference
        distance = math.sqrt((current_x - prev_x)**2 + (current_y - prev_y)**2)
        time_diff = current_time - prev_time
        
        # If time difference is too small, skip check to avoid division by zero
        if time_diff < 0.05:  # 50ms
            return False
        
        # Calculate implied speed (m/s)
        implied_speed = distance / time_diff
        
        # For a mower/robot, speeds > 5 m/s (18 km/h) are unrealistic
        max_realistic_speed = 5.0  # m/s
        
        # Also check absolute distance jump
        if distance > self.max_gps_jump_distance or implied_speed > max_realistic_speed:
            self.get_logger().warn(f"GPS jump detected: {distance:.2f}m in {time_diff:.2f}s (speed: {implied_speed:.2f}m/s)")
            return True
        
        return False

    def is_rtk_stable(self):
        """Check if RTK has been stable for required duration"""
        if self.rtk_fix_status != 1:
            self.rtk_stable_since = None
            return False
        
        current_time = time.time()
        
        if self.rtk_stable_since is None:
            self.rtk_stable_since = current_time
            return False
        
        return (current_time - self.rtk_stable_since) >= self.rtk_stability_time

    def smooth_gps_position(self, x, y, phi):
        """Apply smoothing filter to GPS position"""
        current_time = time.time()
        
        # Add current reading to buffer
        self.gps_position_buffer.append((x, y, phi, current_time))
        
        # Remove old readings (keep only recent ones)
        while len(self.gps_position_buffer) > self.gps_buffer_size:
            self.gps_position_buffer.pop(0)
        
        # If buffer is not full, return current reading
        if len(self.gps_position_buffer) < 3:
            return x, y, phi
        
        # Calculate weighted average (more weight to recent readings)
        total_weight = 0
        weighted_x = 0
        weighted_y = 0
        weighted_phi_x = 0  # Use sin/cos for angle averaging
        weighted_phi_y = 0
        
        for i, (buf_x, buf_y, buf_phi, buf_time) in enumerate(self.gps_position_buffer):
            # Give more weight to recent readings
            weight = i + 1
            weighted_x += buf_x * weight
            weighted_y += buf_y * weight
            weighted_phi_x += math.cos(buf_phi) * weight
            weighted_phi_y += math.sin(buf_phi) * weight
            total_weight += weight
        
        # Calculate averages
        smoothed_x = weighted_x / total_weight
        smoothed_y = weighted_y / total_weight
        smoothed_phi = math.atan2(weighted_phi_y / total_weight, weighted_phi_x / total_weight)
        
        return smoothed_x, smoothed_y, smoothed_phi

    def should_update_transformation(self, gps_accuracy):
        """Determine if coordinate transformation should be updated - now allows continuous updates"""
        current_time = time.time()
        
        # If no transformation exists, always update (if other conditions are met)
        if not hasattr(self, '_coordinate_transform_established'):
            return True
        
        # Allow continuous updates when RTK is good - removed time lock
        # Check if enough time has passed since last update (very short interval now)
        if (self.last_transformation_time is not None and 
            current_time - self.last_transformation_time < self.transformation_lock_time):
            return False
        
        # Allow updates more frequently - only require accuracy to be reasonable, not necessarily better
        # This enables continuous alignment as long as GPS quality is acceptable
        if gps_accuracy <= self.gps_quality_threshold:
            return True
        
        return False

    def should_apply_gps_offset(self):
        """Check if we should apply GPS offset with comprehensive quality checks"""
        # Basic requirements
        if not (self.gps_reference['is_set'] and self.rtk_fix_status == 1 and self.robot_model is not None):
            return False
        
        # GPS quality requirements
        if not self.is_gps_quality_acceptable():
            return False
        
        # RTK stability requirements
        if not self.is_rtk_stable():
            return False
        
        # GPS jump detection
        gps_x = self.robot_model.navigation_gnss.x_local
        gps_y = self.robot_model.navigation_gnss.y_local
        
        if self.detect_gps_jump(gps_x, gps_y):
            return False
        
        # Transformation update requirements
        gps_accuracy = self.robot_model.navigation_gnss.accuracy
        if not self.should_update_transformation(gps_accuracy):
            return False
        
        return True

    def reset_slam_offset(self):
        """Reset the coordinate transformation (useful if you want to re-establish the GPS reference)"""
        # Reset coordinate transformation
        if hasattr(self, '_coordinate_transform_established'):
            delattr(self, '_coordinate_transform_established')
        if hasattr(self, '_slam_to_gps_translation_x'):
            delattr(self, '_slam_to_gps_translation_x')
        if hasattr(self, '_slam_to_gps_translation_y'):
            delattr(self, '_slam_to_gps_translation_y')
        if hasattr(self, '_slam_to_gps_rotation'):
            delattr(self, '_slam_to_gps_rotation')
        
        # Reset reference positions for seamless handover
        if hasattr(self, '_last_gps_x'):
            delattr(self, '_last_gps_x')
        if hasattr(self, '_last_gps_y'):
            delattr(self, '_last_gps_y')
        if hasattr(self, '_last_gps_phi'):
            delattr(self, '_last_gps_phi')
        if hasattr(self, '_last_slam_x'):
            delattr(self, '_last_slam_x')
        if hasattr(self, '_last_slam_y'):
            delattr(self, '_last_slam_y')
        if hasattr(self, '_last_slam_phi'):
            delattr(self, '_last_slam_phi')
        
        # Reset GPS tracking variables
        self.previous_gps_position = None
        self.rtk_stable_since = None
        self.last_transformation_time = None
        self.transformation_quality = None
        self.gps_position_buffer = []
        
        # Reset heading tracking
        self.previous_heading = None
        self.accumulated_heading_change = 0.0
        self.last_slam_heading = None
        
        self.get_logger().info("Coordinate transformation reset - will be re-established on next stable RTK fix")

    def apply_gps_offset(self, slam_x, slam_y, slam_phi):
        """Apply coordinate transformation to align SLAM map with GPS local map coordinate system"""
        # Check if we should use GPS coordinates with all quality checks
        use_gps = self.should_apply_gps_offset()
        
        if use_gps:
            # High-quality RTK fix available - use GPS coordinates from navigation_gnss (x_local, y_local)
            raw_gps_x = self.robot_model.navigation_gnss.x_local
            raw_gps_y = self.robot_model.navigation_gnss.y_local
            raw_gps_phi = self.robot_model.navigation_gnss.heading
            
            # Apply GPS smoothing
            gps_x, gps_y, gps_phi = self.smooth_gps_position(raw_gps_x, raw_gps_y, raw_gps_phi)
            
            # Update GPS position tracking
            current_time = time.time()
            self.previous_gps_position = (gps_x, gps_y, current_time)
            
            # Establish transformation when high-quality RTK is first achieved
            if not hasattr(self, '_coordinate_transform_established'):
                # Calculate transformation: SLAM -> GPS
                self._slam_to_gps_translation_x = gps_x - slam_x
                self._slam_to_gps_translation_y = gps_y - slam_y
                self._slam_to_gps_rotation = self.normalize_angle_diff(gps_phi - slam_phi)
                self._coordinate_transform_established = True
                
                # Record transformation quality and time
                self.transformation_quality = self.robot_model.navigation_gnss.accuracy
                self.last_transformation_time = current_time
                
                self.get_logger().info(f"SLAM->GPS transformation established: "
                                     f"T=({self._slam_to_gps_translation_x:.3f}, {self._slam_to_gps_translation_y:.3f}), "
                                     f"R={self._slam_to_gps_rotation:.3f}rad ({self._slam_to_gps_rotation*57.3:.1f}°), "
                                     f"Quality={self.transformation_quality:.2f}m")
            else:
                # Continuously update transformation when RTK is available
                if self.should_update_transformation(self.robot_model.navigation_gnss.accuracy):
                    # Calculate new transformation
                    new_translation_x = gps_x - slam_x
                    new_translation_y = gps_y - slam_y
                    new_rotation = self.normalize_angle_diff(gps_phi - slam_phi)
                    
                    # Apply smoothing to prevent jitter
                    alpha = 0.1  # Smoothing factor (0.1 = slow adaptation, 0.9 = fast adaptation)
                    self._slam_to_gps_translation_x = (1 - alpha) * self._slam_to_gps_translation_x + alpha * new_translation_x
                    self._slam_to_gps_translation_y = (1 - alpha) * self._slam_to_gps_translation_y + alpha * new_translation_y
                    
                    # Handle angle smoothing carefully
                    angle_diff = self.normalize_angle_diff(new_rotation - self._slam_to_gps_rotation)
                    self._slam_to_gps_rotation = self._slam_to_gps_rotation + alpha * angle_diff
                    
                    # Normalize final angle
                    while self._slam_to_gps_rotation > math.pi:
                        self._slam_to_gps_rotation -= 2 * math.pi
                    while self._slam_to_gps_rotation < -math.pi:
                        self._slam_to_gps_rotation += 2 * math.pi
                    
                    # Update quality and time
                    old_quality = self.transformation_quality
                    self.transformation_quality = self.robot_model.navigation_gnss.accuracy
                    self.last_transformation_time = current_time
                    
                    # Log updates less frequently to avoid spam
                    if not hasattr(self, '_last_transform_log') or current_time - self._last_transform_log > 2.0:
                        self.get_logger().info(f"SLAM->GPS transformation updated continuously: "
                                             f"T=({self._slam_to_gps_translation_x:.3f}, {self._slam_to_gps_translation_y:.3f}), "
                                             f"R={self._slam_to_gps_rotation:.3f}rad, Quality={self.transformation_quality:.2f}m")
                        self._last_transform_log = current_time
            
            # Store reference positions for seamless handover
            self._last_gps_x = gps_x
            self._last_gps_y = gps_y
            self._last_gps_phi = gps_phi
            self._last_slam_x = slam_x
            self._last_slam_y = slam_y
            self._last_slam_phi = slam_phi
            
            # Filter GPS heading for smoothness
            if self.previous_heading is not None:
                gps_phi = self.filter_heading(gps_phi, self.previous_heading)
            
            self.previous_heading = gps_phi
            return gps_x, gps_y, gps_phi
        else:
            # GPS not reliable - use seamless handover with SLAM deltas or raw SLAM
            gps_rejection_reasons = []
            
            # Detailed logging of why GPS was rejected
            if not self.gps_reference['is_set']:
                gps_rejection_reasons.append("no GPS reference")
            if self.rtk_fix_status != 1:
                gps_rejection_reasons.append(f"RTK status={self.rtk_fix_status}")
            if self.robot_model is None:
                gps_rejection_reasons.append("no robot model")
            elif not self.is_gps_quality_acceptable():
                gnss = self.robot_model.navigation_gnss
                gps_rejection_reasons.append(f"poor quality (acc={gnss.accuracy:.2f}m, sats={gnss.num_sats}, hdop={gnss.hdop:.1f})")
            elif not self.is_rtk_stable():
                if self.rtk_stable_since:
                    stable_time = time.time() - self.rtk_stable_since
                    gps_rejection_reasons.append(f"RTK unstable ({stable_time:.1f}s < {self.rtk_stability_time}s)")
                else:
                    gps_rejection_reasons.append("RTK just acquired")
            elif self.robot_model and self.detect_gps_jump(self.robot_model.navigation_gnss.x_local, self.robot_model.navigation_gnss.y_local):
                gps_rejection_reasons.append("GPS jump detected")
            elif not self.should_update_transformation(self.robot_model.navigation_gnss.accuracy if self.robot_model else float('inf')):
                gps_rejection_reasons.append("transformation locked")
            
            # Log rejection reason occasionally
            if hasattr(self, '_last_rejection_log_time'):
                if time.time() - self._last_rejection_log_time > 5.0:  # Every 5 seconds
                    rejection_reason = ", ".join(gps_rejection_reasons)
                    self.get_logger().info(f"GPS rejected: {rejection_reason}")
                    self._last_rejection_log_time = time.time()
            else:
                self._last_rejection_log_time = time.time()
            
            # Use seamless handover with SLAM deltas if transformation exists
            if hasattr(self, '_coordinate_transform_established') and hasattr(self, '_last_gps_x'):
                # Calculate SLAM movement from last known reference position
                slam_delta_x = slam_x - self._last_slam_x
                slam_delta_y = slam_y - self._last_slam_y
                slam_delta_phi = self.normalize_angle_diff(slam_phi - self._last_slam_phi)
                
                # Transform the movement deltas using standard rotation matrix
                cos_rot = math.cos(self._slam_to_gps_rotation)
                sin_rot = math.sin(self._slam_to_gps_rotation)
                # Use standard rotation matrix since X-axis is working correctly
                transformed_delta_x = slam_delta_x * cos_rot - slam_delta_y * sin_rot
                transformed_delta_y = slam_delta_x * sin_rot + slam_delta_y * cos_rot
                
                # Apply transformed deltas to last known GPS position
                transformed_x = self._last_gps_x + transformed_delta_x
                transformed_y = self._last_gps_y + transformed_delta_y
                transformed_phi = self._last_gps_phi + slam_delta_phi
                
                # Normalize heading
                while transformed_phi > math.pi:
                    transformed_phi -= 2 * math.pi
                while transformed_phi < -math.pi:
                    transformed_phi += 2 * math.pi
                
                # Filter heading for smoothness
                if self.previous_heading is not None:
                    transformed_phi = self.filter_heading(transformed_phi, self.previous_heading)
                
                self.previous_heading = transformed_phi
                return transformed_x, transformed_y, transformed_phi
            else:
                # No transformation established yet - use raw SLAM
                self.previous_heading = slam_phi
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

            # Simple debug output for GPS data correspondence check
            if self.robot_model is not None:
                # Only log every 10th message to reduce spam
                if self._message_count % 10 == 0:
                    gps_x = self.robot_model.navigation_gnss.x_local
                    gps_y = self.robot_model.navigation_gnss.y_local
                    gps_phi = self.robot_model.navigation_gnss.heading
                    
                    # Simple 3-line debug output
                    self.get_logger().info(f"RTAB: ({raw_x:.3f}, {raw_y:.3f}, {mower_phi:.3f})")
                    self.get_logger().info(f"GPS:  ({gps_x:.3f}, {gps_y:.3f}, {gps_phi:.3f})")
                    self.get_logger().info(f"OUT:  ({x:.3f}, {y:.3f}, {phi:.3f})")
            
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
