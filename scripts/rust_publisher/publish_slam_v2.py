#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import zmq
import struct
import threading
import time
import math
import msgpack
import numpy as np
from dataclasses import dataclass
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from collections import deque
import json
import os

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

@dataclass
class CoordinateDataPoint:
    """Store a single coordinate data point for analysis"""
    timestamp: float
    gps_x: float
    gps_y: float
    gps_phi: float
    slam_x: float
    slam_y: float
    slam_phi: float
    rtk_fix: int

def parse_robot_model(data: dict) -> RobotModel:
    return RobotModel(
        info=Info(**data["info"]),
        robot_status=RobotStatus(**data["robot_status"]),
        navigation_gnss=NavigationGNSS(**data["navigation_gnss"]),
        power_management=PowerManagement(**data["power_management"]),
        emergency_active=data["emergency_active"],
        dynamics=RobotDynamics(**data["dynamics"]),
    )

class CoordinateTransformationCalculator:
    """Calculate coordinate transformations between GPS and SLAM systems"""
    
    def __init__(self):
        self.gps_points = []
        self.slam_points = []
        self.transformation_matrix = None
        self.rotation_angle = None
        self.translation = None
        
    def add_point_pair(self, gps_x, gps_y, slam_x, slam_y):
        """Add a pair of corresponding GPS and SLAM coordinates"""
        self.gps_points.append([gps_x, gps_y])
        self.slam_points.append([slam_x, slam_y])
        
    def calculate_transformation(self):
        """Calculate the optimal transformation from SLAM to GPS coordinates"""
        if len(self.gps_points) < 2:
            return False, "Need at least 2 point pairs for transformation calculation"
            
        gps_array = np.array(self.gps_points)
        slam_array = np.array(self.slam_points)
        
        # Calculate centroids
        gps_centroid = np.mean(gps_array, axis=0)
        slam_centroid = np.mean(slam_array, axis=0)
        
        # Center the points
        gps_centered = gps_array - gps_centroid
        slam_centered = slam_array - slam_centroid
        
        # Calculate cross-covariance matrix
        H = slam_centered.T @ gps_centered
        
        # Use SVD to find rotation
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        
        # Ensure proper rotation (det(R) = 1)
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
            
        # Calculate translation
        t = gps_centroid - R @ slam_centroid
        
        self.rotation_matrix = R
        self.rotation_angle = math.atan2(R[1, 0], R[0, 0])
        self.translation = t
        
        # Calculate transformation error
        transformed_slam = (R @ slam_array.T).T + t
        errors = np.linalg.norm(transformed_slam - gps_array, axis=1)
        avg_error = np.mean(errors)
        max_error = np.max(errors)
        
        return True, {
            'rotation_matrix': R,
            'rotation_angle_rad': self.rotation_angle,
            'rotation_angle_deg': math.degrees(self.rotation_angle),
            'translation': t,
            'average_error': avg_error,
            'max_error': max_error,
            'num_points': len(self.gps_points)
        }
        
    def transform_point(self, slam_x, slam_y):
        """Transform a SLAM point to GPS coordinates using calculated transformation"""
        if self.rotation_matrix is None or self.translation is None:
            return slam_x, slam_y
            
        slam_point = np.array([slam_x, slam_y])
        transformed = self.rotation_matrix @ slam_point + self.translation
        return transformed[0], transformed[1]
        
    def test_different_axis_conventions(self, slam_x, slam_y, slam_phi, gps_x, gps_y, gps_phi):
        """Test different axis conventions to find the best match"""
        conventions = [
            ("Original", slam_x, slam_y, slam_phi),
            ("X↔Y swap", slam_y, slam_x, slam_phi),
            ("Y negated", slam_x, -slam_y, slam_phi),
            ("X negated", -slam_x, slam_y, slam_phi),
            ("Both negated", -slam_x, -slam_y, slam_phi),
            ("X↔Y + Y neg", slam_y, -slam_x, slam_phi),
            ("X↔Y + X neg", -slam_y, slam_x, slam_phi),
            ("GPS convention", slam_y, -slam_x, slam_phi - math.pi/2),
            ("180° rotation", -slam_x, -slam_y, slam_phi + math.pi),
            ("90° CCW", -slam_y, slam_x, slam_phi + math.pi/2),
            ("90° CW", slam_y, -slam_x, slam_phi - math.pi/2),
            ("270° CCW", slam_y, -slam_x, slam_phi - math.pi/2)
        ]
        
        results = []
        for name, tx, ty, tphi in conventions:
            pos_distance = math.sqrt((gps_x - tx)**2 + (gps_y - ty)**2)
            angle_diff = abs(self._normalize_angle_diff(gps_phi - tphi))
            score = pos_distance + angle_diff  # Combined score
            results.append((name, tx, ty, tphi, pos_distance, angle_diff, score))
            
        # Sort by combined score (lower is better)
        results.sort(key=lambda x: x[6])
        return results
        
    def _normalize_angle_diff(self, angle_diff):
        """Normalize angle difference to [-pi, pi] range"""
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        return angle_diff

class SLAMToZMQPublisher(Node):
    def __init__(self):
        super().__init__('slam_to_zmq_publisher_debug')
        
        # Conversion factor from radians to degrees
        self.CONVERT = 57.29577951308232
        
        # SLAM data structure
        self.slam = {
            "validity": False,
            "robot_pose_x": 0.0,
            "robot_pose_y": 0.0,
            "robot_pose_phi": 0.0
        }
        
        # GPS reference point
        self.gps_reference = {
            "lat": None,
            "lon": None,
            "alt": None,
            "is_set": False
        }
        
        # Robot model data storage
        self.robot_model = None
        self.rtk_fix_status = 0
        
        # Data collection for coordinate system analysis
        self.coordinate_data_points = deque(maxlen=1000)  # Store last 1000 points
        self.rtk_data_points = deque(maxlen=100)  # Store RTK-quality points
        self.transformation_calculator = CoordinateTransformationCalculator()
        self.coordinate_analysis_complete = False
        self.best_transformation = None
        
        # Coordinate transformation parameters (will be calculated automatically)
        self.coordinate_transform_established = False
        self.slam_to_gps_translation_x = 0.0
        self.slam_to_gps_translation_y = 0.0
        self.slam_to_gps_rotation = 0.0
        self.slam_to_gps_matrix = None
        
        # Best coordinate convention found through analysis
        self.best_convention = None
        self.axis_swap_needed = False
        self.x_negate = False
        self.y_negate = False
        self.heading_offset = 0.0
        
        # Reference positions for seamless handover
        self.last_rtk_gps_x = None
        self.last_rtk_gps_y = None
        self.last_rtk_gps_phi = None
        self.last_rtk_slam_x = None
        self.last_rtk_slam_y = None
        self.last_rtk_slam_phi = None
        
        # Camera to mower transformation (will be auto-detected)
        self.camera_to_mower_heading_offset = math.pi  # Initial guess
        
        # Debugging and logging
        self.debug_file_path = "/tmp/slam_gps_debug.json"
        self.analysis_results_path = "/tmp/coordinate_analysis_results.json"
        
        # ZMQ setup for publishing SLAM data
        self.SLAM_PORT = 27745
        self.zmq_context = zmq.Context()
        self.publisher_socket = self.zmq_context.socket(zmq.PUB)
        self.publisher_socket.bind(b"tcp://*:%d" % self.SLAM_PORT)
        self.get_logger().info(f"ZMQ Publisher bound to tcp://*:{self.SLAM_PORT}")
        
        # ZMQ setup for receiving GPS reference data
        self.gps_ref_socket = self.zmq_context.socket(zmq.SUB)
        self.gps_ref_endpoint = "tcp://192.168.232.55:27746"
        self.gps_ref_socket.connect(self.gps_ref_endpoint)
        self.gps_ref_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.gps_ref_socket.setsockopt(zmq.RCVTIMEO, 50)
        self.get_logger().info(f"Connected to GPS reference at: {self.gps_ref_endpoint}")
        
        # ZMQ setup for receiving robot model data
        self.socket = self.zmq_context.socket(zmq.SUB)
        self.endpoint = "tcp://192.168.232.50:35000"
        self.socket.connect(self.endpoint)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.socket.setsockopt(zmq.RCVTIMEO, 50)
        
        # ROS2 subscriber for RTAB-Map odometry
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/rtabmap/odometry_raw',  # This is the raw odometry from RTAB-Map
            self.odometry_callback,
            10
        )
        
        # Alternative topics to try if the main one doesn't work
        self.alternative_topics = [
            '/rtabmap/odom',
            '/rtabmap/odometry',
            '/odom',
            '/odometry/filtered'
        ]
        
        # Timers
        self.gps_ref_timer = self.create_timer(0.1, self.check_gps_reference)
        self.robot_model_timer = self.create_timer(0.1, self.check_robot_model)
        self.publish_timer = self.create_timer(0.1, self.publish_slam_data)
        self.debug_timer = self.create_timer(5.0, self.debug_status)
        self.analysis_timer = self.create_timer(2.0, self.perform_coordinate_analysis)
        
        # Statistics
        self._message_count = 0
        self._rtk_message_count = 0
        
        self.get_logger().info("SLAM to ZMQ Publisher with Debug initialized")
        self.get_logger().info("Collecting data points for coordinate system analysis...")
        self.get_logger().info(f"Debug data will be saved to: {self.debug_file_path}")
        self.get_logger().info(f"Analysis results will be saved to: {self.analysis_results_path}")

    def apply_coordinate_convention(self, slam_x, slam_y, slam_phi):
        """Apply the best coordinate convention found through analysis"""
        if self.best_convention is None:
            return slam_x, slam_y, slam_phi
            
        # Apply the transformation based on best convention found
        if self.best_convention == "X↔Y swap":
            return slam_y, slam_x, slam_phi
        elif self.best_convention == "Y negated":
            return slam_x, -slam_y, slam_phi
        elif self.best_convention == "X negated":
            return -slam_x, slam_y, slam_phi
        elif self.best_convention == "Both negated":
            return -slam_x, -slam_y, slam_phi
        elif self.best_convention == "X↔Y + Y neg":
            return slam_y, -slam_x, slam_phi
        elif self.best_convention == "X↔Y + X neg":
            return -slam_y, slam_x, slam_phi
        elif self.best_convention == "GPS convention":
            return slam_y, -slam_x, slam_phi - math.pi/2
        elif self.best_convention == "180° rotation":
            return -slam_x, -slam_y, slam_phi + math.pi
        elif self.best_convention == "90° CCW":
            return -slam_y, slam_x, slam_phi + math.pi/2
        elif self.best_convention == "90° CW":
            return slam_y, -slam_x, slam_phi - math.pi/2
        elif self.best_convention == "270° CCW":
            return slam_y, -slam_x, slam_phi - math.pi/2
        else:
            return slam_x, slam_y, slam_phi

    def perform_coordinate_analysis(self):
        """Analyze collected data to determine coordinate system transformation"""
        if self.coordinate_analysis_complete or len(self.rtk_data_points) < 3:
            return
            
        self.get_logger().info(f"Performing coordinate analysis with {len(self.rtk_data_points)} RTK data points...")
        
        # Test different coordinate conventions for each data point
        convention_scores = {}
        
        for data_point in self.rtk_data_points:
            if data_point.rtk_fix != 1:
                continue
                
            results = self.transformation_calculator.test_different_axis_conventions(
                data_point.slam_x, data_point.slam_y, data_point.slam_phi,
                data_point.gps_x, data_point.gps_y, data_point.gps_phi
            )
            
            # Accumulate scores for each convention
            for name, tx, ty, tphi, pos_dist, angle_diff, score in results:
                if name not in convention_scores:
                    convention_scores[name] = []
                convention_scores[name].append(score)
        
        # Find the convention with the lowest average score
        best_convention = None
        best_avg_score = float('inf')
        
        for convention, scores in convention_scores.items():
            if len(scores) > 0:
                avg_score = sum(scores) / len(scores)
                if avg_score < best_avg_score:
                    best_avg_score = avg_score
                    best_convention = convention
        
        if best_convention:
            self.best_convention = best_convention
            self.get_logger().info(f"Best coordinate convention found: {best_convention} (avg score: {best_avg_score:.3f})")
            
            # Calculate transformation matrix using the best convention
            self.transformation_calculator.gps_points.clear()
            self.transformation_calculator.slam_points.clear()
            
            for data_point in self.rtk_data_points:
                if data_point.rtk_fix == 1:
                    corrected_x, corrected_y, _ = self.apply_coordinate_convention(
                        data_point.slam_x, data_point.slam_y, data_point.slam_phi
                    )
                    self.transformation_calculator.add_point_pair(
                        data_point.gps_x, data_point.gps_y,
                        corrected_x, corrected_y
                    )
            
            success, result = self.transformation_calculator.calculate_transformation()
            if success:
                self.best_transformation = result
                self.coordinate_analysis_complete = True
                self.save_analysis_results()
                
                self.get_logger().info("=== COORDINATE ANALYSIS COMPLETE ===")
                self.get_logger().info(f"Best convention: {best_convention}")
                self.get_logger().info(f"Rotation angle: {result['rotation_angle_deg']:.1f}°")
                self.get_logger().info(f"Translation: ({result['translation'][0]:.3f}, {result['translation'][1]:.3f})")
                self.get_logger().info(f"Average error: {result['average_error']:.3f}m")
                self.get_logger().info(f"Max error: {result['max_error']:.3f}m")
                
                # Set transformation parameters
                self.slam_to_gps_rotation = result['rotation_angle_rad']
                self.slam_to_gps_translation_x = result['translation'][0]
                self.slam_to_gps_translation_y = result['translation'][1]
                self.coordinate_transform_established = True
            else:
                self.get_logger().error(f"Failed to calculate transformation: {result}")

    def save_analysis_results(self):
        """Save analysis results to file"""
        if self.best_transformation is None:
            return
            
        results = {
            'best_convention': self.best_convention,
            'transformation': {
                'rotation_angle_rad': self.best_transformation['rotation_angle_rad'],
                'rotation_angle_deg': self.best_transformation['rotation_angle_deg'],
                'translation_x': float(self.best_transformation['translation'][0]),
                'translation_y': float(self.best_transformation['translation'][1]),
                'average_error': self.best_transformation['average_error'],
                'max_error': self.best_transformation['max_error'],
                'num_points': self.best_transformation['num_points']
            },
            'data_points_used': len(self.rtk_data_points),
            'timestamp': time.time()
        }
        
        try:
            with open(self.analysis_results_path, 'w') as f:
                json.dump(results, f, indent=2)
            self.get_logger().info(f"Analysis results saved to: {self.analysis_results_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save analysis results: {e}")

    def load_previous_analysis(self):
        """Load previous analysis results if available"""
        try:
            if os.path.exists(self.analysis_results_path):
                with open(self.analysis_results_path, 'r') as f:
                    results = json.load(f)
                
                self.best_convention = results['best_convention']
                transform = results['transformation']
                self.slam_to_gps_rotation = transform['rotation_angle_rad']
                self.slam_to_gps_translation_x = transform['translation_x']
                self.slam_to_gps_translation_y = transform['translation_y']
                self.coordinate_transform_established = True
                self.coordinate_analysis_complete = True
                
                self.get_logger().info("Loaded previous coordinate analysis results:")
                self.get_logger().info(f"Convention: {self.best_convention}")
                self.get_logger().info(f"Rotation: {transform['rotation_angle_deg']:.1f}°")
                self.get_logger().info(f"Translation: ({transform['translation_x']:.3f}, {transform['translation_y']:.3f})")
                
                return True
        except Exception as e:
            self.get_logger().warn(f"Could not load previous analysis: {e}")
        return False

    def check_robot_model(self):
        """Check for robot model data from ZMQ"""
        try:
            msg = self.socket.recv(zmq.NOBLOCK)
            unpacked = msgpack.unpackb(msg, raw=False)
            self.robot_model = parse_robot_model(unpacked)
            self.rtk_fix_status = self.robot_model.robot_status.rtk_fix
            
        except zmq.Again:
            pass
        except Exception as e:
            self.get_logger().error(f"Failed to parse robot model data: {e}")

    def check_gps_reference(self):
        """Check for GPS reference point from ZMQ"""
        try:
            data = self.gps_ref_socket.recv(zmq.NOBLOCK)
            lla_ref = struct.unpack('>ddd', data)
            
            if not self.gps_reference['is_set']:
                self.gps_reference['lat'] = self.CONVERT * lla_ref[0]
                self.gps_reference['lon'] = self.CONVERT * lla_ref[1]
                self.gps_reference['alt'] = lla_ref[2]
                self.gps_reference['is_set'] = True
                
                self.get_logger().info(f"GPS reference point set: Lat={self.gps_reference['lat']:.8f}°, "
                                     f"Lon={self.gps_reference['lon']:.8f}°, Alt={self.gps_reference['alt']:.2f}m")
                
        except zmq.Again:
            pass
        except struct.error as e:
            self.get_logger().error(f"Failed to parse GPS reference data: {e}")

    def quaternion_to_yaw(self, quat):
        """Convert quaternion to yaw angle in radians"""
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

    def camera_to_mower_heading(self, camera_heading):
        """Convert camera heading to mower heading (will be auto-calibrated)"""
        mower_heading = camera_heading + self.camera_to_mower_heading_offset
        
        while mower_heading > math.pi:
            mower_heading -= 2 * math.pi
        while mower_heading < -math.pi:
            mower_heading += 2 * math.pi
            
        return mower_heading

    def should_apply_gps_offset(self):
        """Check if we should apply GPS offset"""
        return (self.gps_reference['is_set'] and 
                self.rtk_fix_status == 1 and 
                self.robot_model is not None)

    def apply_gps_offset(self, slam_x, slam_y, slam_phi):
        """Apply coordinate transformation with auto-calibrated system"""
        
        # Apply coordinate convention correction if analysis is complete
        if self.coordinate_analysis_complete:
            corrected_slam_x, corrected_slam_y, corrected_slam_phi = self.apply_coordinate_convention(
                slam_x, slam_y, slam_phi
            )
        else:
            corrected_slam_x, corrected_slam_y, corrected_slam_phi = slam_x, slam_y, slam_phi
        
        if self.should_apply_gps_offset():
            # RTK fix available - use GPS coordinates directly
            gps_x = self.robot_model.robot_status.x_map
            gps_y = self.robot_model.robot_status.y_map
            gps_phi = self.robot_model.robot_status.heading
            
            # Store data point for analysis
            if not self.coordinate_analysis_complete:
                data_point = CoordinateDataPoint(
                    timestamp=time.time(),
                    gps_x=gps_x, gps_y=gps_y, gps_phi=gps_phi,
                    slam_x=slam_x, slam_y=slam_y, slam_phi=slam_phi,
                    rtk_fix=self.rtk_fix_status
                )
                self.rtk_data_points.append(data_point)
                self._rtk_message_count += 1
            
            # Establish transformation when coordinate analysis is complete
            if (self.coordinate_analysis_complete and 
                not self.coordinate_transform_established):
                
                # Use the calculated transformation
                if self.best_transformation:
                    self.slam_to_gps_rotation = self.best_transformation['rotation_angle_rad']
                    self.slam_to_gps_translation_x = self.best_transformation['translation'][0]
                    self.slam_to_gps_translation_y = self.best_transformation['translation'][1]
                    self.coordinate_transform_established = True
                    
                    self.get_logger().info("Applied calculated coordinate transformation")
            
            # Store reference positions for seamless handover
            self.last_rtk_gps_x = gps_x
            self.last_rtk_gps_y = gps_y
            self.last_rtk_gps_phi = gps_phi
            self.last_rtk_slam_x = corrected_slam_x
            self.last_rtk_slam_y = corrected_slam_y
            self.last_rtk_slam_phi = corrected_slam_phi
            
            return gps_x, gps_y, gps_phi
            
        else:
            # RTK not available - use seamless handover with corrected SLAM coordinates
            if (self.coordinate_transform_established and 
                self.last_rtk_gps_x is not None):
                
                # Calculate SLAM movement from last known reference position
                slam_delta_x = corrected_slam_x - self.last_rtk_slam_x
                slam_delta_y = corrected_slam_y - self.last_rtk_slam_y
                slam_delta_phi = self.normalize_angle_diff(corrected_slam_phi - self.last_rtk_slam_phi)
                
                # Apply calculated transformation to deltas
                if self.best_transformation:
                    R = self.best_transformation['rotation_matrix']
                    delta_vector = np.array([slam_delta_x, slam_delta_y])
                    transformed_delta = R @ delta_vector
                    transformed_delta_x = transformed_delta[0]
                    transformed_delta_y = transformed_delta[1]
                else:
                    # Fallback to simple rotation
                    cos_rot = math.cos(self.slam_to_gps_rotation)
                    sin_rot = math.sin(self.slam_to_gps_rotation)
                    transformed_delta_x = slam_delta_x * cos_rot - slam_delta_y * sin_rot
                    transformed_delta_y = slam_delta_x * sin_rot + slam_delta_y * cos_rot
                
                # Apply transformed deltas to last known GPS position
                transformed_x = self.last_rtk_gps_x + transformed_delta_x
                transformed_y = self.last_rtk_gps_y + transformed_delta_y
                transformed_phi = self.last_rtk_gps_phi + slam_delta_phi
                
                # Normalize heading
                while transformed_phi > math.pi:
                    transformed_phi -= 2 * math.pi
                while transformed_phi < -math.pi:
                    transformed_phi += 2 * math.pi
                
                return transformed_x, transformed_y, transformed_phi
            else:
                # No transformation established yet - use corrected SLAM coordinates
                return corrected_slam_x, corrected_slam_y, corrected_slam_phi

    def odometry_callback(self, msg):
        """Callback function for ROS odometry messages"""
        self._message_count += 1
        
        try:
            if not hasattr(msg, 'pose') or not hasattr(msg.pose, 'pose'):
                self.get_logger().warn("Invalid odometry message structure")
                self.slam["validity"] = False
                return
                
            # Extract position
            raw_x = msg.pose.pose.position.x
            raw_y = msg.pose.pose.position.y
            z = msg.pose.pose.position.z
            
            # Extract orientation and convert quaternion to euler angle (yaw)
            quat = msg.pose.pose.orientation
            raw_phi = self.quaternion_to_yaw(quat)
            
            # Convert camera heading to mower heading
            mower_phi = self.camera_to_mower_heading(raw_phi)
            
            # Store all coordinate data for debugging
            current_data = CoordinateDataPoint(
                timestamp=time.time(),
                gps_x=self.robot_model.robot_status.x_map if self.robot_model else 0.0,
                gps_y=self.robot_model.robot_status.y_map if self.robot_model else 0.0,
                gps_phi=self.robot_model.robot_status.heading if self.robot_model else 0.0,
                slam_x=raw_x, slam_y=raw_y, slam_phi=mower_phi,
                rtk_fix=self.rtk_fix_status
            )
            self.coordinate_data_points.append(current_data)
            
            # Apply GPS offset with coordinate system corrections
            x, y, phi = self.apply_gps_offset(raw_x, raw_y, mower_phi)
            
            # Update SLAM data
            self.slam["validity"] = True
            self.slam["robot_pose_x"] = x
            self.slam["robot_pose_y"] = y
            self.slam["robot_pose_phi"] = phi

            # Enhanced logging with coordinate system analysis
            self.log_coordinate_debug(raw_x, raw_y, mower_phi, x, y, phi)
            
        except Exception as e:
            self.get_logger().error(f"Error processing odometry data: {e}")
            self.slam["validity"] = False

    def log_coordinate_debug(self, raw_x, raw_y, raw_phi, final_x, final_y, final_phi):
        """Enhanced coordinate system debugging"""
        if self.robot_model and self.rtk_fix_status == 1:
            gps_x = self.robot_model.robot_status.x_map
            gps_y = self.robot_model.robot_status.y_map
            gps_phi = self.robot_model.robot_status.heading
            
            self.get_logger().info("=== COORDINATE DEBUG ===")
            self.get_logger().info(f"Raw RTAB:    x={raw_x:.3f}, y={raw_y:.3f}, φ={raw_phi:.3f}rad ({math.degrees(raw_phi):.1f}°)")
            self.get_logger().info(f"GPS Map:     x={gps_x:.3f}, y={gps_y:.3f}, φ={gps_phi:.3f}rad ({math.degrees(gps_phi):.1f}°)")
            self.get_logger().info(f"Final Out:   x={final_x:.3f}, y={final_y:.3f}, φ={final_phi:.3f}rad ({math.degrees(final_phi):.1f}°)")
            
            if self.coordinate_analysis_complete:
                self.get_logger().info(f"Using convention: {self.best_convention}")
                if self.best_transformation:
                    self.get_logger().info(f"Transform error: avg={self.best_transformation['average_error']:.3f}m, max={self.best_transformation['max_error']:.3f}m")
            
            # Position and heading differences
            pos_diff = math.sqrt((gps_x - final_x)**2 + (gps_y - final_y)**2)
            heading_diff = abs(self.normalize_angle_diff(gps_phi - final_phi))
            self.get_logger().info(f"Differences: pos={pos_diff:.3f}m, heading={math.degrees(heading_diff):.1f}°")
            
            # Test different conventions in real-time if analysis not complete
            if not self.coordinate_analysis_complete:
                results = self.transformation_calculator.test_different_axis_conventions(
                    raw_x, raw_y, raw_phi, gps_x, gps_y, gps_phi
                )
                self.get_logger().info("Convention test results (top 3):")
                for i, (name, tx, ty, tphi, pos_dist, angle_diff, score) in enumerate(results[:3]):
                    self.get_logger().info(f"  {i+1}. {name:15s}: pos_err={pos_dist:.3f}m, ang_err={math.degrees(angle_diff):.1f}°, score={score:.3f}")

    def debug_status(self):
        """Enhanced debug status with coordinate analysis info"""
        self.get_logger().info(f"=== STATUS ===")
        self.get_logger().info(f"Messages: SLAM={self._message_count}, RTK={self._rtk_message_count}")
        self.get_logger().info(f"RTK Fix: {self.rtk_fix_status} ({'GOOD' if self.rtk_fix_status == 1 else 'BAD'})")
        self.get_logger().info(f"GPS Ref Set: {self.gps_reference['is_set']}")
        self.get_logger().info(f"Data Points: Total={len(self.coordinate_data_points)}, RTK={len(self.rtk_data_points)}")
        
        if self.coordinate_analysis_complete:
            self.get_logger().info(f"Analysis: COMPLETE - Using {self.best_convention}")
            if self.best_transformation:
                self.get_logger().info(f"Transform: rot={self.best_transformation['rotation_angle_deg']:.1f}°, "
                                     f"trans=({self.best_transformation['translation'][0]:.3f}, {self.best_transformation['translation'][1]:.3f})")
        else:
            self.get_logger().info(f"Analysis: IN PROGRESS (need {max(0, 3-len(self.rtk_data_points))} more RTK points)")
        
        # Current pose info
        self.get_logger().info(f"Current pose: x={self.slam['robot_pose_x']:.3f}, y={self.slam['robot_pose_y']:.3f}, φ={self.slam['robot_pose_phi']:.3f}")
        
        if self._message_count == 0:
            self.get_logger().warn("No RTAB-Map odometry messages received!")
            self.get_logger().info("Available topics:")
            # You could add topic listing here if needed

    def save_debug_data(self):
        """Save debug data to file"""
        debug_data = {
            'timestamp': time.time(),
            'coordinate_data_points': [
                {
                    'timestamp': dp.timestamp,
                    'gps_x': dp.gps_x, 'gps_y': dp.gps_y, 'gps_phi': dp.gps_phi,
                    'slam_x': dp.slam_x, 'slam_y': dp.slam_y, 'slam_phi': dp.slam_phi,
                    'rtk_fix': dp.rtk_fix
                } for dp in list(self.coordinate_data_points)
            ],
            'analysis_complete': self.coordinate_analysis_complete,
            'best_convention': self.best_convention,
            'transformation_established': self.coordinate_transform_established
        }
        
        try:
            with open(self.debug_file_path, 'w') as f:
                json.dump(debug_data, f, indent=2)
        except Exception as e:
            self.get_logger().error(f"Failed to save debug data: {e}")

    def publish_slam_data(self):
        """Publish SLAM data to ZMQ"""
        try:
            validity = 1 if self.slam["validity"] else 0
            packed_data = struct.pack(
                ">Bddd", 
                validity,
                self.slam["robot_pose_x"],
                self.slam["robot_pose_y"],
                self.slam["robot_pose_phi"]
            )
            
            self.publisher_socket.send(packed_data, zmq.NOBLOCK)
            
        except zmq.Again:
            pass
        except Exception as e:
            self.get_logger().error(f"Error publishing SLAM data: {e}")

    def shutdown(self):
        """Clean shutdown with data saving"""
        self.get_logger().info("Shutting down SLAM to ZMQ Publisher")
        
        # Save final debug data
        self.save_debug_data()
        if self.coordinate_analysis_complete:
            self.save_analysis_results()
        
        # Close ZMQ sockets
        self.publisher_socket.close()
        self.socket.close()
        self.gps_ref_socket.close()
        self.zmq_context.term()

def main():
    rclpy.init()
    
    publisher = None
    try:
        publisher = SLAMToZMQPublisher()
        
        # Try to load previous analysis results
        publisher.load_previous_analysis()
        
        # Keep the node running
        rclpy.spin(publisher)
        
    except KeyboardInterrupt:
        if publisher:
            publisher.get_logger().info("SLAM to ZMQ Publisher interrupted by user")
    except Exception as e:
        if publisher:
            publisher.get_logger().error(f"Error in main: {e}")
    finally:
        if publisher:
            publisher.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
