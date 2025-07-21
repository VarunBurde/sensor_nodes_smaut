#!/usr/bin/env python3
"""
Robot Data Publisher - Publishes robot sensor data (GPS, pose, etc.) as ROS2 messages
"""

from dataclasses import dataclass
import zmq
import msgpack
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from std_msgs.msg import Header, Float32, Bool, Int32
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import math

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

class RobotDataPublisher(Node):
    def __init__(self):
        super().__init__('robot_data_publisher')
        
        # Create publishers for different data types
        self.gps_publisher = self.create_publisher(NavSatFix, '/robot/gps/fix', 10)
        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/robot/pose', 10)
        self.emergency_publisher = self.create_publisher(Bool, '/robot/emergency_active', 10)
        self.autonomous_publisher = self.create_publisher(Bool, '/robot/autonomous_state', 10)
        self.wheel_angle_publisher = self.create_publisher(Float32, '/robot/wheel_angle', 10)
        self.diagnostics_publisher = self.create_publisher(DiagnosticArray, '/robot/diagnostics', 10)
        
        # ZMQ setup for robot data
        self.zmq_context = zmq.Context()
        self.socket = self.zmq_context.socket(zmq.SUB)
        self.endpoint = "tcp://192.168.232.50:35000"
        self.socket.connect(self.endpoint)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.socket.setsockopt(zmq.RCVTIMEO, 50)  # 50ms timeout
        
        # Statistics
        self.message_count = 0
        self.last_stats_time = time.time()
        self.stats_interval = 5.0
        
        # Create timer for processing messages
        self.timer = self.create_timer(0.01, self.process_zmq_messages)  # 100Hz
        
        self.get_logger().info(f'Robot data publisher started, listening on {self.endpoint}')
        self.get_logger().info('Publishing robot data to multiple ROS2 topics')

    def __del__(self):
        """Cleanup ZMQ resources"""
        try:
            if hasattr(self, 'socket'):
                self.socket.close()
            if hasattr(self, 'zmq_context'):
                self.zmq_context.term()
        except Exception as e:
            pass  # Ignore cleanup errors

    def parse_robot_model(self, data: dict) -> RobotModel:
        """Parse robot model data from ZMQ message"""
        return RobotModel(
            info=Info(**data["info"]),
            robot_status=RobotStatus(**data["robot_status"]),
            navigation_gnss=NavigationGNSS(**data["navigation_gnss"]),
            power_management=PowerManagement(**data["power_management"]),
            emergency_active=data["emergency_active"],
            dynamics=RobotDynamics(**data["dynamics"]),
        )

    def euler_to_quaternion(self, heading):
        """Convert heading (yaw) in radians to quaternion"""
        yaw = heading
        # Assuming roll and pitch are 0
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(0 * 0.5)  # pitch = 0
        sp = math.sin(0 * 0.5)
        cr = math.cos(0 * 0.5)  # roll = 0
        sr = math.sin(0 * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

    def publish_gps_data(self, robot: RobotModel, nav_data: NavigationGNSS, timestamp):
        """Publish GPS data as NavSatFix message"""
        gps_msg = NavSatFix()
        
        # Header
        gps_msg.header.stamp = timestamp
        gps_msg.header.frame_id = "gps_frame"
        
        # GPS coordinates - Convert from scaled integers to decimal degrees
        # Assuming the incoming data is scaled by 10^7 (common GPS format)
        gps_msg.latitude = nav_data.lat / 10000000.0  # Convert to decimal degrees
        gps_msg.longitude = nav_data.lon / 10000000.0  # Convert to decimal degrees
        gps_msg.altitude = nav_data.alt
        
        # Status - Set based on the RTK fix status
        if robot.robot_status.rtk_fix == 2:  # RTK fix
            gps_msg.status.status = NavSatStatus.STATUS_GBAS_FIX  # 2
        elif nav_data.num_sats > 3:  # Regular GPS fix
            gps_msg.status.status = NavSatStatus.STATUS_FIX  # 0
        else:  # No fix
            gps_msg.status.status = NavSatStatus.STATUS_NO_FIX  # -1
        
        # Service - Assume both GPS and Galileo (common for modern receivers in Europe)
        gps_msg.status.service = NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GALILEO  # 1 + 8 = 9
        
        # Covariance (diagonal elements based on accuracy and DOP values)
        position_covariance = [0.0] * 9
        variance = (nav_data.accuracy ** 2) * (nav_data.hdop ** 2)
        position_covariance[0] = variance  # East variance
        position_covariance[4] = variance  # North variance
        position_covariance[8] = (nav_data.accuracy ** 2) * (nav_data.vdop ** 2)  # Up variance
        gps_msg.position_covariance = position_covariance
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        
        self.gps_publisher.publish(gps_msg)

    def publish_robot_pose(self, robot_status: RobotStatus, timestamp):
        """Publish robot pose with covariance"""
        pose_msg = PoseWithCovarianceStamped()
        
        # Header
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = "wheel_frame"
        
        # Position
        pose_msg.pose.pose.position = Point(x=robot_status.x_map, y=robot_status.y_map, z=0.0)
        
        # Orientation (convert heading to quaternion)
        pose_msg.pose.pose.orientation = self.euler_to_quaternion(robot_status.heading)
        
        # Covariance (6x6 matrix for pose)
        covariance = [0.0] * 36
        # Set diagonal elements for position and orientation uncertainty
        covariance[0] = 0.1  # x variance
        covariance[7] = 0.1  # y variance
        covariance[14] = 0.1  # z variance
        covariance[21] = 0.05  # roll variance
        covariance[28] = 0.05  # pitch variance
        covariance[35] = 0.1  # yaw variance
        pose_msg.pose.covariance = covariance
        
        self.pose_publisher.publish(pose_msg)

    def publish_diagnostics(self, robot: RobotModel, timestamp):
        """Publish diagnostic information"""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = timestamp
        
        # GPS diagnostics
        gps_status = DiagnosticStatus()
        gps_status.name = "GPS"
        gps_status.message = f"Satellites: {robot.navigation_gnss.num_sats}, RTK: {robot.robot_status.rtk_fix}"
        gps_status.hardware_id = robot.info.serial_number
        
        if robot.navigation_gnss.num_sats >= 8 and robot.robot_status.rtk_fix == 2:
            gps_status.level = DiagnosticStatus.OK
        elif robot.navigation_gnss.num_sats >= 4:
            gps_status.level = DiagnosticStatus.WARN
        else:
            gps_status.level = DiagnosticStatus.ERROR
        
        gps_status.values = [
            KeyValue(key="num_satellites", value=str(robot.navigation_gnss.num_sats)),
            KeyValue(key="hdop", value=str(robot.navigation_gnss.hdop)),
            KeyValue(key="pdop", value=str(robot.navigation_gnss.pdop)),
            KeyValue(key="rtk_fix", value=str(robot.robot_status.rtk_fix)),
            KeyValue(key="accuracy", value=str(robot.navigation_gnss.accuracy))
        ]
        
        # System diagnostics
        system_status = DiagnosticStatus()
        system_status.name = "System"
        system_status.message = f"Emergency: {robot.emergency_active}, Autonomous: {robot.robot_status.autonomous_state}"
        system_status.hardware_id = robot.info.serial_number
        
        if not robot.emergency_active:
            system_status.level = DiagnosticStatus.OK
        else:
            system_status.level = DiagnosticStatus.ERROR
        
        system_status.values = [
            KeyValue(key="emergency_active", value=str(robot.emergency_active)),
            KeyValue(key="autonomous_state", value=str(robot.robot_status.autonomous_state)),
            KeyValue(key="serial_number", value=robot.info.serial_number)
        ]
        
        diag_array.status = [gps_status, system_status]
        self.diagnostics_publisher.publish(diag_array)

    def process_zmq_messages(self):
        """Process incoming ZMQ messages and publish to ROS2 topics"""
        try:
            # Non-blocking receive
            msg = self.socket.recv(zmq.NOBLOCK)
            
            # Unpack MessagePack data
            unpacked = msgpack.unpackb(msg, raw=False)
            robot = self.parse_robot_model(unpacked)
            
            # Update statistics
            self.message_count += 1
            
            # Create timestamp
            current_time = self.get_clock().now().to_msg()
            
            # Publish all data types
            self.publish_gps_data(robot, robot.navigation_gnss, current_time)
            self.publish_robot_pose(robot.robot_status, current_time)
            
            # Publish simple messages
            emergency_msg = Bool()
            emergency_msg.data = robot.emergency_active
            self.emergency_publisher.publish(emergency_msg)
            
            autonomous_msg = Bool()
            autonomous_msg.data = robot.robot_status.autonomous_state
            self.autonomous_publisher.publish(autonomous_msg)
            
            wheel_angle_msg = Float32()
            wheel_angle_msg.data = robot.robot_status.wheel_angle
            self.wheel_angle_publisher.publish(wheel_angle_msg)
            
            # Publish diagnostics
            self.publish_diagnostics(robot, current_time)
            
            # Print statistics periodically
            if time.time() - self.last_stats_time > self.stats_interval:
                self.get_logger().info(
                    f'Published robot data to ROS2 topics. '
                    f'Received {self.message_count} messages in the last {self.stats_interval} seconds.'
                )
                self.get_logger().info(
                    f'Current state - GPS Sats: {robot.navigation_gnss.num_sats}, '
                    f'Speed: {robot.robot_status.speed:.2f}m/s, '
                    f'Emergency: {robot.emergency_active}'
                )
                self.last_stats_time = time.time()
                self.message_count = 0
            
        except zmq.Again:
            # No message available, continue
            pass
        except Exception as e:
            self.get_logger().error(f"Failed to process message: {e}")

    def destroy_node(self):
        """Clean up ZMQ resources"""
        if hasattr(self, 'socket'):
            self.socket.close()
        if hasattr(self, 'zmq_context'):
            self.zmq_context.term()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    robot_data_publisher = RobotDataPublisher()
    
    try:
        rclpy.spin(robot_data_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        robot_data_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
