import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import time

class RadarPointCloudListener(Node):
    def __init__(self):
        super().__init__('radar_pointcloud_listener')

        # Subscribe to the radar topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/radar/pointcloud',
            self.listener_callback,
            10
        )

        # Start time and message count
        self.start_time = self.get_clock().now()
        self.message_count = 0
        self.duration_seconds = 60  # Measure over 1 minute

        self.get_logger().info('Radar listener started. Counting messages for 60 seconds...')

        # Create a timer to stop after the duration
        self.timer = self.create_timer(1.0, self.check_time_elapsed)

    def listener_callback(self, msg):
        # Increment message count
        self.message_count += 1

    def check_time_elapsed(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9

        if elapsed >= self.duration_seconds:
            self.get_logger().info(f'Received {self.message_count} messages in {self.duration_seconds} seconds.')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    listener = RadarPointCloudListener()
    rclpy.spin(listener)
    listener.destroy_node()

if __name__ == '__main__':
    main()