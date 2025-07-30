import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
# import open3d as o3d


class PointCloudRoiCollector(Node):
    def __init__(self):
        super().__init__('roi_pointcloud_collector')

        # Size of the tracked objects in cm, x,y,z with respect to the map orientation
        # need to take it in sensor orinetation.. but maybe 
        roi_R1_dim = [5.2, 5.2, 5.2]
        roi_R2_dim = [5.2, 5.2, 5.2]
        roi_R3_dim = [18.0, 18.0, 14.0]
        roi_R4_dim = [29.5, 16.0, 16.0]
        roi_Q1_dim = [21.0, 5.0, 25.0]
        roi_Q2_dim = [21.0, 5.0, 25.0]
        roi_Q3_dim = [21.0, 5.0, 25.0]
        roi_Q4_dim = [21.0, 5.0, 25.0]

        roi_R1_pos = [201.4, 300.0, 124.0]
        roi_R2_pos = [-216.2, 428.7, 223.0]
        roi_R3_pos = [-164.8, 110.6, 162.5]
        roi_R4_pos = [142.5, 182.4, 264.0]
        roi_Q1_pos = [-203.4, 557.3, 158.0]
        roi_Q2_pos = [-114.5, 675.1, 110.6]
        roi_Q3_pos = [-11.6, 728.7, 197.3]
        roi_Q4_pos = [107.2, 642.0, 176.0]              

        # Subscribing to LiDAR data
        self.subscription = self.create_subscription(
            PointCloud2,
            '/unilidar/cloud',  # Update topic name if needed
            self.pointcloud_callback,
            10
        )

        # Publisher for RViz marker
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.create_timer(1.0, self.publish_marker)

        # ROI configuration (center and size of a box)
        self.roi_center = np.array([2.0, 1.0, 2])
        self.roi_size = np.array([0.5, 0.5, 0.5])  # Width, Height, Depth

        # Accumulated points
        self.filtered_points = []

        self.get_logger().info("Node initialized and listening...")

    def pointcloud_callback(self, msg):
        # Convert incoming PointCloud2 message to xyz points
        points = np.array([
            [x, y, z]
            for x, y, z in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ])

        # Axis-aligned bounding box filter
        min_bound = self.roi_center - self.roi_size / 2.0
        max_bound = self.roi_center + self.roi_size / 2.0
        in_box = np.all((points >= min_bound) & (points <= max_bound), axis=1)
        inside_points = points[in_box]

        if inside_points.size > 0:
            self.filtered_points.append(inside_points)

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "unilidar_lidar"  # Match with your LiDAR frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "roi"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Marker position
        marker.pose.position.x = self.roi_center[0]
        marker.pose.position.y = self.roi_center[1]
        marker.pose.position.z = self.roi_center[2]
        marker.pose.orientation.w = 1.0

        # Marker dimensions
        marker.scale.x = self.roi_size[0]
        marker.scale.y = self.roi_size[1]
        marker.scale.z = self.roi_size[2]

        # Color and transparency
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.3

        self.marker_pub.publish(marker)

    def save_results(self):
        if self.filtered_points:
            all_points = np.vstack(self.filtered_points)
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(all_points)
            o3d.io.write_point_cloud("filtered_roi_output.ply", pcd)
            self.get_logger().info(f"Saved {len(all_points)} points to filtered_roi_output.ply")
        else:
            self.get_logger().warn("No points collected in ROI.")

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudRoiCollector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down. Saving filtered data...")
        #node.save_results()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
