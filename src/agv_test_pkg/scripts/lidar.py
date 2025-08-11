#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LidarAngleFilter(Node):
    def __init__(self):
        super().__init__('lidar_angle_filter')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.publisher = self.create_publisher(LaserScan, '/scan_filter', 10)
        self.min_angle = math.radians(-100)
        self.max_angle = math.radians(100)

    def scan_callback(self, msg):
        filtered_scan = LaserScan()
        filtered_scan.header = msg.header
        filtered_scan.angle_min = max(self.min_angle, msg.angle_min)
        filtered_scan.angle_max = min(self.max_angle, msg.angle_max)
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time
        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max

        expected_points = 720
        total_angle = filtered_scan.angle_max - filtered_scan.angle_min
        num_points = int(total_angle / msg.angle_increment) + 1


        start_index = int((filtered_scan.angle_min - msg.angle_min) / msg.angle_increment)
        end_index = start_index + num_points - 1

        # Đảm bảo end_index không vượt quá kích thước mảng
        if end_index >= len(msg.ranges):
            end_index = len(msg.ranges) - 1
            num_points = end_index - start_index + 1

        # Lọc dữ liệu ranges và intensities
        filtered_scan.ranges = msg.ranges[start_index:end_index + 1]
        filtered_scan.intensities = msg.intensities[start_index:end_index + 1]

        if len(filtered_scan.ranges) != expected_points:
            self.get_logger().warn(f"Number of points {len(filtered_scan.ranges)} does not match expected {expected_points}")

        self.publisher.publish(filtered_scan)
        # self.get_logger().info("START LIDAR")

def main(args=None):
    rclpy.init(args=args)
    node = LidarAngleFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()