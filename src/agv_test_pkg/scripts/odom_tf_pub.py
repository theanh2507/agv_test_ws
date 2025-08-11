#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdomTFPublisher(Node):
    def __init__(self):
        super().__init__('odom_tf_publisher')

        # Tạo TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',                
            self.odom_callback,
            10
        )

    def odom_callback(self, msg):
        # Tạo một thông điệp TF
        t = TransformStamped()
        
        # Gán timestamp
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"

        # Gán vị trí từ dữ liệu Odometry
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0

        # Gán orientation từ Odometry
        t.transform.rotation = msg.pose.pose.orientation

        # Phát TF
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
