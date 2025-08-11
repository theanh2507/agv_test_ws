#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped, PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.msg import BehaviorTreeLog

class MultiPointMarker(Node):
    def __init__(self):
        super().__init__('multi_point_marker')

        self.points = []  
        self.goal_index = 0  
        self.is_navigating = False
        self.goal_sent = False

        # publish diem da chon len rviz
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        # publish goal cho robot di chuyen
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.create_subscription(PointStamped, "/clicked_point", self.clicked_point_callback, 10)

        # nhan tin hieu tu GUI thi robot bat dau di chuyen
        self.create_subscription(Bool, "/start_navigation", self.start_callback, 10)
        # self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.orientation_callback, 10)
        self.create_subscription(BehaviorTreeLog, "/behavior_tree_log", self.goal_result_callback, 10)

        # Timer
        self.create_timer(0.5, self.publish_marker)
        self.create_timer(0.5, self.process_goal)

        self.orientation_z = 0.0
        self.orientation_w = 1.0

    def clicked_point_callback(self, msg: PointStamped):
        self.points.append((msg.point.x, msg.point.y))
        self.get_logger().info(f"Added point: ({msg.point.x}, {msg.point.y})")

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "multi_points"                  # namespace cua marker
        marker.id = 0
        marker.type = Marker.SPHERE_LIST            # kieu danh sach cac hinh cau (SPHERE_LIST)
        marker.action = Marker.ADD                  # add vao rviz

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        for x, y in self.points:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.0
            marker.points.append(p)

        self.marker_pub.publish(marker)

    def start_callback(self, msg: Bool):
        if msg.data and not self.is_navigating and self.points:
            self.is_navigating = True
            self.goal_index = 0
            self.goal_sent = False
            self.get_logger().info("Start navigation received.")

    def process_goal(self):
        if self.is_navigating and not self.goal_sent and self.goal_index < len(self.points):
            x, y = self.points[self.goal_index]
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.position.z = 0.0
            goal.pose.orientation.z = self.orientation_z
            goal.pose.orientation.w = self.orientation_w
            self.goal_pub.publish(goal)
            self.goal_sent = True
            self.get_logger().info(f"Sent goal to ({x}, {y})")

    def goal_result_callback(self, msg: BehaviorTreeLog):
        for event in msg.event_log:
            if event.node_name == "NavigateWithReplanning" and event.current_status == "SUCCESS":
                if self.is_navigating and self.goal_sent:
                    # remove point
                    if self.goal_index < len(self.points):
                        self.points.pop(self.goal_index)

                    # di chuyen den het cac diem trong mang
                    if self.points:
                        self.goal_sent = False
                    else:
                        self.get_logger().info("All goals completed.")
                        self.is_navigating = False
                        self.goal_sent = False
                        self.goal_index = 0

    def orientation_callback(self, msg: PoseWithCovarianceStamped):
        self.orientation_z = 0.0
        self.orientation_w = 1.0

def main(args=None):
    rclpy.init(args=args)
    node = MultiPointMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

