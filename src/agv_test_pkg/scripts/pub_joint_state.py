#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from math import pi

class CmdVelToJointStateNode(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_joint_state_node")

        self.radius_wheel = 0.12  
        self.distance_two_wheel = 0.315 

        # Biến để lưu trữ vị trí và vận tốc của bánh xe
        self.angle_left_vel = 0.0
        self.angle_right_vel = 0.0
        self.angle_left_position = 0.0
        self.angle_right_position = 0.0

        self.current_time = self.get_clock().now().to_msg().sec
        self.past_time = self.current_time

        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10
        )

        # Publisher để xuất bản dữ liệu lên /joint_states
        self.joint_state_publisher = self.create_publisher(
            JointState,
            "/joint_states",
            10
        )

        # Timer để định kỳ xuất bản joint states
        self.timer = self.create_timer(0.1, self.publish_joint_state)

        self.get_logger().info("Node cmd_vel_to_joint_state đã khởi động!")

    def cmd_vel_callback(self, msg: Twist):
        # Nhận vận tốc tuyến tính và vận tốc góc từ /cmd_vel
        linear_vel = msg.linear.x  # Vận tốc tuyến tính (m/s)
        angular_vel = msg.angular.z  # Vận tốc góc (rad/s)

        # Tính vận tốc góc của từng bánh xe (rad/s)
        self.angle_right_vel = (linear_vel + (angular_vel * self.distance_two_wheel / 2)) / self.radius_wheel
        self.angle_left_vel = (linear_vel - (angular_vel * self.distance_two_wheel / 2)) / self.radius_wheel

        # self.get_logger().info(f"linear_x: {linear_vel} angular_vel: {angular_vel}")

    def publish_joint_state(self):
        # Lấy thời gian hiện tại
        self.current_time = self.get_clock().now().to_msg().sec

        # Tính khoảng thời gian giữa hai lần gọi
        if self.current_time - self.past_time > 0:
            delta_t = self.current_time - self.past_time

            # Tích lũy vị trí của bánh xe dựa trên vận tốc góc
            self.angle_left_position += self.angle_left_vel * delta_t               # goc quay
            self.angle_right_position += self.angle_right_vel * delta_t

            # Tính vận tốc tuyến tính của bánh xe (m/s) để xuất bản
            wheel_left_vel = self.angle_left_vel * self.radius_wheel
            wheel_right_vel = self.angle_right_vel * self.radius_wheel

            # Tạo thông điệp JointState
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = ['left_wheel_joint', 'right_wheel_joint'] 
            joint_state.position = [self.angle_left_position, self.angle_right_position]  # Vị trí (rad)
            joint_state.velocity = [wheel_left_vel, wheel_right_vel]  # Vận tốc (m/s)
            joint_state.effort = []  

            # Xuất bản thông điệp JointState
            self.joint_state_publisher.publish(joint_state)

            # Cập nhật thời gian trước đó
            self.past_time = self.current_time

            # self.get_logger().info(f"Đã xuất bản joint states: left_vel={wheel_left_vel:.2f}, right_vel={wheel_right_vel:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToJointStateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

