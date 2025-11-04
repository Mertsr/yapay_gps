#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np

class OdomRepublisher(Node):
    def __init__(self):
        super().__init__('odom_to_wheel_odom_linear_only')
        in_topic  = self.declare_parameter('in_topic',  '/odom').get_parameter_value().string_value
        out_topic = self.declare_parameter('out_topic', '/wheel/odometry').get_parameter_value().string_value
        self.sub = self.create_subscription(Odometry, in_topic, self.cb, 10)
        self.pub = self.create_publisher(Odometry, out_topic, 10)
        self.get_logger().info(f"Republishing ONLY linear.x from {in_topic} -> {out_topic}")

    def cb(self, msg: Odometry):
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id

        # Pose'u aynen geçiyoruz
        out.pose = msg.pose

        # Twist: sadece ileri–geri
        out.twist.twist.linear.x = msg.twist.twist.linear.x
        out.twist.twist.linear.y = 0.0
        out.twist.twist.linear.z = 0.0
        out.twist.twist.angular.x = 0.0
        out.twist.twist.angular.y = 0.0
        out.twist.twist.angular.z = 0.0

        # Kovaryans: yalnızca vx için belirsizlik, diğerleri büyük belirsizlik
        cov = np.full((6,6), 99999.0)
        cov[0,0] = 0.05**2  # vx (m/s)
        out.twist.covariance = cov.flatten().tolist()

        self.pub.publish(out)

def main():
    rclpy.init()
    node = OdomRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
