#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

def main(args=None):
    rclpy.init(args=args)
    node = Node('simple_x_movement')

    cmd_vel_pub = node.create_publisher(Twist, 'cmd_vel', 10)
    msg = Twist()
    msg.linear.x = 0.2  # Forward speed in m/s
    msg.angular.z = 0.0 # No rotation

    try:
        while rclpy.ok():
            cmd_vel_pub.publish(msg)
            node.get_logger().info('Publishing forward velocity: 0.2 m/s')
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.1)  # 10 Hz
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
