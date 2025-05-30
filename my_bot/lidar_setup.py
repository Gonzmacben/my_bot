#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import board
import busio
import adafruit_vl53l0x

class VL53Node(Node):
    def __init__(self):
        super().__init__('vl53_node')
        self.publisher_ = self.create_publisher(Float32, 'vl53_distance', 10)
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.vl53 = adafruit_vl53l0x.VL53L0X(self.i2c)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        distance = float(self.vl53.range)
        msg = Float32()
        msg.data = distance
        self.publisher_.publish(msg)
        self.get_logger().info(f'Distancia publicada: {distance} mm')

def main(args=None):
    rclpy.init(args=args)
    node = VL53Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
