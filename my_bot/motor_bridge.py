#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial

def main():
    rclpy.init()

    node = Node('motor_bridge')

    port = node.declare_parameter('port', '/dev/ttyACM0').value
    baud = node.declare_parameter('baud', 57600).value
    timeout = node.declare_parameter('timeout', 0.1).value

    try:
        ser = serial.Serial(port, baudrate=baud, timeout=timeout)
        node.get_logger().info(f'Connected to Arduino on {port} at {baud} baud')
    except serial.SerialException as e:
        node.get_logger().error(f'Failed to open serial port: {e}')
        rclpy.shutdown()
        return

    def read_serial():
        if ser.in_waiting:
            try:
                line = ser.readline().decode('utf-8').strip()
                node.get_logger().info(f'Received: {line}')
            except Exception as e:
                node.get_logger().warn(f'Serial read error: {e}')

    timer = node.create_timer(0.02, read_serial)  # 50 Hz

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    ser.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
