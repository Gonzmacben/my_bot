#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import serial
import threading

node = None
ser = None
serial_lock = threading.Lock()
pub_encoder = None

def cmd_vel_callback(msg):
    linear_vel = msg.linear.x
    # Convert linear velocity (m/s) to ticks per PID interval
    wheel_diameter = 0.065  # meters
    ticks_per_rev = 3018
    pid_interval = 0.033  # seconds

    wheel_circumference = 3.14159 * wheel_diameter
    ticks_per_sec = (linear_vel / wheel_circumference) * ticks_per_rev
    target_ticks = int(ticks_per_sec * pid_interval)

    cmd_str = f"V:{target_ticks}\n"
    with serial_lock:
        try:
            ser.write(cmd_str.encode('utf-8'))
            node.get_logger().debug(f"Sent: {cmd_str.strip()}")
        except Exception as e:
            node.get_logger().error(f"Serial write error: {e}")

def read_serial_loop():
    while rclpy.ok():
        try:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8').strip()
                if line.startswith("ENC:"):
                    try:
                        ticks = int(line[4:])
                        msg = Int32()
                        msg.data = ticks
                        pub_encoder.publish(msg)
                        node.get_logger().debug(f"Encoder ticks: {ticks}")
                    except ValueError:
                        node.get_logger().warn(f"Invalid encoder data: {line}")
                else:
                    node.get_logger().info(f"Arduino: {line}")
        except Exception as e:
            node.get_logger().warn(f"Serial read error: {e}")

def main(args=None):
    global node, ser, pub_encoder

    rclpy.init(args=args)
    node = Node("motor_bridge")

    port = node.declare_parameter("port", "/dev/ttyUSB0").value
    baud = node.declare_parameter("baud", 57600).value

    try:
        ser = serial.Serial(port, baudrate=baud, timeout=0.1)
        node.get_logger().info(f"Connected to Arduino on {port} at {baud} baud")
    except Exception as e:
        node.get_logger().error(f"Failed to open serial port: {e}")
        rclpy.shutdown()
        return

    pub_encoder = node.create_publisher(Int32, "motor/encoder_ticks", 10)
    node.create_subscription(Twist, "cmd_vel", cmd_vel_callback, 10)

    thread = threading.Thread(target=read_serial_loop, daemon=True)
    thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.get_logger().info("Shutting down motor_bridge node...")
    ser.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
