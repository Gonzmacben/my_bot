#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import time

class MotorSequenceNode(Node):
    def __init__(self):
        super().__init__('motor_sequence_node')

        # === SERIAL CONFIGURATION ===
        self.serial_port = '/dev/ttyACM0'  # Adjust as needed
        self.baud_rate = 57600
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            time.sleep(2)  # Wait for Arduino reset
            self.get_logger().info(f"Opened serial port {self.serial_port} at {self.baud_rate} baud")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise e

        # === CONTROL PARAMETERS ===
        self.num_motors = 4
        self.target_rpm = 0.0
        self.pause_duration = 1.0  # seconds between phases
        self.target_counts = 3000  # encoder ticks per phase

        # === STATE VARIABLES ===
        self.current_encoder = [0.0] * self.num_motors
        self.reset_confirmed = False
        self.current_phase = ""
        self.motors_active = False

        # Serial read buffer for incomplete lines
        self.serial_buffer = ""

    def send_command(self, cmd):
        try:
            full_cmd = cmd + '\n'
            self.ser.write(full_cmd.encode('utf-8'))
            self.ser.flush()
            self.get_logger().info(f"Sent command: {repr(full_cmd)}")
            time.sleep(0.05)  # Small delay to allow Arduino to process
        except Exception as e:
            self.get_logger().error(f"Failed to send command '{cmd}': {e}")

    def reset_encoder(self):
        self.reset_confirmed = False
        self.ser.reset_input_buffer()
        self.send_command("RESET")

    def send_rpm_all(self, rpm):
        rpm_cmd = ','.join([str(float(rpm))] * self.num_motors)
        cmd = f"RPM:{rpm_cmd}"
        self.get_logger().info(f"Preparing to send RPM command: {cmd}")
        self.send_command(cmd)
        self.motors_active = (rpm != 0)

    def stop_all_motors(self):
        self.send_command("STOP")
        self.motors_active = False
        self.get_logger().info("Motors stopped.")

    def switch_linear_actuators(self):
        self.send_command("SWITCH")
        self.get_logger().info("Sent SWITCH command to linear actuators")

    def process_line(self, line):
        line = line.strip()
        if not line:
            return

        self.get_logger().debug(f"Processing line: {repr(line)}")

        if line == "RESET_OK":
            self.reset_confirmed = True
            self.get_logger().info("[INFO] RESET confirmation received")

        elif line.startswith("ENC:"):
            try:
                values = line[4:].split(',')
                for i in range(min(self.num_motors, len(values))):
                    self.current_encoder[i] = float(values[i])
                if self.motors_active:
                    self.get_logger().info(f"[{self.current_phase}] ENC: {self.current_encoder}")
            except Exception as e:
                self.get_logger().warn(f"Error processing encoder line '{line}': {e}")

        else:
            self.get_logger().debug(f"Ignored line: {repr(line)}")

    def read_serial_lines(self):
        try:
            while self.ser.in_waiting:
                data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                self.serial_buffer += data
                while '\n' in self.serial_buffer:
                    line, self.serial_buffer = self.serial_buffer.split('\n', 1)
                    self.process_line(line)
        except Exception as e:
            self.get_logger().warn(f"Error reading serial: {e}")

    def minimal_test_loop(self):
        self.get_logger().info("Starting minimal test loop: sending RPM=45.0 for 10 seconds")
        self.send_rpm_all(45.0)
        start_time = time.time()
        while time.time() - start_time < 10:
            self.read_serial_lines()
            time.sleep(0.1)
        self.stop_all_motors()
        self.get_logger().info("Minimal test loop completed.")

    def run_sequence(self):
        # Placeholder for your full motor sequence logic
        self.get_logger().info("Full sequence not implemented in this debug node.")

    def main_loop(self):
        self.get_logger().info("Sequence starting...")
        self.reset_encoder()

        timeout = time.time() + 5.0  # 5 seconds timeout for reset confirmation
        while not self.reset_confirmed and time.time() < timeout:
            self.read_serial_lines()
            time.sleep(0.05)

        if not self.reset_confirmed:
            self.get_logger().error("Reset confirmation not received! Aborting.")
            return

        # Run minimal test loop to verify RPM command sending and feedback
        self.minimal_test_loop()

        # Optionally, run your full sequence here:
        # self.run_sequence()

        self.get_logger().info("Sequence completed.")

def main(args=None):
    rclpy.init(args=args)
    node = MotorSequenceNode()

    try:
        node.main_loop()
    except KeyboardInterrupt:
        node.stop_all_motors()
        node.get_logger().info("Interrupted by user.")
    finally:
        node.stop_all_motors()
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
