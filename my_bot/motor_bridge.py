#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import time

class MotorSequenceNode(Node):
    def __init__(self):
        super().__init__('motor_sequence_node')

        self.serial_port = '/dev/ttyACM0'
        self.baud_rate = 57600
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            time.sleep(2)
            self.get_logger().info(f"Opened serial port {self.serial_port} at {self.baud_rate} baud")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise e

        self.num_motors = 4
        self.target_rpm = 80.0
        self.pause_duration = 1.0
        self.target_counts = 12000

        self.current_encoder = [0.0] * self.num_motors
        self.reset_confirmed = False
        self.current_phase = ""
        self.motors_active = False
        self.serial_buffer = ""

    def send_command(self, cmd):
        try:
            full_cmd = cmd + '\n'
            self.ser.write(full_cmd.encode('utf-8'))
            self.ser.flush()
            self.get_logger().info(f"Sent command: {repr(full_cmd)}")
            time.sleep(0.05)
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

    def switch_linear_actuators_back(self):
        self.send_command("SWITCH_BACK")
        self.get_logger().info("Sent SWITCH_BACK command to linear actuators")

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

    def wait_for_confirmation(self, attr_name, timeout=5.0):
        deadline = time.time() + timeout
        while not getattr(self, attr_name) and time.time() < deadline:
            self.read_serial_lines()
            time.sleep(0.05)
        return getattr(self, attr_name)

    def routine_one(self):
        self.get_logger().info("Starting the first routine...")
        self.reset_encoder()
        if not self.wait_for_confirmation('reset_confirmed'):
            self.get_logger().error("Failed to confirm reset, aborting routine one.")
            return

        self.switch_linear_actuators()
        wait_start = time.time()
        wait_duration = 4
        self.get_logger().info(f"Waiting {wait_duration}s for linear actuators to complete forward switch...")
        while time.time() - wait_start < wait_duration:
            self.read_serial_lines()
            time.sleep(0.05)

        self.current_phase = "MOVING FORWARD"
        start_counts = self.current_encoder.copy()
        self.send_rpm_all(self.target_rpm)
        while any(abs(self.current_encoder[i] - start_counts[i]) < (self.target_counts*2) for i in range(self.num_motors)):
            self.read_serial_lines()
            time.sleep(0.05)
        self.stop_all_motors()

        self.current_phase = "MOVING BACKWARD"
        start_counts = self.current_encoder.copy()
        self.send_rpm_all(-self.target_rpm)
        while any(abs(self.current_encoder[i] - start_counts[i]) < (self.target_counts*2) for i in range(self.num_motors)):
            self.read_serial_lines()
            time.sleep(0.05)
        self.stop_all_motors()

        self.switch_linear_actuators_back()
        wait_start = time.time()
        self.get_logger().info(f"Waiting {wait_duration}s for linear actuators to complete reverse switch...")
        while time.time() - wait_start < wait_duration:
            self.read_serial_lines()
            time.sleep(0.05)
    
    def routine_two(self):
      
        self.switch_linear_actuators()
        wait_start = time.time()
        wait_duration = 4
        self.get_logger().info(f"Waiting {wait_duration}s for linear actuators to complete forward switch...")
        while time.time() - wait_start < wait_duration:
            self.read_serial_lines()
            time.sleep(0.05)


        self.switch_linear_actuators_back()
        wait_start = time.time()
        self.get_logger().info(f"Waiting {wait_duration}s for linear actuators to complete reverse switch...")
        while time.time() - wait_start < wait_duration:
            self.read_serial_lines()
            time.sleep(0.05)

        self.current_encoder.copy()
        self.send_rpm_all(-self.target_rpm)
        while any(abs(self.current_encoder[i] - start_counts[i]) < (self.target_counts) for i in range(self.num_motors)):
            self.read_serial_lines()
            time.sleep(0.05)
        self.stop_all_motors()

        self.switch_linear_actuators_back()
        wait_start = time.time()
        self.get_logger().info(f"Waiting {wait_duration}s for linear actuators to complete reverse switch...")
        while time.time() - wait_start < wait_duration:
            self.read_serial_lines()
            time.sleep(0.05)
def main(args=None):
    rclpy.init(args=args)
    node = MotorSequenceNode()

    try:
        node.get_logger().info("Calling routine one...")
        node.routine_one()
        node.get_logger().info("Waiting 10 seconds before starting routine two...")
        for i in range(100):  # 10 seconds, print a dot every 0.1s
            time.sleep(0.1)
            if i % 10 == 0:
                node.get_logger().info(".")
            node.read_serial_lines()
        node.get_logger().info("Calling routine two...")
        node.routine_two()
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
