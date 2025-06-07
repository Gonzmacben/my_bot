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
            time.sleep(2)  # Allow Arduino to reset
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
        self.stop_confirmed = False
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
        self.stop_confirmed = False
        self.send_command("STOP")
        self.motors_active = False
        self.get_logger().info("Sent STOP command, waiting for STOP_OK")

    # Inverted actuator commands:
    def switch_linear_actuators(self):
        # Send SWITCH_BACK to extend actuators (inverted)
        self.get_logger().info("Sending SWITCH_BACK command to extend actuators (inverted)")
        self.ser.reset_input_buffer()
        self.send_command("SWITCH_BACK")
        time.sleep(0.1)

    def switch_linear_actuators_back(self):
        # Send SWITCH to retract actuators (inverted)
        self.get_logger().info("Sending SWITCH command to retract actuators (inverted)")
        self.ser.reset_input_buffer()
        self.send_command("SWITCH")
        time.sleep(0.1)

    def process_line(self, line):
        line = line.strip()
        if not line:
            return

        self.get_logger().debug(f"Processing line: {repr(line)}")

        if line == "RESET_OK":
            self.reset_confirmed = True
            self.get_logger().info("[INFO] RESET confirmation received")

        elif line == "STOP_OK":
            self.stop_confirmed = True
            self.get_logger().info("[INFO] STOP confirmation received")

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

    def pause(self, duration, phase_name):
        self.current_phase = phase_name
        pause_start = time.time()
        self.get_logger().info(f"Pausing for {duration}s during phase: {phase_name}")
        while time.time() - pause_start < duration:
            self.read_serial_lines()
            time.sleep(0.05)

    def move_distance(self, ticks, rpm, phase_name):
        self.current_phase = phase_name
        start_counts = self.current_encoder.copy()
        self.send_rpm_all(rpm)
        self.get_logger().info(f"Moving {phase_name} for {ticks} ticks at {rpm} RPM")
        while any(abs(self.current_encoder[i] - start_counts[i]) < ticks for i in range(self.num_motors)):
            self.read_serial_lines()
            time.sleep(0.05)
        self.stop_all_motors()
        if not self.wait_for_confirmation('stop_confirmed', timeout=3):
            self.get_logger().warn("STOP_OK not received after stopping motors.")

    def routine_one(self):
        self.get_logger().info("Starting Routine One")

        self.reset_encoder()
        if not self.wait_for_confirmation('reset_confirmed'):
            self.get_logger().error("Failed to confirm reset, aborting Routine One.")
            return

        self.switch_linear_actuators()
        self.pause(4, "Actuators extending")

        self.move_distance(self.target_counts, self.target_rpm, "MOVING FORWARD")

        self.pause(self.pause_duration, "PAUSE 1")

        self.move_distance(self.target_counts, -self.target_rpm, "MOVING BACKWARD")

        self.pause(self.pause_duration, "PAUSE 2")

        self.switch_linear_actuators_back()
        self.pause(4, "Actuators retracting")

        self.get_logger().info("Routine One completed")

    def routine_two(self):
        self.get_logger().info("Starting Routine Two")

        self.reset_encoder()
        if not self.wait_for_confirmation('reset_confirmed'):
            self.get_logger().error("Failed to confirm reset, aborting Routine Two.")
            return

        self.switch_linear_actuators()
        self.pause(4, "Actuators extending")

        self.move_distance(self.target_counts * 2, self.target_rpm, "MOVING FORWARD - LONG")

        self.switch_linear_actuators_back()
        self.pause(4, "Actuators retracting")

        small_move_ticks = int(self.target_counts * 0.3)
        self.move_distance(small_move_ticks, self.target_rpm, "MOVING FORWARD - SMALL")

        self.switch_linear_actuators()
        self.pause(4, "Actuators extending")

        self.move_distance(self.target_counts // 2, -self.target_rpm, "MOVING BACKWARD - HALF")

        self.switch_linear_actuators_back()
        self.pause(4, "Actuators retracting")

        self.move_distance(self.target_counts // 2, -self.target_rpm, "MOVING BACKWARD - FINAL")

        self.get_logger().info("Routine Two completed")

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
