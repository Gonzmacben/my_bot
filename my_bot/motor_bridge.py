import rclpy
from rclpy.node import Node
import serial
import time

# === SERIAL CONFIGURATION ===
serial_port = '/dev/ttyACM0'  # Adjust as needed
baud_rate = 57600
ser = serial.Serial(serial_port, baud_rate, timeout=0.1)

# === CONTROL PARAMETERS ===
num_motors = 4
target_rpm = 15.0
pause_duration = 1.0  # seconds between phases
target_counts = 3000  # encoder ticks per phase

# === GLOBAL VARIABLES ===
current_encoder = [0] * num_motors
reset_confirmed = False
current_phase = ""
motors_active = False  # Flag indicating if motors are active

def send_command(cmd):
    ser.write((cmd + '\n').encode('utf-8'))

def reset_encoder():
    global reset_confirmed
    reset_confirmed = False
    ser.reset_input_buffer()  # Clear buffer before reset
    send_command("RESET")

def send_rpm_all(rpm):
    global motors_active
    rpm_cmd = ','.join([str(rpm)] * num_motors)
    cmd = f"RPM:{rpm_cmd}"
    send_command(cmd)
    motors_active = (rpm != 0)
    if motors_active:
        node.get_logger().info(f"RPM sent to all motors: {cmd}")

def stop_all_motors():
    global motors_active
    send_command("STOP")  
    motors_active = False
    node.get_logger().info("Motors stopped.")

def switch_linear_actuators():
    send_command("SWITCH")
    node.get_logger().info("Sent SWITCH command to linear actuators")

def read_encoder_callback():
    global current_encoder, reset_confirmed
    try:
        while ser.in_waiting:
            line = ser.readline().decode('utf-8').strip()
            if line == "RESET_OK":
                reset_confirmed = True
                node.get_logger().info("[INFO] RESET confirmation received")

            elif line.startswith("ENC:"):
                try:
                    values = line[4:].split(',')
                    for i in range(min(num_motors, len(values))):
                        current_encoder[i] = int(values[i])
                    if motors_active:
                        node.get_logger().info(f"[{current_phase}] ENC: {current_encoder}")
                except Exception as e:
                    node.get_logger().warn(f"Error processing serial line '{line}': {e}")

            elif line.lstrip('-').isdigit():
                if motors_active:
                    node.get_logger().info(f"[{current_phase}] Unknown encoder value: {line}")

            else:
                node.get_logger().debug(f"Ignored: {line}")
    except Exception as e:
        node.get_logger().warn(f"Error reading serial: {e}")

def run_sequence():
    global current_phase, current_encoder

    # === Phase 1: Move Forward ===
    current_phase = "MOVING FORWARD"
    initial_encoders = current_encoder.copy()
    send_rpm_all(target_rpm)

    while all(abs(current_encoder[i] - initial_encoders[i]) < target_counts for i in range(num_motors)):
        read_encoder_callback()
    stop_all_motors()

    # === Phase 2: Pause ===
    current_phase = "PAUSE 1"
    start_time = time.time()
    while time.time() - start_time < pause_duration:
        read_encoder_callback()

    # Then trigger the linear actuator switch sequence
    switch_linear_actuators()
    # Wait enough time for linear actuator to complete (about 21 seconds)
    wait_time = 25
    node.get_logger().info(f"Waiting {wait_time} seconds for linear actuators to complete...")
    start_wait = time.time()
    while time.time() - start_wait < wait_time:
        break

    # === Phase 3: Move Backward ===
    current_phase = "MOVING BACKWARD"
    initial_encoders = current_encoder.copy()
    send_rpm_all(-target_rpm)

    while all(abs(current_encoder[i] - initial_encoders[i]) < target_counts for i in range(num_motors)):
        read_encoder_callback()
    stop_all_motors()

    # === Phase 4: Final Pause ===
    current_phase = "PAUSE 2"
    start_time = time.time()
    while time.time() - start_time < pause_duration:
        read_encoder_callback()

def main():
    global node
    rclpy.init()
    node = Node("motor_sequence_node")

    try:
        node.get_logger().info("Sequence starting...")
        reset_encoder()

        timeout = time.time() + 2.0
        while not reset_confirmed and time.time() < timeout:
            read_encoder_callback()

        # Run the original forward/backward sequence
        run_sequence()

        node.get_logger().info("Sequence completed.")

    except KeyboardInterrupt:
        stop_all_motors()
        node.get_logger().info("Interrupted by user.")
    finally:
        stop_all_motors()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
