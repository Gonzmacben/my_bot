import rclpy
from rclpy.node import Node
import serial
import time

class MotorSequenceNode(Node):
    def __init__(self):
        super().__init__('motor_sequence_node')

        # Configura el puerto serial
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)

        # Estado interno
        self.reset_received = False
        self.encoders = None

        # Buffer para lectura serial
        self.serial_buffer = ""

        # Empieza la secuencia
        self.timer = self.create_timer(0.05, self.loop)

        # Fases
        self.phase = 0
        self.phase_start_time = self.get_clock().now()

        self.send_command("RESET\n")

    def send_command(self, command):
        self.serial_port.write(command.encode())
        self.get_logger().info(f"Sent command: '{command.strip()}'")

    def process_serial_line(self, line):
        if line.startswith("ENC:"):
            try:
                values = line[4:].split(",")
                if len(values) == 4:
                    self.encoders = [float(v) for v in values]
                    self.get_logger().info(f"[MOVING FORWARD] ENC: {self.encoders}")
            except ValueError:
                self.get_logger().warn(f"Could not parse encoder values: '{line}'")

        elif line == "RESET_OK":
            self.reset_received = True
            self.get_logger().info("[INFO] RESET confirmation received")

        elif line.strip() != "":
            self.get_logger().debug(f"Ignored line: {line}")

    def read_serial_lines(self):
        try:
            data = self.serial_port.read(self.serial_port.in_waiting or 1).decode(errors='ignore')
            self.serial_buffer += data
            lines = self.serial_buffer.split('\n')

            # Procesar todas las líneas completas menos la última
            for line in lines[:-1]:
                self.process_serial_line(line.strip())

            # Guardar última línea incompleta en buffer
            self.serial_buffer = lines[-1]
        except Exception as e:
            self.get_logger().error(f"Serial read error: {e}")

    def loop(self):
        self.read_serial_lines()

        now = self.get_clock().now()

        # Fase 0: Esperando RESET
        if self.phase == 0:
            if self.reset_received:
                self.get_logger().info("Received RESET_OK confirmation, continuing sequence.")
                self.phase = 1
                self.phase_start_time = now
                self.reset_received = False
                self.send_command("FWD\n")

        # Fase 1: Mover hacia adelante durante 2s
        elif self.phase == 1:
            if (now - self.phase_start_time).nanoseconds / 1e9 > 2.0:
                self.send_command("STOP\n")
                self.get_logger().info("Motors stopped.")
                time.sleep(0.1)
                self.send_command("RESET\n")
                self.phase = 2
                self.phase_start_time = now

        # Fase 2: Esperando segundo RESET_OK
        elif self.phase == 2:
            if self.reset_received:
                self.get_logger().info("Phase 1 complete: Moved forward.")
                self.phase = 3
                self.phase_start_time = now
                self.reset_received = False

        # Fase 3: Pausa de 1s
        elif self.phase == 3:
            if (now - self.phase_start_time).nanoseconds / 1e9 > 1.0:
                self.get_logger().info("Phase 2 complete: Pause done.")
                self.send_command("SWITCH\n")
                self.phase = 4
                self.phase_start_time = now

        # Fase 4: Esperar 4s por actuadores lineales
        elif self.phase == 4:
            if (now - self.phase_start_time).nanoseconds / 1e9 > 4.0:
                self.get_logger().info("Actuadores lineales deben haber completado el cambio.")
                self.send_command("RESET\n")
                self.phase = 5

        # Fase 5: Esperando tercer RESET_OK
        elif self.phase == 5:
            if self.reset_received:
                self.get_logger().info("Phase 3 complete: Cambio a ruedas mecanum listo.")
                self.phase = 6
                self.reset_received = False
                # Aquí puedes continuar la lógica...

def main(args=None):
    rclpy.init(args=args)
    node = MotorSequenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
