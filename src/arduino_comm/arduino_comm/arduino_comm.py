import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import threading

class SerialMotorControl(Node):
    def __init__(self):
        super().__init__('serial_motor_control')
        # Subscribe to the left and right track velocity topics
        self.left_track_sub = self.create_subscription(Float32, 'left_track', self.left_track_callback, 10)
        self.right_track_sub = self.create_subscription(Float32, 'right_track', self.right_track_callback, 10)

        # Initialize serial communication with Arduino
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.left_speed = 0.0
        self.right_speed = 0.0

        # Start a separate thread to continuously read from the serial port
        self.serial_thread = threading.Thread(target=self.read_from_arduino)
        self.serial_thread.daemon = True
        self.serial_thread.start()

        # Timer to send commands to Arduino periodically
        self.create_timer(0.1, self.send_to_arduino)  # 10 Hz

    def left_track_callback(self, msg):
        self.left_speed = msg.data

    def right_track_callback(self, msg):
        self.right_speed = msg.data

    def send_to_arduino(self):
        # Map speed values (-1.0 to 1.0) to motor steps or PWM values
        left_motor_command = int(self.left_speed * 255)  # Scale -1.0 to 1.0 to -255 to 255
        right_motor_command = int(self.right_speed * 255)

        # Send the commands to Arduino as a single line, e.g., "L100R-100\n"
        command = f"{left_motor_command},{right_motor_command}\n"
        self.get_logger().info(command)
        self.serial_port.write(command.encode('utf-8'))

    def read_from_arduino(self):
        while True:
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8').strip()
                self.get_logger().info(f"Received from Arduino: {line}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialMotorControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
