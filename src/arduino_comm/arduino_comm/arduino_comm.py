import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

RATE_LIMIT = 0.1  # Minimum time between serial commands in seconds
STOP_THRESHOLD = 0.01  # Threshold to consider the command as a stop

class ArduinoCommNode(Node):
    def __init__(self):
        super().__init__('arduino_comm')

        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.declare_parameter('serial_port', '/dev/ttyUSB0')  # Replace with your Arduino port
        self.declare_parameter('baud_rate', 9600)  # Match the baud rate in your Arduino code

        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        try:
            self.serial_conn = serial.Serial(serial_port, baud_rate, timeout=1)
            time.sleep(2)  # Wait for the Arduino to initialize
            self.get_logger().info(f'Connected to Arduino on {serial_port} at {baud_rate} baud.')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to Arduino on {serial_port}: {e}')
            raise

        self.last_command_time = time.time()
        self.last_sent_command = None

    def cmd_vel_callback(self, msg):
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z

        max_speed = 500  # Maximum motor speed in steps per second
        left_motor_speed = int(max_speed * (linear_speed - angular_speed))
        right_motor_speed = int(max_speed * (linear_speed + angular_speed))

        left_motor_speed = max(-max_speed, min(max_speed, left_motor_speed))
        right_motor_speed = max(-max_speed, min(max_speed, right_motor_speed))

        command = f"{left_motor_speed},{right_motor_speed},{left_motor_speed},{right_motor_speed}\n"

        current_time = time.time()
        is_stop_command = abs(linear_speed) < STOP_THRESHOLD and abs(angular_speed) < STOP_THRESHOLD

        # Always send stop commands immediately or send commands at least every RATE_LIMIT seconds
        if is_stop_command or current_time - self.last_command_time >= RATE_LIMIT:
            try:
                self.serial_conn.write(command.encode('utf-8'))
                self.get_logger().info(f'Sent to Arduino: {command.strip()}')
                self.last_command_time = current_time
                self.last_sent_command = command
            except serial.SerialException as e:
                self.get_logger().error(f'Failed to send command to Arduino: {e}')

    def destroy_node(self):
        if self.serial_conn.is_open:
            self.serial_conn.close()
            self.get_logger().info('Serial connection to Arduino closed.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoCommNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
