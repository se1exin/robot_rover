import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

MAX_SPEED = 500
RATE_LIMIT = 0.025 
TIMEOUT = 3.0  # Time in seconds to stop the motors if no Twist command is received

class ArduinoCommNode(Node):
    def __init__(self):
        super().__init__('arduino_comm')

        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.declare_parameter('serial_port', '/dev/ttyACM0')  # Replace with your Arduino port
        self.declare_parameter('baud_rate', 115200)  # Match the baud rate in your Arduino code

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
        self.last_publish_time = time.time()
        self.last_sent_command = None

    def cmd_vel_callback(self, msg):
        current_time = time.time()
        # Compute motor speeds from the Twist message
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z

        left_motor_speed = int(MAX_SPEED * (linear_speed - angular_speed))
        right_motor_speed = int(MAX_SPEED * (linear_speed + angular_speed))

        # Clamp motor speeds to the maximum limits
        left_motor_speed = max(-MAX_SPEED, min(MAX_SPEED, left_motor_speed))
        right_motor_speed = max(-MAX_SPEED, min(MAX_SPEED, right_motor_speed))

        # Format the command to send to the Arduino
        command = f"{left_motor_speed},{right_motor_speed},{left_motor_speed},{right_motor_speed}\n"
        self.last_command_time = current_time  # Update the time of the last command

        # Only send the command if it has changed since the last sent command
        if command != self.last_sent_command and current_time - self.last_publish_time >= RATE_LIMIT:
            try:
                self.serial_conn.write(command.encode('utf-8'))
                self.get_logger().info(f'Sent to Arduino: {command.strip()}')
                self.last_sent_command = command
                self.last_publish_time = current_time
                
            except serial.SerialException as e:
                self.get_logger().error(f'Failed to send command to Arduino: {e}')

    def stop_motors(self):
        """Send a stop command to the Arduino."""
        stop_command = "0,0,0,0\n"
        try:
            self.serial_conn.write(stop_command.encode('utf-8'))
            self.get_logger().info('Sent STOP command to Arduino.')
            self.last_sent_command = stop_command
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to send STOP command to Arduino: {e}')

    def check_timeout(self):
        """Check if the timeout has been exceeded and stop the motors if necessary."""
        current_time = time.time()
        if current_time - self.last_command_time > TIMEOUT:
            self.stop_motors()

    def destroy_node(self):
        """Ensure the motors are stopped and the serial connection is closed on shutdown."""
        self.stop_motors()
        if self.serial_conn.is_open:
            self.serial_conn.close()
            self.get_logger().info('Serial connection to Arduino closed.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoCommNode()

    # Add a periodic timer to check for timeouts
    def check_timeout_callback():
        node.check_timeout()

    # Create a timer to periodically check for timeouts (every 0.1 seconds)
    timer_period = 0.1  # seconds
    timer = node.create_timer(timer_period, check_timeout_callback)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
