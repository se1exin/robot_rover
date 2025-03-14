import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import gpiod
import time

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')

        # Declare and get parameters
        self.declare_parameter('servo_gpio', 18)  # Default GPIO pin
        self.declare_parameter('servo_topic', 'servo_angle')  # Default topic

        self.servo_gpio = self.get_parameter('servo_gpio').get_parameter_value().integer_value
        self.servo_topic = self.get_parameter('servo_topic').get_parameter_value().string_value

        # GPIO Chip
        self.CHIP = "gpiochip0"

        # PWM parameters
        self.PWM_FREQUENCY = 50  # Hz
        self.PERIOD_NS = int(1e9 / self.PWM_FREQUENCY)  # Convert Hz to nanoseconds
        self.MIN_DUTY_CYCLE = 0.05  # 5% (approx 0°)
        self.MAX_DUTY_CYCLE = 0.10  # 10% (approx 180°)

        # Setup GPIO
        self.chip = gpiod.Chip(self.CHIP)
        self.line = self.chip.get_line(self.servo_gpio)
        self.line.request(consumer="servo", type=gpiod.LINE_REQ_DIR_OUT)

        # ROS 2 Subscriber (Topic Name is Configurable)
        self.subscription = self.create_subscription(
            Int32,
            self.servo_topic,
            self.angle_callback,
            10
        )
        self.get_logger().info(f'Servo Controller Node started.')
        self.get_logger().info(f'Listening on topic: {self.servo_topic}')
        self.get_logger().info(f'Using GPIO Pin: {self.servo_gpio}')
        
        self.set_angle(90)

    def angle_to_duty_cycle(self, angle):
        """Convert angle (0-180) to duty cycle."""
        if 0 <= angle <= 180:
            return self.MIN_DUTY_CYCLE + (angle / 180) * (self.MAX_DUTY_CYCLE - self.MIN_DUTY_CYCLE)
        else:
            self.get_logger().warn("Invalid angle received. Must be between 0 and 180.")
            return None

    def set_angle(self, angle):
        """Generate PWM signal to move the servo to the specified angle."""
        duty_cycle = self.angle_to_duty_cycle(angle)
        if duty_cycle is None:
            return

        high_time_ns = int(duty_cycle * self.PERIOD_NS)
        low_time_ns = self.PERIOD_NS - high_time_ns

        self.get_logger().info(f"Setting angle {angle}° -> Duty Cycle: {duty_cycle*100:.1f}%")

        # Generate PWM signal manually
        for _ in range(20):  # Run for ~0.5 sec to stabilize position
            self.line.set_value(1)
            time.sleep(high_time_ns / 1e9)
            self.line.set_value(0)
            time.sleep(low_time_ns / 1e9)

    def angle_callback(self, msg):
        """Callback function for the subscriber."""
        angle = msg.data
        if 0 <= angle <= 180:
            self.set_angle(angle)
        else:
            self.get_logger().warn("Received out-of-range angle. Must be between 0 and 180.")

    def cleanup(self):
        """Release the GPIO line on shutdown."""
        self.line.release()
        self.get_logger().info("GPIO cleanup done.")

def main(args=None):
    rclpy.init(args=args)
    node = ServoController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("\nShutting down...")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
