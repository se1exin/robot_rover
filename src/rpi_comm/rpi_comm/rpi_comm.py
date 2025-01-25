import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import threading
import time


class StepperMotorNode(Node):
    def __init__(self):
        super().__init__('stepper_motor_node')

        # Declare parameters for GPIO pins
        self.declare_parameter('left_motor_step_pin', 17)
        self.declare_parameter('left_motor_dir_pin', 27)
        self.declare_parameter('left_motor_enable_pin', 22)
        self.declare_parameter('right_motor_step_pin', 23)
        self.declare_parameter('right_motor_dir_pin', 24)
        self.declare_parameter('right_motor_enable_pin', 25)

        # Get GPIO pin configurations
        self.left_motor_step_pin = self.get_parameter('left_motor_step_pin').value
        self.left_motor_dir_pin = self.get_parameter('left_motor_dir_pin').value
        self.left_motor_enable_pin = self.get_parameter('left_motor_enable_pin').value
        self.right_motor_step_pin = self.get_parameter('right_motor_step_pin').value
        self.right_motor_dir_pin = self.get_parameter('right_motor_dir_pin').value
        self.right_motor_enable_pin = self.get_parameter('right_motor_enable_pin').value

        # Initialize GPIO pins
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.left_motor_step_pin, GPIO.OUT)
        GPIO.setup(self.left_motor_dir_pin, GPIO.OUT)
        GPIO.setup(self.left_motor_enable_pin, GPIO.OUT)
        GPIO.setup(self.right_motor_step_pin, GPIO.OUT)
        GPIO.setup(self.right_motor_dir_pin, GPIO.OUT)
        GPIO.setup(self.right_motor_enable_pin, GPIO.OUT)

        # Enable both motors by default
        GPIO.output(self.left_motor_enable_pin, GPIO.HIGH)  # Disable motor initially
        GPIO.output(self.right_motor_enable_pin, GPIO.HIGH)  # Disable motor initially

        # Set up subscriptions
        self.create_subscription(Float32, 'left_track_speed', self.left_track_callback, 10)
        self.create_subscription(Float32, 'right_track_speed', self.right_track_callback, 10)

        # Initialize motor speeds
        self.left_speed = 0
        self.right_speed = 0

        # Create threads for motor control
        self.left_motor_thread = threading.Thread(target=self.run_motor, args=(self.left_motor_step_pin, self.left_motor_dir_pin, self.left_motor_enable_pin, lambda: self.left_speed, "Left Motor"))
        self.right_motor_thread = threading.Thread(target=self.run_motor, args=(self.right_motor_step_pin, self.right_motor_dir_pin, self.right_motor_enable_pin, lambda: self.right_speed, "Right Motor"))

        self.left_motor_thread.daemon = True
        self.right_motor_thread.daemon = True

        self.left_motor_thread.start()
        self.right_motor_thread.start()

        self.get_logger().info("Stepper motor node started.")

    def left_track_callback(self, msg):
        self.left_speed = msg.data
        self.get_logger().debug(f"Left track speed updated: {self.left_speed}")

    def right_track_callback(self, msg):
        self.right_speed = msg.data
        self.get_logger().debug(f"Right track speed updated: {self.right_speed}")

    def run_motor(self, step_pin, dir_pin, enable_pin, speed_getter, motor_name):
        while True:
            speed = speed_getter()
            if speed == 0:
                # Disable the motor if speed is 0
                GPIO.output(enable_pin, GPIO.HIGH)
                self.get_logger().info(f"{motor_name} disabled (speed = 0).")
                time.sleep(0.1)  # Sleep briefly to prevent CPU overload
                continue
            else:
                # Enable the motor
                GPIO.output(enable_pin, GPIO.LOW)
                self.get_logger().info(f"{motor_name} enabled (speed = {speed}).")

            # Set direction
            direction = GPIO.HIGH if speed > 0 else GPIO.LOW
            GPIO.output(dir_pin, direction)
            if direction == GPIO.HIGH:
                self.get_logger().debug(f"{motor_name} direction set to FORWARD.")
            else:
                self.get_logger().debug(f"{motor_name} direction set to REVERSE.")

            # Calculate delay between steps (speed control)
            delay = 1.0 / max(abs(speed), 1)  # Prevent division by zero

            # Step the motor
            GPIO.output(step_pin, GPIO.HIGH)
            time.sleep(delay / 2.0)  # Half the delay for HIGH signal
            GPIO.output(step_pin, GPIO.LOW)
            time.sleep(delay / 2.0)  # Half the delay for LOW signal

            self.get_logger().debug(f"{motor_name} stepped (delay = {delay:.4f}s).")

    def destroy_node(self):
        # Clean up GPIO pins on shutdown
        GPIO.cleanup()
        self.get_logger().info("GPIO cleaned up.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StepperMotorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
