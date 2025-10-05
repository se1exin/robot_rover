import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import gpiod
import threading
import time

# Encoder constants
PULSES_PER_REV = 10
DEGREES_PER_PULSE = 360.0 / PULSES_PER_REV

class StepperMotorNode(Node):
    def __init__(self):
        super().__init__('stepper_motor_node')

        # 17HS13-0404S-PG5 Stepper
        # https://www.omc-stepperonline.com/nema-17-stepper-motor-bipolar-l-33mm-w-gear-ratio-5-1-planetary-gearbox-17hs13-0404s-pg5

        # Declare parameters for GPIO pins
        self.declare_parameter('left_motor_step_pin', 27)
        self.declare_parameter('left_motor_dir_pin', 17)
        self.declare_parameter('left_motor_enable_pin', 22)
        self.declare_parameter('left_motor_encoder_pin', 24)
        self.declare_parameter('right_motor_step_pin', 6)
        self.declare_parameter('right_motor_dir_pin', 5)
        self.declare_parameter('right_motor_enable_pin', 13)
        self.declare_parameter('right_motor_encoder_pin', 25)

        # Get GPIO pin configurations
        self.left_motor_step_pin = self.get_parameter('left_motor_step_pin').value
        self.left_motor_dir_pin = self.get_parameter('left_motor_dir_pin').value
        self.left_motor_enable_pin = self.get_parameter('left_motor_enable_pin').value
        self.left_motor_encoder_pin = self.get_parameter('left_motor_encoder_pin').value
        self.right_motor_step_pin = self.get_parameter('right_motor_step_pin').value
        self.right_motor_dir_pin = self.get_parameter('right_motor_dir_pin').value
        self.right_motor_enable_pin = self.get_parameter('right_motor_enable_pin').value
        self.right_motor_encoder_pin = self.get_parameter('right_motor_encoder_pin').value

        # Initialize gpiod chips and lines
        self.chip = gpiod.Chip('gpiochip0')  # Default GPIO chip
        self.lines = {
            self.left_motor_step_pin: self.chip.get_line(self.left_motor_step_pin),
            self.left_motor_dir_pin: self.chip.get_line(self.left_motor_dir_pin),
            self.left_motor_enable_pin: self.chip.get_line(self.left_motor_enable_pin),
            self.right_motor_step_pin: self.chip.get_line(self.right_motor_step_pin),
            self.right_motor_dir_pin: self.chip.get_line(self.right_motor_dir_pin),
            self.right_motor_enable_pin: self.chip.get_line(self.right_motor_enable_pin),
        }

        # Configure GPIO lines as outputs
        for pin, line in self.lines.items():
            line.request(consumer="StepperMotorNode", type=gpiod.LINE_REQ_DIR_OUT)

        # Enable both motors by default
        self.lines[self.left_motor_enable_pin].set_value(1)  # Disable motor initially
        self.lines[self.right_motor_enable_pin].set_value(1)  # Disable motor initially

        # Set up subscriptions
        self.create_subscription(Float32, 'left_track_delay', self.left_track_callback, 10)
        self.create_subscription(Float32, 'right_track_delay', self.right_track_callback, 10)

        # Setup encoder angle publishers
        self.left_motor_angle_pub = self.create_publisher(Float32, 'left_motor_angle', 10)
        self.right_motor_angle_pub = self.create_publisher(Float32, 'right_motor_angle', 10)

        # Initialize degree counters
        self.left_motor_degrees = 0.0
        self.right_motor_degrees = 0.0
        self.left_pulse_count = 0
        self.right_pulse_count = 0

        # Lock for thread safety
        self.lock = threading.Lock()

        # Initialize motor speeds
        self.left_speed = 0
        self.right_speed = 0

        # Create threads for motor control
        self.left_motor_thread = threading.Thread(target=self.run_motor, args=(
            self.left_motor_step_pin, self.left_motor_dir_pin, self.left_motor_enable_pin, lambda: self.left_speed, "Left Motor", True))
        self.right_motor_thread = threading.Thread(target=self.run_motor, args=(
            self.right_motor_step_pin, self.right_motor_dir_pin, self.right_motor_enable_pin, lambda: self.right_speed, "Right Motor", False))

        self.left_encoder_thread = threading.Thread(target=self.encoder_monitor, args=(self.left_motor_encoder_pin, "left"))
        self.right_encoder_thread = threading.Thread(target=self.encoder_monitor, args=(self.right_motor_encoder_pin, "right"))

        self.left_motor_thread.daemon = True
        self.right_motor_thread.daemon = True

        self.left_encoder_thread.daemon = True
        self.right_encoder_thread.daemon = True

        self.left_motor_thread.start()
        self.right_motor_thread.start()

        self.left_encoder_thread.start()
        self.right_encoder_thread.start()

        # Timer to publish updates at a regular interval
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.update_angles)

        self.get_logger().info("Stepper motor node started.")

    def left_track_callback(self, msg):
        self.left_speed = msg.data
        self.get_logger().debug(f"Left track speed updated: {self.left_speed}")

    def right_track_callback(self, msg):
        self.right_speed = msg.data
        self.get_logger().debug(f"Right track speed updated: {self.right_speed}")

    def run_motor(self, step_pin, dir_pin, enable_pin, speed_getter, motor_name, is_left):
        step_counter = 0

        while True:
            speed = speed_getter()
            if speed == 0:
                # Disable the motor if speed is 0
                self.lines[enable_pin].set_value(1)
                # self.get_logger().info(f"{motor_name} disabled (speed = 0).")
                time.sleep(0.1)  # Sleep briefly to prevent CPU overload
            else:
                # Enable the motor
                self.lines[enable_pin].set_value(0)
                # self.get_logger().info(f"{motor_name} enabled (speed = {speed}).")

                # Set direction
                direction = 1 if speed > 0 else 0
                self.lines[dir_pin].set_value(direction)

                # Calculate delay between steps (speed control)
                delay = max(abs(speed), 1) / 100000  # Prevent division by zero

                # Step the motor
                self.lines[step_pin].set_value(1)
                time.sleep(delay)  # Half the delay for HIGH signal
                self.lines[step_pin].set_value(0)
                time.sleep(delay)  # Half the delay for LOW signal


    def update_angles(self):
        self.left_motor_angle_pub.publish(Float32(data=self.left_motor_degrees))
        self.right_motor_angle_pub.publish(Float32(data=self.right_motor_degrees))

    def encoder_monitor(self, line_offset, name):

        line = self.chip.get_line(line_offset)
        line.request(consumer=f"{name}-encoder", type=gpiod.LINE_REQ_EV_FALLING_EDGE)

        while True:
            if line.event_wait(5):
                event = line.event_read()
                if event.type == gpiod.LineEvent.FALLING_EDGE:
                    with self.lock:
                        if name == "left":
                            if self.left_speed >= 0:
                                self.left_pulse_count += 1
                            else:
                                self.left_pulse_count -= 1
                            degrees = self.left_pulse_count * DEGREES_PER_PULSE
                            self.left_motor_degrees = degrees % 360

                        elif name == "right":
                            if self.right_speed >= 0:
                                self.right_pulse_count += 1
                            else:
                                self.right_pulse_count -= 1
                            degrees = self.right_pulse_count * DEGREES_PER_PULSE
                            self.right_motor_degrees = degrees % 360
            else:
                self.get_logger().debug(f"[{name.upper()}] No pulse in 5 seconds...")


    def destroy_node(self):
        # Release GPIO lines on shutdown
        for line in self.lines.values():
            line.release()
        self.get_logger().info("GPIO lines released.")
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
