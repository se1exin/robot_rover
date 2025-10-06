import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import time
import gpiod
import threading

# Constants
PULSES_PER_REV = 10
DEGREES_PER_PULSE = 360.0 / PULSES_PER_REV

class EncoderMonitorNode(Node):
    def __init__(self):
        super().__init__('encoder_monitor_node')
        self.last_pulse_time = 0
        self.min_pulse_interval = 0.005  # 5 ms debounce (adjust as needed)

        # Declare parameters
        self.declare_parameter('encoder_pin', 24)
        self.declare_parameter('angle_topic', 'motor_angle')
        self.declare_parameter('delay_topic', 'track_delay')  # e.g., 'left_track_delay'

        # Get parameters
        self.encoder_pin = self.get_parameter('encoder_pin').value
        self.angle_topic = self.get_parameter('angle_topic').value
        self.delay_topic = self.get_parameter('delay_topic').value

        # GPIO setup
        self.chip = gpiod.Chip('gpiochip0')
        self.line = self.chip.get_line(self.encoder_pin)
        self.line.request(consumer="encoder-monitor", type=gpiod.LINE_REQ_EV_BOTH_EDGES)

        self.get_logger().info(f"Encoder monitor started on GPIO {self.encoder_pin}, listening to '{self.delay_topic}'")

        # Initialize state
        self.pulse_count = 0
        self.degrees = 0.0
        self.direction = 1  # default direction

        # Publisher for angle
        self.angle_pub = self.create_publisher(Float32, self.angle_topic, 10)

        # Subscriber to delay topic to infer direction
        self.create_subscription(Float32, self.delay_topic, self.delay_callback, 10)

        # Start encoder monitoring thread
        self.encoder_thread = threading.Thread(target=self.encoder_loop)
        self.encoder_thread.daemon = True
        self.encoder_thread.start()

        # Timer to publish angle periodically
        self.create_timer(0.1, self.publish_angle)

    def delay_callback(self, msg):
        self.direction = 1 if msg.data >= 0 else -1

    def encoder_loop(self):
        while rclpy.ok():
            if self.line.event_wait(5):
                event = self.line.event_read()
                now = time.time()

                if (now - self.last_pulse_time) >= self.min_pulse_interval:
                    if event.type == gpiod.LineEvent.FALLING_EDGE:
                        self.pulse_count += self.direction
                        self.degrees = (self.pulse_count * DEGREES_PER_PULSE) % 360
                        self.last_pulse_time = now
                else:
                    self.get_logger().debug("Debounced extra pulse.")
            else:
                # self.get_logger().warn(f"No encoder pulses on GPIO {self.encoder_pin} in 5 seconds.")
                pass

    def publish_angle(self):
        msg = Float32()
        msg.data = self.degrees
        self.angle_pub.publish(msg)

    def destroy_node(self):
        self.line.release()
        self.get_logger().info("Encoder GPIO released.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EncoderMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Encoder monitor node interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
