import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from time import time  # Import time to track timestamps

DEAD_ZONE = 0.1

class XboxControl(Node):
    def __init__(self):
        super().__init__('xbox_control')

        # Subscribe to the Xbox controller's Joy topic
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Publishers for left and right track velocities
        self.left_track_pub = self.create_publisher(Float32, 'left_track_delay', 10)
        self.right_track_pub = self.create_publisher(Float32, 'right_track_delay', 10)

        # Rate limit time in seconds (default to 1 second)
        self.rate_limit_time = 0.02  # Set this value as needed or make it configurable
        self.last_publish_time = 0.0  # Keep track of the last published time (initialized to 0)

        self.last_left = 0.0
        self.last_right = 0.0

        # Variables to track the B button state and speed factor
        self.b_button_last_state = 0  # Last state of the B button (0 = not pressed, 1 = pressed)
        self.speed_multiplier = 1.0  # Default to normal speed

    def joy_callback(self, msg):
        # Check if enough time has passed since the last publish
        current_time = time()
        if current_time - self.last_publish_time < self.rate_limit_time:
            return  # If not enough time has passed, skip this callback

        # Update the last publish time
        self.last_publish_time = current_time

        # Detect B button state (index 1 for "B" button in most Xbox controllers)
        # self.get_logger().info(msg.buttons)
        b_button_state = msg.buttons[1]  # 0 = not pressed, 1 = pressed
        if b_button_state != self.b_button_last_state:
            # If the B button state has changed (toggled)
            self.b_button_last_state = b_button_state
            if b_button_state == 1:  # On button press, toggle speed multiplier
                self.speed_multiplier = 0.5 if self.speed_multiplier == 1.0 else 1.0
                self.get_logger().info(f"B button toggled. Speed multiplier set to {self.speed_multiplier}")

        # Map left stick's vertical axis to left track, and right stick's vertical axis to right track
        left_track_speed = msg.axes[1]  # Left stick's vertical axis
        right_track_speed = msg.axes[4]  # Right stick's vertical axis

        if abs(left_track_speed) < DEAD_ZONE:
            left_track_speed = 0.0
        
        if abs(right_track_speed) < DEAD_ZONE:
            right_track_speed = 0.0

        # Apply the speed multiplier
        left_track_speed *= self.speed_multiplier
        right_track_speed *= self.speed_multiplier

        # Publish left track speed if it has changed
        if self.last_left != left_track_speed:
            self.last_left = left_track_speed
            left_delay = self.map_to_delay(left_track_speed)
            self.left_track_pub.publish(Float32(data=left_delay))

        # Publish right track speed if it has changed
        if self.last_right != right_track_speed:
            self.last_right = right_track_speed
            right_delay = self.map_to_delay(right_track_speed)
            self.right_track_pub.publish(Float32(data=right_delay))
        
        self.get_logger().info(f"left: {self.last_left}, right: {self.last_right}")

    def map_to_delay(self, speed_factor, min_delay=50, max_delay=100):
        """
        Maps a speed factor between -1.0 and 1.0 to a delay for a stepper motor.
        
        Parameters:
            speed_factor (float): Speed factor between -1.0 and 1.0. 
                                0.0 means stopped (max delay), 1.0 means full forward speed (min delay),
                                -1.0 means full reverse speed (negative min delay).
            min_delay (int): The minimum delay in milliseconds at full speed (default: 100 ms).
            max_delay (int): The maximum delay in milliseconds at stop (default: 500 ms).
            
        Returns:
            float: The computed delay in milliseconds. Negative values represent reverse motion.
        """
        if speed_factor < -1.0 or speed_factor > 1.0:
            self.get_logger().error("speed_factor must be between -1.0 and 1.0")

            return 0.0
        
        if speed_factor == 0.0:
            return 0.0  # No motion

        # Use the absolute value of speed_factor to compute the delay
        delay = max_delay - (abs(speed_factor) * (max_delay - min_delay))

        # Return the delay with the same sign as speed_factor
        return delay if speed_factor > 0 else -delay


def main(args=None):
    rclpy.init(args=args)
    node = XboxControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
