import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import time

SIGNIFICANT_CHANGE_THRESHOLD = 0.05  # Minimum change needed to update cmd_vel
STOP_THRESHOLD = 0.01  # Threshold to consider input stopped
MAX_HOLD_TIME = 0.5  # Maximum time to hold the same value before forcing a resend

class JoystickControlNode(Node):
    def __init__(self):
        super().__init__('joystick_control')

        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.declare_parameter('linear_speed_scale', 1.0)  # Scaling for forward/backward speed
        self.declare_parameter('angular_speed_scale', 1.0)  # Scaling for turning speed
        self.declare_parameter('joystick_deadzone', 0.1)  # Deadzone to ignore small joystick inputs

        self.smoothed_linear_speed = 0.0
        self.smoothed_angular_speed = 0.0
        self.previous_linear_speed = 0.0
        self.previous_angular_speed = 0.0
        self.last_update_time = time.time()

    def apply_deadzone(self, value, deadzone):
        # Ignore small joystick inputs (inside the deadzone)
        if abs(value) < deadzone:
            return 0.0
        return (value - deadzone) / (1.0 - deadzone) if value > 0.0 else (value + deadzone) / (1.0 - deadzone)

    def joy_callback(self, msg):
        # Read joystick axes for linear (forward/backward) and angular (left/right) motion
        raw_linear_speed = msg.axes[1]  # Left stick vertical axis
        raw_angular_speed = msg.axes[3]  # Right stick horizontal axis

        # Apply deadzone to filter small input noise
        deadzone = self.get_parameter('joystick_deadzone').get_parameter_value().double_value
        linear_speed = self.apply_deadzone(raw_linear_speed, deadzone)
        angular_speed = self.apply_deadzone(raw_angular_speed, deadzone)

        # Apply scaling factors for linear and angular speeds
        linear_scale = self.get_parameter('linear_speed_scale').get_parameter_value().double_value
        angular_scale = self.get_parameter('angular_speed_scale').get_parameter_value().double_value

        scaled_linear_speed = linear_speed * linear_scale
        scaled_angular_speed = angular_speed * angular_scale

        # Differential steering: calculate left and right wheel speeds
        left_speed = scaled_linear_speed - scaled_angular_speed
        right_speed = scaled_linear_speed + scaled_angular_speed

        # Smooth the speeds using exponential smoothing
        alpha = 0.1  # Smoothing factor (can be adjusted)
        self.smoothed_linear_speed = alpha * scaled_linear_speed + (1 - alpha) * self.smoothed_linear_speed
        self.smoothed_angular_speed = alpha * scaled_angular_speed + (1 - alpha) * self.smoothed_angular_speed

        # Compute differences to detect significant changes
        linear_difference = abs(self.smoothed_linear_speed - self.previous_linear_speed)
        angular_difference = abs(self.smoothed_angular_speed - self.previous_angular_speed)

        current_time = time.time()
        time_since_last_update = current_time - self.last_update_time

        # Update cmd_vel topic if there is a significant change, a stop command, or the same value persists for too long
        if (
            linear_difference > SIGNIFICANT_CHANGE_THRESHOLD
            or angular_difference > SIGNIFICANT_CHANGE_THRESHOLD
            or abs(self.smoothed_linear_speed) < STOP_THRESHOLD
            and abs(self.smoothed_angular_speed) < STOP_THRESHOLD
            or time_since_last_update >= MAX_HOLD_TIME
        ):
            twist_msg = Twist()
            twist_msg.linear.x = (left_speed + right_speed) / 2  # Average linear speed
            twist_msg.angular.z = (right_speed - left_speed) / 2  # Angular difference for turning
            self.cmd_vel_pub.publish(twist_msg)

            # Store the last speeds and update time
            self.previous_linear_speed = self.smoothed_linear_speed
            self.previous_angular_speed = self.smoothed_angular_speed
            self.last_update_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = JoystickControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
