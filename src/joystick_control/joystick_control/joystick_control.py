import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import time

SIGNIFICANT_CHANGE_THRESHOLD = 0.05  # Minimum change needed to update cmd_vel
STOP_THRESHOLD = 0.05  # Threshold to consider input stopped
MAX_HOLD_TIME = 0.05  # Maximum time to hold the same value before forcing a resend

class JoystickControlNode(Node):
    def __init__(self):
        super().__init__('joystick_control')

        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.declare_parameter('linear_speed_scale', 1.0)
        self.declare_parameter('angular_speed_scale', 1.0)
        self.declare_parameter('joystick_deadzone', 0.1)

        self.previous_linear_speed = 0.0
        self.previous_angular_speed = 0.0
        self.last_update_time = time.time()

    def apply_deadzone(self, value, deadzone):
        if abs(value) < deadzone:
            return 0.0
        return (value - deadzone) / (1.0 - deadzone) if value > 0.0 else (value + deadzone) / (1.0 - deadzone)

    def joy_callback(self, msg):
        raw_linear_speed = msg.axes[1]
        raw_angular_speed = msg.axes[3]

        deadzone = self.get_parameter('joystick_deadzone').get_parameter_value().double_value
        linear_speed = self.apply_deadzone(raw_linear_speed, deadzone)
        angular_speed = self.apply_deadzone(raw_angular_speed, deadzone)

        linear_scale = self.get_parameter('linear_speed_scale').get_parameter_value().double_value
        angular_scale = self.get_parameter('angular_speed_scale').get_parameter_value().double_value

        scaled_linear_speed = linear_speed * linear_scale
        scaled_angular_speed = angular_speed * angular_scale

        linear_difference = abs(scaled_linear_speed - self.previous_linear_speed)
        angular_difference = abs(scaled_angular_speed - self.previous_angular_speed)

        current_time = time.time()
        time_since_last_update = current_time - self.last_update_time

        if (
            linear_difference > SIGNIFICANT_CHANGE_THRESHOLD
            or angular_difference > SIGNIFICANT_CHANGE_THRESHOLD
            or abs(scaled_linear_speed) < STOP_THRESHOLD
            and abs(scaled_angular_speed) < STOP_THRESHOLD
            or time_since_last_update >= MAX_HOLD_TIME
        ):
            twist_msg = Twist()
            twist_msg.linear.x = scaled_linear_speed
            twist_msg.angular.z = scaled_angular_speed
            self.cmd_vel_pub.publish(twist_msg)

            self.previous_linear_speed = scaled_linear_speed
            self.previous_angular_speed = scaled_angular_speed
            self.last_update_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = JoystickControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
