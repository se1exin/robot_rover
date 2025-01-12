import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import time

RESEND_INTERVAL = 0.1  # Resend the message every 1 second, even if values haven't changed


class JoystickControlNode(Node):
    def __init__(self):
        super().__init__('joystick_control')

        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.declare_parameter('linear_speed_scale', 1.0)
        self.declare_parameter('angular_speed_scale', 1.0)
        self.declare_parameter('joystick_deadzone', 0.1)

        # Variables to track the previous joystick values and time of last message
        self.previous_linear_speed = 0.0
        self.previous_angular_speed = 0.0
        self.last_publish_time = time.time()

    def apply_deadzone(self, value, deadzone):
        if abs(value) < deadzone:
            return 0.0
        return (value - deadzone) / (1.0 - deadzone) if value > 0.0 else (value + deadzone) / (1.0 - deadzone)

    def joy_callback(self, msg):
        # Get the current time
        current_time = time.time()

        # Read joystick axes for linear (forward/backward) and angular (left/right) motion
        raw_linear_speed = msg.axes[1]
        raw_angular_speed = msg.axes[3]

        # Apply deadzone to filter out small input noise
        deadzone = self.get_parameter('joystick_deadzone').get_parameter_value().double_value
        linear_speed = self.apply_deadzone(raw_linear_speed, deadzone)
        angular_speed = self.apply_deadzone(raw_angular_speed, deadzone)

        # Apply scaling factors to adjust the range of linear and angular speeds
        linear_scale = self.get_parameter('linear_speed_scale').get_parameter_value().double_value
        angular_scale = self.get_parameter('angular_speed_scale').get_parameter_value().double_value

        scaled_linear_speed = linear_speed * linear_scale
        scaled_angular_speed = angular_speed * angular_scale

        # Check if the joystick values have changed or if the resend interval has passed
        if (
            scaled_linear_speed != self.previous_linear_speed
            or scaled_angular_speed != self.previous_angular_speed
            or current_time - self.last_publish_time >= RESEND_INTERVAL
        ):
            # Create and publish the Twist message
            twist_msg = Twist()
            twist_msg.linear.x = scaled_linear_speed
            twist_msg.angular.z = scaled_angular_speed
            self.cmd_vel_pub.publish(twist_msg)

            # Update previous values and the last publish time
            self.previous_linear_speed = scaled_linear_speed
            self.previous_angular_speed = scaled_angular_speed
            self.last_publish_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = JoystickControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
