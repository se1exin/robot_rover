import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

class XboxControl(Node):
    def __init__(self):
        super().__init__('xbox_control')
        # Subscribe to the Xbox controller's Joy topic
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        # Publishers for left and right track velocities
        self.left_track_pub = self.create_publisher(Float32, 'left_track', 10)
        self.right_track_pub = self.create_publisher(Float32, 'right_track', 10)

    def joy_callback(self, msg):
        # Map left stick's vertical axis to left track, and right stick's vertical axis to right track
        left_track_speed = msg.axes[1]  # Left stick's vertical axis
        right_track_speed = msg.axes[4]  # Right stick's vertical axis

        # Publish the speeds
        self.left_track_pub.publish(Float32(data=left_track_speed))
        self.right_track_pub.publish(Float32(data=right_track_speed))

def main(args=None):
    rclpy.init(args=args)
    node = XboxControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
