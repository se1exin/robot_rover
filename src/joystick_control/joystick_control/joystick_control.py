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
        self.left_track_pub = self.create_publisher(Float32, 'left_track_delay', 10)
        self.right_track_pub = self.create_publisher(Float32, 'right_track_delay', 10)

    def joy_callback(self, msg):
        # Map left stick's vertical axis to left track, and right stick's vertical axis to right track
        left_track_speed = msg.axes[1]  # Left stick's vertical axis
        right_track_speed = msg.axes[4]  # Right stick's vertical axis

        # Joy values are between 0.0 and 1.0
        # We want to map this to delay for the stepper motors, with 0 being not moving and 1.0 being full speed
        left_delay =  self.map_to_delay(left_track_speed)
        right_delay =  self.map_to_delay(right_track_speed)

        self.get_logger().info(f"left: {left_delay}, {left_track_speed}")

        # Publish the speeds
        self.left_track_pub.publish(Float32(data=left_delay))
        self.right_track_pub.publish(Float32(data=right_delay))

    def map_to_delay(self, speed_factor, min_delay=300):
        """
        Maps a speed factor between 0.0 and 1.0 to a delay for a stepper motor.
        
        Parameters:
            speed_factor (float): Speed factor between 0.0 and 1.0. 
                                0.0 means stopped, 1.0 means full speed.
            min_delay (int): The minimum delay in milliseconds at full speed (default: 300 ms).
            
        Returns:
            float: The computed delay in milliseconds. Returns a very large delay for 0.0.
        """
        if speed_factor < -1.0 or speed_factor > 1.0:
            self.get_logger().error(f"speed_factor must be between 0.0 and 1.0  ({speed_factor})")
        
        if speed_factor == 0.0:
            return 0.0  # Infinite delay for stopped motor
        
        # Map the speed factor to the delay
        return min_delay / speed_factor

def main(args=None):
    rclpy.init(args=args)
    node = XboxControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
