import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class DiffDrivePubSub(Node):
    def __init__(self):
        super().__init__('diffdrive_pubsub_node')

        # Parameters for the robot, such as wheel separation and radius
        # self.declare_parameter('wheel_separation', 0.245)
        # self.declare_parameter('wheel_radius', 0.0412)
        self.declare_parameter('wheel_separation', 1.0)
        self.declare_parameter('wheel_radius', 1.0)

        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value

        # Publishers for wheel velocities
        self.left_wheel_pub = self.create_publisher(Float32, 'left_track_delay', 10)
        self.right_wheel_pub = self.create_publisher(Float32, 'right_track_delay', 10)

        # Subscriber to the velocity command
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocities
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Compute the wheel velocities
        left_wheel_velocity = (linear_vel - angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
        right_wheel_velocity = (linear_vel + angular_vel * self.wheel_separation / 2.0) / self.wheel_radius

        left_delay = self.map_to_delay(left_wheel_velocity / 15)
        right_delay = self.map_to_delay(right_wheel_velocity / 15)

        # self.get_logger().info(f"{left_wheel_velocity} {left_wheel_velocity / 15}")
        # Publish the wheel velocities
        left_msg = Float32()
        right_msg = Float32()
        left_msg.data = left_delay
        right_msg.data = right_delay

        self.left_wheel_pub.publish(left_msg)
        self.right_wheel_pub.publish(right_msg)

    
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
    node = DiffDrivePubSub()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()