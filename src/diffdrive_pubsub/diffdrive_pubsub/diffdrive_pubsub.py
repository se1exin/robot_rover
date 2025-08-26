import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math
import time


class DiffDrivePubSub(Node):
    def __init__(self):
        super().__init__('diffdrive_pubsub_node')

        # Parameters for the robot, such as wheel separation and radius
        self.declare_parameter('wheel_separation', 0.35)
        self.declare_parameter('wheel_radius', 0.038)

        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        
        # Stepper motor: 17HS13-0404S-PG5
        self.step_angle = 0.35  # degrees per step
        self.gear_ratio = 5.18  # gearbox ratio
        self.wheel_diameter = self.wheel_radius * 2
        self.wheel_circumference = self.wheel_diameter * 3.14159  # Pi * diameter
        self.steps_per_revolution = 360 / self.step_angle  # steps needed for one full motor revolution

        # Publishers for wheel velocities
        self.left_wheel_pub = self.create_publisher(Float32, 'left_track_delay', 10)
        self.right_wheel_pub = self.create_publisher(Float32, 'right_track_delay', 10)

        # Subscribe to motor angle topics
        self.create_subscription(Float32, 'left_motor_angle', self.left_angle_callback, 10)
        self.create_subscription(Float32, 'right_motor_angle', self.right_angle_callback, 10)

        # State to track previous angle and time
        self.left_last_angle = None
        self.left_last_time = None
        self.right_last_angle = None
        self.right_last_time = None
        self.left_velocity = 0.0
        self.right_velocity = 0.0

        # Publisher for joint states
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Publisher for odometry
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # Transform broadcaster for odometry
        self.odom_broadcaster = TransformBroadcaster(self)

        # Subscriber to the velocity command
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Initialize joint state
        self.joint_state = JointState()
        self.joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        self.joint_state.position = [0.0, 0.0]
        self.joint_state.velocity = [0.0, 0.0]

        # Odometry variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Timer to publish updates at a regular interval
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.update)

    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocities
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        motor_a, motor_b = self.map_cmd_vel_to_motor_speed_factors(linear_vel, angular_vel)

        left_delay = self.map_to_delay(motor_a)
        right_delay = self.map_to_delay(motor_b)

        # self.get_logger().info(f"left_delay: {left_delay}, right_delay: {right_delay}")

        # Publish the wheel velocities
        left_msg = Float32()
        right_msg = Float32()
        left_msg.data = left_delay
        right_msg.data = right_delay

        self.left_wheel_pub.publish(left_msg)
        self.right_wheel_pub.publish(right_msg)
    
    def map_cmd_vel_to_motor_speed_factors(self, x, y):
        # Ensure x and y are within the expected range [-1.0, 1.0]
        x = max(-1.0, min(1.0, x))
        y = max(-1.0, min(1.0, y))
        
        # Calculate motor speeds considering full stop and reverse at extreme Y values
        motor_a_speed = (x * (1 - abs(y))) - y
        motor_b_speed = (x * (1 - abs(y))) + y
        
        # Normalize the motor speeds to be within [-1.0, 1.0]
        max_speed = max(abs(motor_a_speed), abs(motor_b_speed), 1.0)
        motor_a_speed /= max_speed
        motor_b_speed /= max_speed

        return motor_a_speed, motor_b_speed
    

    def map_to_delay(self, speed_factor, min_delay=80, max_delay=220):
        """
        Maps a speed factor between -1.0 and 1.0 to a delay for a stepper motor.
        
        Parameters:
            speed_factor (float): Speed factor between -1.0 and 1.0. 
                                  0.0 means stopped (max delay), 1.0 means full forward speed (min delay),
                                  -1.0 means full reverse speed (negative min delay).
            min_delay (int): The minimum delay in milliseconds at full speed (default: 50 ms).
            max_delay (int): The maximum delay in milliseconds at stop (default: 100 ms).
            
        Returns:
            float: The computed delay in milliseconds. Negative values represent reverse motion.
        """
        if speed_factor < -1.0 or speed_factor > 1.0:
            self.get_logger().error(f"speed_factor must be between -1.0 and 1.0 (recieved {speed_factor})")
            return 0.0

        if speed_factor == 0.0:
            return 0.0  # No motion

        # Use the absolute value of speed_factor to compute the delay
        delay = max_delay - (abs(speed_factor) * (max_delay - min_delay))

        # Return the delay with the same sign as speed_factor
        return delay if speed_factor > 0 else -delay
    
    def left_angle_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds
        if self.left_last_time is not None and self.left_last_angle is not None:
            time_diff = (current_time - self.left_last_time) / 1e9  # Convert nanoseconds to seconds
            angle_diff = self.angle_difference(msg.data, self.left_last_angle)  # Calculate wrapped angle difference
            velocity = angle_diff / time_diff  # rad/s

            self.left_velocity = velocity
            self.joint_state.velocity = [self.left_velocity, self.right_velocity]
        
        self.left_last_angle = msg.data
        self.left_last_time = current_time

    def right_angle_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds
        if self.right_last_time is not None and self.right_last_angle is not None:
            time_diff = (current_time - self.right_last_time) / 1e9  # Convert nanoseconds to seconds
            angle_diff = self.angle_difference(msg.data, self.right_last_angle)  # Calculate wrapped angle difference
            velocity = angle_diff / time_diff  # rad/s

            self.right_velocity = velocity
            self.joint_state.velocity = [self.left_velocity, self.right_velocity]
        
        self.right_last_angle = msg.data
        self.right_last_time = current_time

    def angle_difference(self, current_angle, last_angle):
        """Calculate the minimum difference between two angles, handling wraparound."""
        diff = (current_angle - last_angle + 180) % 360 - 180
        return math.radians(diff)  # Convert difference from degrees to radians

    def update(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert nanoseconds to seconds

        # Calculate wheel angular velocity (rad/s) from motor velocity (rad/s)
        motor_v_left = self.joint_state.velocity[0]
        motor_v_right = self.joint_state.velocity[1]
        wheel_v_left = motor_v_left
        wheel_v_right = motor_v_right

        # Convert angular velocity to linear velocity
        v_left = wheel_v_left * self.wheel_radius * 0.98  # 0.9 accounts for slippage
        v_right = wheel_v_right * self.wheel_radius * 0.98  # 0.9 accounts for slippage

        v = (v_right + v_left) / 2.0
        omega = ((v_right - v_left) / self.wheel_separation)

        delta_x = v * math.cos(self.theta) * dt
        delta_y = v * math.sin(self.theta) * dt
        delta_theta = omega * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Create quaternion from yaw
        odom_quat = self.create_quaternion_from_yaw(self.theta)

        # Publish the transform over tf
        odom_trans = TransformStamped()

        odom_trans.header.stamp = current_time.to_msg()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'base_link'

        odom_trans.transform.translation.x = self.x
        odom_trans.transform.translation.y = self.y
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation = odom_quat

        # Send the transform
        self.odom_broadcaster.sendTransform(odom_trans)

        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'

        # Set the position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = odom_quat

        # Set the velocity
        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = omega

        # Publish odometry
        self.odom_pub.publish(odom)

        # Update joint positions by integrating velocity over time
        self.joint_state.position[0] += self.joint_state.velocity[0] * dt
        self.joint_state.position[1] += self.joint_state.velocity[1] * dt

        # Update timestamp
        self.joint_state.header.stamp = current_time.to_msg()

        # Publish the joint state
        self.joint_state_pub.publish(self.joint_state)

        # Update last time
        self.last_time = current_time

    def create_quaternion_from_yaw(self, yaw):
        """Create a quaternion from a yaw angle."""
        half_yaw = yaw * 0.5
        quat = TransformStamped()
        quat.transform.rotation.x = 0.0
        quat.transform.rotation.y = 0.0
        quat.transform.rotation.z = math.sin(half_yaw)
        quat.transform.rotation.w = math.cos(half_yaw)
        return quat.transform.rotation


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
