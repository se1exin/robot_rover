import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from math import pi
from .fm1828_driver import FM1828Driver, NUM_READINGS_PER_REV


class FM1828Publisher(Node):
    def __init__(self):
        super().__init__("fm1828_publisher")

        self.publisher_ = self.create_publisher(LaserScan, "scan", 10)
        self.timer_period = 0.01

        # Create driver with callback
        self.driver = FM1828Driver(data_callback=self.publish_laser_scan)

        # Start reading data continuously
        self.create_timer(self.timer_period, self.driver.read_serial_data)

    def publish_laser_scan(self, scan_data):
        """Publishes the LaserScan message with the collected range and intensity data."""
        try:
            ranges, intensities = scan_data
            angle_increment = 2 * pi / NUM_READINGS_PER_REV

            scan_msg = LaserScan()
            scan_msg.header.stamp = self.get_clock().now().to_msg()
            scan_msg.header.frame_id = "laser_frame"

            scan_msg.angle_min = -pi
            scan_msg.angle_max = pi
            scan_msg.angle_increment = angle_increment
            scan_msg.time_increment = 0.0
            scan_msg.scan_time = self.timer_period
            scan_msg.range_min = 0.1
            scan_msg.range_max = 15.0

            scan_msg.ranges = ranges
            scan_msg.intensities = intensities

            self.publisher_.publish(scan_msg)
        except Exception as e:
            self.get_logger().error(f"Error while publishing LaserScan: {e}")


def main(args=None):
    rclpy.init(args=args)

    fm1828_publisher = FM1828Publisher()

    try:
        rclpy.spin(fm1828_publisher)
    except KeyboardInterrupt:
        fm1828_publisher.get_logger().info("KeyboardInterrupt detected, shutting down.")
    finally:
        fm1828_publisher.driver.close()
        fm1828_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
