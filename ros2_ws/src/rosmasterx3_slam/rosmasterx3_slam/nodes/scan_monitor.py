import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry


class ScanMonitor(Node):

    def __init__(self):
        super().__init__('scan_monitor')

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10)

        self.get_logger().info("Scan monitor node started")

    def scan_callback(self, msg):
        self.get_logger().info("Received LiDAR scan")

    def odom_callback(self, msg):
        self.get_logger().info("Received odometry")

    def imu_callback(self, msg):
        self.get_logger().info("Received IMU data")


def main(args=None):
    rclpy.init(args=args)
    node = ScanMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
