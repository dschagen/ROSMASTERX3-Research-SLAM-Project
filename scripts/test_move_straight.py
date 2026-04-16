#!/usr/bin/env python3
"""
Quick movement test — run this to find which linear.x sign drives the robot forward.
Press Ctrl+C to stop.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


LINEAR_X = 0.10   # change to -0.10 if robot goes backwards


class MoveStraight(Node):
    def __init__(self):
        super().__init__('move_straight_test')
        self.pub = self.create_publisher(Twist, '/cmd_vel_safe', 10)
        self.timer = self.create_timer(0.05, self.publish)
        self.get_logger().info(f'Publishing linear.x={LINEAR_X} — Ctrl+C to stop')

    def publish(self):
        cmd = Twist()
        cmd.linear.x = float(LINEAR_X)
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = MoveStraight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        for _ in range(20):
            node.pub.publish(stop)
            time.sleep(0.02)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
