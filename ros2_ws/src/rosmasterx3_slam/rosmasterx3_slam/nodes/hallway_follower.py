#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist


class HallwayFollower(Node):
    def __init__(self):
        super().__init__('hallway_follower')

        self.subscription = self.create_subscription(
            String,
            '/hallway_direction',
            self.direction_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_safe', 10)

        self.latest_direction = 'NONE'

        # Publish commands on a timer so motion is continuous
        self.timer = self.create_timer(0.1, self.publish_command)

        self.get_logger().info('Hallway follower started. Listening to /hallway_direction')

    def direction_callback(self, msg: String) -> None:
        self.latest_direction = msg.data

    def publish_command(self) -> None:
        cmd = Twist()

        if self.latest_direction == 'CENTER':
            cmd.linear.x = 0.02
            cmd.angular.z = 0.0

        elif self.latest_direction == 'LEFT':
            cmd.linear.x = 0.015
            cmd.angular.z = 0.08

        elif self.latest_direction == 'RIGHT':
            cmd.linear.x = 0.015
            cmd.angular.z = -0.08

        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = HallwayFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_cmd = Twist()
        node.cmd_pub.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
