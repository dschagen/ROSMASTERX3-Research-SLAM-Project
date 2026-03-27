#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import cv2
import numpy as np
from collections import deque


class HallwayDetector(Node):
    def __init__(self):
        super().__init__('hallway_detector')

        self.bridge = CvBridge()

        # Use sensor-data QoS for camera topics
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

        # Publish hallway bias for follower node
        self.direction_pub = self.create_publisher(String, '/hallway_direction', 10)

        # Debug / state
        self.received_first_frame = False
        self.direction_history = deque(maxlen=7)
        self.last_reported_direction = 'NONE'

        # Tunable parameters
        self.edge_low = 60
        self.edge_high = 160
        self.center_bias = 0.90
        self.diff_threshold = 150
        self.show_debug = True

        self.get_logger().info(
            'Hallway detector v5 started. Using full image from /camera/color/image_raw'
        )

    def image_callback(self, msg: Image) -> None:
        if not self.received_first_frame:
            self.get_logger().info('Received first camera frame.')
            self.received_first_frame = True

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return

        height, width, _ = frame.shape

        # Full-frame processing
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)
        edges = cv2.Canny(blurred, self.edge_low, self.edge_high)

        third = width // 3

        # Full-image thirds
        left_full = edges[:, :third]
        center_full = edges[:, third:2 * third]
        right_full = edges[:, 2 * third:]

        # Lower-half thirds get extra weight
        lower_start = height // 2
        left_lower = edges[lower_start:, :third]
        center_lower = edges[lower_start:, third:2 * third]
        right_lower = edges[lower_start:, 2 * third:]

        left_score = (
            int(np.count_nonzero(left_full)) +
            2 * int(np.count_nonzero(left_lower))
        )
        center_score = (
            int(np.count_nonzero(center_full)) +
            2 * int(np.count_nonzero(center_lower))
        )
        right_score = (
            int(np.count_nonzero(right_full)) +
            2 * int(np.count_nonzero(right_lower))
        )

        adjusted_center = int(center_score * self.center_bias)

        # Lower score means more open space
        scores = {
            'LEFT': left_score,
            'CENTER': adjusted_center,
            'RIGHT': right_score
        }

        raw_direction = min(scores, key=scores.get)

        # If left/right are too similar, prefer center
        lr_diff = abs(left_score - right_score)
        if lr_diff < self.diff_threshold:
            direction = 'CENTER'
        else:
            direction = raw_direction

        # Flip labels to match your camera orientation
        if direction == 'LEFT':
            direction = 'RIGHT'
        elif direction == 'RIGHT':
            direction = 'LEFT'

        if raw_direction == 'LEFT':
            raw_direction = 'RIGHT'
        elif raw_direction == 'RIGHT':
            raw_direction = 'LEFT'

        # Smooth the output over several frames
        self.direction_history.append(direction)
        stable_direction = self.majority_vote(self.direction_history)

        # Publish direction every callback
        msg_out = String()
        msg_out.data = stable_direction
        self.direction_pub.publish(msg_out)

        if stable_direction != self.last_reported_direction:
            self.get_logger().info(
                f'Stable path estimate: {stable_direction} | '
                f'L={left_score} C={center_score} R={right_score} '
                f'raw={raw_direction} lr_diff={lr_diff}'
            )
            self.last_reported_direction = stable_direction

        if self.show_debug:
            # Show original image with overlays
            debug = frame.copy()

            # Region lines
            cv2.line(debug, (third, 0), (third, height), (0, 255, 0), 2)
            cv2.line(debug, (2 * third, 0), (2 * third, height), (0, 255, 0), 2)

            # Lower-half reference line
            cv2.line(debug, (0, lower_start), (width, lower_start), (255, 0, 0), 2)

            # Labels
            cv2.putText(
                debug,
                f'Stable: {stable_direction}',
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 255),
                2
            )

            cv2.putText(
                debug,
                f'L={left_score} C={center_score} R={right_score}',
                (20, 75),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (255, 255, 0),
                2
            )

            cv2.putText(
                debug,
                'LEFT',
                (20, height - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )

            cv2.putText(
                debug,
                'CENTER',
                (third + 20, height - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )

            cv2.putText(
                debug,
                'RIGHT',
                (2 * third + 20, height - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )

            try:
                cv2.imshow('Hallway Detector Debug', debug)
                cv2.waitKey(1)
            except Exception as e:
                self.get_logger().warn(f'Debug window failed: {e}')
                self.show_debug = False

    @staticmethod
    def majority_vote(history: deque) -> str:
        if not history:
            return 'NONE'

        counts = {}
        for item in history:
            counts[item] = counts.get(item, 0) + 1

        return max(counts, key=counts.get)


def main(args=None):
    rclpy.init(args=args)
    node = HallwayDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
