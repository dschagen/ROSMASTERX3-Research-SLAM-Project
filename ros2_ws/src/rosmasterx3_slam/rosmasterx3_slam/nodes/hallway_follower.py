#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String


class HallwayFollower(Node):
    def __init__(self):
        super().__init__('hallway_follower')

        self.declare_parameter('linear_speed_max', 0.12)
        self.declare_parameter('linear_speed_min', 0.04)
        self.declare_parameter('angular_speed_max', 0.6)
        self.declare_parameter('turn_gain', 0.85)
        self.declare_parameter('pid_kp', 0.55)
        self.declare_parameter('pid_ki', 0.02)
        self.declare_parameter('pid_kd', 0.08)
        self.declare_parameter('pid_integral_max', 0.35)
        self.declare_parameter('adaptive_turn_scale', 0.65)
        self.declare_parameter('front_angle_min_deg', -40.0)
        self.declare_parameter('front_angle_max_deg', 40.0)
        self.declare_parameter('stop_distance_m', 0.22)
        self.declare_parameter('slow_distance_m', 0.55)
        self.declare_parameter('lidar_timeout_sec', 0.5)
        self.declare_parameter('camera_timeout_sec', 0.45)
        self.declare_parameter('require_lidar_before_move', False)
        self.declare_parameter('control_period_sec', 0.05)
        self.declare_parameter('discrete_fallback_threshold', 0.06)

        self._linear_max = self.get_parameter('linear_speed_max').value
        self._linear_min = self.get_parameter('linear_speed_min').value
        self._angular_max = self.get_parameter('angular_speed_max').value
        self._turn_gain = self.get_parameter('turn_gain').value
        self._kp = self.get_parameter('pid_kp').value
        self._ki = self.get_parameter('pid_ki').value
        self._kd = self.get_parameter('pid_kd').value
        self._i_max = self.get_parameter('pid_integral_max').value
        self._adaptive_scale = self.get_parameter('adaptive_turn_scale').value
        self._front_min_rad = math.radians(
            self.get_parameter('front_angle_min_deg').value
        )
        self._front_max_rad = math.radians(
            self.get_parameter('front_angle_max_deg').value
        )
        self._stop_d = self.get_parameter('stop_distance_m').value
        self._slow_d = self.get_parameter('slow_distance_m').value
        self._lidar_timeout = self.get_parameter('lidar_timeout_sec').value
        self._camera_timeout = self.get_parameter('camera_timeout_sec').value
        self._require_lidar = self.get_parameter('require_lidar_before_move').value
        period = float(self.get_parameter('control_period_sec').value)
        self._fb_thresh = float(self.get_parameter('discrete_fallback_threshold').value)

        self.create_subscription(
            Float32,
            '/hallway_steering_bias',
            self.bias_callback,
            10,
        )
        self.create_subscription(
            String,
            '/hallway_direction',
            self.direction_callback,
            10,
        )
        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10,
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_safe', 10)

        self.latest_direction = 'NONE'
        self.steering_bias = 0.0
        self.bias_prev = 0.0
        self.bias_prev_time = None
        self._pid_integral = 0.0

        self._forward_range_m = float('nan')
        self._scan_received = False
        self._last_scan_time = None
        self._last_camera_time = None

        self.timer = self.create_timer(period, self.publish_command)

        self.get_logger().info(
            'Hallway follower: PID + adaptive speed + LiDAR margins + watchdog. '
            'Subscribes /hallway_steering_bias, /hallway_direction, /scan'
        )

    def bias_callback(self, msg: Float32) -> None:
        self.steering_bias = float(msg.data)
        self._last_camera_time = time.monotonic()

    def direction_callback(self, msg: String) -> None:
        self.latest_direction = msg.data

    def scan_callback(self, msg: LaserScan) -> None:
        self._forward_range_m = self._min_range_in_front(msg)
        self._scan_received = True
        self._last_scan_time = time.monotonic()

    def _min_range_in_front(self, scan: LaserScan) -> float:
        amin = scan.angle_min
        ainc = scan.angle_increment
        n = len(scan.ranges)
        rmin = float('inf')
        for i in range(n):
            ang = amin + i * ainc
            if ang < self._front_min_rad or ang > self._front_max_rad:
                continue
            r = scan.ranges[i]
            if math.isnan(r) or math.isinf(r):
                continue
            if r < scan.range_min or r > scan.range_max:
                continue
            rmin = min(rmin, r)
        return rmin if rmin != float('inf') else float('nan')

    def _reset_pid(self) -> None:
        self._pid_integral = 0.0
        self.bias_prev = 0.0
        self.bias_prev_time = None

    def _discrete_bias_fallback(self) -> float:
        if self.latest_direction == 'LEFT':
            return 0.85
        if self.latest_direction == 'RIGHT':
            return -0.85
        return 0.0

    def _pid_step(self, bias: float, dt: float) -> float:
        err = bias
        self._pid_integral = float(
            max(-self._i_max, min(self._i_max, self._pid_integral + err * dt))
        )
        d_term = (bias - self.bias_prev) / dt if dt > 1e-6 else 0.0
        out = self._kp * err + self._ki * self._pid_integral + self._kd * d_term
        self.bias_prev = bias
        return max(-self._angular_max, min(self._angular_max, out * self._turn_gain))

    def publish_command(self) -> None:
        now = time.monotonic()
        cmd = Twist()

        # Minimal gating: only require first camera frame
        if self._last_camera_time is None:
            return

        bias = self.steering_bias
        if (
            abs(bias) < self._fb_thresh
            and self.latest_direction in ('LEFT', 'RIGHT', 'CENTER')
        ):
            bias = self._discrete_bias_fallback()

        if self.bias_prev_time is None:
            dt = 0.05
        else:
            dt = max(1e-3, now - self.bias_prev_time)
        self.bias_prev_time = now

        angular_z = self._pid_step(bias, dt)

        turn_mag = abs(angular_z) / max(self._angular_max, 1e-6)
        bias_mag = abs(bias)
        speed_factor = 1.0 - self._adaptive_scale * max(turn_mag, bias_mag)
        speed_factor = max(0.0, min(1.0, speed_factor))

        linear_x = self._linear_min + (self._linear_max - self._linear_min) * speed_factor

        lidar_scale = 1.0
        if not math.isnan(self._forward_range_m):
            if self._forward_range_m <= self._stop_d:
                linear_x = 0.0
                angular_z = 0.0
                self._reset_pid()
            elif self._forward_range_m < self._slow_d:
                lidar_scale = max(
                    0.0,
                    (self._forward_range_m - self._stop_d)
                    / max(self._slow_d - self._stop_d, 1e-6),
                )
                linear_x *= lidar_scale

        if self.latest_direction == 'NONE':
            linear_x = 0.0
            angular_z = 0.0

        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
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
