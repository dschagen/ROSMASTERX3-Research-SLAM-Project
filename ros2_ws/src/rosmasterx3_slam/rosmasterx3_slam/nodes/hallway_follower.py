#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class HallwayFollower(Node):
    def __init__(self):
        super().__init__('hallway_follower')

        # --- speed ---
        self.declare_parameter('linear_speed_max', 0.10)
        self.declare_parameter('linear_speed_min', 0.05)
        self.declare_parameter('angular_speed_max', 0.35)
        self.declare_parameter('adaptive_turn_scale', 0.35)

        # --- PID (softened to prevent sway) ---
        self.declare_parameter('turn_gain', 0.45)
        self.declare_parameter('pid_kp', 0.25)
        self.declare_parameter('pid_ki', 0.008)
        self.declare_parameter('pid_kd', 0.04)
        self.declare_parameter('pid_integral_max', 0.20)

        # --- front LiDAR sector (widened) ---
        self.declare_parameter('front_angle_min_deg', -60.0)
        self.declare_parameter('front_angle_max_deg', 60.0)
        self.declare_parameter('stop_distance_m', 0.20)
        self.declare_parameter('slow_distance_m', 0.50)
        self.declare_parameter('wall_backoff_linear', -0.06)
        self.declare_parameter('wall_min_turn', 0.25)
        self.declare_parameter('wall_turn_boost', 1.5)

        # --- side-wall LiDAR sectors (new: keeps robot centered) ---
        self.declare_parameter('side_inner_deg', 30.0)
        self.declare_parameter('side_outer_deg', 150.0)
        self.declare_parameter('side_wall_gain', 0.35)
        self.declare_parameter('side_wall_threshold_m', 0.60)

        # --- misc ---
        self.declare_parameter('control_period_sec', 0.05)
        self.declare_parameter('diag_interval_sec', 3.0)

        self._linear_max = self.get_parameter('linear_speed_max').value
        self._linear_min = self.get_parameter('linear_speed_min').value
        self._angular_max = self.get_parameter('angular_speed_max').value
        self._adaptive_scale = self.get_parameter('adaptive_turn_scale').value
        self._turn_gain = self.get_parameter('turn_gain').value
        self._kp = self.get_parameter('pid_kp').value
        self._ki = self.get_parameter('pid_ki').value
        self._kd = self.get_parameter('pid_kd').value
        self._i_max = self.get_parameter('pid_integral_max').value

        self._front_min_rad = math.radians(self.get_parameter('front_angle_min_deg').value)
        self._front_max_rad = math.radians(self.get_parameter('front_angle_max_deg').value)
        self._stop_d = self.get_parameter('stop_distance_m').value
        self._slow_d = self.get_parameter('slow_distance_m').value
        self._wall_backoff = float(self.get_parameter('wall_backoff_linear').value)
        self._wall_min_turn = float(self.get_parameter('wall_min_turn').value)
        self._wall_turn_boost = float(self.get_parameter('wall_turn_boost').value)

        self._side_inner_rad = math.radians(self.get_parameter('side_inner_deg').value)
        self._side_outer_rad = math.radians(self.get_parameter('side_outer_deg').value)
        self._side_gain = float(self.get_parameter('side_wall_gain').value)
        self._side_thresh = float(self.get_parameter('side_wall_threshold_m').value)

        period = float(self.get_parameter('control_period_sec').value)
        self._diag_interval = float(self.get_parameter('diag_interval_sec').value)

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_safe', 10)

        self.bias_prev = 0.0
        self.bias_prev_time = None
        self._pid_integral = 0.0

        self._front_range = float('nan')
        self._left_range = float('nan')
        self._right_range = float('nan')
        self._scan_received = False
        self._last_diag_time = 0.0

        self.timer = self.create_timer(period, self.publish_command)

        self.get_logger().info(
            'Hallway follower v2: PID on side-wall LiDAR, front sector ±60°'
        )

    # ---- callbacks ----

    def scan_callback(self, msg: LaserScan) -> None:
        self._front_range = self._sector_min(msg, self._front_min_rad, self._front_max_rad)
        self._left_range = self._sector_min(msg, self._side_inner_rad, self._side_outer_rad)
        self._right_range = self._sector_min(msg, -self._side_outer_rad, -self._side_inner_rad)
        self._scan_received = True

    # ---- LiDAR helpers ----

    @staticmethod
    def _sector_min(scan: LaserScan, ang_lo: float, ang_hi: float) -> float:
        amin = scan.angle_min
        ainc = scan.angle_increment
        rmin = float('inf')
        for i in range(len(scan.ranges)):
            ang = amin + i * ainc
            if ang < ang_lo or ang > ang_hi:
                continue
            r = scan.ranges[i]
            if math.isnan(r) or math.isinf(r):
                continue
            if r < scan.range_min or r > scan.range_max:
                continue
            rmin = min(rmin, r)
        return rmin if rmin != float('inf') else float('nan')

    def _side_wall_correction(self) -> float:
        """Push robot away from the closer side wall."""
        def proximity(r: float) -> float:
            if math.isnan(r) or r > self._side_thresh:
                return 0.0
            return max(0.0, (self._side_thresh - r) / self._side_thresh)

        lp = proximity(self._left_range)
        rp = proximity(self._right_range)
        return self._side_gain * (rp - lp)

    # ---- PID ----

    def _reset_pid(self) -> None:
        self._pid_integral = 0.0
        self.bias_prev = 0.0
        self.bias_prev_time = None

    def _pid_step(self, bias: float, dt: float) -> float:
        self._pid_integral = float(
            max(-self._i_max, min(self._i_max, self._pid_integral + bias * dt))
        )
        d_term = (bias - self.bias_prev) / dt if dt > 1e-6 else 0.0
        out = self._kp * bias + self._ki * self._pid_integral + self._kd * d_term
        self.bias_prev = bias
        return max(-self._angular_max, min(self._angular_max, out * self._turn_gain))

    # ---- diagnostics ----

    def _log_diag(self, now: float) -> None:
        if (now - self._last_diag_time) < self._diag_interval:
            return
        self._last_diag_time = now
        f = self._front_range
        l = self._left_range
        r = self._right_range
        fs = f'{f:.2f}' if not math.isnan(f) else 'nan'
        ls = f'{l:.2f}' if not math.isnan(l) else 'nan'
        rs = f'{r:.2f}' if not math.isnan(r) else 'nan'
        self.get_logger().info(
            f'DIAG | front={fs} left={ls} right={rs} scan_rx={self._scan_received}'
        )

    # ---- main control loop ----

    def publish_command(self) -> None:
        now = time.monotonic()
        cmd = Twist()
        self._log_diag(now)

        # side-wall LiDAR correction
        bias = self._side_wall_correction()

        # PID
        if self.bias_prev_time is None:
            dt = 0.05
        else:
            dt = max(1e-3, now - self.bias_prev_time)
        self.bias_prev_time = now

        angular_z = self._pid_step(bias, dt)

        # adaptive forward speed: less speed during turns
        turn_mag = abs(angular_z) / max(self._angular_max, 1e-6)
        speed_factor = 1.0 - self._adaptive_scale * turn_mag
        speed_factor = max(0.0, min(1.0, speed_factor))
        linear_x = self._linear_min + (self._linear_max - self._linear_min) * speed_factor

        # front LiDAR: slow down or back off + forced turn
        if not math.isnan(self._front_range):
            if self._front_range <= self._stop_d:
                linear_x = self._wall_backoff
                self._reset_pid()

                # Pick escape direction from side LiDAR (more space = turn toward it).
                # Fall back to current angular_z sign if no side data.
                lr = self._left_range
                rr = self._right_range
                have_left = not math.isnan(lr)
                have_right = not math.isnan(rr)

                if have_left and have_right:
                    turn_sign = 1.0 if lr >= rr else -1.0
                elif have_left:
                    turn_sign = 1.0
                elif have_right:
                    turn_sign = -1.0
                else:
                    turn_sign = 1.0 if angular_z >= 0 else -1.0

                angular_z = turn_sign * max(self._wall_min_turn, abs(angular_z) * self._wall_turn_boost)
                angular_z = max(-self._angular_max, min(self._angular_max, angular_z))

            elif self._front_range < self._slow_d:
                scale = (self._front_range - self._stop_d) / max(self._slow_d - self._stop_d, 1e-6)
                linear_x *= max(0.0, scale)

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
