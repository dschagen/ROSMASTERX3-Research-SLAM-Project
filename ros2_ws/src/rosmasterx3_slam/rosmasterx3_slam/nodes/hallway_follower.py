#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan


class WallFollower(Node):
    def __init__(self):
        super().__init__('hallway_follower')

        # --- wall selection ---
        self.declare_parameter('wall_side', 'left')
        self.declare_parameter('follow_any_wall', True)
        self.declare_parameter('wall_target_m', 0.30)

        # --- forward speed ---
        self.declare_parameter('cruise_speed', 0.03)
        self.declare_parameter('search_speed', 0.02)
        self.declare_parameter('linear_scale', 1.0)

        # --- side-wall PID ---
        self.declare_parameter('kp_side', 0.20)
        self.declare_parameter('ki_side', 0.01)
        self.declare_parameter('kd_side', 0.40)
        self.declare_parameter('angular_max', 0.20)
        self.declare_parameter('integral_max', 0.10)
        self.declare_parameter('error_deadband_m', 0.06)

        # --- clear-path straight driving ---
        self.declare_parameter('straight_front_min_m', 0.90)
        self.declare_parameter('straight_pid_error_m', 0.08)

        # --- front obstacle ---
        self.declare_parameter('front_stop_m', 0.30)
        self.declare_parameter('front_slow_m', 0.60)
        self.declare_parameter('front_half_angle_deg', 40.0)

        # --- corner mode ---
        self.declare_parameter('corner_open_m', 0.25)
        self.declare_parameter('corner_turn', 0.12)
        self.declare_parameter('corner_turn_min', 0.06)
        self.declare_parameter('corner_speed', 0.025)
        self.declare_parameter('corner_diag_target_m', 0.50)
        self.declare_parameter('corner_kp', 0.30)
        self.declare_parameter('corner_timeout_sec', 5.0)
        self.declare_parameter('corner_exit_tol_m', 0.08)

        # --- turn mode ---
        self.declare_parameter('turn_angular', 0.08)
        self.declare_parameter('turn_min_time', 0.50)
        self.declare_parameter('turn_max_time', 2.0)

        # --- settle mode ---
        self.declare_parameter('settle_duration_sec', 2.0)

        # --- exploration mode ---
        self.declare_parameter('explore_speed', 0.08)
        self.declare_parameter('explore_turn_speed', 0.25)
        self.declare_parameter('explore_angle_tol', 0.20)
        self.declare_parameter('explore_goal_dist', 0.40)
        self.declare_parameter('explore_stuck_sec', 6.0)

        # --- recovery mode ---
        self.declare_parameter('recovery_backup_sec', 0.8)
        self.declare_parameter('recovery_turn_sec', 1.2)
        self.declare_parameter('recovery_backup_speed', -0.07)
        self.declare_parameter('recovery_turn_speed', 0.25)

        # --- LiDAR geometry ---
        self.declare_parameter('angle_offset_deg', 180)
        self.declare_parameter('side_inner_deg', 60.0)
        self.declare_parameter('side_outer_deg', 120.0)
        self.declare_parameter('diag_inner_deg', 30.0)
        self.declare_parameter('diag_outer_deg', 65.0)
        self.declare_parameter('side_percentile', 25.0)
        self.declare_parameter('diag_percentile', 30.0)
        self.declare_parameter('opp_percentile', 25.0)

        # --- follow recovery ---
        self.declare_parameter('recenter_slow_error_m', 0.10)
        self.declare_parameter('recenter_speed_scale_min', 0.40)

        # --- misc ---
        self.declare_parameter('control_period_sec', 0.05)
        self.declare_parameter('diag_interval_sec', 3.0)

        # --- startup ---
        self.declare_parameter('prescan_count', 20)
        self.declare_parameter('wall_switch_cooldown_sec', 0.8)

        # ---- read params ----
        wall_side_str = self.get_parameter('wall_side').value
        self._preferred_wall_side = wall_side_str
        self._follow_any_wall = bool(self.get_parameter('follow_any_wall').value)
        self._wall_sign = 1.0 if wall_side_str == 'left' else -1.0
        self._wall_target = float(self.get_parameter('wall_target_m').value)

        self._cruise = float(self.get_parameter('cruise_speed').value)
        self._search_spd = float(self.get_parameter('search_speed').value)
        self._linear_scale = float(self.get_parameter('linear_scale').value)

        self._kp = float(self.get_parameter('kp_side').value)
        self._ki = float(self.get_parameter('ki_side').value)
        self._kd = float(self.get_parameter('kd_side').value)
        self._ang_max = float(self.get_parameter('angular_max').value)
        self._integral_max = float(self.get_parameter('integral_max').value)
        self._error_deadband = float(self.get_parameter('error_deadband_m').value)
        self._straight_front_min = float(self.get_parameter('straight_front_min_m').value)
        self._straight_pid_error = float(self.get_parameter('straight_pid_error_m').value)

        self._front_stop = float(self.get_parameter('front_stop_m').value)
        self._front_slow = float(self.get_parameter('front_slow_m').value)
        half = math.radians(self.get_parameter('front_half_angle_deg').value)
        self._front_lo = -half
        self._front_hi = half

        self._corner_open = float(self.get_parameter('corner_open_m').value)
        self._corner_turn = float(self.get_parameter('corner_turn').value)
        self._corner_turn_min = float(self.get_parameter('corner_turn_min').value)
        self._corner_speed = float(self.get_parameter('corner_speed').value)
        self._corner_diag_target = float(self.get_parameter('corner_diag_target_m').value)
        self._corner_kp = float(self.get_parameter('corner_kp').value)
        self._corner_timeout = float(self.get_parameter('corner_timeout_sec').value)
        self._corner_exit_tol = float(self.get_parameter('corner_exit_tol_m').value)

        self._turn_ang = float(self.get_parameter('turn_angular').value)
        self._turn_min_t = float(self.get_parameter('turn_min_time').value)
        self._turn_max_t = float(self.get_parameter('turn_max_time').value)
        self._settle_duration = float(self.get_parameter('settle_duration_sec').value)

        self._explore_spd = float(self.get_parameter('explore_speed').value)
        self._explore_turn_spd = float(self.get_parameter('explore_turn_speed').value)
        self._explore_angle_tol = float(self.get_parameter('explore_angle_tol').value)
        self._explore_goal_dist = float(self.get_parameter('explore_goal_dist').value)
        self._explore_stuck_sec = float(self.get_parameter('explore_stuck_sec').value)

        self._recovery_backup_sec = float(self.get_parameter('recovery_backup_sec').value)
        self._recovery_turn_sec = float(self.get_parameter('recovery_turn_sec').value)
        self._recovery_backup_spd = float(self.get_parameter('recovery_backup_speed').value)
        self._recovery_turn_spd = float(self.get_parameter('recovery_turn_speed').value)

        self._angle_offset = math.radians(self.get_parameter('angle_offset_deg').value)
        self._side_percentile = float(self.get_parameter('side_percentile').value)
        self._diag_percentile = float(self.get_parameter('diag_percentile').value)
        self._opp_percentile = float(self.get_parameter('opp_percentile').value)
        self._recenter_slow_error = float(self.get_parameter('recenter_slow_error_m').value)
        self._recenter_speed_scale_min = float(
            self.get_parameter('recenter_speed_scale_min').value
        )

        si = math.radians(self.get_parameter('side_inner_deg').value)
        so = math.radians(self.get_parameter('side_outer_deg').value)
        di = math.radians(self.get_parameter('diag_inner_deg').value)
        do_ = math.radians(self.get_parameter('diag_outer_deg').value)

        self._left_side_lo = min(si, so)
        self._left_side_hi = max(si, so)
        self._right_side_lo = -self._left_side_hi
        self._right_side_hi = -self._left_side_lo
        self._left_diag_lo = min(di, do_)
        self._left_diag_hi = max(di, do_)
        self._right_diag_lo = -self._left_diag_hi
        self._right_diag_hi = -self._left_diag_lo

        self._set_wall_geometry_from_sign()

        period = float(self.get_parameter('control_period_sec').value)
        self._diag_interval = float(self.get_parameter('diag_interval_sec').value)
        self._prescan_target = int(self.get_parameter('prescan_count').value)
        self._wall_switch_cooldown = float(
            self.get_parameter('wall_switch_cooldown_sec').value
        )

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Point, '/exploration_goal', self._goal_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_safe', 10)

        self._prescan_done = False
        self._scan_count = 0

        self._front_range = float('nan')
        self._side_range = float('nan')
        self._diag_range = float('nan')
        self._rear_range = float('nan')
        self._opp_range = float('nan')
        self._left_side_range = float('nan')
        self._right_side_range = float('nan')
        self._left_diag_range = float('nan')
        self._right_diag_range = float('nan')
        self._scan_received = False

        self._calibrated = False
        self._calib_samples = []
        self._calib_target = 5

        self._goal_dx = None
        self._goal_dy = None
        self._goal_yaw = None

        self._prev_err = 0.0
        self._filtered_err = 0.0
        self._prev_raw_err = 0.0
        self._integral_err = 0.0
        self._prev_time = None
        self._mode = 'follow'
        self._mode_start = None
        self._last_diag_time = 0.0
        self._last_wall_switch_time = 0.0

        self.timer = self.create_timer(period, self.publish_command)
        self.get_logger().info(
            f'Wall follower ready - wall={wall_side_str} target={self._wall_target:.2f}m'
        )
        self.get_logger().info(
            f'Waiting for {self._prescan_target} scans before moving...'
        )

    def _set_wall_geometry_from_sign(self) -> None:
        s = self._wall_sign
        self._side_lo = self._left_side_lo if s > 0.0 else self._right_side_lo
        self._side_hi = self._left_side_hi if s > 0.0 else self._right_side_hi
        self._diag_lo = self._left_diag_lo if s > 0.0 else self._right_diag_lo
        self._diag_hi = self._left_diag_hi if s > 0.0 else self._right_diag_hi

    def scan_callback(self, msg: LaserScan) -> None:
        off = self._angle_offset
        self._front_range = self._sector_min(msg, self._front_lo, self._front_hi, off)
        self._left_side_range = self._sector_percentile(
            msg, self._left_side_lo, self._left_side_hi, self._side_percentile, off
        )
        self._right_side_range = self._sector_percentile(
            msg, self._right_side_lo, self._right_side_hi, self._side_percentile, off
        )
        self._left_diag_range = self._sector_percentile(
            msg, self._left_diag_lo, self._left_diag_hi, self._diag_percentile, off
        )
        self._right_diag_range = self._sector_percentile(
            msg, self._right_diag_lo, self._right_diag_hi, self._diag_percentile, off
        )
        self._refresh_active_wall_ranges()

        rear_l = self._sector_min(msg, math.radians(140.0), math.pi, off)
        rear_r = self._sector_min(msg, -math.pi, math.radians(-140.0), off)
        if math.isnan(rear_l) and math.isnan(rear_r):
            self._rear_range = float('nan')
        elif math.isnan(rear_l):
            self._rear_range = rear_r
        elif math.isnan(rear_r):
            self._rear_range = rear_l
        else:
            self._rear_range = min(rear_l, rear_r)
        self._scan_received = True

        if not self._prescan_done:
            self._scan_count += 1
            if self._scan_count % 5 == 0:
                self.get_logger().info(
                    f'Pre-scan: {self._scan_count}/{self._prescan_target} scans collected...'
                )
            if self._scan_count >= self._prescan_target:
                self._prescan_done = True
                self.get_logger().info(
                    f'Pre-scan complete ({self._scan_count} scans) - starting calibration'
                )

    def _goal_callback(self, msg: Point) -> None:
        self._goal_dx = msg.x
        self._goal_dy = msg.y
        self._goal_yaw = msg.z

    @staticmethod
    def _sector_min(scan: LaserScan, ang_lo: float, ang_hi: float, offset: float = 0.0) -> float:
        amin = scan.angle_min
        ainc = scan.angle_increment
        two_pi = 2.0 * math.pi
        rmin = float('inf')
        for i in range(len(scan.ranges)):
            ang = ((amin + i * ainc - offset + math.pi) % two_pi) - math.pi
            if ang < ang_lo or ang > ang_hi:
                continue
            r = scan.ranges[i]
            if math.isnan(r) or math.isinf(r) or r < scan.range_min or r > scan.range_max:
                continue
            if r < rmin:
                rmin = r
        return rmin if rmin != float('inf') else float('nan')

    @staticmethod
    def _sector_percentile(
        scan: LaserScan,
        ang_lo: float,
        ang_hi: float,
        percentile: float,
        offset: float = 0.0,
    ) -> float:
        amin = scan.angle_min
        ainc = scan.angle_increment
        two_pi = 2.0 * math.pi
        values = []
        for i in range(len(scan.ranges)):
            ang = ((amin + i * ainc - offset + math.pi) % two_pi) - math.pi
            if ang < ang_lo or ang > ang_hi:
                continue
            r = scan.ranges[i]
            if math.isnan(r) or math.isinf(r) or r < scan.range_min or r > scan.range_max:
                continue
            values.append(r)
        if not values:
            return float('nan')
        values.sort()
        pct = max(0.0, min(100.0, percentile)) / 100.0
        idx = int(round((len(values) - 1) * pct))
        return values[idx]

    def _refresh_active_wall_ranges(self) -> None:
        if self._wall_sign > 0.0:
            self._side_range = self._left_side_range
            self._diag_range = self._left_diag_range
            self._opp_range = self._right_side_range
        else:
            self._side_range = self._right_side_range
            self._diag_range = self._right_diag_range
            self._opp_range = self._left_side_range

    def _set_active_wall(self, wall_side: str, now: float) -> None:
        self._wall_sign = 1.0 if wall_side == 'left' else -1.0
        self._set_wall_geometry_from_sign()
        self._refresh_active_wall_ranges()
        self._last_wall_switch_time = now
        self._prev_err = 0.0
        self._filtered_err = 0.0
        self._prev_raw_err = 0.0
        self._integral_err = 0.0
        self.get_logger().info(f'Active wall -> {wall_side}')

    def _try_switch_wall(self, now: float) -> bool:
        if not self._follow_any_wall:
            return False
        if (now - self._last_wall_switch_time) < self._wall_switch_cooldown:
            return False

        current_visible = not math.isnan(self._side_range)
        other_visible = not math.isnan(self._opp_range)
        if current_visible or not other_visible:
            return False

        new_side = 'right' if self._wall_sign > 0.0 else 'left'
        self._set_active_wall(new_side, now)
        return True

    def _forward_speed(self, front: float) -> float:
        if math.isnan(front) or front >= self._front_slow:
            return self._cruise
        if front <= self._front_stop:
            return 0.0
        t = (front - self._front_stop) / (self._front_slow - self._front_stop)
        return self._cruise * t

    @staticmethod
    def _clamp(v: float, limit: float) -> float:
        return max(-limit, min(limit, v))

    @staticmethod
    def _norm_angle(a: float) -> float:
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    def publish_command(self) -> None:
        if not self._scan_received:
            return

        if not self._prescan_done:
            self.cmd_pub.publish(Twist())
            return

        if not self._calibrated:
            if not math.isnan(self._side_range):
                self._calib_samples.append(self._side_range)
                self.get_logger().info(
                    f'Calibrating... sample {len(self._calib_samples)}/{self._calib_target}: '
                    f'{self._side_range:.3f}m'
                )
            if len(self._calib_samples) >= self._calib_target:
                self._wall_target = sum(self._calib_samples) / len(self._calib_samples)
                self._calibrated = True
                self.get_logger().info(
                    f'Wall target calibrated to {self._wall_target:.3f}m - starting navigation'
                )
            else:
                self.cmd_pub.publish(Twist())
                return

        now = time.monotonic()
        dt = 0.05 if self._prev_time is None else max(1e-3, now - self._prev_time)
        self._prev_time = now

        front = self._front_range
        side = self._side_range
        diag = self._diag_range

        if self._try_switch_wall(now):
            side = self._side_range
            diag = self._diag_range

        wall_seen = not math.isnan(side)
        front_blocked = not math.isnan(front) and front <= self._front_stop
        front_clear = not front_blocked
        goal_available = self._goal_dx is not None and self._goal_yaw is not None
        elapsed = now - (self._mode_start if self._mode_start is not None else now)
        rear_blocked = not math.isnan(self._rear_range) and self._rear_range <= self._front_stop

        if self._mode == 'follow':
            if front_blocked:
                self._switch('turn', now)
            elif wall_seen and side > self._wall_target + self._corner_open:
                self._switch('corner', now)
            elif not wall_seen:
                if goal_available:
                    self._switch('explore', now)
                else:
                    self._switch('search', now)

        elif self._mode == 'corner':
            if front_blocked:
                self._switch('turn', now)
            elif elapsed >= self._corner_timeout:
                if goal_available:
                    self._switch('explore', now)
                else:
                    self._switch('search', now)
            else:
                side_aligned = wall_seen and abs(side - self._wall_target) <= self._corner_exit_tol
                if side_aligned:
                    self._switch('follow', now)

        elif self._mode == 'turn':
            if elapsed >= self._turn_max_t:
                self._switch('search', now)
            elif elapsed >= self._turn_min_t and front_clear:
                self._switch('settle', now)

        elif self._mode == 'settle':
            if elapsed >= self._settle_duration:
                if self._try_switch_wall(now):
                    wall_seen = not math.isnan(self._side_range)
                if wall_seen:
                    self._switch('follow', now)
                else:
                    self._switch('search', now)

        elif self._mode == 'search':
            if self._try_switch_wall(now):
                wall_seen = not math.isnan(self._side_range)
            if wall_seen:
                self._switch('follow', now)
            elif front_blocked:
                self._switch('turn', now)
            elif goal_available:
                self._switch('explore', now)

        elif self._mode == 'explore':
            if wall_seen:
                self._switch('follow', now)
            elif front_blocked:
                self._switch('turn', now)
            elif elapsed > self._explore_stuck_sec:
                self.get_logger().warn('Explore stuck - entering recovery')
                self._switch('recovery_backup', now)
            elif goal_available:
                goal_dist = math.hypot(self._goal_dx, self._goal_dy)
                if goal_dist < self._explore_goal_dist:
                    self.get_logger().info(
                        f'Frontier goal reached ({goal_dist:.2f}m) - returning to search'
                    )
                    self._switch('search', now)

        elif self._mode == 'recovery_backup':
            if rear_blocked or elapsed >= self._recovery_backup_sec:
                self._switch('recovery_turn', now)

        elif self._mode == 'recovery_turn':
            if elapsed >= self._recovery_turn_sec:
                self._switch('settle', now)

        cmd = Twist()

        if self._mode == 'turn':
            cmd.linear.x = 0.0
            cmd.angular.z = float(-self._wall_sign * self._turn_ang)

        elif self._mode == 'settle':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        elif self._mode == 'corner':
            if not math.isnan(diag):
                diag_err = diag - self._corner_diag_target
                arc_turn = self._corner_turn_min + self._corner_kp * diag_err
            else:
                arc_turn = self._corner_turn_min
            arc_turn = self._clamp(arc_turn, self._corner_turn)
            arc_turn = max(self._corner_turn_min, arc_turn)
            cmd.linear.x = float(self._corner_speed * self._linear_scale)
            cmd.angular.z = float(self._wall_sign * arc_turn)

        elif self._mode == 'search':
            cmd.linear.x = float(self._search_spd * self._linear_scale)
            cmd.angular.z = float(self._wall_sign * self._corner_turn * 0.5)

        elif self._mode == 'explore':
            if goal_available:
                heading_to_goal = math.atan2(self._goal_dy, self._goal_dx)
                heading_err = self._norm_angle(heading_to_goal - self._goal_yaw)
                if abs(heading_err) > self._explore_angle_tol:
                    cmd.linear.x = 0.0
                    cmd.angular.z = float(
                        self._clamp(
                            self._explore_turn_spd * math.copysign(1.0, heading_err),
                            self._explore_turn_spd,
                        )
                    )
                else:
                    cmd.linear.x = float(self._explore_spd * self._linear_scale)
                    cmd.angular.z = float(self._clamp(0.3 * heading_err, self._ang_max))
            else:
                cmd.linear.x = float(self._search_spd * self._linear_scale)
                cmd.angular.z = 0.0

        elif self._mode == 'recovery_backup':
            cmd.linear.x = float(self._recovery_backup_spd)
            cmd.angular.z = 0.0

        elif self._mode == 'recovery_turn':
            cmd.linear.x = 0.0
            cmd.angular.z = float(-self._wall_sign * self._recovery_turn_spd)

        else:  # follow
            speed = self._forward_speed(front)
            follow_err_mag = 0.0
            if wall_seen:
                raw_err = side - self._wall_target
                follow_err_mag = abs(raw_err)
                clear_straight_path = (
                    (math.isnan(front) or front >= self._straight_front_min)
                    and abs(raw_err) <= self._straight_pid_error
                )
                if clear_straight_path:
                    angular_z = 0.0
                    self._filtered_err = 0.0
                    self._prev_raw_err = raw_err
                    self._integral_err = 0.0
                else:
                    if abs(raw_err) < self._error_deadband:
                        eff_err = 0.0
                    else:
                        eff_err = raw_err - math.copysign(self._error_deadband, raw_err)

                    self._filtered_err = eff_err
                    self._integral_err = self._clamp(
                        self._integral_err + eff_err * dt,
                        self._integral_max,
                    )
                    d_term = self._clamp((raw_err - self._prev_raw_err) / dt, 2.0)

                    raw = self._wall_sign * (
                        self._kp * self._filtered_err
                        + self._ki * self._integral_err
                        + self._kd * d_term
                    )
                    angular_z = self._clamp(raw, self._ang_max)
                    self._prev_err = self._filtered_err
                    self._prev_raw_err = raw_err
            else:
                angular_z = 0.0

            if self._recenter_slow_error > 1e-3:
                recenter_frac = min(1.0, follow_err_mag / self._recenter_slow_error)
                recenter_scale = 1.0 - (
                    (1.0 - self._recenter_speed_scale_min) * recenter_frac
                )
                speed *= recenter_scale

            cmd.linear.x = float(speed * self._linear_scale)
            cmd.angular.z = float(angular_z)

        self._log_diag(now, front, side, diag)
        self.cmd_pub.publish(cmd)

    def _switch(self, mode: str, now: float) -> None:
        self.get_logger().info(f'Mode: {self._mode} -> {mode}')
        self._mode = mode
        self._mode_start = now
        self._prev_err = 0.0
        self._filtered_err = 0.0
        self._prev_raw_err = 0.0
        self._integral_err = 0.0

    def _log_diag(self, now: float, front: float, side: float, diag: float) -> None:
        if (now - self._last_diag_time) < self._diag_interval:
            return
        self._last_diag_time = now
        fmt = lambda v: f'{v:.2f}' if not math.isnan(v) else 'nan'
        goal_str = (
            f'goal=({self._goal_dx:.2f},{self._goal_dy:.2f})'
            if self._goal_dx is not None
            else 'goal=none'
        )
        clear_straight_path = (
            not math.isnan(side)
            and (math.isnan(front) or front >= self._straight_front_min)
            and abs(side - self._wall_target) <= self._straight_pid_error
        )
        ctrl_str = 'straight' if clear_straight_path else 'pid'
        wall_str = 'left' if self._wall_sign > 0.0 else 'right'
        self.get_logger().info(
            f'DIAG | mode={self._mode:16s} wall={wall_str:5s} ctrl={ctrl_str:8s} '
            f'front={fmt(front)} side={fmt(side)} opp={fmt(self._opp_range)} '
            f'diag={fmt(diag)} {goal_str}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        for _ in range(20):
            node.cmd_pub.publish(stop)
            time.sleep(0.02)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
