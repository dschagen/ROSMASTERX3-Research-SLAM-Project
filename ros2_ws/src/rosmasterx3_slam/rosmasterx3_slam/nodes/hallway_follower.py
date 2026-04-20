#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan


class WallFollower(Node):
    """
    LiDAR-based wall follower with integrated exploration mode.

    This is the ONLY node that publishes to /cmd_vel_safe.

    Modes:
        follow   — PD wall tracking (normal hallway following)
        corner   — wall suddenly opens up, wrap around it
        turn     — front obstacle, spin away
        search   — wall lost, no exploration goal available, slow forward + turn
        explore  — wall lost, exploration goal available, drive toward goal heading
        recovery — stuck in explore mode, back up and turn before retrying
    """

    def __init__(self):
        super().__init__('hallway_follower')

        # --- wall selection ---
        self.declare_parameter('wall_side', 'left')
        self.declare_parameter('wall_target_m', 0.30)

        # --- forward speed ---
        self.declare_parameter('cruise_speed', 0.10)
        self.declare_parameter('search_speed', 0.07)
        self.declare_parameter('linear_scale', 1.0)

        # --- side-wall PD ---
        self.declare_parameter('kp_side', 0.40)
        self.declare_parameter('kd_side', 0.80)
        self.declare_parameter('angular_max', 0.35)

        # --- front obstacle ---
        self.declare_parameter('front_stop_m', 0.25)
        self.declare_parameter('front_slow_m', 0.55)
        self.declare_parameter('front_half_angle_deg', 40.0)

        # --- corner mode ---
        self.declare_parameter('corner_open_m', 0.15)
        self.declare_parameter('corner_turn', 0.18)

        # --- turn mode ---
        self.declare_parameter('turn_angular', 0.20)
        self.declare_parameter('turn_min_time', 0.30)
        self.declare_parameter('turn_max_time', 3.0)

        # --- exploration mode ---
        self.declare_parameter('explore_speed', 0.08)        # forward speed while exploring
        self.declare_parameter('explore_turn_speed', 0.25)   # turning speed toward goal
        self.declare_parameter('explore_angle_tol', 0.20)    # radians — stop turning within this
        self.declare_parameter('explore_goal_dist', 0.40)    # metres — goal considered reached

        # --- recovery mode (triggered when stuck in explore) ---
        self.declare_parameter('recovery_backup_sec', 0.8)   # seconds to back up
        self.declare_parameter('recovery_turn_sec', 1.2)     # seconds to turn after backing up
        self.declare_parameter('recovery_backup_speed', -0.07)  # m/s backward
        self.declare_parameter('recovery_turn_speed', 0.25)  # rad/s during recovery turn
        self.declare_parameter('explore_stuck_sec', 6.0)     # seconds before recovery triggers

        # --- LiDAR geometry ---
        self.declare_parameter('angle_offset_deg', 180)
        self.declare_parameter('side_inner_deg', 60.0)
        self.declare_parameter('side_outer_deg', 120.0)
        self.declare_parameter('diag_inner_deg', 30.0)
        self.declare_parameter('diag_outer_deg', 65.0)

        # --- misc ---
        self.declare_parameter('control_period_sec', 0.05)
        self.declare_parameter('diag_interval_sec', 3.0)

        # ---- read all params ----
        wall_side_str = self.get_parameter('wall_side').value
        self._wall_sign = 1.0 if wall_side_str == 'left' else -1.0
        self._wall_target = float(self.get_parameter('wall_target_m').value)

        self._cruise = float(self.get_parameter('cruise_speed').value)
        self._search_spd = float(self.get_parameter('search_speed').value)
        self._linear_scale = float(self.get_parameter('linear_scale').value)

        self._kp = float(self.get_parameter('kp_side').value)
        self._kd = float(self.get_parameter('kd_side').value)
        self._ang_max = float(self.get_parameter('angular_max').value)

        self._front_stop = float(self.get_parameter('front_stop_m').value)
        self._front_slow = float(self.get_parameter('front_slow_m').value)
        half = math.radians(self.get_parameter('front_half_angle_deg').value)
        self._front_lo = -half
        self._front_hi = half

        self._corner_open = float(self.get_parameter('corner_open_m').value)
        self._corner_turn = float(self.get_parameter('corner_turn').value)

        self._turn_ang = float(self.get_parameter('turn_angular').value)
        self._turn_min_t = float(self.get_parameter('turn_min_time').value)
        self._turn_max_t = float(self.get_parameter('turn_max_time').value)

        self._explore_spd = float(self.get_parameter('explore_speed').value)
        self._explore_turn = float(self.get_parameter('explore_turn_speed').value)
        self._explore_tol = float(self.get_parameter('explore_angle_tol').value)
        self._explore_goal_dist = float(self.get_parameter('explore_goal_dist').value)

        self._recovery_backup_sec = float(self.get_parameter('recovery_backup_sec').value)
        self._recovery_turn_sec = float(self.get_parameter('recovery_turn_sec').value)
        self._recovery_backup_spd = float(self.get_parameter('recovery_backup_speed').value)
        self._recovery_turn_spd = float(self.get_parameter('recovery_turn_speed').value)
        self._explore_stuck_sec = float(self.get_parameter('explore_stuck_sec').value)

        self._angle_offset = math.radians(self.get_parameter('angle_offset_deg').value)

        si = math.radians(self.get_parameter('side_inner_deg').value)
        so = math.radians(self.get_parameter('side_outer_deg').value)
        di = math.radians(self.get_parameter('diag_inner_deg').value)
        do_ = math.radians(self.get_parameter('diag_outer_deg').value)

        s = self._wall_sign
        self._side_lo = min(s * si, s * so)
        self._side_hi = max(s * si, s * so)
        self._diag_lo = min(s * di, s * do_)
        self._diag_hi = max(s * di, s * do_)

        period = float(self.get_parameter('control_period_sec').value)
        self._diag_interval = float(self.get_parameter('diag_interval_sec').value)

        # ---- subscriptions ----
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Point, '/exploration_goal', self._goal_callback, 10)

        # ---- publisher — ONLY node writing to /cmd_vel_safe ----
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_safe', 10)

        # ---- sensor state ----
        self._front_range = float('nan')
        self._side_range = float('nan')
        self._diag_range = float('nan')
        self._scan_received = False

        # ---- startup calibration ----
        self._calibrated = False
        self._calib_samples = []
        self._calib_target = 5

        # ---- exploration goal ----
        self._explore_goal_x: float = None
        self._explore_goal_y: float = None
        self._robot_yaw = 0.0  # updated from odom via exploration_goal messages
        self._has_goal = False

        # ---- controller state ----
        self._prev_err = 0.0
        self._prev_time = None
        self._mode = 'follow'   # follow | corner | turn | search | explore | recovery
        self._mode_start = None
        self._last_diag_time = 0.0

        # ---- recovery state ----
        self._recovery_phase = 'backup'   # 'backup' | 'turn'
        self._recovery_phase_start = None

        self.timer = self.create_timer(period, self.publish_command)
        self.get_logger().info(
            f'Wall follower ready — wall={wall_side_str}  target={self._wall_target:.2f}m'
        )

    # ------------------------------------------------------------------ callbacks --

    def scan_callback(self, msg: LaserScan) -> None:
        off = self._angle_offset
        self._front_range = self._sector_min(msg, self._front_lo, self._front_hi, off)
        self._side_range = self._sector_min(msg, self._side_lo, self._side_hi, off)
        self._diag_range = self._sector_min(msg, self._diag_lo, self._diag_hi, off)
        self._scan_received = True

    def _goal_callback(self, msg: Point) -> None:
        """Receive exploration goal from frontier explorer."""
        self._explore_goal_x = msg.x
        self._explore_goal_y = msg.y
        # z encodes the robot's current yaw (sent by explorer from odom)
        self._robot_yaw = msg.z
        self._has_goal = True

    # ------------------------------------------------------------------ helpers --

    @staticmethod
    def _sector_min(scan: LaserScan, ang_lo: float, ang_hi: float,
                    offset: float = 0.0) -> float:
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

    def _angle_to_goal(self) -> float:
        """Compute heading error to exploration goal in radians."""
        if not self._has_goal:
            return 0.0
        dx = self._explore_goal_x
        dy = self._explore_goal_y
        target_yaw = math.atan2(dy, dx)
        err = target_yaw - self._robot_yaw
        while err >  math.pi: err -= 2 * math.pi
        while err < -math.pi: err += 2 * math.pi
        return err

    # ------------------------------------------------------------------ control --

    def publish_command(self) -> None:
        if not self._scan_received:
            return

        # ---- startup calibration ----
        if not self._calibrated:
            if not math.isnan(self._side_range):
                self._calib_samples.append(self._side_range)
                self.get_logger().info(
                    f'Calibrating... sample {len(self._calib_samples)}'
                    f'/{self._calib_target}: {self._side_range:.3f}m'
                )
            if len(self._calib_samples) >= self._calib_target:
                self._wall_target = sum(self._calib_samples) / len(self._calib_samples)
                self._calibrated = True
                self.get_logger().info(
                    f'Wall target calibrated to {self._wall_target:.3f}m'
                )
            else:
                self.cmd_pub.publish(Twist())
                return

        now = time.monotonic()
        dt = 0.05 if self._prev_time is None else max(1e-3, now - self._prev_time)
        self._prev_time = now

        front = self._front_range
        side  = self._side_range
        diag  = self._diag_range

        wall_seen     = not math.isnan(side)
        front_blocked = not math.isnan(front) and front <= self._front_stop
        front_clear   = not front_blocked

        # ---- mode transitions ----

        if self._mode == 'follow':
            if front_blocked:
                self._switch('turn', now)
            elif wall_seen and side > self._wall_target + self._corner_open:
                self._switch('corner', now)
            elif not wall_seen:
                # wall lost — go to explore if we have a goal, else search
                if self._has_goal:
                    self._switch('explore', now)
                else:
                    self._switch('search', now)

        elif self._mode == 'corner':
            if front_blocked:
                self._switch('turn', now)
            else:
                diag_close = not math.isnan(diag) and diag <= self._wall_target + self._corner_open
                side_back  = wall_seen and side <= self._wall_target + self._corner_open
                if diag_close or side_back:
                    self._switch('follow', now)

        elif self._mode == 'turn':
            elapsed = now - (self._mode_start or now)
            if elapsed >= self._turn_max_t:
                self._switch('search', now)
            elif elapsed >= self._turn_min_t and front_clear:
                if wall_seen:
                    self._switch('follow', now)
                elif self._has_goal:
                    self._switch('explore', now)
                else:
                    self._switch('search', now)

        elif self._mode == 'search':
            if wall_seen:
                self._switch('follow', now)
            elif front_blocked:
                self._switch('turn', now)
            elif self._has_goal:
                # explorer provided a goal while we were searching
                self._switch('explore', now)

        elif self._mode == 'explore':
            if wall_seen:
                # found a wall — resume normal following
                self._switch('follow', now)
            elif front_blocked:
                self._switch('turn', now)
            else:
                # check if stuck too long
                elapsed = now - (self._mode_start or now)
                if elapsed > self._explore_stuck_sec:
                    self.get_logger().warn('Explore mode stuck — triggering recovery')
                    self._recovery_phase = 'backup'
                    self._recovery_phase_start = now
                    self._switch('recovery', now)
                # check if goal reached (distance encoded in goal x/y as relative)
                elif self._has_goal:
                    dist = math.hypot(self._explore_goal_x, self._explore_goal_y)
                    if dist < self._explore_goal_dist:
                        self.get_logger().info('Exploration goal reached')
                        self._has_goal = False
                        self._switch('search', now)

        elif self._mode == 'recovery':
            elapsed = now - (self._recovery_phase_start or now)
            if self._recovery_phase == 'backup':
                if elapsed >= self._recovery_backup_sec:
                    self._recovery_phase = 'turn'
                    self._recovery_phase_start = now
            elif self._recovery_phase == 'turn':
                if elapsed >= self._recovery_turn_sec:
                    # recovery done — go explore or search
                    if self._has_goal:
                        self._switch('explore', now)
                    else:
                        self._switch('search', now)

        # ---- compute output ----
        cmd = Twist()

        if self._mode == 'turn':
            cmd.linear.x  = 0.0
            cmd.angular.z = float(-self._wall_sign * self._turn_ang)

        elif self._mode == 'corner':
            speed = self._forward_speed(front)
            cmd.linear.x  = float(speed * self._linear_scale)
            cmd.angular.z = float(self._wall_sign * self._corner_turn)

        elif self._mode == 'search':
            cmd.linear.x  = float(self._search_spd * self._linear_scale)
            cmd.angular.z = float(self._wall_sign * self._corner_turn * 0.5)

        elif self._mode == 'explore':
            angle_err = self._angle_to_goal()
            if abs(angle_err) > self._explore_tol:
                # turn toward goal
                cmd.linear.x  = 0.0
                cmd.angular.z = float(self._explore_turn * (1.0 if angle_err > 0 else -1.0))
            else:
                # drive toward goal with gentle heading correction
                cmd.linear.x  = float(self._explore_spd * self._linear_scale)
                cmd.angular.z = float(self._clamp(0.5 * angle_err, self._ang_max))

        elif self._mode == 'recovery':
            if self._recovery_phase == 'backup':
                cmd.linear.x  = float(self._recovery_backup_spd * self._linear_scale)
                cmd.angular.z = 0.0
            elif self._recovery_phase == 'turn':
                cmd.linear.x  = 0.0
                cmd.angular.z = float(-self._wall_sign * self._recovery_turn_spd)

        else:  # follow
            speed = self._forward_speed(front)
            if wall_seen:
                err    = side - self._wall_target
                d_term = self._clamp((err - self._prev_err) / dt, 10.0)
                raw    = self._wall_sign * (self._kp * err + self._kd * d_term)
                angular_z = self._clamp(raw, self._ang_max)
                self._prev_err = err
            else:
                angular_z = 0.0
            cmd.linear.x  = float(speed * self._linear_scale)
            cmd.angular.z = float(angular_z)

        self._log_diag(now, front, side, diag)
        self.cmd_pub.publish(cmd)

    def _switch(self, mode: str, now: float) -> None:
        if mode != self._mode:
            self.get_logger().info(f'Mode: {self._mode} → {mode}')
        self._mode = mode
        self._mode_start = now
        self._prev_err = 0.0

    # ------------------------------------------------------------------ diagnostics --

    def _log_diag(self, now: float, front: float, side: float, diag: float) -> None:
        if (now - self._last_diag_time) < self._diag_interval:
            return
        self._last_diag_time = now
        fmt = lambda v: f'{v:.2f}' if not math.isnan(v) else 'nan'
        goal_str = (f'goal=({self._explore_goal_x:.2f},{self._explore_goal_y:.2f})'
                    if self._has_goal else 'goal=none')
        self.get_logger().info(
            f'DIAG | mode={self._mode:8s} front={fmt(front)} '
            f'side={fmt(side)} diag={fmt(diag)} {goal_str}'
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
