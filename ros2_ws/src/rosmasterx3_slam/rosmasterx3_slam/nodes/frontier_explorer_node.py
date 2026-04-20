#!/usr/bin/env python3

"""
Frontier Explorer Node

Responsibilities:
  - Read /map occupancy grid from SLAM Toolbox
  - Find frontier cells (free cells adjacent to unknown cells)
  - Cluster frontiers, rank by distance, blacklist visited ones
  - Publish the best frontier as a relative goal to /exploration_goal
  - Never publish to /cmd_vel_safe — the hallway_follower handles all motion

Topic interface:
  Subscribes:  /map  (nav_msgs/OccupancyGrid)
               /odom (nav_msgs/Odometry)
  Publishes:   /exploration_goal (geometry_msgs/Point)
                 x = dx to goal (world frame, relative to robot)
                 y = dy to goal (world frame, relative to robot)
                 z = current robot yaw (so follower can compute heading)
"""

import math
import time

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Point

FREE            =   0
UNKNOWN         =  -1
OCCUPIED_THRESH =  50


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        self.declare_parameter('update_rate_sec', 3.0)
        self.declare_parameter('min_frontier_size', 5)
        self.declare_parameter('goal_reached_dist_m', 0.40)
        self.declare_parameter('stuck_timeout_sec', 8.0)
        self.declare_parameter('visited_radius_m', 0.50)
        self.declare_parameter('exploration_complete_sec', 6.0)
        self.declare_parameter('publish_rate_sec', 0.2)

        self._update_rate    = float(self.get_parameter('update_rate_sec').value)
        self._min_frontier   = int(self.get_parameter('min_frontier_size').value)
        self._goal_dist      = float(self.get_parameter('goal_reached_dist_m').value)
        self._stuck_timeout  = float(self.get_parameter('stuck_timeout_sec').value)
        self._visited_radius = float(self.get_parameter('visited_radius_m').value)
        self._done_timeout   = float(self.get_parameter('exploration_complete_sec').value)
        self._pub_rate       = float(self.get_parameter('publish_rate_sec').value)

        self._map: OccupancyGrid = None
        self._robot_x   = 0.0
        self._robot_y   = 0.0
        self._robot_yaw = 0.0
        self._odom_received = False
        self._map_received  = False

        self._goal_wx: float = None
        self._goal_wy: float = None
        self._goal_set_time: float = None

        self._visited: list = []
        self._no_frontier_since: float = None
        self._exploration_done = False

        self.create_subscription(OccupancyGrid, '/map',  self._map_callback,  10)
        self.create_subscription(Odometry,      '/odom', self._odom_callback, 10)

        self._goal_pub = self.create_publisher(Point, '/exploration_goal', 10)

        self.create_timer(self._update_rate, self._exploration_step)
        self.create_timer(self._pub_rate,    self._publish_goal)

        self.get_logger().info('Frontier explorer ready — waiting for map and odometry...')

    # ------------------------------------------------------------------ callbacks

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self._map = msg
        self._map_received = True

    def _odom_callback(self, msg: Odometry) -> None:
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        self._odom_received = True

    # ------------------------------------------------------------------ frontier detection

    def _find_frontiers(self):
        if self._map is None:
            return []

        info = self._map.info
        data = self._map.data
        w    = info.width
        h    = info.height
        res  = info.resolution
        ox   = info.origin.position.x
        oy   = info.origin.position.y

        def cell(r, c):
            if 0 <= r < h and 0 <= c < w:
                v = data[r * w + c]
                if v == -1:
                    return UNKNOWN
                return 0 if v <= OCCUPIED_THRESH else 100
            return 100

        frontier_cells = set()
        for r in range(1, h - 1):
            for c in range(1, w - 1):
                if cell(r, c) != FREE:
                    continue
                for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    if cell(r + dr, c + dc) == UNKNOWN:
                        frontier_cells.add((r, c))
                        break

        if not frontier_cells:
            return []

        visited_cells = set()
        clusters = []

        for start in frontier_cells:
            if start in visited_cells:
                continue
            cluster = []
            queue = [start]
            while queue:
                curr = queue.pop()
                if curr in visited_cells:
                    continue
                visited_cells.add(curr)
                if curr in frontier_cells:
                    cluster.append(curr)
                    r, c = curr
                    for dr, dc in [(-1,0),(1,0),(0,-1),(0,1),
                                   (-1,-1),(-1,1),(1,-1),(1,1)]:
                        nb = (r + dr, c + dc)
                        if nb not in visited_cells:
                            queue.append(nb)
            if len(cluster) >= self._min_frontier:
                clusters.append(cluster)

        if not clusters:
            return []

        candidates = []
        for cluster in clusters:
            avg_r = sum(p[0] for p in cluster) / len(cluster)
            avg_c = sum(p[1] for p in cluster) / len(cluster)
            wx = ox + (avg_c + 0.5) * res
            wy = oy + (avg_r + 0.5) * res

            if self._is_visited(wx, wy):
                continue

            dist = math.hypot(wx - self._robot_x, wy - self._robot_y)
            candidates.append((dist, wx, wy))

        candidates.sort(key=lambda x: x[0])
        return [(wx, wy) for _, wx, wy in candidates]

    def _is_visited(self, wx: float, wy: float) -> bool:
        for vx, vy in self._visited:
            if math.hypot(wx - vx, wy - vy) < self._visited_radius:
                return True
        return False

    # ------------------------------------------------------------------ exploration logic

    def _exploration_step(self) -> None:
        if not self._odom_received or not self._map_received:
            return
        if self._exploration_done:
            return

        now = time.monotonic()

        if self._goal_wx is not None:
            dist = math.hypot(self._robot_x - self._goal_wx,
                              self._robot_y - self._goal_wy)

            if dist < self._goal_dist:
                self.get_logger().info(
                    f'Frontier reached: ({self._goal_wx:.2f}, {self._goal_wy:.2f})'
                )
                self._visited.append((self._goal_wx, self._goal_wy))
                self._goal_wx = None
                self._goal_wy = None

            elif (now - self._goal_set_time) > self._stuck_timeout:
                self.get_logger().warn(
                    f'Stuck on frontier ({self._goal_wx:.2f}, {self._goal_wy:.2f}) '
                    f'— blacklisting'
                )
                self._visited.append((self._goal_wx, self._goal_wy))
                self._goal_wx = None
                self._goal_wy = None

        if self._goal_wx is None:
            frontiers = self._find_frontiers()

            if frontiers:
                self._no_frontier_since = None
                self._goal_wx, self._goal_wy = frontiers[0]
                self._goal_set_time = now
                self.get_logger().info(
                    f'New frontier: ({self._goal_wx:.2f}, {self._goal_wy:.2f}) '
                    f'| {len(frontiers)} available | {len(self._visited)} visited'
                )
            else:
                if self._no_frontier_since is None:
                    self._no_frontier_since = now
                    self.get_logger().info('No new frontiers — waiting...')
                elif (now - self._no_frontier_since) > self._done_timeout:
                    self.get_logger().info(
                        '=== Exploration complete — no frontiers remaining ==='
                    )
                    self._exploration_done = True

    def _publish_goal(self) -> None:
        """Publish current goal at 5Hz so follower always has fresh heading data."""
        if not self._odom_received:
            return
        if self._goal_wx is None or self._exploration_done:
            return

        msg = Point()
        msg.x = self._goal_wx - self._robot_x   # relative dx in world frame
        msg.y = self._goal_wy - self._robot_y   # relative dy in world frame
        msg.z = self._robot_yaw                  # robot yaw for heading computation
        self._goal_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
