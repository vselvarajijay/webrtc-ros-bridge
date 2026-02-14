#!/usr/bin/env python3
"""
Frontier-based exploration goal publisher.

Subscribes to /map (and optionally /map_odom). Finds frontier cells (free cells
adjacent to unknown), clusters them, selects a goal (e.g. cluster centroid),
publishes /goal_pose as PoseStamped in map frame.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry


# OccupancyGrid constants
FREE = 0
OCCUPIED = 100
UNKNOWN = -1


def _yaw_from_quat(ox: float, oy: float, oz: float, ow: float) -> float:
    return math.atan2(2.0 * (ow * oz + ox * oy), 1.0 - 2.0 * (oy * oy + oz * oz))


class FrontierNode(Node):
    """Detect frontiers on /map, publish goal pose for exploration."""

    def __init__(self):
        super().__init__("frontier_node")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("odom_topic", "/map_odom")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("update_rate_hz", 1.0)
        self.declare_parameter("frontier_min_cluster_size", 5)
        self.declare_parameter("max_goal_distance", 8.0)
        self.declare_parameter("min_goal_distance", 0.5)

        self._map_topic = self.get_parameter("map_topic").value
        self._odom_topic = self.get_parameter("odom_topic").value
        self._goal_topic = self.get_parameter("goal_topic").value
        self._update_rate_hz = self.get_parameter("update_rate_hz").value
        self._frontier_min_cluster_size = self.get_parameter("frontier_min_cluster_size").value
        self._max_goal_distance = self.get_parameter("max_goal_distance").value
        self._min_goal_distance = self.get_parameter("min_goal_distance").value

        self._last_map: OccupancyGrid | None = None
        self._robot_pose: tuple[float, float, float] | None = None  # (x, y, theta)

        self._map_sub = self.create_subscription(
            OccupancyGrid,
            self._map_topic,
            self._map_callback,
            10,
        )
        self._odom_sub = self.create_subscription(
            Odometry,
            self._odom_topic,
            self._odom_callback,
            10,
        )
        self._goal_pub = self.create_publisher(PoseStamped, self._goal_topic, 10)
        self.create_timer(1.0 / self._update_rate_hz, self._publish_goal_callback)

        self.get_logger().info(
            "frontier_node: map=%s odom=%s goal=%s rate=%.1f Hz"
            % (self._map_topic, self._odom_topic, self._goal_topic, self._update_rate_hz)
        )

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self._last_map = msg

    def _odom_callback(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        o = msg.pose.pose.orientation
        theta = _yaw_from_quat(o.x, o.y, o.z, o.w)
        self._robot_pose = (x, y, theta)

    def _find_frontier_cells(
        self, data: np.ndarray, w: int, h: int
    ) -> list[tuple[int, int]]:
        """Return list of (gx, gy) that are free and have at least one unknown neighbor."""
        frontiers = []
        for gy in range(1, h - 1):
            for gx in range(1, w - 1):
                if int(data[gy, gx]) != FREE:
                    continue
                # Check 4-neighbors for unknown
                if (
                    int(data[gy - 1, gx]) == UNKNOWN
                    or int(data[gy + 1, gx]) == UNKNOWN
                    or int(data[gy, gx - 1]) == UNKNOWN
                    or int(data[gy, gx + 1]) == UNKNOWN
                ):
                    frontiers.append((gx, gy))
        return frontiers

    def _cluster_frontiers(
        self, cells: list[tuple[int, int]], res: float
    ) -> list[list[tuple[int, int]]]:
        """Cluster frontier cells by distance (same cluster if within ~2 cell diagonal)."""
        if not cells:
            return []
        max_dist = 2.5 * res  # ~2 cells
        clusters: list[list[tuple[int, int]]] = []
        used = set()

        def dist(a: tuple[int, int], b: tuple[int, int]) -> float:
            return res * math.hypot(a[0] - b[0], a[1] - b[1])

        for seed in cells:
            if seed in used:
                continue
            cluster = [seed]
            used.add(seed)
            stack = [seed]
            while stack:
                cx, cy = stack.pop()
                for nx, ny in [
                    (cx - 1, cy),
                    (cx + 1, cy),
                    (cx, cy - 1),
                    (cx, cy + 1),
                ]:
                    if (nx, ny) in used:
                        continue
                    if (nx, ny) not in cells:
                        continue
                    used.add((nx, ny))
                    cluster.append((nx, ny))
                    stack.append((nx, ny))
            clusters.append(cluster)
        return clusters

    def _select_goal_cluster(
        self,
        clusters: list[list[tuple[int, int]]],
        res: float,
        ox: float,
        oy: float,
    ) -> tuple[float, float] | None:
        """Pick best cluster: centroid in map frame. Prefer larger, reachable clusters."""
        if not clusters or self._robot_pose is None:
            return None
        rx, ry, _ = self._robot_pose
        best = None
        best_score = -1.0
        for cluster in clusters:
            if len(cluster) < self._frontier_min_cluster_size:
                continue
            cx = sum(c[0] for c in cluster) / len(cluster)
            cy = sum(c[1] for c in cluster) / len(cluster)
            wx = ox + cx * res
            wy = oy + cy * res
            dist = math.hypot(wx - rx, wy - ry)
            if dist < self._min_goal_distance or dist > self._max_goal_distance:
                continue
            # Score: prefer larger clusters and closer (so we make progress)
            score = len(cluster) * 0.5 + (self._max_goal_distance - dist) * 0.5
            if score > best_score:
                best_score = score
                best = (wx, wy)
        return best

    def _publish_goal_callback(self) -> None:
        if self._last_map is None:
            return
        grid = self._last_map
        w = grid.info.width
        h = grid.info.height
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        data = np.array(grid.data, dtype=np.int8).reshape((h, w))

        frontiers = self._find_frontier_cells(data, w, h)
        if not frontiers:
            return
        clusters = self._cluster_frontiers(frontiers, res)
        if not self._robot_pose:
            return
        goal = self._select_goal_cluster(clusters, res, ox, oy)
        if goal is None:
            return
        gx, gy = goal
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(gx)
        msg.pose.position.y = float(gy)
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        self._goal_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
