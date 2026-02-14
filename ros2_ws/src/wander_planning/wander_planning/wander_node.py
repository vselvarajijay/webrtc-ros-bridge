#!/usr/bin/env python3
"""
Reactive wander + obstacle avoid with pose tracking, stuck detection,
visited-cell bias, and optional goal-directed exploration.

Subscribes to occupancy grid (and /map_odom when using map frame),
chooses forward or turn based on free space in left/center/right sectors,
publishes velocity commands to /robot/control.
"""

import json
import math
import random
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool, String


def _yaw_from_quat(ox: float, oy: float, oz: float, ow: float) -> float:
    """Extract yaw from quaternion (assuming 2D rotation)."""
    return math.atan2(2.0 * (ow * oz + ox * oy), 1.0 - 2.0 * (oy * oy + oz * oz))


def _normalize_angle(a: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class WanderNode(Node):
    """Reactive wander with pose, stuck recovery, visited bias, and optional goal drive."""

    def __init__(self):
        super().__init__("wander_node")

        self.declare_parameter("grid_topic", "/occupancy/grid")
        self.declare_parameter("control_topic", "/robot/control")
        self.declare_parameter("odom_topic", "/map_odom")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("linear_speed", 0.15)
        self.declare_parameter("angular_speed", 0.4)
        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("safety_row_ratio", 0.25)
        self.declare_parameter("safety_clearance", 50)
        self.declare_parameter("center_free_ratio_min", 0.5)
        self.declare_parameter("center_passable_ratio_min", 0.2)  # forward if this fraction of center is passable (path clear)
        self.declare_parameter("min_center_passable_cells", 3)  # or forward if at least this many passable cells in center
        self.declare_parameter("grid_stale_timeout_s", 1.0)
        self.declare_parameter("enable_topic", "/wander/enable")
        # Stuck detection
        self.declare_parameter("stuck_distance_m", 0.08)
        self.declare_parameter("stuck_cycles", 30)
        self.declare_parameter("stuck_recovery_angular", 0.8)
        self.declare_parameter("stuck_recovery_duration_s", 1.5)
        # Visited cells
        self.declare_parameter("use_visited_bias", True)
        self.declare_parameter("visited_cells_max", 800)
        # Robot-centric sector (map frame)
        self.declare_parameter("front_band_min_m", 0.3)
        self.declare_parameter("front_band_max_m", 2.0)
        self.declare_parameter("sector_angle_deg", 18.0)  # left/center/right cone; wider = more "in front" counts as center
        # Goal-driven
        self.declare_parameter("goal_reached_distance_m", 0.5)
        self.declare_parameter("goal_timeout_s", 30.0)
        self.declare_parameter("drive_to_goal_linear_max", 0.2)
        self.declare_parameter("drive_to_goal_k_angular", 1.2)

        self._grid_topic = self.get_parameter("grid_topic").value
        self._control_topic = self.get_parameter("control_topic").value
        self._odom_topic = self.get_parameter("odom_topic").value
        self._goal_topic = self.get_parameter("goal_topic").value
        self._linear_speed = self.get_parameter("linear_speed").value
        self._angular_speed = self.get_parameter("angular_speed").value
        self._control_rate_hz = self.get_parameter("control_rate_hz").value
        self._safety_row_ratio = self.get_parameter("safety_row_ratio").value
        self._safety_clearance = self.get_parameter("safety_clearance").value
        self._center_free_ratio_min = self.get_parameter("center_free_ratio_min").value
        self._center_passable_ratio_min = self.get_parameter("center_passable_ratio_min").value
        self._min_center_passable_cells = self.get_parameter("min_center_passable_cells").value
        self._grid_stale_timeout_s = self.get_parameter("grid_stale_timeout_s").value
        self._enable_topic = self.get_parameter("enable_topic").value
        self._stuck_distance_m = self.get_parameter("stuck_distance_m").value
        self._stuck_cycles = self.get_parameter("stuck_cycles").value
        self._stuck_recovery_angular = self.get_parameter("stuck_recovery_angular").value
        self._stuck_recovery_duration_s = self.get_parameter("stuck_recovery_duration_s").value
        self._use_visited_bias = self.get_parameter("use_visited_bias").value
        self._visited_cells_max = self.get_parameter("visited_cells_max").value
        self._front_band_min_m = self.get_parameter("front_band_min_m").value
        self._front_band_max_m = self.get_parameter("front_band_max_m").value
        self._sector_angle_deg = self.get_parameter("sector_angle_deg").value
        self._goal_reached_distance_m = self.get_parameter("goal_reached_distance_m").value
        self._goal_timeout_s = self.get_parameter("goal_timeout_s").value
        self._drive_to_goal_linear_max = self.get_parameter("drive_to_goal_linear_max").value
        self._drive_to_goal_k_angular = self.get_parameter("drive_to_goal_k_angular").value

        self._enabled = False
        self._last_grid: OccupancyGrid | None = None
        self._last_grid_time_ns: int = 0
        self._turn_alternate = 1

        # Pose and visited cells (for map frame)
        self._robot_pose: tuple[float, float, float] | None = None  # (x, y, theta)
        self._visited_cells: set[tuple[int, int]] = set()
        self._visited_deque: deque = deque(maxlen=self._visited_cells_max + 1)

        # Stuck detection
        self._last_pose: tuple[float, float] | None = None
        self._stuck_counter = 0
        self._recovery_cycles_left = 0
        self._recovery_angular_sign = 1.0

        # Goal
        self._current_goal: tuple[float, float] | None = None
        self._goal_stamp_ns: int | None = None

        self._grid_sub = self.create_subscription(
            OccupancyGrid,
            self._grid_topic,
            self._grid_callback,
            10,
        )
        self._enable_sub = self.create_subscription(
            Bool,
            self._enable_topic,
            self._enable_callback,
            10,
        )
        self._odom_sub = self.create_subscription(
            Odometry,
            self._odom_topic,
            self._odom_callback,
            10,
        )
        self._goal_sub = self.create_subscription(
            PoseStamped,
            self._goal_topic,
            self._goal_callback,
            10,
        )
        self._control_pub = self.create_publisher(String, self._control_topic, 10)

        period_ns = int(1e9 / self._control_rate_hz)
        self._timer = self.create_timer(period_ns / 1e9, self._control_callback)

        self.get_logger().info(
            "wander_node: grid=%s odom=%s goal=%s control=%s enable=%s rate=%.1f Hz"
            % (
                self._grid_topic,
                self._odom_topic,
                self._goal_topic,
                self._control_topic,
                self._enable_topic,
                self._control_rate_hz,
            )
        )

    def _enable_callback(self, msg: Bool) -> None:
        self._enabled = msg.data

    def _grid_callback(self, msg: OccupancyGrid) -> None:
        self._last_grid = msg
        self._last_grid_time_ns = self.get_clock().now().nanoseconds

    def _odom_callback(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        o = msg.pose.pose.orientation
        theta = _yaw_from_quat(o.x, o.y, o.z, o.w)
        self._robot_pose = (x, y, theta)
        if self._last_grid is not None:
            info = self._last_grid.info
            ox = info.origin.position.x
            oy = info.origin.position.y
            res = info.resolution
            gx = int((x - ox) / res)
            gy = int((y - oy) / res)
            cell = (gx, gy)
            if cell not in self._visited_cells:
                self._visited_cells.add(cell)
                self._visited_deque.append(cell)
                if len(self._visited_deque) > self._visited_cells_max:
                    old = self._visited_deque.popleft()
                    self._visited_cells.discard(old)

    def _goal_callback(self, msg: PoseStamped) -> None:
        self._current_goal = (msg.pose.position.x, msg.pose.position.y)
        self._goal_stamp_ns = self.get_clock().now().nanoseconds

    def _compute_sectors_map_frame(
        self,
        data: np.ndarray,
        w: int,
        h: int,
        res: float,
        ox: float,
        oy: float,
        rx: float,
        ry: float,
        theta: float,
    ) -> tuple[int, int, int, int, int, int, int, int, int]:
        """Compute left/center/right free and passable counts in robot frame.
        Passable = free or unknown (not occupied). Returns (..., center_passable, left_passable, right_passable).
        """
        sector_rad = math.radians(self._sector_angle_deg)
        left_free = center_free = right_free = 0
        left_passable = right_passable = 0
        left_visited = right_visited = 0
        center_total = 0
        center_passable = 0
        for gy in range(h):
            for gx in range(w):
                wx = ox + gx * res + res * 0.5
                wy = oy + gy * res + res * 0.5
                dx = (wx - rx) * math.cos(theta) + (wy - ry) * math.sin(theta)
                dy = -(wx - rx) * math.sin(theta) + (wy - ry) * math.cos(theta)
                dist = math.hypot(dx, dy)
                if dist < self._front_band_min_m or dist > self._front_band_max_m:
                    continue
                angle = math.atan2(dy, dx)
                val = int(data[gy, gx])
                free = 0 <= val < self._safety_clearance
                passable = val != 100  # free or unknown (not occupied)
                cell = (gx, gy)
                visited = cell in self._visited_cells
                if -sector_rad <= angle <= sector_rad:
                    center_total += 1
                    if free:
                        center_free += 1
                    if passable:
                        center_passable += 1
                elif sector_rad < angle <= 2 * sector_rad:
                    if free:
                        left_free += 1
                        if visited:
                            left_visited += 1
                    if passable:
                        left_passable += 1
                elif -2 * sector_rad <= angle < -sector_rad:
                    if free:
                        right_free += 1
                        if visited:
                            right_visited += 1
                    if passable:
                        right_passable += 1
        return (left_free, center_free, right_free, center_total, left_visited, right_visited, center_passable, left_passable, right_passable)

    def _compute_sectors_robot_centric(
        self, data: np.ndarray, w: int, h: int
    ) -> tuple[int, int, int, int, int, int, int, int, int]:
        """Robot-centric grid (e.g. /occupancy/grid): first rows = front, thirds = L/C/R.
        Returns (..., center_passable, left_passable, right_passable).
        """
        safety_rows = max(1, int(h * self._safety_row_ratio))
        band = data[:safety_rows, :]
        third = max(1, w // 3)
        left_slice = band[:, 0:third]
        center_slice = band[:, third : 2 * third]
        right_slice = band[:, 2 * third : w]
        left_free = int(np.sum((left_slice >= 0) & (left_slice < self._safety_clearance)))
        center_free = int(
            np.sum((center_slice >= 0) & (center_slice < self._safety_clearance))
        )
        right_free = int(
            np.sum((right_slice >= 0) & (right_slice < self._safety_clearance))
        )
        center_total = center_slice.size
        center_passable = int(np.sum(center_slice != 100))
        left_passable = int(np.sum(left_slice != 100))
        right_passable = int(np.sum(right_slice != 100))
        return (left_free, center_free, right_free, center_total, 0, 0, center_passable, left_passable, right_passable)

    def _reactive_behavior(
        self,
        left_free: int,
        center_free: int,
        right_free: int,
        center_total: int,
        left_visited: int,
        right_visited: int,
        center_passable: int,
        left_passable: int,
        right_passable: int,
    ) -> tuple[float, float]:
        """Go forward when path in front is clear; when turning, turn toward longest path (most passable)."""
        center_passable_ratio = center_passable / center_total if center_total else 0.0
        # Forward if enough green/passable in front (ratio or minimum cell count)
        if center_passable_ratio >= self._center_passable_ratio_min or center_passable >= self._min_center_passable_cells:
            return (self._linear_speed, 0.0)
        # Turn toward the longest path (side with more passable space)
        if left_passable >= right_passable and left_passable > 0:
            self._turn_alternate = 1
            return (0.0, self._angular_speed)
        if right_passable > 0:
            self._turn_alternate = -1
            return (0.0, -self._angular_speed)
        # Both sides blocked: alternate to escape
        angular = self._angular_speed * self._turn_alternate
        self._turn_alternate *= -1
        return (0.0, angular)

    def _drive_to_goal(self) -> tuple[float, float] | None:
        """Return (linear, angular) toward current goal, or None if no valid goal."""
        if self._current_goal is None or self._robot_pose is None:
            return None
        now_ns = self.get_clock().now().nanoseconds
        if self._goal_stamp_ns is not None and (now_ns - self._goal_stamp_ns) / 1e9 > self._goal_timeout_s:
            self._current_goal = None
            self._goal_stamp_ns = None
            return None
        gx, gy = self._current_goal
        rx, ry, theta = self._robot_pose
        dx = gx - rx
        dy = gy - ry
        dist = math.hypot(dx, dy)
        if dist < self._goal_reached_distance_m:
            self._current_goal = None
            self._goal_stamp_ns = None
            return None
        heading = math.atan2(dy, dx)
        err = _normalize_angle(heading - theta)
        linear = min(dist * 0.3, self._drive_to_goal_linear_max)
        angular = self._drive_to_goal_k_angular * err
        angular = max(-self._angular_speed * 2.0, min(self._angular_speed * 2.0, angular))
        return (linear, angular)

    def _control_callback(self) -> None:
        if not self._enabled:
            self._publish_stop()
            return
        now_ns = self.get_clock().now().nanoseconds
        if self._last_grid is None:
            self._publish_stop()
            return
        if self._grid_stale_timeout_s > 0:
            age_s = (now_ns - self._last_grid_time_ns) / 1e9
            if age_s > self._grid_stale_timeout_s:
                self._publish_stop()
                return

        grid = self._last_grid
        w, h = grid.info.width, grid.info.height
        if w < 3 or h < 2:
            self._publish_stop()
            return

        data = np.array(grid.data, dtype=np.int8).reshape((h, w))
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        in_map_frame = (grid.header.frame_id == "map") and (self._robot_pose is not None)

        if in_map_frame:
            rx, ry, theta = self._robot_pose
            left_free, center_free, right_free, center_total, left_visited, right_visited, center_passable, left_passable, right_passable = (
                self._compute_sectors_map_frame(data, w, h, res, ox, oy, rx, ry, theta)
            )
        else:
            left_free, center_free, right_free, center_total, left_visited, right_visited, center_passable, left_passable, right_passable = (
                self._compute_sectors_robot_centric(data, w, h)
            )

        center_passable_ratio = center_passable / center_total if center_total else 0.0

        # Stuck detection: update counter and possibly start recovery
        if self._robot_pose is not None:
            x, y, _ = self._robot_pose
            if self._last_pose is not None:
                dist = math.hypot(x - self._last_pose[0], y - self._last_pose[1])
                if dist < self._stuck_distance_m:
                    self._stuck_counter += 1
                else:
                    self._stuck_counter = 0
            self._last_pose = (x, y)

        # Recovery in progress
        if self._recovery_cycles_left > 0:
            self._recovery_cycles_left -= 1
            if self._recovery_cycles_left == 0:
                self._current_goal = None
                self._goal_stamp_ns = None
            self._publish_control(0.0, self._stuck_recovery_angular * self._recovery_angular_sign)
            return

        # Start recovery if stuck
        if self._stuck_counter >= self._stuck_cycles:
            self._stuck_counter = 0
            self._recovery_cycles_left = int(
                self._stuck_recovery_duration_s * self._control_rate_hz
            )
            self._recovery_angular_sign = random.choice([-1.0, 1.0])
            self._publish_control(0.0, self._stuck_recovery_angular * self._recovery_angular_sign)
            return

        # Goal-driven with reactive fallback when obstacle ahead
        cmd_linear, cmd_angular = None, None
        if self._current_goal is not None and center_passable_ratio >= self._center_passable_ratio_min:
            drive = self._drive_to_goal()
            if drive is not None:
                cmd_linear, cmd_angular = drive
        if cmd_linear is None:
            cmd_linear, cmd_angular = self._reactive_behavior(
                left_free, center_free, right_free, center_total, left_visited, right_visited,
                center_passable, left_passable, right_passable
            )
        self._publish_control(cmd_linear, cmd_angular)

    def _publish_control(self, linear: float, angular: float) -> None:
        cmd = {"linear": float(linear), "angular": float(angular), "lamp": 0}
        msg = String()
        msg.data = json.dumps(cmd)
        self._control_pub.publish(msg)

    def _publish_stop(self) -> None:
        self._publish_control(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = WanderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
