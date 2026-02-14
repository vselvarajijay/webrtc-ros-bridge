#!/usr/bin/env python3
"""
Build 2D occupancy grid from ORB-SLAM3 map points.

Subscribes to /map_points (PointCloud2) and /camera_pose (PoseStamped).
Assumes map_points in map frame. Fits ground plane (z = ax + by + c),
points above plane = obstacles, rasterize to 2D grid, publish /map.
"""

import struct
import threading
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2


def pointcloud2_to_xyz(msg: PointCloud2) -> np.ndarray:
    """Extract Nx3 (x, y, z) float32 from PointCloud2. Returns (N, 3)."""
    # Find offsets for x, y, z (assume FLOAT32 = 7)
    ox = oy = oz = None
    for f in msg.fields:
        if f.name == "x":
            ox = f.offset
        elif f.name == "y":
            oy = f.offset
        elif f.name == "z":
            oz = f.offset
    if ox is None or oy is None or oz is None:
        return np.zeros((0, 3), dtype=np.float32)
    n = len(msg.data) // msg.point_step
    xs = np.zeros(n, dtype=np.float32)
    ys = np.zeros(n, dtype=np.float32)
    zs = np.zeros(n, dtype=np.float32)
    for i in range(n):
        base = i * msg.point_step
        xs[i] = struct.unpack_from("f", msg.data, base + ox)[0]
        ys[i] = struct.unpack_from("f", msg.data, base + oy)[0]
        zs[i] = struct.unpack_from("f", msg.data, base + oz)[0]
    return np.column_stack([xs, ys, zs])


class SlamOccupancyNode(Node):
    """Build occupancy grid from SLAM map points (ground plane + obstacles above)."""

    def __init__(self):
        super().__init__("slam_occupancy_node")

        self.declare_parameter("map_points_topic", "/map_points")
        self.declare_parameter("camera_pose_topic", "/camera_pose")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("occupancy_grid_topic", "/occupancy/grid")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("grid_resolution", 0.05)
        self.declare_parameter("grid_width_m", 20.0)
        self.declare_parameter("grid_height_m", 20.0)
        self.declare_parameter("origin_x_m", -10.0)
        self.declare_parameter("origin_y_m", -10.0)
        self.declare_parameter("ground_height_threshold", 0.15)
        self.declare_parameter("publish_occupancy_grid", False)

        self._lock = threading.Lock()
        self._latest_points: Optional[np.ndarray] = None
        self._latest_pose: Optional[PoseStamped] = None

        map_pts_topic = self.get_parameter("map_points_topic").value
        pose_topic = self.get_parameter("camera_pose_topic").value
        map_topic = self.get_parameter("map_topic").value
        occ_topic = self.get_parameter("occupancy_grid_topic").value

        self._map_pub = self.create_publisher(OccupancyGrid, map_topic, 10)
        self.create_subscription(
            PointCloud2,
            map_pts_topic,
            self._points_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            pose_topic,
            self._pose_callback,
            10,
        )
        self.get_logger().info(
            "slam_occupancy: %s + %s -> %s",
            map_pts_topic,
            pose_topic,
            map_topic,
        )

        if self.get_parameter("publish_occupancy_grid").value:
            self._occ_pub = self.create_publisher(OccupancyGrid, occ_topic, 10)
            self.get_logger().info("Also publishing to %s", occ_topic)
        else:
            self._occ_pub = None

    def _points_callback(self, msg: PointCloud2) -> None:
        xyz = pointcloud2_to_xyz(msg)
        with self._lock:
            self._latest_points = xyz if xyz.size > 0 else None

    def _pose_callback(self, msg: PoseStamped) -> None:
        with self._lock:
            self._latest_pose = msg

    def _fit_ground_plane(self, xyz: np.ndarray) -> tuple[float, float, float]:
        """Fit plane z = a*x + b*y + c via least squares. Return (a, b, c)."""
        if xyz.shape[0] < 3:
            return 0.0, 0.0, 0.0
        x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        A = np.column_stack([x, y, np.ones_like(x)])
        (a, b, c), _, _, _ = np.linalg.lstsq(A, z, rcond=None)
        return float(a), float(b), float(c)

    def _run(self) -> None:
        with self._lock:
            points = self._latest_points
            pose = self._latest_pose
        if points is None or points.shape[0] < 10:
            return
        if pose is None:
            return

        a, b, c = self._fit_ground_plane(points)
        # Signed height above plane: z - (a*x + b*y + c)
        z_pred = a * points[:, 0] + b * points[:, 1] + c
        height_above = points[:, 2] - z_pred
        threshold = self.get_parameter("ground_height_threshold").value
        obstacles = points[height_above > threshold]

        if obstacles.size == 0:
            return

        resolution = self.get_parameter("grid_resolution").value
        width_m = self.get_parameter("grid_width_m").value
        height_m = self.get_parameter("grid_height_m").value
        ox = self.get_parameter("origin_x_m").value
        oy = self.get_parameter("origin_y_m").value
        frame_id = self.get_parameter("frame_id").value

        cols = max(1, int(round(width_m / resolution)))
        rows = max(1, int(round(height_m / resolution)))

        # -1 unknown, 0 free, 100 occupied
        grid = np.full((rows, cols), -1, dtype=np.int8)

        # Obstacle (x, y) -> cell (cx, cy)
        cx = ((obstacles[:, 0] - ox) / resolution).astype(np.int32)
        cy = ((obstacles[:, 1] - oy) / resolution).astype(np.int32)
        valid = (cx >= 0) & (cx < cols) & (cy >= 0) & (cy < rows)
        cx, cy = cx[valid], cy[valid]
        grid[cy, cx] = 100

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.info.resolution = resolution
        msg.info.width = cols
        msg.info.height = rows
        msg.info.origin.position.x = ox
        msg.info.origin.position.y = oy
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.flatten().tolist()

        self._map_pub.publish(msg)
        if self._occ_pub is not None:
            self._occ_pub.publish(msg)

    def run_timer_callback(self) -> None:
        self._run()

    def start_timer(self) -> None:
        self._timer = self.create_timer(0.2, self.run_timer_callback)


def main(args=None):
    rclpy.init(args=args)
    node = SlamOccupancyNode()
    node.start_timer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
