#!/usr/bin/env python3
"""
Depth-to-occupancy node: subscribes to metric depth image, publishes robot-centric occupancy grid.
Placeholder implementation: publishes an empty grid until depth processing is implemented.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image


class DepthToOccupancyNode(Node):
    """Convert metric depth (/depth/image_raw) to robot-centric occupancy grid."""

    def __init__(self):
        super().__init__("depth_to_occupancy_node")
        self.declare_parameter("depth_topic", "/depth/image_raw")
        self.declare_parameter("occupancy_topic", "/occupancy/grid")
        self.declare_parameter("frame_id", "camera_link")
        depth_topic = self.get_parameter("depth_topic").value
        occ_topic = self.get_parameter("occupancy_topic").value
        self._pub = self.create_publisher(OccupancyGrid, occ_topic, 10)
        self._sub = self.create_subscription(Image, depth_topic, self._depth_callback, 10)
        self.get_logger().info(
            "depth_to_occupancy: %s -> %s (placeholder)",
            depth_topic,
            occ_topic,
        )

    def _depth_callback(self, msg: Image) -> None:
        # Placeholder: publish empty grid; replace with depth-to-BEV occupancy logic
        grid = OccupancyGrid()
        grid.header = msg.header
        grid.header.frame_id = self.get_parameter("frame_id").value
        grid.info.resolution = 0.05
        grid.info.width = 80
        grid.info.height = 120
        grid.info.origin.position.x = -2.0
        grid.info.origin.position.y = 0.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [-1] * (80 * 120)
        self._pub.publish(grid)


def main(args=None):
    rclpy.init(args=args)
    node = DepthToOccupancyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
