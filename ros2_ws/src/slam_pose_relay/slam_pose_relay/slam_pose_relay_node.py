#!/usr/bin/env python3
"""
SLAM pose relay: subscribe to ORB-SLAM3 camera pose and optionally /occupancy/grid;
publish /map_odom (Odometry) and optionally /map (republish occupancy grid).
Enables the app Map card to show robot pose and grid when using mono SLAM.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry


class SlamPoseRelayNode(Node):
    """Relay SLAM camera pose to /map_odom; optionally republish occupancy grid as /map."""

    def __init__(self):
        super().__init__("slam_pose_relay_node")

        self.declare_parameter("camera_pose_topic", "/camera_pose")
        self.declare_parameter("occupancy_grid_topic", "/occupancy/grid")
        self.declare_parameter("map_odom_topic", "/map_odom")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("map_frame_id", "map")
        self.declare_parameter("child_frame_id", "camera_link")
        self.declare_parameter("republish_occupancy_as_map", True)

        camera_pose_topic = self.get_parameter("camera_pose_topic").value
        occupancy_grid_topic = self.get_parameter("occupancy_grid_topic").value
        map_odom_topic = self.get_parameter("map_odom_topic").value
        map_topic = self.get_parameter("map_topic").value
        republish = self.get_parameter("republish_occupancy_as_map").value

        self._odom_pub = self.create_publisher(Odometry, map_odom_topic, 10)
        self.create_subscription(
            PoseStamped,
            camera_pose_topic,
            self._pose_callback,
            10,
        )
        self.get_logger().info(
            "Relay: %s -> %s (frame_id=%s, child_frame_id=%s)",
            camera_pose_topic,
            map_odom_topic,
            self.get_parameter("map_frame_id").value,
            self.get_parameter("child_frame_id").value,
        )

        self._map_pub = None
        if republish:
            self._map_pub = self.create_publisher(OccupancyGrid, map_topic, 10)
            self.create_subscription(
                OccupancyGrid,
                occupancy_grid_topic,
                self._grid_callback,
                10,
            )
            self.get_logger().info("Relay: %s -> %s", occupancy_grid_topic, map_topic)
        else:
            self.get_logger().info("Republish occupancy as map disabled")

    def _pose_callback(self, msg: PoseStamped) -> None:
        """Convert PoseStamped (SLAM camera pose) to Odometry for /map_odom."""
        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = self.get_parameter("map_frame_id").value
        odom.child_frame_id = self.get_parameter("child_frame_id").value
        odom.pose.pose = msg.pose
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = 0.0
        self._odom_pub.publish(odom)

    def _grid_callback(self, msg: OccupancyGrid) -> None:
        """Republish occupancy grid to /map for bridge and Map card."""
        if self._map_pub is not None:
            out = OccupancyGrid()
            out.header = msg.header
            if out.header.frame_id == "":
                out.header.frame_id = self.get_parameter("map_frame_id").value
            out.info = msg.info
            out.data = msg.data
            self._map_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SlamPoseRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
