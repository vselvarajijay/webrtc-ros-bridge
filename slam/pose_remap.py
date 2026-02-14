#!/usr/bin/env python3
"""
Remap ORB-SLAM3 pose topic to /camera_pose for webrtc-ros-bridge.
Subscribe: /robot_pose_slam (geometry_msgs/PoseStamped) from suchetanrs wrapper.
Publish:   /camera_pose (geometry_msgs/PoseStamped).
Run in the SLAM container so the rest of the stack sees /camera_pose.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


def main():
    rclpy.init()
    node = Node("slam_pose_remap")
    pub = node.create_publisher(PoseStamped, "/camera_pose", 10)
    node.create_subscription(
        PoseStamped,
        "/robot_pose_slam",
        lambda msg: pub.publish(msg),
        10,
    )
    node.get_logger().info("Remap: /robot_pose_slam -> /camera_pose")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
