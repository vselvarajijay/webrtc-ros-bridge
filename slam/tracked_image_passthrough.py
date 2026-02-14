#!/usr/bin/env python3
"""
Passthrough: subscribe /robot/video/front, publish /tracked_image.
When ORB-SLAM3 runs in the same network, the app SLAM card can show the camera feed.
Real tracked overlay would require wrapper support; this gives a usable default.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def main():
    rclpy.init()
    node = Node("tracked_image_passthrough")
    pub = node.create_publisher(Image, "/tracked_image", 10)
    node.create_subscription(
        Image,
        "/robot/video/front",
        lambda msg: pub.publish(msg),
        10,
    )
    node.get_logger().info("Passthrough: /robot/video/front -> /tracked_image")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
