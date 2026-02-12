#!/usr/bin/env python3
"""
ROS2 -> App-server bridge: subscribes to ROS2 topics and POSTs to /api/ingest.

Run in a container with ROS2 (rclpy). Set APP_SERVER_URL (e.g. http://app_server:8001).
"""

import json
import logging
import os
import sys
import urllib.error
import urllib.request
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

APP_SERVER_URL = os.environ.get("APP_SERVER_URL", "http://localhost:8001")
INGEST_URL = f"{APP_SERVER_URL.rstrip('/')}/api/ingest"


def post_message(payload: dict) -> bool:
    """POST payload to app-server /api/ingest."""
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
        INGEST_URL,
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            return 200 <= resp.status < 300
    except urllib.error.URLError as e:
        logger.warning("POST to app-server failed: %s", e)
        return False


class Ros2AppBridge(Node):
    """Subscribes to /chatter and forwards to app-server."""

    def __init__(self):
        super().__init__("ros2_app_bridge")
        self.sub = self.create_subscription(
            String,
            "/chatter",
            self.callback,
            10,
        )
        self.get_logger().info(f"Bridge: /chatter -> {INGEST_URL}")

    def callback(self, msg: String) -> None:
        payload = {
            "topic": "/chatter",
            "data": msg.data,
            "timestamp": datetime.now().isoformat(),
            "type": "std_msgs/String",
        }
        if not post_message(payload):
            self.get_logger().warning("Failed to post message to app-server")


def main() -> None:
    if not APP_SERVER_URL:
        logger.error("APP_SERVER_URL not set")
        sys.exit(1)
    rclpy.init()
    node = Ros2AppBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
