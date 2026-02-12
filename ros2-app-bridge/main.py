#!/usr/bin/env python3
"""
ROS2 -> App-server bridge: subscribes to ROS2 topics and POSTs to /api/ingest.
Also exposes HTTP POST /control to receive control commands and publish to /robot/control.

Run in a container with ROS2 (rclpy). Set APP_SERVER_URL (e.g. http://app_server:8001).
Control server listens on CONTROL_PORT (default 9000).
"""

import json
import logging
import os
import queue
import sys
import threading
import urllib.error
import urllib.request
from datetime import datetime
from http.server import BaseHTTPRequestHandler, HTTPServer

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

APP_SERVER_URL = os.environ.get("APP_SERVER_URL", "http://localhost:8001")
INGEST_URL = f"{APP_SERVER_URL.rstrip('/')}/api/ingest"
CONTROL_PORT = int(os.environ.get("CONTROL_PORT", "9000"))

# Thread-safe queue for control commands (HTTP thread pushes, ROS2 timer drains)
control_queue: queue.Queue = queue.Queue()


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


def make_control_handler(q: queue.Queue):
    """Factory for HTTP handler that pushes command dicts into the queue."""

    class ControlHandler(BaseHTTPRequestHandler):
        def log_message(self, format, *args):
            logger.info("%s - %s", self.address_string(), format % args)

        def do_POST(self):
            if self.path != "/control":
                self.send_response(404)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(b'{"error": "Not Found"}\n')
                return
            content_length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_length) if content_length else b""
            try:
                data = json.loads(body.decode("utf-8"))
                cmd = data.get("command")
                if not isinstance(cmd, dict):
                    self.send_response(400)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(b'{"error": "command must be an object"}\n')
                    return
                q.put(cmd)
                response_body = b'{"ok": true}\n'
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(response_body)
                logger.info("Control response: 200 OK %s", response_body.decode("utf-8").strip())
            except (json.JSONDecodeError, ValueError) as e:
                logger.warning("Invalid control body: %s", e)
                self.send_response(400)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"error": str(e)}).encode("utf-8") + b"\n")

    return ControlHandler


def run_control_server(port: int, q: queue.Queue) -> None:
    """Run HTTP server for POST /control in this thread."""
    handler = make_control_handler(q)
    with HTTPServer(("0.0.0.0", port), handler) as server:
        logger.info("Control server listening on port %s", port)
        server.serve_forever()


class Ros2AppBridge(Node):
    """Subscribes to /chatter and forwards to app-server; publishes /robot/control from control queue."""

    def __init__(self):
        super().__init__("ros2_app_bridge")
        self.sub = self.create_subscription(
            String,
            "/chatter",
            self.callback,
            10,
        )
        self.control_pub = self.create_publisher(String, "/robot/control", 10)
        self.control_timer = self.create_timer(0.1, self._drain_control_queue)
        self.get_logger().info(f"Bridge: /chatter -> {INGEST_URL}")
        self.get_logger().info("Bridge: POST /control -> /robot/control")

    def callback(self, msg: String) -> None:
        payload = {
            "topic": "/chatter",
            "data": msg.data,
            "timestamp": datetime.now().isoformat(),
            "type": "std_msgs/String",
        }
        if not post_message(payload):
            self.get_logger().warning("Failed to post message to app-server")

    def _drain_control_queue(self) -> None:
        """Drain control queue and publish to /robot/control (called from ROS2 executor)."""
        try:
            while True:
                cmd = control_queue.get_nowait()
                msg = String()
                msg.data = json.dumps(cmd)
                self.control_pub.publish(msg)
                self.get_logger().info(f"Published control to /robot/control: {cmd}")
        except queue.Empty:
            pass


def main() -> None:
    if not APP_SERVER_URL:
        logger.error("APP_SERVER_URL not set")
        sys.exit(1)
    rclpy.init()
    node = Ros2AppBridge()

    # Start control HTTP server in background thread
    server_thread = threading.Thread(
        target=run_control_server,
        args=(CONTROL_PORT, control_queue),
        daemon=True,
    )
    server_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
