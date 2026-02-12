#!/usr/bin/env python3
"""
ROS2 -> App-server bridge: subscribes to ROS2 topics and POSTs to /api/ingest.
Also exposes HTTP POST /control to receive control commands and publish to /robot/control.
Subscribes to /robot/video/front and POSTs raw BGR frames to app-server (single encode: WebRTC only).

Run in a container with ROS2 (rclpy). Set APP_SERVER_URL (e.g. http://app_server:8001).
Control server listens on CONTROL_PORT (default 9000).
"""

import json
import logging
import os
import queue
import sys
import threading
import time
import urllib.error
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from http.server import BaseHTTPRequestHandler, HTTPServer

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

APP_SERVER_URL = os.environ.get("APP_SERVER_URL", "http://localhost:8001")
INGEST_URL = f"{APP_SERVER_URL.rstrip('/')}/api/ingest"
VIDEO_FRAME_RAW_URL = f"{APP_SERVER_URL.rstrip('/')}/api/video/frame/raw"
CONTROL_PORT = int(os.environ.get("CONTROL_PORT", "9000"))

# Target ~30 FPS; send raw BGR so app-server does single encode (WebRTC only)
VIDEO_FRAME_TARGET_FPS = 30.0
VIDEO_FRAME_MIN_INTERVAL_S = 1.0 / VIDEO_FRAME_TARGET_FPS

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


def image_to_bgr_bytes(msg: Image) -> tuple[bytes, int, int] | None:
    """Convert sensor_msgs/Image (bgr8 or rgb8) to raw BGR bytes. Returns (bytes, width, height) or None."""
    if not HAS_CV2:
        return None
    try:
        h, w = msg.height, msg.width
        if msg.encoding in ("bgr8", "rgb8"):
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
            if msg.encoding == "rgb8":
                arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            return arr.tobytes(), w, h
        return None
    except Exception as e:
        logger.warning("image_to_bgr_bytes failed: %s", e)
        return None


def post_video_frame_raw_sync(bgr_body: bytes, width: int, height: int) -> None:
    """POST raw BGR bytes to app-server /api/video/frame/raw (blocking; run in executor). Single encode at server."""
    try:
        req = urllib.request.Request(
            VIDEO_FRAME_RAW_URL,
            data=bgr_body,
            headers={
                "Content-Type": "application/octet-stream",
                "X-Width": str(width),
                "X-Height": str(height),
            },
            method="POST",
        )
        urllib.request.urlopen(req, timeout=2)
    except urllib.error.URLError as e:
        logger.debug("POST video frame raw failed: %s", e)


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
    """Subscribes to /chatter and /robot/video/front; forwards to app-server; publishes /robot/control from control queue.
    Video: encode at up to 30 FPS, POST in background (latest-frame only) for real-time low-latency stream."""

    def __init__(self):
        super().__init__("ros2_app_bridge")
        self.sub = self.create_subscription(
            String,
            "/chatter",
            self.callback,
            10,
        )
        self.control_pub = self.create_publisher(String, "/robot/control", 10)
        # Drain control queue at 50 Hz for minimal latency (robot ← ROS2 ← webapp)
        self.control_timer = self.create_timer(0.02, self._drain_control_queue)
        self._last_video_encode_time = 0.0
        self._video_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="video_post")
        self._video_lock = threading.Lock()
        self._pending_raw: tuple[bytes, int, int] | None = None
        self._post_future = None
        if HAS_CV2:
            # Queue 1 = always process latest frame only (reduces latency)
            self.video_sub = self.create_subscription(
                Image,
                "/robot/video/front",
                self._video_callback,
                1,
            )
            self.get_logger().info(
                f"Bridge: /robot/video/front -> {VIDEO_FRAME_RAW_URL} raw BGR (target {VIDEO_FRAME_TARGET_FPS} FPS, single encode at server)"
            )
        else:
            self.get_logger().warn("cv2 not available; front camera streaming disabled")
        self.get_logger().info(f"Bridge: /chatter -> {INGEST_URL}")
        self.get_logger().info("Bridge: POST /control -> /robot/control")

    def _on_video_post_done(self, future) -> None:
        """When a POST completes, send pending frame if any (keeps stream real-time)."""
        with self._video_lock:
            pending = self._pending_raw
            self._pending_raw = None
            self._post_future = None
        if pending is not None:
            bgr_bytes, w, h = pending
            f = self._video_executor.submit(post_video_frame_raw_sync, bgr_bytes, w, h)
            f.add_done_callback(self._on_video_post_done)
            with self._video_lock:
                self._post_future = f

    def _video_callback(self, msg: Image) -> None:
        """Throttle to target FPS; POST raw BGR in executor (latest-frame only). No encode here → single encode at server."""
        now = time.monotonic()
        if now - self._last_video_encode_time < VIDEO_FRAME_MIN_INTERVAL_S:
            return
        raw = image_to_bgr_bytes(msg)
        if raw is None:
            return
        bgr_bytes, w, h = raw
        self._last_video_encode_time = now
        with self._video_lock:
            in_flight = self._post_future is not None and not self._post_future.done()
            if in_flight:
                self._pending_raw = (bgr_bytes, w, h)
                return
            f = self._video_executor.submit(post_video_frame_raw_sync, bgr_bytes, w, h)
            f.add_done_callback(self._on_video_post_done)
            self._post_future = f

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
        """Drain control queue and publish to /robot/control (50 Hz for real-time)."""
        try:
            while True:
                cmd = control_queue.get_nowait()
                msg = String()
                msg.data = json.dumps(cmd)
                self.control_pub.publish(msg)
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
        if HAS_CV2 and hasattr(node, "_video_executor"):
            node._video_executor.shutdown(wait=False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
