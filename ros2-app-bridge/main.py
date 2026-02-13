#!/usr/bin/env python3
"""
ROS2 -> App-server bridge: subscribes to ROS2 topics and forwards via WebRTC (preferred) or HTTP (fallback).

ARCHITECTURE:
- WebRTC is the PRIMARY method for video streaming and control (low latency, real-time)
- HTTP POST is kept as a FALLBACK for debugging and compatibility when WebRTC is unavailable
- Video: WebRTC video tracks (preferred) -> HTTP POST /api/video/frame/raw (fallback)
- Control: WebRTC data channel (preferred) -> HTTP POST /control (fallback)

Run in a container with ROS2 (rclpy). Set APP_SERVER_URL (e.g. http://app_server:8001).
Control server listens on CONTROL_PORT (default 9000).
"""

import asyncio
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
from fractions import Fraction
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import Callable, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# WebRTC support
try:
    from av import VideoFrame
    from aiortc import MediaStreamTrack, RTCPeerConnection, RTCSessionDescription
    import httpx
    HAS_WEBRTC = True
    logger.info("WebRTC libraries imported successfully")
except ImportError as e:
    HAS_WEBRTC = False
    logger.error(f"WebRTC import failed: {e}", exc_info=True)
    logger.error("WebRTC will not be available - using HTTP fallback only")
    MediaStreamTrack = None
    RTCPeerConnection = None
    RTCSessionDescription = None
    VideoFrame = None
    httpx = None

APP_SERVER_URL = os.environ.get("APP_SERVER_URL", "http://localhost:8001")
INGEST_URL = f"{APP_SERVER_URL.rstrip('/')}/api/ingest"
VIDEO_FRAME_RAW_URL = f"{APP_SERVER_URL.rstrip('/')}/api/video/frame/raw"
VIDEO_OCCUPANCY_RAW_URL = f"{APP_SERVER_URL.rstrip('/')}/api/video/occupancy/frame/raw"
WEBRTC_OFFER_URL = f"{APP_SERVER_URL.rstrip('/')}/api/webrtc/ros2-bridge/offer"
CONTROL_PORT = int(os.environ.get("CONTROL_PORT", "9000"))

# Target ~30 FPS; send raw BGR so app-server does single encode (WebRTC only)
VIDEO_FRAME_TARGET_FPS = 30.0
VIDEO_FRAME_MIN_INTERVAL_S = 1.0 / VIDEO_FRAME_TARGET_FPS

# Thread-safe queue for control commands (HTTP thread pushes, ROS2 timer drains)
control_queue: queue.Queue = queue.Queue()

# WebRTC peer connections and data channels
if HAS_WEBRTC:
    webrtc_pc_front: Optional[RTCPeerConnection] = None
    webrtc_pc_occupancy: Optional[RTCPeerConnection] = None
    webrtc_data_channel: Optional = None
    webrtc_connected = False
    webrtc_lock = threading.Lock()
else:
    webrtc_pc_front = None
    webrtc_pc_occupancy = None
    webrtc_data_channel = None
    webrtc_connected = False
    webrtc_lock = threading.Lock()  # Still create lock even if WebRTC not available


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
    """Convert sensor_msgs/Image (bgr8, rgb8, or 8UC3) to raw BGR bytes. Returns (bytes, width, height) or None."""
    if not HAS_CV2:
        return None
    try:
        h, w = msg.height, msg.width
        if msg.encoding in ("bgr8", "rgb8"):
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
            if msg.encoding == "rgb8":
                arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            return arr.tobytes(), w, h
        # cv_bridge sometimes uses 8UC3 for 3-channel uint8; treat as BGR
        if msg.encoding == "8UC3":
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3)).copy()
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


def post_occupancy_frame_raw_sync(bgr_body: bytes, width: int, height: int) -> None:
    """
    POST raw BGR bytes to app-server /api/video/occupancy/frame/raw (blocking; run in executor).
    
    NOTE: This is a FALLBACK method. WebRTC video tracks are preferred for video streaming.
    HTTP POST is kept for debugging and compatibility when WebRTC is not available.
    """
    try:
        req = urllib.request.Request(
            VIDEO_OCCUPANCY_RAW_URL,
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
        logger.debug("POST occupancy frame raw failed: %s", e)


# WebRTC LiveFrameTrack for forwarding ROS2 video frames
if HAS_WEBRTC:
    class LiveFrameTrack(MediaStreamTrack):
        """Video track that yields frames from ROS2 topics as av.VideoFrame."""
        kind = "video"
        
        def __init__(self, get_latest_bgr: Callable[[], Optional[np.ndarray]]):
            super().__init__()
            self._get_latest_bgr = get_latest_bgr
            self._pts = 0
            # time_base must be a Fraction, not a float
            self._time_base = Fraction(1, int(VIDEO_FRAME_TARGET_FPS))
        
        async def recv(self):
            bgr = self._get_latest_bgr()
            if bgr is not None:
                arr = bgr.copy()
            else:
                # Placeholder gray frame
                arr = np.full((480, 640, 3), 128, dtype=np.uint8)
            frame = VideoFrame.from_ndarray(arr, format="bgr24")
            frame.pts = self._pts
            frame.time_base = self._time_base
            self._pts += 1
            return frame


async def setup_webrtc_connection(stream_type: str, get_latest_bgr: Callable[[], Optional[np.ndarray]]) -> bool:
    """Setup WebRTC peer connection to App Server for video streaming."""
    if not HAS_WEBRTC:
        logger.warning("WebRTC not available, skipping WebRTC connection setup")
        return False
    
    global webrtc_pc_front, webrtc_pc_occupancy, webrtc_data_channel, webrtc_connected
    
    try:
        pc = RTCPeerConnection()
        
        # Create video track
        track = LiveFrameTrack(get_latest_bgr)
        pc.addTrack(track)
        logger.info(f"WebRTC {stream_type} track added to peer connection")
        
        # Create data channel for control (only on front connection)
        if stream_type == "front":
            dc = pc.createDataChannel("control", ordered=True)
            webrtc_data_channel = dc
            
            @dc.on("open")
            def on_open():
                global webrtc_connected
                logger.info("WebRTC control data channel opened successfully")
                webrtc_connected = True
            
            @dc.on("close")
            def on_close():
                global webrtc_connected
                logger.warning("WebRTC control data channel closed")
                webrtc_connected = False
            
            @dc.on("error")
            def on_error(error):
                logger.error("WebRTC control data channel error: %s", error)
            
            @dc.on("message")
            def on_message(message):
                # Control commands from App Server via WebRTC data channel
                try:
                    if isinstance(message, str):
                        data = json.loads(message)
                        cmd = data.get("command")
                        if isinstance(cmd, dict):
                            logger.info("WebRTC control command received from App Server: linear=%.2f, angular=%.2f, lamp=%d",
                                       cmd.get("linear", 0), cmd.get("angular", 0), cmd.get("lamp", 0))
                            # Put command in queue for ROS2 publishing
                            control_queue.put(cmd)
                        else:
                            logger.warning("WebRTC control message missing 'command' field")
                    else:
                        logger.warning("WebRTC control message is not a string: %s", type(message))
                except json.JSONDecodeError as e:
                    logger.error("WebRTC control message JSON parse error: %s", e)
                except Exception as e:
                    logger.error("WebRTC control message handling error: %s", e)
        
        # Create offer
        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)
        
        # Send offer to App Server
        async with httpx.AsyncClient(timeout=10.0) as client:
            response = await client.post(
                WEBRTC_OFFER_URL,
                json={
                    "sdp": pc.localDescription.sdp,
                    "type": pc.localDescription.type,
                },
            )
            if response.status_code != 200:
                logger.error("WebRTC offer failed: %s %s", response.status_code, response.text)
                await pc.close()
                return False
            
            answer_data = response.json()
            answer = RTCSessionDescription(sdp=answer_data["sdp"], type=answer_data["type"])
            await pc.setRemoteDescription(answer)
        
        # Store peer connection
        with webrtc_lock:
            if stream_type == "front":
                webrtc_pc_front = pc
            elif stream_type == "occupancy":
                webrtc_pc_occupancy = pc
        
        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            state = pc.connectionState
            logger.info(f"WebRTC {stream_type} connection state changed: {state}")
            if state in ("failed", "closed", "disconnected"):
                logger.warning(f"WebRTC {stream_type} connection {state}, will attempt reconnection")
                with webrtc_lock:
                    if stream_type == "front":
                        webrtc_pc_front = None
                    elif stream_type == "occupancy":
                        webrtc_pc_occupancy = None
                await pc.close()
        
        logger.info(f"WebRTC {stream_type} connection established successfully")
        return True
        
    except Exception as e:
        logger.error(f"Failed to setup WebRTC {stream_type} connection: {e}", exc_info=True)
        return False


def make_control_handler(q: queue.Queue):
    """
    Factory for HTTP handler that pushes command dicts into the queue.
    
    NOTE: This is a FALLBACK handler. WebRTC data channel is preferred for control.
    HTTP is kept for debugging and compatibility when WebRTC is not available.
    """

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
    """Subscribes to /chatter and /robot/video/front; forwards to app-server via WebRTC (preferred) or HTTP (fallback).
    Video: forwards via WebRTC video tracks (preferred) or HTTP POST (fallback).
    Control: receives via WebRTC data channel (preferred) or HTTP POST (fallback)."""

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
        
        # Latest BGR frames for WebRTC tracks
        self._latest_front_bgr: Optional[np.ndarray] = None
        self._latest_occupancy_bgr: Optional[np.ndarray] = None
        self._bgr_lock = threading.Lock()
        
        # HTTP fallback (keep for compatibility)
        self._last_video_encode_time = 0.0
        self._video_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="video_post")
        self._video_lock = threading.Lock()
        self._pending_raw: tuple[bytes, int, int] | None = None
        self._post_future = None
        self._last_occupancy_encode_time = 0.0
        self._occupancy_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="occupancy_post")
        self._occupancy_lock = threading.Lock()
        self._pending_occupancy_raw: tuple[bytes, int, int] | None = None
        self._occupancy_post_future = None
        
        # WebRTC event loop (run in separate thread)
        self._webrtc_loop: Optional[asyncio.AbstractEventLoop] = None
        self._webrtc_thread: Optional[threading.Thread] = None
        if HAS_CV2:
            # Use sensor_data QoS (BEST_EFFORT) to match video and occupancy publishers
            sensor_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )
            # Queue depth 10; we throttle in callback to target FPS
            self.video_sub = self.create_subscription(
                Image,
                "/robot/video/front",
                self._video_callback,
                sensor_qos,
            )
            self.get_logger().info("✓ Subscribed to ROS2 topic: /robot/video/front")
            self.get_logger().info(
                f"Bridge: /robot/video/front -> {VIDEO_FRAME_RAW_URL} raw BGR (target {VIDEO_FRAME_TARGET_FPS} FPS, single encode at server)"
            )
            self.occupancy_sub = self.create_subscription(
                Image,
                "/occupancy/annotated_image",
                self._occupancy_callback,
                sensor_qos,
            )
            self.get_logger().info("✓ Subscribed to ROS2 topic: /occupancy/annotated_image")
            self.get_logger().info(
                f"Bridge: /occupancy/annotated_image -> {VIDEO_OCCUPANCY_RAW_URL} raw BGR (target {VIDEO_FRAME_TARGET_FPS} FPS)"
            )
        else:
            self.get_logger().warn("cv2 not available; front camera streaming disabled")
        self.get_logger().info(f"Bridge: /chatter -> {INGEST_URL}")
        self.get_logger().info("Bridge: POST /control -> /robot/control (HTTP fallback)")
        
        # Initialize WebRTC connections if available
        if HAS_WEBRTC and HAS_CV2:
            self._init_webrtc()
        else:
            if not HAS_WEBRTC:
                self.get_logger().warn("WebRTC not available, using HTTP fallback only")
            if not HAS_CV2:
                self.get_logger().warn("cv2 not available, WebRTC disabled")
    
    def _init_webrtc(self):
        """Initialize WebRTC connections to App Server in background thread."""
        def run_webrtc_loop():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            self._webrtc_loop = loop

            def _asyncio_exception_handler(loop, context):
                exc = context.get("exception")
                if exc is not None and isinstance(exc, asyncio.InvalidStateError):
                    msg = context.get("message", "")
                    handle_str = str(context.get("handle", ""))
                    if "Transaction" in msg or "stun" in msg or "Transaction" in handle_str or "__retry" in handle_str:
                        logger.debug("aioice STUN retry race (ignored): %s", msg or handle_str)
                        return
                loop.default_exception_handler(context)
            loop.set_exception_handler(_asyncio_exception_handler)

            # Setup front camera connection
            def get_front_bgr():
                with self._bgr_lock:
                    return self._latest_front_bgr.copy() if self._latest_front_bgr is not None else None
            
            # Setup occupancy connection
            def get_occupancy_bgr():
                with self._bgr_lock:
                    return self._latest_occupancy_bgr.copy() if self._latest_occupancy_bgr is not None else None
            
            async def setup_connections():
                # Wait a bit for ROS2 to initialize
                await asyncio.sleep(2)
                
                # Setup front camera WebRTC connection
                front_success = await setup_webrtc_connection("front", get_front_bgr)
                if front_success:
                    self.get_logger().info("WebRTC front camera connection established")
                else:
                    self.get_logger().warn("WebRTC front camera connection failed, using HTTP fallback")
                
                # Setup occupancy WebRTC connection
                occupancy_success = await setup_webrtc_connection("occupancy", get_occupancy_bgr)
                if occupancy_success:
                    self.get_logger().info("WebRTC occupancy connection established")
                else:
                    self.get_logger().warn("WebRTC occupancy connection failed, using HTTP fallback")
            
            try:
                loop.run_until_complete(setup_connections())
                # Keep loop running for reconnection logic
                loop.run_forever()
            except Exception as e:
                self.get_logger().error(f"WebRTC loop error: {e}", exc_info=True)
            finally:
                loop.close()
        
        self._webrtc_thread = threading.Thread(target=run_webrtc_loop, daemon=True, name="webrtc-loop")
        self._webrtc_thread.start()
        self.get_logger().info("WebRTC initialization thread started")

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
        """Update latest BGR frame for WebRTC track, fallback to HTTP POST if WebRTC not available."""
        # Log every callback to verify it's being called
        if not hasattr(self, '_callback_count_front'):
            self._callback_count_front = 0
        self._callback_count_front += 1
        if self._callback_count_front <= 5 or self._callback_count_front % 100 == 0:
            self.get_logger().info(f"Front camera callback #{self._callback_count_front} triggered - msg encoding: {msg.encoding}, size: {msg.width}x{msg.height}")
        
        raw = image_to_bgr_bytes(msg)
        if raw is None:
            self.get_logger().warn(f"Front camera frame conversion failed - encoding: {msg.encoding}")
            return
        bgr_bytes, w, h = raw
        # Log first frame and then every 30 frames (once per second at 30fps) to reduce spam
        if not hasattr(self, '_frame_count_front'):
            self._frame_count_front = 0
        self._frame_count_front += 1
        if self._frame_count_front == 1 or self._frame_count_front % 30 == 0:
            self.get_logger().info(f"Received front camera frame #{self._frame_count_front} from ROS2: {w}x{h}, encoding={msg.encoding}")
        
        # Convert bytes to numpy array for WebRTC track
        bgr_array = np.frombuffer(bgr_bytes, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        
        # Update latest BGR frame for WebRTC track
        with self._bgr_lock:
            self._latest_front_bgr = bgr_array
        
        # Always use HTTP fallback to ensure frames reach app-server
        # WebRTC is preferred but HTTP ensures reliability
        use_http = True
        
        if use_http:
            now = time.monotonic()
            if now - self._last_video_encode_time < VIDEO_FRAME_MIN_INTERVAL_S:
                return
            self._last_video_encode_time = now
            with self._video_lock:
                in_flight = self._post_future is not None and not self._post_future.done()
                if in_flight:
                    self._pending_raw = (bgr_bytes, w, h)
                    return
                # Log every 30 frames to reduce spam
                if not hasattr(self, '_http_frame_count_front'):
                    self._http_frame_count_front = 0
                self._http_frame_count_front = getattr(self, '_http_frame_count_front', 0) + 1
                if self._http_frame_count_front == 1 or self._http_frame_count_front % 30 == 0:
                    self.get_logger().info(f"Forwarding front camera frame #{self._http_frame_count_front} via HTTP: {w}x{h}, size={len(bgr_bytes)} bytes")
                f = self._video_executor.submit(post_video_frame_raw_sync, bgr_bytes, w, h)
                f.add_done_callback(self._on_video_post_done)
                self._post_future = f

    def _on_occupancy_post_done(self, future) -> None:
        """When occupancy POST completes, send pending frame if any."""
        with self._occupancy_lock:
            pending = self._pending_occupancy_raw
            self._pending_occupancy_raw = None
            self._occupancy_post_future = None
        if pending is not None:
            bgr_bytes, w, h = pending
            f = self._occupancy_executor.submit(post_occupancy_frame_raw_sync, bgr_bytes, w, h)
            f.add_done_callback(self._on_occupancy_post_done)
            with self._occupancy_lock:
                self._occupancy_post_future = f

    def _occupancy_callback(self, msg: Image) -> None:
        """Update latest occupancy BGR frame for WebRTC track, fallback to HTTP POST if WebRTC not available."""
        # Log every callback to verify it's being called
        if not hasattr(self, '_callback_count_occupancy'):
            self._callback_count_occupancy = 0
        self._callback_count_occupancy += 1
        if self._callback_count_occupancy <= 5 or self._callback_count_occupancy % 100 == 0:
            self.get_logger().info(f"Occupancy callback #{self._callback_count_occupancy} triggered - msg encoding: {msg.encoding}, size: {msg.width}x{msg.height}")
        
        raw = image_to_bgr_bytes(msg)
        if raw is None:
            self.get_logger().warn(
                "Occupancy frame skipped (encoding=%s); expected bgr8/rgb8/8UC3"
                % getattr(msg, "encoding", "?"),
                throttle_duration_sec=10.0,
            )
            return
        bgr_bytes, w, h = raw
        # Log first frame and then every 30 frames to reduce spam
        if not hasattr(self, '_frame_count_occupancy'):
            self._frame_count_occupancy = 0
        self._frame_count_occupancy += 1
        if self._frame_count_occupancy == 1 or self._frame_count_occupancy % 30 == 0:
            self.get_logger().info(f"Received occupancy frame #{self._frame_count_occupancy} from ROS2: {w}x{h}, encoding={msg.encoding}")
        
        # Convert bytes to numpy array for WebRTC track
        bgr_array = np.frombuffer(bgr_bytes, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        
        # Update latest BGR frame for WebRTC track
        with self._bgr_lock:
            self._latest_occupancy_bgr = bgr_array
        
        # Always use HTTP fallback to ensure frames reach app-server
        # WebRTC is preferred but HTTP ensures reliability
        use_http = True
        
        if use_http:
            now = time.monotonic()
            if now - self._last_occupancy_encode_time < VIDEO_FRAME_MIN_INTERVAL_S:
                return
            self._last_occupancy_encode_time = now
            with self._occupancy_lock:
                in_flight = self._occupancy_post_future is not None and not self._occupancy_post_future.done()
                if in_flight:
                    self._pending_occupancy_raw = (bgr_bytes, w, h)
                    return
                # Log every 30 frames to reduce spam
                if not hasattr(self, '_http_frame_count_occupancy'):
                    self._http_frame_count_occupancy = 0
                self._http_frame_count_occupancy = getattr(self, '_http_frame_count_occupancy', 0) + 1
                if self._http_frame_count_occupancy == 1 or self._http_frame_count_occupancy % 30 == 0:
                    self.get_logger().info(f"Forwarding occupancy frame #{self._http_frame_count_occupancy} via HTTP: {w}x{h}, size={len(bgr_bytes)} bytes")
                f = self._occupancy_executor.submit(post_occupancy_frame_raw_sync, bgr_bytes, w, h)
                f.add_done_callback(self._on_occupancy_post_done)
                self._occupancy_post_future = f

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
        """Drain control queue and publish to /robot/control (50 Hz for real-time).
        When a stop (linear=0, angular=0) is seen, clear any pending commands so only stop is applied immediately."""
        try:
            while True:
                cmd = control_queue.get_nowait()
                is_stop = (float(cmd.get("linear", 0)) == 0.0 and float(cmd.get("angular", 0)) == 0.0)
                if is_stop:
                    # Drop any queued commands so robot stops immediately; publish only this stop
                    try:
                        while True:
                            control_queue.get_nowait()
                    except queue.Empty:
                        pass
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
        if HAS_CV2 and hasattr(node, "_occupancy_executor"):
            node._occupancy_executor.shutdown(wait=False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
