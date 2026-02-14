#!/usr/bin/env python3
"""
Pure FastAPI app-server: WebSocket for frontend, HTTP ingest for bridge.

No ROS2 dependencies. The bridge service (separate container) subscribes to
ROS2 and POSTs messages here; we broadcast to WebSocket clients.
"""

import asyncio
import base64
import io
import json
import logging
import os
import time
import urllib.error
import urllib.request
from contextlib import asynccontextmanager
from datetime import datetime
from fractions import Fraction
from typing import Callable, List, Set

import httpx
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, StreamingResponse
from pydantic import BaseModel

# WebRTC: live video track and signaling
try:
    import numpy as np
    from av import VideoFrame
    from aiortc import MediaStreamTrack, RTCDataChannel, RTCPeerConnection, RTCSessionDescription
    HAS_WEBRTC = True
except ImportError:
    HAS_WEBRTC = False
    np = None
    VideoFrame = None
    MediaStreamTrack = None
    RTCDataChannel = None
    RTCPeerConnection = None
    RTCSessionDescription = None

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Store active WebSocket connections (frontend clients)
active_connections: Set[WebSocket] = set()

# MJPEG front camera stream: latest frame and per-client queues
latest_front_jpeg: bytes | None = None
video_stream_subscribers: List[asyncio.Queue[bytes]] = []
_last_frame_post_time: float = 0.0  # when we last received a frame from ros2_app_bridge (POST)
# WebRTC: pre-decoded BGR (updated once per POST) so track recv() doesn't decode every time → lower latency
latest_front_bgr: "np.ndarray | None" = None

# Occupancy stream: annotated video (floor overlay + BEV) from ros2_app_bridge
latest_occupancy_bgr: "np.ndarray | None" = None
video_stream_subscribers_occupancy: List[asyncio.Queue[bytes]] = []
_last_occupancy_frame_time: float | None = None  # monotonic time when we last received a frame

# Map and robot pose from ros2-app-bridge (POST /api/map/update)
latest_map: dict | None = None  # { width, height, resolution, origin: {x,y}, data }
latest_robot_pose: dict | None = None  # { x, y, theta }

# Minimal 1x1 gray JPEG so stream always sends something before first real frame
_PLACEHOLDER_JPEG = base64.b64decode(
    "/9j/4AAQSkZJRgABAQEASABIAAD/2wBDAAgGBgcGBQgHBwcJCQgKDBQNDAsLDBkSEw8UHRofHh0aHBwgJC4nICIsIxwcKDcpLDAxNDQ0Hyc5PTgyPC4zNDL/2wBDAQkJCQwLDBgNDRgyIRwhMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjL/wAARCAABAAEDASIAAhEBAxEB/8QAFQABAQAAAAAAAAAAAAAAAAAAAAv/xAAUEAEAAAAAAAAAAAAAAAAAAAAA/8QAFQEBAQAAAAAAAAAAAAAAAAAAAAX/2gAMAwEAAhEDEQA/ALJ//9k="
)

# URL of ros2-app-bridge control endpoint (e.g. http://ros2_app_bridge:9000)
ROS2_APP_BRIDGE_URL = os.environ.get("ROS2_APP_BRIDGE_URL", "http://ros2_app_bridge:9000").rstrip("/")
# Optional: Frodobots service URL for fallback front camera (when ROS2 path has no frames)
FRODOBOTS_URL = os.environ.get("FRODOBOTS_URL", "").rstrip("/")
_last_control_forward_warning_time: float = 0.0
CONTROL_FORWARD_WARNING_INTERVAL = 10.0  # log at most once per 10s when bridge unreachable
def _video_fallback_interval() -> float:
    """Fetch interval in seconds; env VIDEO_FALLBACK_INTERVAL overrides (e.g. 0.033 for 30 FPS)."""
    v = os.environ.get("VIDEO_FALLBACK_INTERVAL")
    if v:
        try:
            return float(v)
        except ValueError:
            pass
    return 0.033  # ~30 FPS default for real-time streaming
VIDEO_FALLBACK_INTERVAL = _video_fallback_interval()

# WebRTC: peer connections for cleanup on shutdown
webrtc_pcs: Set = set()

# WebRTC: data channel references keyed by peer connection
webrtc_data_channels: dict = {}

# WebRTC: ROS2 App Bridge peer connections (for forwarding video and control)
ros2_bridge_pcs: dict = {}  # Key: "front" or "occupancy", Value: RTCPeerConnection
ros2_bridge_data_channels: dict = {}  # Key: RTCPeerConnection, Value: RTCDataChannel

# WebRTC statistics tracking
_webrtc_stats = {"webrtc_messages": 0, "http_requests": 0}

# Reused HTTP client for control forwarding (keep-alive, low latency)
_control_http_client: httpx.AsyncClient | None = None


def _jpeg_to_bgr24(jpeg: bytes) -> tuple[np.ndarray, int, int] | None:
    """Decode JPEG bytes to BGR24 numpy array. Returns (array, width, height) or None."""
    if not jpeg or len(jpeg) < 2:
        return None
    try:
        from PIL import Image
        img = Image.open(io.BytesIO(jpeg)).convert("RGB")
        arr = np.array(img, dtype=np.uint8)
        bgr = arr[:, :, ::-1].copy()
        h, w = bgr.shape[:2]
        return bgr, w, h
    except Exception as e:
        logger.debug("jpeg decode failed: %s", e)
        return None


def _bgr_to_jpeg(bgr: np.ndarray, quality: int = 85) -> bytes:
    """Encode BGR24 numpy array to JPEG bytes. Used only for MJPEG subscribers."""
    try:
        from PIL import Image
        rgb = bgr[:, :, ::-1].copy()
        img = Image.fromarray(rgb)
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=quality)
        return buf.getvalue()
    except Exception as e:
        logger.debug("bgr_to_jpeg failed: %s", e)
        return b""


if HAS_WEBRTC:
    class LiveFrameTrack(MediaStreamTrack):
        """Video track that yields the latest frame as av.VideoFrame (30fps). Uses pre-decoded BGR when set to avoid decode-in-recv latency."""

        kind = "video"

        def __init__(self, get_latest_bgr: Callable[[], np.ndarray | None]):
            super().__init__()
            self._get_latest_bgr = get_latest_bgr
            self._pts = 0
            # time_base must be a Fraction, not a float
            self._time_base = Fraction(1, 30)

        async def recv(self):
            bgr = self._get_latest_bgr()
            if bgr is not None:
                arr = bgr.copy()
            else:
                arr = np.full((480, 640, 3), 128, dtype=np.uint8)
            frame = VideoFrame.from_ndarray(arr, format="bgr24")
            frame.pts = self._pts
            frame.time_base = self._time_base
            self._pts += 1
            return frame


def _raw_image_to_jpeg(raw: bytes) -> bytes:
    """Convert raw image bytes (JPEG or PNG) to JPEG. Returns as-is if already JPEG."""
    if raw[:2] == b"\xff\xd8":
        return raw
    try:
        from PIL import Image

        img = Image.open(io.BytesIO(raw))
        if img.mode in ("RGBA", "P"):
            img = img.convert("RGB")
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=85)
        return buf.getvalue()
    except Exception as e:
        logger.debug("_raw_image_to_jpeg: %s", e)
        return raw


async def _video_fallback_loop() -> None:
    """Periodically fetch front frame from Frodobots and push to MJPEG stream."""
    if not FRODOBOTS_URL:
        return
    url = f"{FRODOBOTS_URL}/v2/front"
    logger.info("Video fallback enabled: fetching from %s", url)
    last_log = 0.0
    async with httpx.AsyncClient(timeout=5.0) as client:
        while True:
            try:
                r = await client.get(url)
                if r.status_code != 200:
                    await asyncio.sleep(VIDEO_FALLBACK_INTERVAL)
                    continue
                data = r.json()
                b64 = data.get("front_frame")
                if not b64:
                    await asyncio.sleep(VIDEO_FALLBACK_INTERVAL)
                    continue
                raw = base64.b64decode(b64)
                jpeg = _raw_image_to_jpeg(raw)
                global latest_front_jpeg, latest_front_bgr
                # Prefer ROS2 path: skip fallback update if we got a frame from bridge recently
                if time.monotonic() - _last_frame_post_time < 2.0:
                    await asyncio.sleep(VIDEO_FALLBACK_INTERVAL)
                    continue
                latest_front_jpeg = jpeg
                # Feed WebRTC track when aiortc is available so browser gets real-time frames
                if HAS_WEBRTC:
                    result = _jpeg_to_bgr24(jpeg)
                    if result:
                        latest_front_bgr = result[0].copy()
                chunk = _mjpeg_chunk(jpeg)
                dead: List[int] = []
                for i, q in enumerate(video_stream_subscribers):
                    try:
                        q.put_nowait(chunk)
                    except Exception:
                        dead.append(i)
                for i in reversed(dead):
                    video_stream_subscribers.pop(i)
            except asyncio.CancelledError:
                break
            except Exception as e:
                now = time.monotonic()
                if now - last_log > 10.0:
                    logger.warning("Video fallback fetch failed: %s", e)
                    last_log = now
            await asyncio.sleep(VIDEO_FALLBACK_INTERVAL)


async def broadcast_message(message: dict) -> None:
    """Broadcast message to all connected WebSocket clients."""
    if not active_connections:
        return
    message_json = json.dumps(message)
    disconnected: Set[WebSocket] = set()
    for connection in active_connections:
        try:
            await connection.send_text(message_json)
        except Exception as e:
            logger.error("Error sending to client: %s", e)
            disconnected.add(connection)
    active_connections.difference_update(disconnected)


class IngestMessage(BaseModel):
    """Payload the bridge sends to /api/ingest (topic, data, timestamp, type)."""

    topic: str
    data: str
    timestamp: str | None = None
    type: str = "std_msgs/String"


class ControlCommand(BaseModel):
    """Frodobots control: linear and angular in [-1, 1], lamp 0 or 1."""

    linear: float = 0.0
    angular: float = 0.0
    lamp: int = 0


class ControlBody(BaseModel):
    """POST /api/control body: { \"command\": { linear, angular, lamp } }."""

    command: ControlCommand


_video_fallback_task: asyncio.Task | None = None


def _asyncio_exception_handler(loop, context):
    """Log asyncio exceptions; suppress noisy aioice STUN InvalidStateError."""
    exc = context.get("exception")
    if exc is not None and isinstance(exc, asyncio.InvalidStateError):
        msg = context.get("message", "")
        handle_str = str(context.get("handle", ""))
        if "Transaction" in msg or "stun" in msg or "Transaction" in handle_str or "__retry" in handle_str:
            logger.debug("aioice STUN retry race (ignored): %s", msg or handle_str)
            return
    loop.default_exception_handler(context)


@asynccontextmanager
async def lifespan(app: FastAPI):
    global _video_fallback_task, _control_http_client
    logger.info("App-server starting (pure FastAPI, no ROS2)")
    loop = asyncio.get_running_loop()
    loop.set_exception_handler(_asyncio_exception_handler)
    _control_http_client = httpx.AsyncClient(timeout=2.0)
    if FRODOBOTS_URL:
        _video_fallback_task = asyncio.create_task(_video_fallback_loop())
    yield
    if _video_fallback_task is not None:
        _video_fallback_task.cancel()
        try:
            await _video_fallback_task
        except asyncio.CancelledError:
            pass
    if _control_http_client is not None:
        await _control_http_client.aclose()
        _control_http_client = None
    for pc in list(webrtc_pcs):
        await pc.close()
    webrtc_pcs.clear()
    logger.info("App-server shutting down")


app = FastAPI(title="App Server", lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:5173", "http://localhost:3000"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/")
async def root():
    """Health check."""
    return {
        "status": "running",
        "active_connections": len(active_connections),
    }


@app.post("/api/ingest")
async def ingest(message: IngestMessage):
    """Bridge pushes ROS2 messages here; we broadcast to WebSocket clients."""
    payload = {
        "topic": message.topic,
        "data": message.data,
        "timestamp": message.timestamp or datetime.now().isoformat(),
        "type": message.type,
    }
    await broadcast_message(payload)
    return {"ok": True}


def _mjpeg_chunk(jpeg: bytes) -> bytes:
    """Build one multipart part for MJPEG (boundary + headers + body)."""
    header = (
        b"--frame\r\n"
        b"Content-Type: image/jpeg\r\n"
        b"Content-Length: %d\r\n"
        b"\r\n"
    ) % (len(jpeg),)
    return header + jpeg


@app.post("/api/video/frame")
async def video_frame(request: Request):
    """Accept JPEG from legacy clients; decode to BGR for WebRTC and broadcast to MJPEG."""
    global latest_front_jpeg, latest_front_bgr, _last_frame_post_time
    body = await request.body()
    _last_frame_post_time = time.monotonic()
    if not body:
        return {"ok": False, "error": "empty body"}
    latest_front_jpeg = body
    result = _jpeg_to_bgr24(body)
    if result:
        latest_front_bgr = result[0].copy()
    chunk = _mjpeg_chunk(body)
    dead: List[int] = []
    for i, q in enumerate(video_stream_subscribers):
        try:
            q.put_nowait(chunk)
        except Exception:
            dead.append(i)
    for i in reversed(dead):
        video_stream_subscribers.pop(i)
    return {"ok": True}


@app.post("/api/video/frame/raw")
async def video_frame_raw(request: Request):
    """Accept raw BGR bytes from ros2_app_bridge. Single encode: WebRTC only (JPEG only for MJPEG subscribers)."""
    global latest_front_bgr, _last_frame_post_time
    body = await request.body()
    _last_frame_post_time = time.monotonic()
    width = request.headers.get("X-Width")
    height = request.headers.get("X-Height")
    if not body or not width or not height:
        return {"ok": False, "error": "empty body or missing X-Width/X-Height"}
    try:
        w, h = int(width), int(height)
    except ValueError:
        return {"ok": False, "error": "invalid X-Width or X-Height"}
    expected = h * w * 3
    if len(body) != expected:
        return {"ok": False, "error": f"body length {len(body)} != {expected}"}
    
    # Log first frame and every 30th frame to reduce spam
    if not hasattr(video_frame_raw, '_frame_count'):
        video_frame_raw._frame_count = 0
    video_frame_raw._frame_count += 1
    if video_frame_raw._frame_count == 1 or video_frame_raw._frame_count % 30 == 0:
        logger.info(f"Received front camera frame #{video_frame_raw._frame_count} via HTTP: {w}x{h}, size={len(body)} bytes")
    
    arr = np.frombuffer(body, dtype=np.uint8).reshape((h, w, 3)).copy()
    latest_front_bgr = arr
    
    # Send to MJPEG subscribers
    if video_stream_subscribers:
        jpeg = _bgr_to_jpeg(arr)
        if jpeg:
            chunk = _mjpeg_chunk(jpeg)
            dead: List[int] = []
            for i, q in enumerate(video_stream_subscribers):
                try:
                    q.put_nowait(chunk)
                except Exception:
                    dead.append(i)
            for i in reversed(dead):
                video_stream_subscribers.pop(i)
            if video_frame_raw._frame_count == 1 or video_frame_raw._frame_count % 30 == 0:
                logger.info(f"Sent frame #{video_frame_raw._frame_count} to {len(video_stream_subscribers)} MJPEG subscribers")
    else:
        if video_frame_raw._frame_count == 1 or video_frame_raw._frame_count % 30 == 0:
            logger.debug("Frame #%s received but no MJPEG subscribers (client may connect later)", video_frame_raw._frame_count)
    
    return {"ok": True}


@app.post("/api/video/occupancy/frame/raw")
async def video_occupancy_frame_raw(request: Request):
    """Accept raw BGR bytes from ros2_app_bridge (occupancy annotated image)."""
    global latest_occupancy_bgr, _last_occupancy_frame_time
    _last_occupancy_frame_time = time.monotonic()
    body = await request.body()
    width = request.headers.get("X-Width")
    height = request.headers.get("X-Height")
    if not body or not width or not height:
        return {"ok": False, "error": "empty body or missing X-Width/X-Height"}
    try:
        w, h = int(width), int(height)
    except ValueError:
        return {"ok": False, "error": "invalid X-Width or X-Height"}
    expected = h * w * 3
    if len(body) != expected:
        return {"ok": False, "error": f"body length {len(body)} != {expected}"}
    # Log first frame and every 30th frame to reduce spam
    if not hasattr(video_occupancy_frame_raw, '_frame_count'):
        video_occupancy_frame_raw._frame_count = 0
    video_occupancy_frame_raw._frame_count += 1
    if video_occupancy_frame_raw._frame_count == 1 or video_occupancy_frame_raw._frame_count % 30 == 0:
        logger.info(f"Received occupancy frame #{video_occupancy_frame_raw._frame_count} via HTTP: {w}x{h}, size={len(body)} bytes")
    arr = np.frombuffer(body, dtype=np.uint8).reshape((h, w, 3)).copy()
    latest_occupancy_bgr = arr
    
    # Send to MJPEG subscribers
    if video_stream_subscribers_occupancy:
        jpeg = _bgr_to_jpeg(arr)
        if jpeg:
            chunk = _mjpeg_chunk(jpeg)
            dead: List[int] = []
            for i, q in enumerate(video_stream_subscribers_occupancy):
                try:
                    q.put_nowait(chunk)
                except Exception:
                    dead.append(i)
            for i in reversed(dead):
                video_stream_subscribers_occupancy.pop(i)
            if video_occupancy_frame_raw._frame_count == 1 or video_occupancy_frame_raw._frame_count % 30 == 0:
                logger.info(f"Sent occupancy frame #{video_occupancy_frame_raw._frame_count} to {len(video_stream_subscribers_occupancy)} MJPEG subscribers")
    else:
        if video_occupancy_frame_raw._frame_count == 1 or video_occupancy_frame_raw._frame_count % 30 == 0:
            logger.debug("Occupancy frame #%s received but no MJPEG subscribers (client may connect later)", video_occupancy_frame_raw._frame_count)
    
    return {"ok": True}


async def _stream_generator(client_queue: asyncio.Queue[bytes]) -> bytes:
    """Yield MJPEG chunks until client disconnects; remove queue on exit."""
    try:
        while True:
            yield await client_queue.get()
    finally:
        try:
            video_stream_subscribers.remove(client_queue)
            # Clear any remaining items in the queue to free memory
            while not client_queue.empty():
                try:
                    client_queue.get_nowait()
                except Exception:
                    break
        except ValueError:
            pass


@app.get("/api/video/front/stream")
async def video_front_stream():
    """Long-lived MJPEG stream: server pushes frames to the client."""
    logger.info("MJPEG stream client connected for front camera")
    # Limit queue size to prevent memory buildup (max 5 frames buffered per client)
    client_queue: asyncio.Queue[bytes] = asyncio.Queue(maxsize=5)
    video_stream_subscribers.append(client_queue)
    logger.info(f"Total MJPEG subscribers: {len(video_stream_subscribers)}")
    
    if latest_front_bgr is not None:
        logger.info("Using latest_front_bgr for initial frame")
        initial = _bgr_to_jpeg(latest_front_bgr)
        initial = _mjpeg_chunk(initial) if initial else _mjpeg_chunk(_PLACEHOLDER_JPEG)
    elif latest_front_jpeg is not None:
        logger.info("Using latest_front_jpeg for initial frame")
        initial = _mjpeg_chunk(latest_front_jpeg)
    else:
        logger.warning("No video frames available - sending placeholder")
        initial = _mjpeg_chunk(_PLACEHOLDER_JPEG)
    try:
        client_queue.put_nowait(initial)
    except Exception:
        pass
    return StreamingResponse(
        _stream_generator(client_queue),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


async def _occupancy_stream_generator(client_queue: asyncio.Queue[bytes]) -> bytes:
    """Yield MJPEG chunks for occupancy stream until client disconnects; remove queue on exit."""
    try:
        while True:
            yield await client_queue.get()
    finally:
        try:
            video_stream_subscribers_occupancy.remove(client_queue)
            # Clear any remaining items in the queue to free memory
            while not client_queue.empty():
                try:
                    client_queue.get_nowait()
                except Exception:
                    break
        except ValueError:
            pass


@app.get("/api/video/occupancy/stream")
async def video_occupancy_stream():
    """Long-lived MJPEG stream: occupancy annotated video (floor overlay + BEV)."""
    # Limit queue size to prevent memory buildup (max 5 frames buffered per client)
    client_queue: asyncio.Queue[bytes] = asyncio.Queue(maxsize=5)
    video_stream_subscribers_occupancy.append(client_queue)
    if latest_occupancy_bgr is not None:
        initial = _bgr_to_jpeg(latest_occupancy_bgr)
        initial = _mjpeg_chunk(initial) if initial else _mjpeg_chunk(_PLACEHOLDER_JPEG)
    else:
        initial = _mjpeg_chunk(_PLACEHOLDER_JPEG)
    try:
        client_queue.put_nowait(initial)
    except Exception:
        pass
    return StreamingResponse(
        _occupancy_stream_generator(client_queue),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


@app.get("/api/occupancy/status")
async def occupancy_status():
    """Return whether occupancy frames have been received (for UI waiting state)."""
    return {
        "received_frames": latest_occupancy_bgr is not None,
        "last_frame_at": _last_occupancy_frame_time,
    }


# --- WebRTC signaling (true low-latency video) ---

class WebRTCOfferBody(BaseModel):
    sdp: str
    type: str


@app.post("/api/webrtc/offer")
async def webrtc_offer(body: WebRTCOfferBody):
    """Accept SDP offer from client, add live video track, return SDP answer."""
    if not HAS_WEBRTC:
        return JSONResponse(
            content={"error": "WebRTC not available (install aiortc, av, numpy)"},
            status_code=503,
        )
    offer = RTCSessionDescription(sdp=body.sdp, type=body.type)
    pc = RTCPeerConnection()
    webrtc_pcs.add(pc)

    # Log SDP to check for data channel and video info
    sdp_lines = body.sdp.split('\n')
    has_data_channel = any('m=application' in line for line in sdp_lines)
    has_video = any('m=video' in line for line in sdp_lines)
    logger.info("WebRTC offer received - SDP contains data channel: %s, video: %s", has_data_channel, has_video)
    if has_data_channel:
        logger.debug("SDP data channel lines: %s", [line for line in sdp_lines if 'm=application' in line or 'sctp' in line.lower()])
    if has_video:
        video_lines = [line for line in sdp_lines if 'm=video' in line or ('a=sendrecv' in line or 'a=recvonly' in line or 'a=sendonly' in line)]
        logger.debug("SDP video lines: %s", video_lines[:5])  # First 5 video-related lines

    def get_latest_bgr():
        return latest_front_bgr

    # Only add video track if offer includes video media line
    # Frontend offers may only include data channel, in which case video will use HTTP/MJPEG fallback
    if has_video:
        track = LiveFrameTrack(get_latest_bgr)
        pc.addTrack(track)
        logger.info("WebRTC front camera track added to peer connection (offer includes video)")
    else:
        logger.info("WebRTC offer has no video - video will use HTTP/MJPEG fallback")

    @pc.on("track")
    def on_track(track):
        logger.info("WebRTC front camera: remote track received: %s", track.kind)

    # Register data channel handler BEFORE setRemoteDescription
    @pc.on("datachannel")
    def on_datachannel(channel: RTCDataChannel):
        logger.info("WebRTC data channel received: label=%s, id=%s, readyState=%s", 
                   channel.label, getattr(channel, 'id', 'N/A'), getattr(channel, 'readyState', 'N/A'))
        
        if channel.label == "control":
            # Store reference in dictionary keyed by peer connection
            webrtc_data_channels[pc] = channel
            logger.info("WebRTC control data channel stored for peer connection")
            
            @channel.on("open")
            def on_open():
                logger.info("WebRTC control data channel opened successfully")
                webrtc_data_channels[pc] = channel  # Ensure it's stored
            
            @channel.on("close")
            def on_close():
                logger.warning("WebRTC control data channel closed")
                if pc in webrtc_data_channels:
                    del webrtc_data_channels[pc]
            
            @channel.on("error")
            def on_error(error):
                logger.error("WebRTC control data channel error: %s", error)
            
            @channel.on("message")
            def on_message(message):
                try:
                    logger.debug("WebRTC control message received: %s", message[:100] if isinstance(message, str) else type(message))
                    if isinstance(message, str):
                        data = json.loads(message)
                        cmd = data.get("command")
                        if isinstance(cmd, dict):
                            logger.info("WebRTC control command received: linear=%.2f, angular=%.2f, lamp=%d", 
                                       cmd.get("linear", 0), cmd.get("angular", 0), cmd.get("lamp", 0))
                            _webrtc_stats["webrtc_messages"] += 1
                            asyncio.create_task(_forward_control_to_bridge(cmd))
                        else:
                            logger.warning("WebRTC control message missing 'command' field")
                    else:
                        logger.warning("WebRTC control message is not a string: %s", type(message))
                except json.JSONDecodeError as e:
                    logger.error("WebRTC control message JSON parse error: %s", e)
                except TypeError as e:
                    logger.error("WebRTC control message type error: %s", e)
                except Exception as e:
                    logger.error("WebRTC control message handling error: %s", e)

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        state = pc.connectionState
        logger.info("WebRTC peer connection state changed: %s", state)
        if state in ("failed", "closed", "disconnected"):
            logger.warning("WebRTC peer connection %s, cleaning up", state)
            # Clean up data channel reference
            if pc in webrtc_data_channels:
                del webrtc_data_channels[pc]
            webrtc_pcs.discard(pc)
            await pc.close()
        elif state == "connected":
            # Check data channel state when connection is established
            if pc in webrtc_data_channels:
                dc = webrtc_data_channels[pc]
                logger.info("WebRTC connection established - data channel state: %s", getattr(dc, 'readyState', 'unknown'))
            else:
                logger.warning("WebRTC connection established but no data channel found")

    try:
        logger.info("Setting remote description (data channel handler registered)...")
        await pc.setRemoteDescription(offer)
        
        # Check if data channel was received after setRemoteDescription
        # In aiortc, the datachannel event may fire synchronously during setRemoteDescription
        if pc in webrtc_data_channels:
            logger.info("Data channel found after setRemoteDescription: %s", webrtc_data_channels[pc].label)
        else:
            logger.warning("No data channel found after setRemoteDescription - checking SDP again")
            # Check SDP for data channel info
            local_sdp = offer.sdp if offer and hasattr(offer, 'sdp') and offer.sdp else None
            if local_sdp and 'm=application' in local_sdp:
                logger.info("SDP contains application media line - data channel should be negotiated")
            else:
                logger.warning("SDP does not contain application media line - data channel may not be in offer")
        
        answer = await pc.createAnswer()
        if not answer or not hasattr(answer, 'sdp') or not answer.sdp:
            raise ValueError("createAnswer() returned invalid answer - missing SDP")
        await pc.setLocalDescription(answer)
        
        # Log answer SDP to verify data channel is included
        if answer.sdp:
            answer_sdp_lines = answer.sdp.split('\n')
            answer_has_data_channel = any('m=application' in line for line in answer_sdp_lines)
            logger.info("WebRTC answer created - SDP contains data channel: %s", answer_has_data_channel)
        else:
            logger.warning("WebRTC answer created but SDP is empty")
        
        if not pc.localDescription or not pc.localDescription.sdp:
            raise ValueError("Failed to create local description - missing SDP")
        
        return {
            "sdp": pc.localDescription.sdp,
            "type": pc.localDescription.type,
        }
    except Exception as e:
        logger.error("WebRTC offer failed: %s", e, exc_info=True)
        # Clean up data channel reference on error
        if pc in webrtc_data_channels:
            del webrtc_data_channels[pc]
        webrtc_pcs.discard(pc)
        await pc.close()
        return JSONResponse(content={"error": str(e)}, status_code=500)


@app.post("/api/webrtc/occupancy/offer")
async def webrtc_occupancy_offer(body: WebRTCOfferBody):
    """Accept SDP offer for occupancy stream; return SDP answer with single video track (annotated floor overlay)."""
    if not HAS_WEBRTC:
        return JSONResponse(
            content={"error": "WebRTC not available (install aiortc, av, numpy)"},
            status_code=503,
        )
    offer = RTCSessionDescription(sdp=body.sdp, type=body.type)
    pc = RTCPeerConnection()
    webrtc_pcs.add(pc)

    def get_latest_occupancy_bgr():
        return latest_occupancy_bgr

    # Check if offer has video before adding track
    sdp_lines = body.sdp.split('\n') if body.sdp else []
    has_video = any('m=video' in line for line in sdp_lines)
    
    if has_video:
        track = LiveFrameTrack(get_latest_occupancy_bgr)
        pc.addTrack(track)
        logger.info("WebRTC occupancy track added to peer connection (offer includes video)")
    else:
        logger.info("WebRTC occupancy offer has no video - will use HTTP/MJPEG fallback")

    @pc.on("track")
    def on_track(track):
        logger.info("WebRTC occupancy: remote track received: %s", track.kind)

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        state = pc.connectionState
        logger.info("WebRTC occupancy peer connection state changed: %s", state)
        if state in ("failed", "closed", "disconnected"):
            logger.warning("WebRTC occupancy peer connection %s, cleaning up", state)
            webrtc_pcs.discard(pc)
            await pc.close()

    try:
        await pc.setRemoteDescription(offer)
        answer = await pc.createAnswer()
        if not answer or not hasattr(answer, 'sdp') or not answer.sdp:
            raise ValueError("createAnswer() returned invalid answer - missing SDP")
        await pc.setLocalDescription(answer)
        
        # Log answer SDP to verify data channel is included
        if answer.sdp:
            answer_sdp_lines = answer.sdp.split('\n')
            answer_has_data_channel = any('m=application' in line for line in answer_sdp_lines)
            logger.info("WebRTC occupancy answer created - SDP contains data channel: %s", answer_has_data_channel)
        
        if not pc.localDescription or not pc.localDescription.sdp:
            raise ValueError("Failed to create local description - missing SDP")
        
        return {
            "sdp": pc.localDescription.sdp,
            "type": pc.localDescription.type,
        }
    except Exception as e:
        logger.error("WebRTC occupancy offer failed: %s", e, exc_info=True)
        webrtc_pcs.discard(pc)
        await pc.close()
        return JSONResponse(content={"error": str(e)}, status_code=500)


@app.post("/api/webrtc/ros2-bridge/offer")
async def webrtc_ros2_bridge_offer(body: WebRTCOfferBody):
    """Accept SDP offer from ROS2 App Bridge for video and control forwarding."""
    if not HAS_WEBRTC:
        return JSONResponse(
            content={"error": "WebRTC not available (install aiortc, av, numpy)"},
            status_code=503,
        )
    
    offer = RTCSessionDescription(sdp=body.sdp, type=body.type)
    pc = RTCPeerConnection()
    
    # Determine stream type from SDP or use a parameter (for now, we'll detect from tracks)
    stream_type = "front"  # Default, will be updated when track is received
    
    logger.info("WebRTC ROS2 Bridge offer received for stream type: %s", stream_type)
    
    # Store peer connection
    ros2_bridge_pcs[stream_type] = pc
    
    # Handle incoming video tracks from ROS2 App Bridge
    @pc.on("track")
    def on_track(track):
        if track.kind == "video":
            track_label = getattr(track, 'label', None) or getattr(track, 'id', 'unknown')
            logger.info("WebRTC ROS2 Bridge video track received: %s", track_label)
            # Update latest BGR frame from ROS2 Bridge
            # We'll need to extract frames from the track and update latest_front_bgr or latest_occupancy_bgr
            # For now, log that we received it
            # TODO: Extract frames from track and update latest_front_bgr/latest_occupancy_bgr
    
    # Handle data channel for control commands
    @pc.on("datachannel")
    def on_datachannel(channel: RTCDataChannel):
        logger.info("WebRTC ROS2 Bridge data channel received: label=%s", channel.label)
        
        if channel.label == "control":
            ros2_bridge_data_channels[pc] = channel
            
            @channel.on("open")
            def on_open():
                logger.info("WebRTC ROS2 Bridge control data channel opened successfully")
            
            @channel.on("close")
            def on_close():
                logger.warning("WebRTC ROS2 Bridge control data channel closed")
                if pc in ros2_bridge_data_channels:
                    del ros2_bridge_data_channels[pc]
            
            @channel.on("error")
            def on_error(error):
                logger.error("WebRTC ROS2 Bridge control data channel error: %s", error)
            
            @channel.on("message")
            def on_message(message):
                # Control commands from ROS2 Bridge (if bidirectional needed)
                logger.debug("WebRTC ROS2 Bridge control message received: %s", message[:100] if isinstance(message, str) else type(message))
    
    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        state = pc.connectionState
        logger.info("WebRTC ROS2 Bridge peer connection state changed: %s", state)
        if state in ("failed", "closed", "disconnected"):
            logger.warning("WebRTC ROS2 Bridge peer connection %s, cleaning up", state)
            if pc in ros2_bridge_data_channels:
                del ros2_bridge_data_channels[pc]
            if stream_type in ros2_bridge_pcs:
                del ros2_bridge_pcs[stream_type]
            await pc.close()
    
    try:
        await pc.setRemoteDescription(offer)
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)
        logger.info("WebRTC ROS2 Bridge connection established")
        return {
            "sdp": pc.localDescription.sdp,
            "type": pc.localDescription.type,
        }
    except Exception as e:
        logger.warning("WebRTC ROS2 Bridge offer failed: %s", e)
        if stream_type in ros2_bridge_pcs:
            del ros2_bridge_pcs[stream_type]
        if pc in ros2_bridge_data_channels:
            del ros2_bridge_data_channels[pc]
        await pc.close()
        return JSONResponse(content={"error": str(e)}, status_code=500)


async def _forward_control_to_bridge(cmd: dict) -> bool:
    """
    Forward control command to ros2-app-bridge via WebRTC data channel (preferred) or HTTP (fallback).
    
    WebRTC is the primary method for low-latency control. HTTP is kept as a fallback for debugging
    and compatibility when WebRTC is not available.
    """
    # Try WebRTC data channel first
    for pc, dc in ros2_bridge_data_channels.items():
        if dc.readyState == "open":
            try:
                message = json.dumps({"command": cmd})
                dc.send(message)
                logger.debug("Control command forwarded via WebRTC data channel to ROS2 Bridge")
                return True
            except Exception as e:
                logger.warning("Failed to send via WebRTC data channel: %s, falling back to HTTP", e)
                break
    
    # Fallback to HTTP
    url = f"{ROS2_APP_BRIDGE_URL}/control"
    client = _control_http_client
    if client is None:
        async with httpx.AsyncClient(timeout=2.0) as c:
            r = await c.post(url, json={"command": cmd})
            return 200 <= r.status_code < 300
    try:
        r = await client.post(url, json={"command": cmd})
        if 200 <= r.status_code < 300:
            return True
        logger.warning("Bridge control returned %s: %s", r.status_code, r.text[:200])
        return False
    except Exception as e:
        global _last_control_forward_warning_time
        now = time.monotonic()
        if now - _last_control_forward_warning_time >= CONTROL_FORWARD_WARNING_INTERVAL:
            logger.warning("Forward to ros2-app-bridge failed: %s", e)
            _last_control_forward_warning_time = now
        return False


@app.post("/api/control")
def control(body: ControlBody):
    """Forward control command to ros2-app-bridge (publishes to /robot/control). HTTP fallback endpoint."""
    _webrtc_stats["http_requests"] += 1
    logger.info("HTTP control command received (fallback): linear=%.2f, angular=%.2f, lamp=%d", 
               body.command.linear, body.command.angular, body.command.lamp)
    cmd = {
        "linear": body.command.linear,
        "angular": body.command.angular,
        "lamp": body.command.lamp,
    }
    data = json.dumps({"command": cmd}).encode("utf-8")
    req = urllib.request.Request(
        f"{ROS2_APP_BRIDGE_URL}/control",
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            body_resp = resp.read().decode("utf-8")
            logger.info(
                "Control forward response: status=%s body=%s",
                resp.status,
                body_resp.strip() or "(empty)",
            )
            if 200 <= resp.status < 300:
                return {"ok": True}
            return {"ok": False, "error": f"Bridge returned {resp.status}"}
    except urllib.error.URLError as e:
        global _last_control_forward_warning_time
        now = time.monotonic()
        if now - _last_control_forward_warning_time >= CONTROL_FORWARD_WARNING_INTERVAL:
            logger.warning(
                "Forward to ros2-app-bridge failed: %s (ros2_app_bridge may be down or not reachable)",
                e,
            )
            _last_control_forward_warning_time = now
        return {"ok": False, "error": str(e.reason) if getattr(e, "reason", None) else str(e)}


def _forward_wander_enable(enable: bool) -> dict:
    """Forward wander enable to ros2-app-bridge (publishes to /wander/enable)."""
    data = json.dumps({"enable": enable}).encode("utf-8")
    req = urllib.request.Request(
        f"{ROS2_APP_BRIDGE_URL}/wander/enable",
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            body_resp = resp.read().decode("utf-8")
            logger.info(
                "Wander enable forward: enable=%s status=%s body=%s",
                enable, resp.status, body_resp.strip() or "(empty)",
            )
            if 200 <= resp.status < 300:
                return {"ok": True}
            return {"ok": False, "error": f"Bridge returned {resp.status}"}
    except urllib.error.URLError as e:
        logger.warning("Forward to ros2-app-bridge /wander/enable failed: %s", e)
        return {"ok": False, "error": str(e.reason) if getattr(e, "reason", None) else str(e)}


@app.post("/api/wander/start")
def wander_start():
    """Enable wander (forward to bridge -> /wander/enable true)."""
    return _forward_wander_enable(True)


@app.post("/api/wander/stop")
def wander_stop():
    """Disable wander (forward to bridge -> /wander/enable false)."""
    return _forward_wander_enable(False)


@app.post("/api/map/update")
async def map_update(request: Request):
    """Accept map and pose from ros2-app-bridge; store for GET /api/map and /api/robot_pose."""
    global latest_map, latest_robot_pose
    try:
        body = await request.json()
        if "map" in body and body["map"] is not None:
            latest_map = body["map"]
        if "pose" in body and body["pose"] is not None:
            latest_robot_pose = body["pose"]
    except Exception as e:
        logger.warning("Map update body error: %s", e)
    return {"ok": True}


@app.get("/api/map")
def get_map():
    """Return latest occupancy map (width, height, resolution, origin, data) for UI."""
    if latest_map is None:
        return JSONResponse(status_code=404, content={"error": "No map received yet"})
    return latest_map


@app.get("/api/robot_pose")
def get_robot_pose():
    """Return latest robot pose in map frame (x, y, theta) for UI."""
    if latest_robot_pose is None:
        return JSONResponse(status_code=404, content={"error": "No pose received yet"})
    return latest_robot_pose


@app.get("/api/webrtc/status")
def webrtc_status():
    """Get WebRTC connection status and statistics."""
    active_connections_count = len(webrtc_pcs)
    connections_info = []
    for pc in list(webrtc_pcs):
        connections_info.append({
            "connection_state": pc.connectionState,
            "ice_connection_state": pc.iceConnectionState,
            "ice_gathering_state": pc.iceGatheringState,
        })
    
    return {
        "active_peer_connections": active_connections_count,
        "connections": connections_info,
        "statistics": {
            "webrtc_messages": _webrtc_stats["webrtc_messages"],
            "http_requests": _webrtc_stats["http_requests"],
            "total": _webrtc_stats["webrtc_messages"] + _webrtc_stats["http_requests"],
        },
        "webrtc_usage_percentage": (
            (_webrtc_stats["webrtc_messages"] / max(1, _webrtc_stats["webrtc_messages"] + _webrtc_stats["http_requests"])) * 100
        ) if (_webrtc_stats["webrtc_messages"] + _webrtc_stats["http_requests"]) > 0 else 0,
    }


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket for frontend; receives messages broadcast from ingest."""
    await websocket.accept()
    active_connections.add(websocket)
    logger.info("WebSocket client connected. Total: %s", len(active_connections))

    try:
        await websocket.send_text(
            json.dumps({"type": "status", "message": "Connected to app-server"})
        )
    except Exception:
        # Client may have disconnected before we sent (e.g. refresh, hot reload)
        active_connections.discard(websocket)
        logger.info("WebSocket client disconnected before welcome. Total: %s", len(active_connections))
        return

    try:
        while True:
            data = await websocket.receive_text()
            if data == "ping":
                await websocket.send_text(json.dumps({"type": "pong"}))
            elif data.startswith("test:"):
                await broadcast_message(
                    {
                        "topic": "/test",
                        "data": data[5:],
                        "timestamp": datetime.now().isoformat(),
                        "type": "test",
                    }
                )
    except WebSocketDisconnect:
        active_connections.discard(websocket)
        n = len(active_connections)
        logger.info("WebSocket client disconnected. Total: %s", n)
    except Exception as e:
        logger.error("WebSocket error: %s", e)
        active_connections.discard(websocket)


def main() -> None:
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=8001)


if __name__ == "__main__":
    main()
