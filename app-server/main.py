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
VIDEO_FALLBACK_INTERVAL = 0.15  # fetch from Frodobots every 150ms (~6–7 FPS) when fallback enabled

# WebRTC: peer connections for cleanup on shutdown
webrtc_pcs: Set = set()

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
            self._time_base = 1 / 30

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
                global latest_front_jpeg
                # Prefer ROS2 path: skip fallback update if we got a frame from bridge recently
                if time.monotonic() - _last_frame_post_time < 2.0:
                    await asyncio.sleep(VIDEO_FALLBACK_INTERVAL)
                    continue
                latest_front_jpeg = jpeg
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


@asynccontextmanager
async def lifespan(app: FastAPI):
    global _video_fallback_task, _control_http_client
    logger.info("App-server starting (pure FastAPI, no ROS2)")
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
    arr = np.frombuffer(body, dtype=np.uint8).reshape((h, w, 3)).copy()
    latest_front_bgr = arr
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
    return {"ok": True}


async def _stream_generator(client_queue: asyncio.Queue[bytes]) -> bytes:
    """Yield MJPEG chunks until client disconnects; remove queue on exit."""
    try:
        while True:
            yield await client_queue.get()
    finally:
        try:
            video_stream_subscribers.remove(client_queue)
        except ValueError:
            pass


@app.get("/api/video/front/stream")
async def video_front_stream():
    """Long-lived MJPEG stream: server pushes frames to the client."""
    client_queue: asyncio.Queue[bytes] = asyncio.Queue()
    video_stream_subscribers.append(client_queue)
    if latest_front_bgr is not None:
        initial = _bgr_to_jpeg(latest_front_bgr)
        initial = _mjpeg_chunk(initial) if initial else _mjpeg_chunk(_PLACEHOLDER_JPEG)
    elif latest_front_jpeg is not None:
        initial = _mjpeg_chunk(latest_front_jpeg)
    else:
        initial = _mjpeg_chunk(_PLACEHOLDER_JPEG)
    try:
        client_queue.put_nowait(initial)
    except Exception:
        pass
    return StreamingResponse(
        _stream_generator(client_queue),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


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

    def get_latest_bgr():
        return latest_front_bgr

    track = LiveFrameTrack(get_latest_bgr)
    pc.addTrack(track)

    @pc.on("datachannel")
    def on_datachannel(channel: RTCDataChannel):
        if channel.label == "control":
            @channel.on("message")
            def on_message(message):
                try:
                    if isinstance(message, str):
                        data = json.loads(message)
                        cmd = data.get("command")
                        if isinstance(cmd, dict):
                            asyncio.create_task(_forward_control_to_bridge(cmd))
                except (json.JSONDecodeError, TypeError) as e:
                    logger.debug("WebRTC control message parse error: %s", e)

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        if pc.connectionState in ("failed", "closed", "disconnected"):
            webrtc_pcs.discard(pc)
            await pc.close()

    try:
        await pc.setRemoteDescription(offer)
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)
        return {
            "sdp": pc.localDescription.sdp,
            "type": pc.localDescription.type,
        }
    except Exception as e:
        logger.warning("WebRTC offer failed: %s", e)
        webrtc_pcs.discard(pc)
        await pc.close()
        return JSONResponse(content={"error": str(e)}, status_code=500)


async def _forward_control_to_bridge(cmd: dict) -> bool:
    """Forward control command to ros2-app-bridge (async). Reuses shared client for low latency."""
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
    """Forward control command to ros2-app-bridge (publishes to /robot/control)."""
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
