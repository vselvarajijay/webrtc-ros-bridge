"""
Standardized WebRTC receiver for real-time streaming.

Provides generic WebRTC infrastructure that accepts SDP offers,
receives video/audio/data tracks, and maps them to standardized
camera types using configurable track mapping.
"""

import asyncio
import json
import logging
import threading
import time
from queue import Queue
from typing import Dict, Optional, Callable

try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False
    np = None

from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCDataChannel

from .track_mapper import TrackMapper

logger = logging.getLogger("webrtc_receiver")

CORS_HEADERS = {
    "Access-Control-Allow-Origin": "*",
    "Access-Control-Allow-Methods": "POST, OPTIONS",
    "Access-Control-Allow-Headers": "Content-Type",
}


def _run_loop(
    loop: asyncio.AbstractEventLoop,
    port: int,
    video_queues: Dict[str, Queue],
    track_mapper: TrackMapper,
    video_callback: Optional[Callable],
    telemetry_callback: Optional[Callable],
    ready_event: threading.Event,
):
    """Run the asyncio event loop for WebRTC signaling."""
    asyncio.set_event_loop(loop)
    
    if not logger.handlers:
        handler = logging.StreamHandler()
        handler.setLevel(logging.INFO)
        logger.addHandler(handler)
        logger.setLevel(logging.INFO)

    pcs = set()

    async def forward_video_frames(track, camera_type: str):
        """Forward video frames from a track to the appropriate queue."""
        recv_count = 0
        log_every_n = 30
        try:
            while True:
                try:
                    frame = await track.recv()
                    if frame is None:
                        break
                    recv_count += 1
                    
                    if recv_count == 1:
                        logger.info(f"WebRTC: first {camera_type} video frame received")
                    elif recv_count % log_every_n == 0:
                        logger.debug(f"WebRTC: received {recv_count} {camera_type} frames")
                    
                    if not HAS_NUMPY:
                        logger.error("numpy not available, cannot process video frames")
                        break
                    
                    img = frame.to_ndarray(format="bgr24")
                    timestamp = time.time()
                    
                    # Add to queue
                    queue = video_queues.get(camera_type)
                    if queue:
                        try:
                            # Keep only latest frame (drop older ones)
                            while not queue.empty():
                                try:
                                    queue.get_nowait()
                                except:
                                    pass
                            queue.put_nowait((img, timestamp))
                        except Exception as e:
                            logger.debug(f"Queue full for {camera_type}: {e}")
                    
                    # Call callback if provided
                    if video_callback:
                        try:
                            video_callback(img, camera_type, timestamp)
                        except Exception as e:
                            logger.error(f"Error in video callback: {e}")
                            
                except asyncio.CancelledError:
                    break
                except Exception as e:
                    logger.warning(f"forward_video_frames ({camera_type}): {e}")
                    break
        finally:
            logger.info(f"Video track ended ({camera_type}, total frames: {recv_count})")

    async def handle_data_channel(channel: RTCDataChannel):
        """Handle telemetry data from WebRTC data channel."""
        @channel.on("message")
        def on_message(message):
            try:
                if isinstance(message, str):
                    data = json.loads(message)
                    if telemetry_callback:
                        try:
                            telemetry_callback(data)
                        except Exception as e:
                            logger.error(f"Error in telemetry callback: {e}")
                else:
                    logger.warning("Received non-string message on data channel")
            except json.JSONDecodeError as e:
                logger.warning(f"Failed to parse telemetry data: {e}")
            except Exception as e:
                logger.error(f"Error handling telemetry: {e}")

    async def offer(request):
        """Handle WebRTC SDP offer."""
        if request.method == "OPTIONS":
            return web.Response(status=204, headers=CORS_HEADERS)
        
        try:
            params = await request.json()
            offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
        except Exception as e:
            logger.warning(f"Invalid offer body: {e}")
            return web.Response(status=400, text="Invalid JSON or SDP", headers=CORS_HEADERS)

        pc = RTCPeerConnection()
        pcs.add(pc)

        @pc.on("track")
        def on_track(track):
            """Handle incoming media tracks."""
            if track.kind == "video":
                # Get track ID and label
                track_id = getattr(track, "id", "")
                track_label = getattr(track, "label", "")
                
                # Identify camera type using track mapper
                camera_type = track_mapper.identify_camera_type(str(track_id), track_label)
                
                if camera_type:
                    logger.info(f"Video track received: {camera_type} (ID: {track_id}, Label: {track_label})")
                    asyncio.ensure_future(forward_video_frames(track, camera_type))
                else:
                    logger.warning(f"Video track received but no mapping found (ID: {track_id}, Label: {track_label})")
                    # Use default "front" if no mapping
                    camera_type = "front"
                    logger.info(f"Using default camera type 'front' for track {track_id}")
                    asyncio.ensure_future(forward_video_frames(track, camera_type))
            elif track.kind == "audio":
                logger.info("Audio track received (not currently processed)")
            else:
                logger.info(f"Unknown track kind: {track.kind}")

        @pc.on("datachannel")
        def on_datachannel(channel: RTCDataChannel):
            """Handle data channel for telemetry."""
            logger.info(f"Data channel opened: {channel.label}")
            asyncio.ensure_future(handle_data_channel(channel))

        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            """Handle connection state changes."""
            logger.info(f"Connection state: {pc.connectionState}")
            if pc.connectionState in ("failed", "closed"):
                await pc.close()
                pcs.discard(pc)

        try:
            await pc.setRemoteDescription(offer)
            answer = await pc.createAnswer()
            await pc.setLocalDescription(answer)
            return web.Response(
                content_type="application/json",
                text=json.dumps({
                    "sdp": pc.localDescription.sdp,
                    "type": pc.localDescription.type,
                }),
                headers=CORS_HEADERS,
            )
        except Exception as e:
            logger.error(f"Offer handling failed: {e}")
            await pc.close()
            pcs.discard(pc)
            return web.Response(status=500, text=str(e), headers=CORS_HEADERS)

    async def on_shutdown(app):
        """Cleanup on shutdown."""
        for pc in list(pcs):
            await pc.close()
        pcs.clear()

    async def serve():
        """Start the HTTP server for WebRTC signaling."""
        app = web.Application()
        app.router.add_route("OPTIONS", "/offer", offer)
        app.router.add_post("/offer", offer)
        app.on_shutdown.append(on_shutdown)
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, "0.0.0.0", port)
        await site.start()
        ready_event.set()
        logger.info(f"WebRTC signaling server listening on port {port}")
        while True:
            await asyncio.sleep(3600)

    loop.run_until_complete(serve())


class WebRTCReceiver:
    """
    Standardized WebRTC receiver for SDK bridges.
    
    Accepts WebRTC SDP offers, receives video/audio/data tracks,
    and maps them to standardized camera types using TrackMapper.
    """

    def __init__(
        self,
        port: int = 8080,
        track_mapper: Optional[TrackMapper] = None,
        video_callback: Optional[Callable] = None,
        telemetry_callback: Optional[Callable] = None,
    ):
        """
        Initialize WebRTC receiver.
        
        Args:
            port: Port for WebRTC HTTP signaling server
            track_mapper: TrackMapper instance for camera identification
            video_callback: Optional callback for video frames
                Signature: callback(frame: np.ndarray, camera_type: str, timestamp: float)
            telemetry_callback: Optional callback for telemetry data
                Signature: callback(data: dict)
        """
        self.port = port
        self.track_mapper = track_mapper or TrackMapper({"track_mappings": {}})
        self.video_callback = video_callback
        self.telemetry_callback = telemetry_callback
        
        # Initialize video queues for each camera type
        camera_types = self.track_mapper.get_all_camera_types()
        if not camera_types:
            # Default camera types if none configured
            camera_types = ["front", "rear"]
        
        self.video_queues = {camera_type: Queue(maxsize=2) for camera_type in camera_types}
        
        self._ready = threading.Event()
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(
            target=_run_loop,
            args=(
                self._loop,
                port,
                self.video_queues,
                self.track_mapper,
                video_callback,
                telemetry_callback,
                self._ready,
            ),
            daemon=True,
        )
        self._connected = False

    def start(self):
        """Start the WebRTC receiver."""
        self._thread.start()
        self._ready.wait(timeout=10.0)
        if not self._ready.is_set():
            raise RuntimeError("WebRTC server failed to start within 10s")
        logger.info("WebRTC receiver started")

    def stop(self):
        """Stop the WebRTC receiver."""
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(timeout=5.0)
        logger.info("WebRTC receiver stopped")

    def get_frame(self, camera_type: str = "front"):
        """
        Get the latest video frame for a camera type.
        
        Args:
            camera_type: Standardized camera type ("front", "rear", etc.)
            
        Returns:
            Tuple of (numpy array, timestamp) or None if no frame available
        """
        queue = self.video_queues.get(camera_type)
        if not queue:
            # Create queue if camera type not in mapping
            self.video_queues[camera_type] = Queue(maxsize=2)
            queue = self.video_queues[camera_type]
        
        frame_data = None
        try:
            # Get latest frame (discard older ones)
            while True:
                frame_data = queue.get_nowait()
        except:
            pass
        
        return frame_data

    def is_connected(self) -> bool:
        """Check if any peer is connected."""
        return self._connected
