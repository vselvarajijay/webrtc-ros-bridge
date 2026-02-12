"""
WebRTC video receiver: HTTP server for SDP offer/answer, pushes received frames to a queue.
Runs in a background thread with its own asyncio event loop.
"""

import asyncio
import json
import logging
import threading
from queue import Queue

from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription

logger = logging.getLogger("webrtc_source")

# CORS headers so the web app (different origin) can POST /offer
CORS_HEADERS = {
    "Access-Control-Allow-Origin": "*",
    "Access-Control-Allow-Methods": "POST, OPTIONS",
    "Access-Control-Allow-Headers": "Content-Type",
}


def _run_loop(loop, port, frame_queue, ready_event):
    asyncio.set_event_loop(loop)
    # Ensure WebRTC logs (e.g. "first video frame received") show in the console
    if not logger.handlers:
        handler = logging.StreamHandler()
        handler.setLevel(logging.INFO)
        logger.addHandler(handler)
        logger.setLevel(logging.INFO)

    pcs = set()

    async def forward_frames(track):
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
                        logger.info("WebRTC: first video frame received from browser")
                    elif recv_count % log_every_n == 0:
                        logger.info("WebRTC: received %d frames (video flowing)", recv_count)
                    img = frame.to_ndarray(format="bgr24")
                    try:
                        frame_queue.put_nowait(img)
                    except Exception:
                        pass
                except asyncio.CancelledError:
                    break
                except Exception as e:
                    logger.warning("forward_frames: %s", e)
                    break
        finally:
            logger.info("Video track ended (total frames received: %d)", recv_count)

    async def offer(request):
        if request.method == "OPTIONS":
            return web.Response(status=204, headers=CORS_HEADERS)
        try:
            params = await request.json()
            offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
        except Exception as e:
            logger.warning("Invalid offer body: %s", e)
            return web.Response(status=400, text="Invalid JSON or SDP", headers=CORS_HEADERS)

        pc = RTCPeerConnection()
        pcs.add(pc)

        @pc.on("track")
        def on_track(track):
            if track.kind == "video":
                logger.info("Video track received, forwarding frames")
                asyncio.ensure_future(forward_frames(track))

        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
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
            logger.warning("Offer handling failed: %s", e)
            await pc.close()
            pcs.discard(pc)
            return web.Response(status=500, text=str(e), headers=CORS_HEADERS)

    async def on_shutdown(app):
        for pc in list(pcs):
            await pc.close()
        pcs.clear()

    async def serve():
        app = web.Application()
        app.router.add_route("OPTIONS", "/offer", offer)
        app.router.add_post("/offer", offer)
        app.on_shutdown.append(on_shutdown)
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, "0.0.0.0", port)
        await site.start()
        ready_event.set()
        logger.info("WebRTC signaling server listening on port %s", port)
        while True:
            await asyncio.sleep(3600)

    loop.run_until_complete(serve())


class WebRTCSource:
    """
    WebRTC video source: runs an HTTP server that accepts SDP offers and returns answers.
    Received video frames are put into a thread-safe queue for the ROS node to consume.
    """

    def __init__(self, port=8080):
        self.port = port
        self._frame_queue = Queue(maxsize=2)
        self._ready = threading.Event()
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(
            target=_run_loop,
            args=(self._loop, port, self._frame_queue, self._ready),
            daemon=True,
        )

    def start(self):
        self._thread.start()
        self._ready.wait(timeout=10.0)
        if not self._ready.is_set():
            raise RuntimeError("WebRTC server failed to start within 10s")

    def get_frame(self):
        """Return the latest frame if available, else None. Non-blocking."""
        frame = None
        try:
            while True:
                frame = self._frame_queue.get_nowait()
        except Exception:
            pass
        return frame

    def stop(self):
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(timeout=5.0)
