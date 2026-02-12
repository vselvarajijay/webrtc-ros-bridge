#!/usr/bin/env python3
"""
Pure FastAPI app-server: WebSocket for frontend, HTTP ingest for bridge.

No ROS2 dependencies. The bridge service (separate container) subscribes to
ROS2 and POSTs messages here; we broadcast to WebSocket clients.
"""

import json
import logging
from contextlib import asynccontextmanager
from datetime import datetime
from typing import Set

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Store active WebSocket connections (frontend clients)
active_connections: Set[WebSocket] = set()


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


@asynccontextmanager
async def lifespan(app: FastAPI):
    logger.info("App-server starting (pure FastAPI, no ROS2)")
    yield
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
