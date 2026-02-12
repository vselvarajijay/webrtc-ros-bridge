# Running ROS2 Talker and Viewing Messages in React App

This guide explains how to run the ROS2 talker in Docker and see messages propagate to the React frontend using the **three-service** architecture: **ros2_bridge**, **app_server** (pure FastAPI), and **ros2_app_bridge**.

## Architecture

```
ROS2 Docker Container (ros2_bridge)
    ├── ROS2 Talker → /chatter topic
    └── ROS_DOMAIN_ID: 0 (default)
         ↓ DDS
ros2_app_bridge (separate container)
    ├── Subscribes to /chatter via rclpy
    └── POSTs each message to app_server /api/ingest
         ↓ HTTP
app_server (pure FastAPI, no ROS2)
    └── WebSocket /ws → broadcasts to frontend
         ↓
React Frontend (app)
    └── WebSocket Client → Logs Panel
```

## Step-by-Step Setup

### 1. Start the full stack

```bash
cd ros2_ws
docker-compose up -d
docker-compose ps
```

This starts **ros2_bridge**, **app_server**, **ros2_app_bridge**, and **frodobots_sdk**. The app-server is available at `http://localhost:8001` and WebSocket at `ws://localhost:8001/ws`.

### 2. Set ROS_DOMAIN_ID (for bridge and ROS2)

The **ros2_bridge** and **ros2_app_bridge** must use the same `ROS_DOMAIN_ID`. In `ros2_ws/.env`:

```bash
ROS_DOMAIN_ID=0  # default
```

`docker-compose` passes this into both containers.

### 3. Start ROS2 Talker

**Option A: Helper script**

```bash
cd app-server
./start-talker.sh
```

**Option B: Manual**

```bash
docker exec -it ros2_bridge bash -c "source /opt/ros/humble/setup.bash && ros2 run demo_nodes_cpp talker"
```

**Option C: Python talker**

```bash
docker exec -it ros2_bridge bash -c "source /opt/ros/humble/setup.bash && ros2 run demo_nodes_py talker"
```

### 4. Start React frontend

```bash
cd app
pnpm dev
```

The app connects to `ws://localhost:8001/ws` and shows messages in the logs panel.

## Running app-server locally (no Docker)

For frontend or API testing without the full Docker stack:

```bash
cd app-server
./run.sh
```

Then open the React app. You can push test messages via the WebSocket (`test:...`) or by POSTing to `http://localhost:8001/api/ingest` with body:

```json
{"topic": "/test", "data": "Hello", "timestamp": "2026-02-12T12:00:00", "type": "test"}
```

## Verifying the setup

- **Health check:** `curl http://localhost:8001/` → `{"status":"running","active_connections":...}`
- **ROS2 topics in ros2_bridge:** `docker exec ros2_bridge bash -c "source /opt/ros/humble/setup.bash && ros2 topic list"`
- **Bridge logs:** `docker logs ros2_app_bridge` (should show subscription and POSTs)
- **App-server logs:** `docker logs app_server` (should show POSTs to /api/ingest and WebSocket connections)

## Troubleshooting

- **No messages in React:** Ensure talker is running, then check `docker logs ros2_app_bridge` (DDS and POSTs) and `docker logs app_server` (ingest and WebSocket). Confirm `ROS_DOMAIN_ID` is the same for ros2_bridge and ros2_app_bridge.
- **Bridge can’t reach app_server:** Containers must be on the same Docker network (default compose network). Check `APP_SERVER_URL=http://app_server:8001` for ros2_app_bridge.
- **Testing without ROS2:** Run `./run.sh` in app-server, open React, and send `test:message` over the WebSocket or POST to `/api/ingest`.
