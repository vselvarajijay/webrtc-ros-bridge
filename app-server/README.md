# App-server (pure FastAPI)

FastAPI service with **no ROS2 dependencies**. Exposes WebSocket `/ws` for the frontend and `POST /api/ingest` for the **ros2-app-bridge** service, which subscribes to ROS2 and pushes messages here.

## Local run

```bash
uv sync
./run.sh
# -> http://localhost:8001, ws://localhost:8001/ws
```

## Endpoints

- `GET /` - Health check
- `POST /api/ingest` - Bridge pushes ROS2 messages here (JSON: `topic`, `data`, `timestamp`, `type`)
- `WS /ws` - Frontend connects here; receives broadcast messages

## Docker (with ROS2 stack)

App-server runs as its own container. From repo root:

```bash
cd ros2_ws
docker-compose up -d
```

This starts **ros2_bridge**, **app_server**, **ros2_app_bridge**, and **frodobots**. The bridge subscribes to ROS2 and POSTs to `http://app_server:8001/api/ingest`. Run the talker (e.g. `./start-talker.sh` from app-server) and the React app (`cd app && pnpm dev`). Logs from `/chatter` appear in the UI.

## Architecture

```
ROS2 (/chatter) -> ros2_app_bridge -> POST /api/ingest -> app_server -> WebSocket /ws -> React
```
