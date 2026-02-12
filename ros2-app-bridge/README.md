# ROS2 -> App-server bridge

Subscribes to ROS2 topics (e.g. `/chatter`) and POSTs each message to the app-server `POST /api/ingest` endpoint. Runs as its own Docker service; requires ROS2 (rclpy) and network access to `app_server`.

## Environment

- `APP_SERVER_URL` - Base URL of app-server (e.g. `http://app_server:8001`)
- `ROS_DOMAIN_ID` - Must match the ROS2 stack (default `0`)

## Docker

Built and run via `ros2_ws/docker-compose.yml` as the `ros2_app_bridge` service.
