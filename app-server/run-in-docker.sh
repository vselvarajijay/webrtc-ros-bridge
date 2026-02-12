#!/bin/bash
# App-server is now its own Docker service. Use docker-compose to start everything.
# This script prints how to run the stack (app_server + ros2_app_bridge + ros2_bridge).

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
ROS2_WS="$PROJECT_ROOT/ros2_ws"

echo "App-server runs as its own container (app_server)."
echo "Start the full stack (ROS2 + app_server + ros2_app_bridge) with:"
echo ""
echo "  cd $ROS2_WS"
echo "  docker-compose up -d"
echo "  # Optional: run talker: ./start-talker.sh (from app-server dir)"
echo ""
echo "Then:"
echo "  - App-server: http://localhost:8001"
echo "  - WebSocket:  ws://localhost:8001/ws"
echo ""

if docker ps --format "{{.Names}}" 2>/dev/null | grep -q "app_server"; then
  echo "app_server container is already running."
else
  echo "To start only app_server and ros2_app_bridge (requires ros2_bridge running):"
  echo "  cd $ROS2_WS && docker-compose up -d app_server ros2_app_bridge"
fi
