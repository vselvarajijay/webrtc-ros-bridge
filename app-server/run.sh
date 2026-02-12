#!/bin/bash
# Run the pure FastAPI app-server (no ROS2). For local dev.
# With Docker: app_server and ros2_app_bridge are separate services in ros2_ws/docker-compose.

set -e
cd "$(dirname "${BASH_SOURCE[0]}")"

# Optional: kill process on port 8001
if command -v lsof >/dev/null 2>&1; then
  PIDS=$(lsof -ti:8001 2>/dev/null || true)
  if [ -n "$PIDS" ]; then
    echo "Killing processes on port 8001: $PIDS"
    echo "$PIDS" | xargs kill -9 2>/dev/null || true
    sleep 1
  fi
fi

echo "Starting app-server (pure FastAPI) on http://localhost:8001"
exec uv run python main.py
