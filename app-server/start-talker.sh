#!/bin/bash
# Helper script to start ROS2 talker in Docker container

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS_DIR="$(cd "$SCRIPT_DIR/../ros2_ws" && pwd)"

echo "Starting ROS2 talker in Docker container..."
echo ""

# Check if Docker container is running
if ! docker ps --filter "name=ros2_bridge" --format "{{.Names}}" | grep -q "ros2_bridge"; then
    echo "Error: ros2_bridge container is not running."
    echo "Start it with: cd $ROS2_WS_DIR && docker-compose up -d"
    exit 1
fi

# Run talker in Docker container
echo "Running: ros2 run demo_nodes_cpp talker"
echo ""
docker exec -it ros2_bridge bash -c "source /opt/ros/humble/setup.bash && [ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash || true && ros2 run demo_nodes_cpp talker"
