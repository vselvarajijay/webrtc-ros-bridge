#!/bin/bash
# Open shell in ROS2 bridge container

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

cd "$WORKSPACE_DIR"

# Check if container exists and is running
# Use docker ps directly for more reliable status checking
CONTAINER_STATUS=$(docker ps --filter "name=ros2_bridge" --format "{{.Status}}" 2>/dev/null)
if [ -z "$CONTAINER_STATUS" ] || [[ ! "$CONTAINER_STATUS" =~ ^Up ]]; then
    echo "Error: ros2_bridge container is not running."
    echo "Current status: $CONTAINER_STATUS"
    echo "Start it with: ./scripts/docker-run.sh"
    echo ""
    echo "Checking container logs..."
    docker-compose logs --tail=20 ros2_bridge 2>/dev/null || echo "No logs available"
    exit 1
fi

echo "Opening shell in ROS2 bridge container..."
echo "ROS2 environment will be automatically sourced."
echo ""
# Use bash -l to ensure proper environment, and source ROS2
docker-compose exec ros2_bridge bash -c "source /opt/ros/humble/setup.bash && [ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash || true && exec bash"
