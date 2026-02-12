#!/bin/bash
# Build ROS2 workspace inside Docker container

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

cd "$WORKSPACE_DIR"

# Check if container is running
CONTAINER_STATUS=$(docker ps --filter "name=ros2_bridge" --format "{{.Status}}" 2>/dev/null)
if [ -z "$CONTAINER_STATUS" ] || [[ ! "$CONTAINER_STATUS" =~ ^Up ]]; then
    echo "Error: ros2_bridge container is not running."
    echo "Start it with: ./scripts/docker-run.sh"
    exit 1
fi

echo "Building ROS2 workspace inside Docker container..."
echo ""

# Run build command inside container
docker-compose exec ros2_bridge bash -c "
    cd /workspace && \
    source /opt/ros/humble/setup.bash && \
    echo 'Fixing NumPy version (downgrading from 2.x to 1.x)...' && \
    pip3 install 'numpy>=1.24.0,<2.0.0' --force-reinstall --no-cache-dir --break-system-packages 2>&1 | grep -v 'WARNING\|ERROR.*dependency' || true && \
    echo 'Cleaning previous build...' && \
    rm -rf build/* install/* log/* 2>/dev/null || true && \
    echo 'Building workspace...' && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    echo '' && \
    echo 'Build complete!' && \
    echo 'Sourcing workspace...' && \
    source install/setup.bash && \
    echo 'Verifying packages...' && \
    ros2 pkg list | grep -E '(sdk_bridge_core|frodobots_bridge|camera_node)' && \
    echo '' && \
    echo 'Testing imports...' && \
    python3 -c 'from sdk_bridge_core.base.sdk_bridge_base import SDKBridgeBase; print(\"  ✓ sdk_bridge_core imports OK\")' && \
    python3 -c 'from frodobots_bridge.frodobots_adapter import FrodobotsAdapter; print(\"  ✓ frodobots_bridge imports OK\")' && \
    echo '' && \
    echo '✓ Workspace built successfully!'
"

echo ""
echo "You can now run validation: ./scripts/validate.sh"
