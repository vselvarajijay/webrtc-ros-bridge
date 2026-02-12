#!/bin/bash
# Convenience script to run validation inside Docker container

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

# Determine which validation script to run
VALIDATION_SCRIPT="${1:-validate-all.sh}"

# Map common names to script names
case "$VALIDATION_SCRIPT" in
    build|validate-build)
        VALIDATION_SCRIPT="validate-build.sh"
        ;;
    talker|listener|talker-listener|validate-talker-listener)
        VALIDATION_SCRIPT="validate-talker-listener.sh"
        ;;
    packages|validate-packages)
        VALIDATION_SCRIPT="validate-packages.sh"
        ;;
    all|validate-all|*)
        VALIDATION_SCRIPT="validate-all.sh"
        ;;
esac

echo "Running $VALIDATION_SCRIPT inside Docker container..."
echo ""

# Run validation script inside container
# Scripts directory is mounted as read-only at /workspace/scripts
docker-compose exec ros2_bridge bash -c "cd /workspace && source /opt/ros/humble/setup.bash && [ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash || true && bash /workspace/scripts/$VALIDATION_SCRIPT"
