#!/bin/bash
# View Docker container logs

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

cd "$WORKSPACE_DIR"

# Check for service argument
SERVICE=${1:-}

if [ -z "$SERVICE" ]; then
    echo "Viewing logs for all services..."
    echo "Usage: $0 [ros2_bridge|frodobots]"
    echo ""
    docker-compose logs -f
else
    echo "Viewing logs for $SERVICE..."
    docker-compose logs -f "$SERVICE"
fi
