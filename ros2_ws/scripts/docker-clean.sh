#!/bin/bash
# Clean up Docker containers, volumes, and images

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

cd "$WORKSPACE_DIR"

echo "This will remove containers, volumes, and images."
read -p "Are you sure? (y/N) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Cancelled."
    exit 1
fi

echo "Stopping and removing containers..."
docker-compose down -v

echo "Removing images..."
docker rmi ros2-webrtc-bridge:latest frodobots-sdk:latest 2>/dev/null || true

echo "Cleanup complete!"
