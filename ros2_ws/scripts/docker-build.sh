#!/bin/bash
# Build Docker images for ROS2 bridge

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

cd "$WORKSPACE_DIR"

echo "Building Docker images..."
docker-compose build

echo "Build complete!"
echo "To start containers: ./scripts/docker-run.sh"
