#!/bin/bash
# Start Docker containers

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

cd "$WORKSPACE_DIR"

# Check if .env file exists
if [ ! -f .env ]; then
    echo "Warning: .env file not found. Copying from .env.example..."
    if [ -f .env.example ]; then
        cp .env.example .env
        echo "Please edit .env file with your credentials before continuing."
        exit 1
    else
        echo "Error: .env.example not found. Please create .env file manually."
        exit 1
    fi
fi

echo "Starting Docker containers..."
docker-compose up -d

echo "Waiting for containers to be ready..."
sleep 5

echo "Containers started!"
echo ""
echo "To open shell in ROS2 container: ./scripts/docker-shell.sh"
echo "To view logs: ./scripts/docker-logs.sh"
echo "To stop containers: docker-compose down"
