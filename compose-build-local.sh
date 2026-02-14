#!/usr/bin/env bash
# Use a local Docker builder so builds run on this machine instead of
# a remote builder (e.g. valthera-builder), avoiding slow export/transfer.
# Run from repo root: ./compose-build-local.sh [compose args...]
# Example: ./compose-build-local.sh up --build -d
set -e
cd "$(dirname "$0")"
if docker buildx use default 2>/dev/null; then
  echo "Using local builder: default"
else
  # Docker Desktop: local builder is usually named desktop-linux
  docker buildx use desktop-linux
  echo "Using local builder: desktop-linux"
fi
echo "Running: docker compose $*"
exec docker compose "$@"
