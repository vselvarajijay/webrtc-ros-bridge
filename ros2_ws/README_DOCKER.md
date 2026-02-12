# Docker Setup for ROS2 Bridge

This guide explains how to build and run the ROS2 bridge in Docker on Mac.

## Prerequisites

1. **Docker Desktop for Mac** installed
   - Download from: https://www.docker.com/products/docker-desktop
   - Supports both Intel (x86_64) and Apple Silicon (ARM64) Macs
   - Recommended: Allocate at least 4GB RAM and 2 CPUs in Docker Desktop settings

2. **Docker Compose** (included with Docker Desktop)

## Quick Start

```bash
cd ros2_ws

# 1. Set up environment
cp .env.example .env
# Edit .env with your Frodobots credentials

# 2. Build and start
./scripts/docker-build.sh
./scripts/docker-run.sh

# 3. Open shell in ROS2 container
./scripts/docker-shell.sh

# 4. Inside container, run the bridge
ros2 launch frodobots_bridge frodobots_bridge.launch.py
```

## Architecture

The Docker setup consists of two containers:

1. **ros2_bridge**: ROS2 Humble workspace with bridge packages
   - WebRTC signaling server on port 8080
   - ROS2 topics and services
   - Standardized bridge interface

2. **frodobots_sdk**: Frodobots SDK service
   - FastAPI service on port 8000
   - WebRTC streaming from robot
   - HTTP control endpoints

Containers communicate via Docker's default bridge network using service names.

## Building the Docker Images

### Option 1: Using Helper Script (Recommended)

```bash
./scripts/docker-build.sh
```

### Option 2: Using Docker Compose Directly

```bash
docker-compose build
```

### Option 3: Build Individual Services

```bash
# Build ROS2 bridge only
docker-compose build ros2_bridge

# Build Frodobots SDK only
docker-compose build frodobots_sdk
```

## Running the Containers

### Start All Containers

```bash
./scripts/docker-run.sh
```

Or manually:

```bash
docker-compose up -d
```

### View Running Containers

```bash
docker-compose ps
```

### View Logs

```bash
# All services
./scripts/docker-logs.sh

# Specific service
./scripts/docker-logs.sh ros2_bridge
./scripts/docker-logs.sh frodobots_sdk
```

### Stop Containers

```bash
docker-compose down
```

## Development Workflow

### 1. Initial Setup

```bash
cd ros2_ws
cp .env.example .env
# Edit .env with your credentials:
# - SDK_API_TOKEN
# - BOT_SLUG
# - MISSION_SLUG (optional)
```

### 2. Build and Start

```bash
./scripts/docker-build.sh
./scripts/docker-run.sh
```

### 3. Develop with Live Code Editing

The workspace source code is mounted as a volume, so you can edit files on your Mac and they're immediately available in the container:

```bash
# Edit code on Mac
code src/frodobots_bridge/frodobots_bridge/frodobots_adapter.py

# Rebuild inside container
./scripts/docker-shell.sh
colcon build --symlink-install
```

### 4. Run the Bridge

```bash
# Open shell in container
./scripts/docker-shell.sh

# Inside container (ROS2 is already sourced)
ros2 launch frodobots_bridge frodobots_bridge.launch.py
```

## Inside the Container

Once inside the ROS2 container, ROS2 is already sourced. You can:

```bash
# Check ROS2 environment
echo $ROS_DISTRO  # Should show "humble"

# List available packages
ros2 pkg list | grep bridge

# Run nodes
ros2 run frodobots_bridge frodobots_bridge

# View topics
ros2 topic list

# Monitor topics
ros2 topic echo /robot/video/front
ros2 topic echo /robot/telemetry

# Send control commands
ros2 topic pub /robot/control std_msgs/String '{"data": "{\"command_type\":\"movement\",\"command_data\":\"{\\\"action\\\":\\\"forward\\\"}\"}"}'
```

## Networking

### Inter-Container Communication

Containers communicate via Docker's default bridge network:

- ROS2 bridge → SDK: `http://frodobots_sdk:8000`
- SDK → ROS2 bridge WebRTC: `http://ros2_bridge:8080/offer`

### External Access (from Mac)

- Frodobots SDK: `http://localhost:8000`
- WebRTC Signaling: `http://localhost:8080`
- ROS2 Topics: Accessible if ROS2 installed locally on Mac

### Port Mappings

- **8000**: Frodobots SDK FastAPI service
- **8080**: WebRTC signaling server
- **11311**: ROS master (if needed)

## Mac-Specific Considerations

### 1. Architecture Compatibility

- **Intel Macs**: Native x86_64 support, full performance
- **Apple Silicon (M1/M2/M3)**: Docker Desktop emulates x86_64
  - Works but slower than native
  - Consider native ROS2 installation for better performance

### 2. Performance Optimization

**Docker Desktop Settings**:
- Increase memory allocation: Settings → Resources → Memory (recommend 4GB+)
- Increase CPU allocation: Settings → Resources → CPUs (recommend 2+)
- Enable VirtioFS for better file system performance (Docker Desktop 4.6+)

**Volume Mount Performance**:
- The `docker-compose.yml` uses standard volume mounts
- For better performance, you can add `:cached` or `:delegated` flags:
  ```yaml
  volumes:
    - ./src:/workspace/src:rw,cached
  ```

### 3. File Permissions

Mac file permissions are preserved in Docker volumes. If you encounter permission issues:

```bash
# Fix permissions inside container
docker-compose exec ros2_bridge bash
sudo chown -R $USER:$USER /workspace
```

### 4. X11/Display (for GUI tools like rviz)

If you need GUI tools:

```bash
# Install XQuartz on Mac
brew install --cask xquartz

# Allow connections
xhost +localhost

# Run with X11 forwarding
docker run -it --rm \
  -e DISPLAY=host.docker.internal:0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ros2-webrtc-bridge:latest
```

## Environment Variables

Create `.env` file from `.env.example`:

```bash
cp .env.example .env
```

Required variables:
- `SDK_API_TOKEN`: Your Frodobots API token
- `BOT_SLUG`: Your bot identifier

Optional variables:
- `MISSION_SLUG`: Mission identifier
- `ROS_DOMAIN_ID`: ROS2 domain ID (default: 0)
- `MAP_ZOOM_LEVEL`: Map zoom level (default: 18)

## Troubleshooting

### Port Already in Use

```bash
# Check what's using the port
lsof -i :8080
lsof -i :8000

# Kill the process or change ports in docker-compose.yml
```

### Container Won't Start

```bash
# Check logs
docker-compose logs ros2_bridge
docker-compose logs frodobots_sdk

# Start in foreground to see errors
docker-compose up
```

### Build Fails

```bash
# Clean build (no cache)
docker-compose build --no-cache

# Check Dockerfile syntax
docker build -t test .
```

### WebRTC Connection Issues

1. Verify containers are on same network:
   ```bash
   docker network inspect ros2_bridge_network
   ```

2. Test connectivity:
   ```bash
   docker-compose exec ros2_bridge curl http://frodobots_sdk:8000/
   ```

3. Check firewall settings on Mac

### ROS2 Domain ID Conflicts

If running multiple ROS2 instances:

```bash
# Set different domain ID in .env
ROS_DOMAIN_ID=1
docker-compose up
```

### SDK Not Authenticated

The Frodobots SDK needs to authenticate before the bridge can connect:

```bash
# Check SDK logs
docker-compose logs frodobots_sdk

# Authenticate SDK (if needed)
docker-compose exec frodobots_sdk curl -X POST http://localhost:8000/start-mission
```

## Helper Scripts

All scripts are in `scripts/` directory:

- `docker-build.sh`: Build Docker images
- `docker-run.sh`: Start containers
- `docker-shell.sh`: Open shell in ROS2 container
- `docker-logs.sh`: View logs
- `docker-clean.sh`: Clean up containers and images

## Cleanup

### Stop and Remove Containers

```bash
docker-compose down
```

### Remove Everything (Containers, Volumes, Images)

```bash
./scripts/docker-clean.sh
```

Or manually:

```bash
docker-compose down -v
docker rmi ros2-webrtc-bridge:latest frodobots-sdk:latest
```

## Alternative: Native ROS2 on Mac

For better performance on Mac (especially Apple Silicon), consider installing ROS2 natively:

```bash
# Install ROS2 Humble on macOS
brew tap ros/ros2
brew install ros-humble-desktop

# Use workspace directly
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch frodobots_bridge frodobots_bridge.launch.py
```

## Next Steps

1. Ensure Frodobots SDK is running and authenticated
2. Start the ROS2 bridge container
3. Verify WebRTC connection
4. Monitor ROS2 topics
5. Send control commands

For more information, see the main README.md in the project root.
