# ROS2 Dockerfile for webrtc-ros-bridge
# Multi-architecture support (x86_64 and ARM64)
# Note: ROS2 Humble images are x86_64 only, Docker Desktop will emulate on ARM64 Macs

FROM --platform=linux/amd64 osrf/ros:humble-desktop-full

# Set working directory
WORKDIR /workspace

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool \
    git \
    wget \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 cv_bridge (if not already included)
RUN apt-get update && apt-get install -y \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements file first (for better layer caching)
COPY requirements.txt /workspace/requirements.txt

# Install Python dependencies for WebRTC
RUN pip3 install --no-cache-dir -r /workspace/requirements.txt

# Copy ROS2 workspace source
COPY src/ /workspace/src/

# Build the workspace
RUN cd /workspace && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source ROS2 setup in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /workspace/install/setup.bash" >> ~/.bashrc

# Expose ports
# 8080: WebRTC signaling
# 11311: ROS master (if needed)
EXPOSE 8080 11311

# Default command (keep container running)
CMD ["/bin/bash"]
