# Bridge: ROS2 (rclpy) -> app-server WebRTC/HTTP. Subscribes to /robot/video/front, forwards via WebRTC tracks.
FROM ros:humble

# sensor_msgs for Image; nav_msgs for OccupancyGrid/Odometry (map and pose)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-sensor-msgs \
    ros-humble-nav-msgs \
    python3-pip \
    libavformat-dev \
    libavcodec-dev \
    libavdevice-dev \
    libavutil-dev \
    libswscale-dev \
    libswresample-dev \
    libavfilter-dev \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir numpy opencv-python-headless aiortc av httpx

WORKDIR /app
COPY main.py .

ENV ROS_DOMAIN_ID=0
# Source ROS2 then run bridge (rclpy needs ROS2 environment)
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && exec python3 main.py"]
