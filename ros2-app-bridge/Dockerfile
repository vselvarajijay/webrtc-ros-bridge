# Bridge: ROS2 (rclpy) -> app-server WebRTC/HTTP. Subscribes to /robot/video/front, forwards via WebRTC tracks.
FROM ros:humble

# Only what main.py uses: sensor_msgs (Image), std_msgs (in base), pip. No nav_msgs. PyAV/aiortc use wheels when possible.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-sensor-msgs \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir numpy opencv-python-headless aiortc av httpx

WORKDIR /app
COPY main.py .

ENV ROS_DOMAIN_ID=0
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && exec python3 main.py"]
