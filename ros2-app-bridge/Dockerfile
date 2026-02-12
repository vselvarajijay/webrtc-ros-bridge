# Bridge: ROS2 (rclpy) -> app-server HTTP ingest. Uses ROS2 base image; no extra Python deps.
FROM ros:humble

WORKDIR /app
COPY main.py .

ENV ROS_DOMAIN_ID=0
# Source ROS2 then run bridge (rclpy needs ROS2 environment)
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && exec python3 main.py"]
