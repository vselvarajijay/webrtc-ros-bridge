# ROS2 workspace for frodobots_bridge (WebRTC → ROS2 topics). Built and run via docker-compose.
FROM --platform=linux/amd64 ros:humble-ros-base

WORKDIR /workspace

# Build + run deps: colcon, pip, cv_bridge (frodobots_bridge/sdk_bridge_core). No vcstool/git/wget/curl.
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /workspace/requirements.txt
RUN pip3 install --no-cache-dir -r /workspace/requirements.txt

COPY src/ /workspace/src/
RUN bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /workspace/install/setup.bash" >> ~/.bashrc

EXPOSE 8080
CMD ["/bin/bash"]
