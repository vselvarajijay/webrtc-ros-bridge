# Builds from suchetanrs/ORB-SLAM3-ROS2-Docker (clone + submodules); overlay: remap + params + entrypoint.
# ORB-SLAM3 mono as a Docker service for webrtc-ros-bridge. Subscribes to /robot/video/front,
# publishes /robot_pose_slam; our remap nodes expose /camera_pose and /tracked_image.

FROM --platform=linux/amd64 osrf/ros:humble-desktop-full-jammy

ARG DEBIAN_FRONTEND=noninteractive

# Dependencies (from suchetanrs base + nav2 for launch)
RUN apt-get update && apt-get install -y \
    cmake build-essential git wget unzip pkg-config \
    libeigen3-dev libopencv-dev \
    libgl1-mesa-dev libglew-dev libpython3-dev \
    ros-humble-cv-bridge ros-humble-vision-opencv ros-humble-image-transport \
    ros-humble-nav2-common ros-humble-pcl-ros ros-humble-tf2-ros \
    ros-humble-tf2-eigen ros-humble-tf2-geometry-msgs \
    ros-humble-pcl-conversions ros-humble-std-srvs \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Pangolin v0.9.1 - single-threaded build to avoid "write jobserver: Bad file descriptor" in Docker
RUN cd /tmp && git clone --branch v0.9.1 --depth 1 https://github.com/stevenlovegrove/Pangolin && \
    cd Pangolin && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
             -DCMAKE_CXX_FLAGS=-std=c++14 && \
    make -j1 && make install && ldconfig && \
    cd / && rm -rf /tmp/Pangolin

# Clone upstream repo with submodules (ORB_SLAM3 + slam_msgs + orb_slam3_ros2_wrapper)
# Pin to a tag for stability, e.g. --branch 1.0.0 if available; master = latest.
RUN git clone --recurse-submodules --depth 1 https://github.com/suchetanrs/ORB-SLAM3-ROS2-Docker.git /tmp/slam-repo

# ORB_SLAM3: use upstream submodule at /home/orb/ORB_SLAM3 (FindORB_SLAM3 expects this path).
# Pangolin v0.9.1 needs C++14; upstream ORB_SLAM3 may use C++11 - apply minimal patches.
# Use -Os and -j1 to reduce RAM and avoid OOM in Docker.
RUN cp -r /tmp/slam-repo/ORB_SLAM3 /home/orb/ORB_SLAM3 && \
    cd /home/orb/ORB_SLAM3 && \
    sed -i 's/CHECK_CXX_COMPILER_FLAG("-std=c++11"/CHECK_CXX_COMPILER_FLAG("-std=c++14"/' CMakeLists.txt && \
    sed -i 's/COMPILER_SUPPORTS_CXX11/COMPILER_SUPPORTS_CXX14/g' CMakeLists.txt && \
    sed -i 's/-std=c++11/-std=c++14/g' CMakeLists.txt && \
    sed -i 's/COMPILEDWITHC11/COMPILEDWITHC14/g' CMakeLists.txt && \
    sed -i 's/Using flag -std=c++11/Using flag -std=c++14/' CMakeLists.txt && \
    sed -i 's/-O3/-Os/g' CMakeLists.txt && \
    chmod +x build.sh && sed -i 's/make -j6/make -j1/g' build.sh && \
    ./build.sh

# Colcon workspace: slam_msgs + orb_slam3_ros2_wrapper from upstream (FindORB_SLAM3 = /home/orb/ORB_SLAM3)
RUN mkdir -p /root/colcon_ws/src && \
    cp -r /tmp/slam-repo/slam_msgs /root/colcon_ws/src/ && \
    cp -r /tmp/slam-repo/orb_slam3_ros2_wrapper /root/colcon_ws/src/ && \
    bash -c "source /opt/ros/humble/setup.bash && cd /root/colcon_ws && colcon build --symlink-install --packages-up-to orb_slam3_ros2_wrapper"

# Our overlay: params override (image topic + headless) and remap scripts
COPY params/mono-bridge-params.yaml /root/colcon_ws/install/orb_slam3_ros2_wrapper/share/orb_slam3_ros2_wrapper/params/ros_params/
COPY pose_remap.py tracked_image_passthrough.py /app/
COPY entrypoint.sh /app/
RUN chmod +x /app/entrypoint.sh

WORKDIR /app
ENV ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# Same network as ros2_bridge/frodobots so it can subscribe to /robot/video/front
ENTRYPOINT ["/app/entrypoint.sh"]
