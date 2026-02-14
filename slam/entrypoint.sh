#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /root/colcon_ws/install/setup.bash

# Start remap nodes in background (pose and tracked image passthrough)
python3 /app/pose_remap.py &
python3 /app/tracked_image_passthrough.py &

# Launch ORB-SLAM3 mono: subscribe /robot/video/front, publish /robot_pose_slam
# Params: image_topic_name=/robot/video/front, visualization=false, use_sim_time=false
exec ros2 launch orb_slam3_ros2_wrapper unirobot.launch.py \
  sensor_config:=mono \
  use_sim_time:=false \
  orb_slam3_param_file:=euroc_mono.yaml \
  ros_params_file:=mono-bridge-params.yaml \
  monitor_enabled:=false
