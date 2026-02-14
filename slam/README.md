# ORB-SLAM3 Mono Service (Option A)

This service runs ORB-SLAM3 (mono) in Docker so the webrtc-ros-bridge app can show **pose**, **trajectory**, and the **SLAM camera view** without running SLAM on the host.

## Build source

The SLAM image is built from the **existing** ROS2 package [suchetanrs/ORB-SLAM3-ROS2-Docker](https://github.com/suchetanrs/ORB-SLAM3-ROS2-Docker): the Dockerfile clones that repo (with submodules for ORB_SLAM3, slam_msgs, and orb_slam3_ros2_wrapper) and builds the full stack from it. This repo only adds a **thin overlay**: remap scripts (`/robot_pose_slam` → `/camera_pose`, `/robot/video/front` → `/tracked_image`), params override (`params/mono-bridge-params.yaml`), and the container entrypoint. No separate fork of ORB_SLAM3 or the wrapper is maintained here.

## What it does

- **ORB-SLAM3 mono** (from the upstream clone above) subscribes to `/robot/video/front` and publishes `/robot_pose_slam`.
- **Remap nodes** (our overlay) inside the container:
  - `/robot_pose_slam` → `/camera_pose` (for trajectory and map pose)
  - `/robot/video/front` → `/tracked_image` (passthrough so the SLAM card shows the camera feed)

The app’s Map card shows the SLAM trajectory; the SLAM (tracked) card shows the front camera stream when this service is running.

## Run with SLAM

From the repo root:

```bash
docker compose --profile slam up --build
```

This starts the usual stack (app_server, ros2_app_bridge, ros2_bridge, frodobots) **and** the `slam` service. Ensure `ROS_DOMAIN_ID` matches (default `0`).

To build only the SLAM image:

```bash
docker compose --profile slam build slam
```

## Without the profile

```bash
docker compose up -d
```

starts the stack **without** SLAM. The app still works; the SLAM card and trajectory stay empty until you run with `--profile slam` or another source of `/camera_pose` and `/tracked_image`.

## Camera calibration

The SLAM node uses a default config (EuRoC-style) under `params/`. For better accuracy, replace or add a config that matches your camera (focal length, distortion, resolution) and point the launch to it.

## Notes

- **Tracked image**: This setup publishes the raw front camera as `/tracked_image`. A true “tracked” overlay (keypoints/trajectory on the image) would require changes in the ORB-SLAM3 wrapper.
- **Map points**: `/map_points` is not published continuously by this wrapper; the slam_occupancy node may not get map points unless the wrapper is extended or a different node is used.
- **First build**: Building the SLAM image clones the upstream suchetanrs repo (with submodules) and builds ORB_SLAM3 and the ROS2 wrapper; it can take 15–30 minutes.
- **Out of memory**: The ORB_SLAM3 build is very RAM-heavy. If it fails with `Killed` or `cannot allocate memory`, **increase Docker memory** (Docker Desktop → Settings → Resources → Memory) to **8 GB or more**, close other apps, then run `docker compose --profile slam build slam` again. The Dockerfile uses `-Os` and single-threaded build to reduce memory; if it still OOMs, more RAM is required.
