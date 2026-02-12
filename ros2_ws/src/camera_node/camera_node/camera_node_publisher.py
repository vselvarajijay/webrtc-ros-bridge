#!/usr/bin/env python3
"""Capture camera stream (device or WebRTC) and publish as sensor_msgs/Image."""

import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import ParameterType
from sensor_msgs.msg import Image
from std_msgs.msg import Int64
from cv_bridge import CvBridge


def main(args=None):
    import argparse
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument(
        "--source",
        type=str,
        default="device",
        choices=("device", "webrtc"),
        help="Video source: device (local camera) or webrtc (WebRTC stream)",
    )
    arg_parser.add_argument("--device", type=int, default=0, help="Camera device index (e.g. 0 for /dev/video0)")
    arg_parser.add_argument("--width", type=int, default=640)
    arg_parser.add_argument("--height", type=int, default=480)
    arg_parser.add_argument("--fps", type=float, default=30.0)
    arg_parser.add_argument("--topic", type=str, default="camera/image_raw")
    arg_parser.add_argument("--frame_id", type=str, default="camera_optical_frame")
    arg_parser.add_argument(
        "--webrtc_signaling_port",
        type=int,
        default=8080,
        help="Port for WebRTC HTTP signaling (POST /offer with SDP)",
    )
    parsed, _ = arg_parser.parse_known_args(args=args if args is not None else sys.argv[1:])

    rclpy.init(args=args)

    node = CameraNode(
        source=parsed.source,
        device_index=parsed.device,
        width=parsed.width,
        height=parsed.height,
        fps=parsed.fps,
        topic=parsed.topic,
        frame_id=parsed.frame_id,
        webrtc_signaling_port=parsed.webrtc_signaling_port,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


class CameraNode(Node):
    """Publishes camera frames as sensor_msgs/Image from a device or WebRTC stream."""

    def __init__(
        self,
        source="device",
        device_index=0,
        width=640,
        height=480,
        fps=30.0,
        topic="camera/image_raw",
        frame_id="camera_optical_frame",
        webrtc_signaling_port=8080,
    ):
        super().__init__("camera_node")
        self.declare_parameter("source", source)
        self.declare_parameter("device_index", device_index)
        self.declare_parameter("width", width)
        self.declare_parameter("height", height)
        self.declare_parameter("fps", fps)
        self.declare_parameter("topic", topic)
        self.declare_parameter("frame_id", frame_id)
        self.declare_parameter("webrtc_signaling_port", webrtc_signaling_port)

        def _int_param(name):
            p = self.get_parameter(name).get_parameter_value()
            if p.type == ParameterType.PARAMETER_INTEGER:
                return p.integer_value
            return int(p.string_value) if p.string_value else 0

        def _float_param(name):
            p = self.get_parameter(name).get_parameter_value()
            if p.type == ParameterType.PARAMETER_DOUBLE:
                return p.double_value
            return float(p.string_value) if p.string_value else 30.0

        self._source = self.get_parameter("source").get_parameter_value().string_value or "device"
        device_index = _int_param("device_index")
        width = _int_param("width")
        height = _int_param("height")
        fps = _float_param("fps")
        topic = self.get_parameter("topic").get_parameter_value().string_value
        frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        webrtc_signaling_port = _int_param("webrtc_signaling_port")

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, topic, 10)
        self._frame_count_pub = self.create_publisher(Int64, "/camera/frame_count", 10)
        self._frame_count = 0
        self._log_every_n_frames = 30  # log every N frames so console isn't flooded
        self.frame_id = frame_id
        self._fps = fps
        self.cap = None
        self._webrtc_source = None

        if self._source == "device":
            try:
                import cv2
            except ImportError:
                self.get_logger().error(
                    "opencv-python not installed. Install with: apt-get install python3-opencv"
                )
                sys.exit(1)
            self.cap = cv2.VideoCapture(device_index)
            if not self.cap.isOpened():
                self.get_logger().error(f"Cannot open camera device {device_index}")
                sys.exit(1)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_FPS, fps)
            self.get_logger().info(
                f"Publishing on {topic} from device {device_index} ({width}x{height} @ {fps} Hz)"
            )
        elif self._source == "webrtc":
            try:
                from camera_node.webrtc_source import WebRTCSource
            except ImportError as e:
                self.get_logger().error(
                    f"WebRTC source requires aiortc and aiohttp: pip install aiortc aiohttp. {e}"
                )
                sys.exit(1)
            self._webrtc_source = WebRTCSource(port=webrtc_signaling_port)
            try:
                self._webrtc_source.start()
            except Exception as e:
                self.get_logger().error(f"Failed to start WebRTC signaling server: {e}")
                sys.exit(1)
            self.get_logger().info(
                f"Publishing on {topic} from WebRTC (signaling on port {webrtc_signaling_port})"
            )
        else:
            self.get_logger().error(f"Unknown source: {self._source}")
            sys.exit(1)

        self.timer = self.create_timer(1.0 / fps if fps > 0 else 0.033, self.timer_callback)

    def timer_callback(self):
        if self._source == "device":
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("Failed to read frame", throttle_duration_sec=5.0)
                return
        else:
            frame = self._webrtc_source.get_frame()
            if frame is None:
                return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)
        self._frame_count += 1
        self._frame_count_pub.publish(Int64(data=self._frame_count))
        # Throttled logging: first frame and every N frames
        if self._frame_count == 1:
            self.get_logger().info(
                f"First frame received (shape {frame.shape[1]}x{frame.shape[0]}), publishing to {self.pub.topic_name}"
            )
        elif self._frame_count % self._log_every_n_frames == 0:
            self.get_logger().info(f"Frames received: {self._frame_count} (data flowing)")

    def destroy_node(self, *args, **kwargs):
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        if self._webrtc_source is not None:
            self._webrtc_source.stop()
            self._webrtc_source = None
        super().destroy_node(*args, **kwargs)
