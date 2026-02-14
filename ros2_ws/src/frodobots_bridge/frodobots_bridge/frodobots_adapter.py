#!/usr/bin/env python3
"""
Frodobots SDK bridge adapter for ROS2.

Implements the standardized SDK bridge interface, connecting Frodobots SDK
WebRTC streams and HTTP control to standardized ROS2 topics.
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import json
import logging
import base64
import cv2
import requests
from typing import Optional, Tuple

from sdk_bridge_core.base.sdk_bridge_base import SDKBridgeBase
from sdk_bridge_core_interfaces.msg import StreamTelemetry
from sdk_bridge_core.webrtc import WebRTCReceiver, TrackMapper
from frodobots_bridge.http_control import FrodobotsHTTPControl

logger = logging.getLogger("frodobots_adapter")


class FrodobotsAdapter(SDKBridgeBase):
    """
    Frodobots SDK adapter implementing the standardized bridge interface.
    
    Receives WebRTC streams from Frodobots SDK and publishes to standardized
    ROS2 topics. Handles control commands from ROS2 and sends them to SDK via HTTP.
    """

    def __init__(self, node: Node):
        """Initialize Frodobots adapter."""
        super().__init__(node)
        
        # Get parameters
        self.declare_parameters()
        
        # Initialize components (webrtc_receiver is set via _set_webrtc_receiver from base)
        self.bridge = CvBridge()
        self.http_control = None
        self.sdk_url = None
        
        # Telemetry data cache
        self._latest_telemetry = {}
        
        # HTTP fallback state
        self.use_http_fallback = True
        self.http_fallback_endpoint = "/v2/front"
        self._http_fetch_count_front = 0
        self._http_fetch_count_rear = 0
        self._http_error_count = 0
        
        # Publishers for standardized ROS2 topics
        self.video_pub_front = None
        self.video_pub_rear = None
        self.telemetry_pub = None
        
        # Subscriber for control commands
        self.control_sub = None
        
        # Timer for publishing frames
        self.timer = None

    def declare_parameters(self):
        """Declare ROS2 parameters."""
        # Parameters are declared on the node, not self.node
        # Since we're passed the node, we use it directly
        pass

    def start_streams(self) -> bool:
        """Start receiving streams from Frodobots SDK."""
        try:
            # Declare and get parameters
            self.node.declare_parameter("sdk_url", "http://localhost:8000")
            self.node.declare_parameter("webrtc_signaling_port", 8080)
            self.node.declare_parameter("video_topic_front", "/robot/video/front")
            self.node.declare_parameter("video_topic_rear", "/robot/video/rear")
            self.node.declare_parameter("telemetry_topic", "/robot/telemetry")
            self.node.declare_parameter("control_topic", "/robot/control")
            self.node.declare_parameter("frame_id_front", "robot_front_camera")
            self.node.declare_parameter("frame_id_rear", "robot_rear_camera")
            self.node.declare_parameter("publish_fps", 30.0)
            
            # Get parameter values
            sdk_url = self.node.get_parameter("sdk_url").get_parameter_value().string_value
            self.sdk_url = sdk_url  # Store for HTTP fallback
            webrtc_port = self.node.get_parameter("webrtc_signaling_port").get_parameter_value().integer_value
            
            # HTTP fallback parameters
            self.node.declare_parameter("use_http_fallback", True)
            self.node.declare_parameter("http_fallback_endpoint", "/v2/front")
            self.use_http_fallback = self.node.get_parameter("use_http_fallback").get_parameter_value().bool_value
            self.http_fallback_endpoint = self.node.get_parameter("http_fallback_endpoint").get_parameter_value().string_value
            video_topic_front = self.node.get_parameter("video_topic_front").get_parameter_value().string_value
            video_topic_rear = self.node.get_parameter("video_topic_rear").get_parameter_value().string_value
            telemetry_topic = self.node.get_parameter("telemetry_topic").get_parameter_value().string_value
            control_topic = self.node.get_parameter("control_topic").get_parameter_value().string_value
            frame_id_front = self.node.get_parameter("frame_id_front").get_parameter_value().string_value
            frame_id_rear = self.node.get_parameter("frame_id_rear").get_parameter_value().string_value
            fps = self.node.get_parameter("publish_fps").get_parameter_value().double_value

            # Get WebRTC configuration
            webrtc_config = self.get_webrtc_config()
            if webrtc_config:
                # Create track mapper with Frodobots-specific mappings
                track_mapper = TrackMapper(webrtc_config)
                
                # Create control publisher for WebRTC control commands
                control_pub = self.node.create_publisher(String, control_topic, 10)
                
                # Control callback for WebRTC data channel
                def on_control_command(command: dict):
                    """Handle control command from WebRTC data channel and publish to ROS2."""
                    try:
                        msg = String()
                        msg.data = json.dumps(command)
                        control_pub.publish(msg)
                        self.node.get_logger().info(f"Control command received via WebRTC and published to {control_topic}")
                    except Exception as e:
                        self.node.get_logger().error(f"Error publishing control command: {e}")
                
                # Initialize standardized WebRTC receiver (set via base class setter)
                receiver = WebRTCReceiver(
                    port=webrtc_port,
                    track_mapper=track_mapper,
                    video_callback=self._on_video_frame_received,
                    telemetry_callback=self._on_telemetry_received,
                    control_callback=on_control_command,
                )
                receiver.start()
                self._set_webrtc_receiver(receiver)
            else:
                self.node.get_logger().error("WebRTC configuration not available")
                return False

            # Initialize HTTP control client
            self.http_control = FrodobotsHTTPControl(sdk_url=sdk_url)

            # Create publishers (sensor_data QoS for video so subscribers in other containers receive)
            sensor_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )
            self.video_pub_front = self.node.create_publisher(Image, video_topic_front, sensor_qos)
            self.video_pub_rear = self.node.create_publisher(Image, video_topic_rear, sensor_qos)
            self.telemetry_pub = self.node.create_publisher(StreamTelemetry, telemetry_topic, 10)

            # Create control subscriber
            self.control_sub = self.node.create_subscription(
                String,
                control_topic,
                self._control_callback,
                10,
            )

            # Create timer for publishing frames
            timer_period = 1.0 / fps if fps > 0 else 0.033
            self.timer = self.node.create_timer(timer_period, self._timer_callback)

            self._set_running(True)
            self.node.get_logger().info("Frodobots adapter started successfully")
            self.node.get_logger().info(f"WebRTC signaling on port {webrtc_port}")
            self.node.get_logger().info(f"SDK URL: {sdk_url}")
            self.node.get_logger().info(f"Publishing to: {video_topic_front}, {video_topic_rear}, {telemetry_topic}")
            self.node.get_logger().info(f"Subscribing to: {control_topic}")

            return True

        except Exception as e:
            self.node.get_logger().error(f"Failed to start streams: {e}")
            return False

    def publish_video(self, frame_data: np.ndarray, camera_type: str = "front") -> None:
        """Publish video frame to ROS2."""
        try:
            if camera_type == "front":
                publisher = self.video_pub_front
                frame_id = self.node.get_parameter("frame_id_front").get_parameter_value().string_value
            elif camera_type == "rear":
                publisher = self.video_pub_rear
                frame_id = self.node.get_parameter("frame_id_rear").get_parameter_value().string_value
            else:
                self.node.get_logger().warn(f"Unknown camera type: {camera_type}")
                return

            if publisher is None:
                return

            # Convert numpy array to ROS2 Image message
            msg = self.bridge.cv2_to_imgmsg(frame_data, encoding="bgr8")
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = frame_id
            publisher.publish(msg)
            if not hasattr(self, f'_pub_count_{camera_type}'):
                setattr(self, f'_pub_count_{camera_type}', 0)
            count = getattr(self, f'_pub_count_{camera_type}')
            setattr(self, f'_pub_count_{camera_type}', count + 1)
            if count == 0 or count % 30 == 0:
                self.node.get_logger().info(f"Published {camera_type} camera frame #{count} to ROS2 topic (size: {frame_data.shape})")

        except Exception as e:
            self.node.get_logger().error(f"Error publishing video ({camera_type}): {e}")

    def publish_audio(self, audio_data: any) -> None:
        """Publish audio stream (not currently implemented)."""
        # Audio publishing can be added later if needed
        pass

    def publish_telemetry(self, telemetry_data: dict) -> None:
        """Publish robot telemetry to ROS2."""
        try:
            if self.telemetry_pub is None:
                return

            msg = StreamTelemetry()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"

            # Extract telemetry fields (convert from Frodobots format)
            msg.latitude = float(telemetry_data.get("latitude", 0.0))
            msg.longitude = float(telemetry_data.get("longitude", 0.0))
            msg.heading = float(telemetry_data.get("heading", 0.0))
            msg.speed = float(telemetry_data.get("speed", 0.0))
            msg.status = str(telemetry_data.get("status", "unknown"))

            self.telemetry_pub.publish(msg)

        except Exception as e:
            self.node.get_logger().error(f"Error publishing telemetry: {e}")

    def handle_control_command(self, command: dict) -> bool:
        """Process control command and send to Frodobots SDK."""
        try:
            if self.http_control is None:
                self.node.get_logger().error("HTTP control client not initialized")
                return False

            # Convert standardized ROS2 command to Frodobots format
            frodobots_cmd = self.http_control.convert_ros2_to_frodobots(command)
            
            # Send to SDK
            success = self.http_control.send_command(frodobots_cmd)
            return success

        except Exception as e:
            msg = str(e).strip() or repr(e)
            self.node.get_logger().error(f"Error handling control command: {type(e).__name__}: {msg}")
            return False

    def stop(self) -> None:
        """Stop all streams and cleanup."""
        self._set_running(False)

        if self.timer:
            self.timer.cancel()

        if self.webrtc_receiver:
            self.webrtc_receiver.stop()

        self.node.get_logger().info("Frodobots adapter stopped")

    def _timer_callback(self):
        """Timer callback to publish video frames."""
        if not self.is_running:
            return

        # Front camera: try WebRTC first, fallback to HTTP
        front_frame_data = None
        front_source = None
        if self.webrtc_receiver:
            front_frame_data = self.webrtc_receiver.get_frame("front")
            if front_frame_data is not None:
                front_source = "WebRTC"
        
        # HTTP fallback if WebRTC not available or no frame received
        if front_frame_data is None and self.use_http_fallback:
            front_frame_data = self._fetch_frame_via_http("front")
            if front_frame_data is not None:
                front_source = "HTTP"
                if not hasattr(self, '_http_fallback_logged_front'):
                    self.node.get_logger().info("Using HTTP fallback for front camera frames")
                    self._http_fallback_logged_front = True
        
        if front_frame_data is not None:
            frame, timestamp = front_frame_data
            if not hasattr(self, '_publish_count_front'):
                self._publish_count_front = 0
            self._publish_count_front += 1
            if self._publish_count_front == 1 or self._publish_count_front % 30 == 0:
                self.node.get_logger().info(f"Publishing front camera frame #{self._publish_count_front} to ROS2 topic /robot/video/front (source: {front_source})")
            self.publish_video(frame, "front")
        elif not hasattr(self, '_no_frame_warned_front'):
            self.node.get_logger().warn("No front camera frame available from WebRTC receiver or HTTP fallback")
            self._no_frame_warned_front = True

        # Rear camera: try WebRTC first, fallback to HTTP (skip if not available)
        # Only attempt rear camera if WebRTC receiver has frames (rear camera may not be available)
        rear_frame_data = None
        if self.webrtc_receiver:
            rear_frame_data = self.webrtc_receiver.get_frame("rear")
        
        # Skip HTTP fallback for rear camera if it consistently fails (reduce log spam)
        # Only try HTTP if we haven't seen consistent failures
        if rear_frame_data is None and self.use_http_fallback:
            if not hasattr(self, '_rear_camera_disabled'):
                rear_frame_data = self._fetch_frame_via_http("rear")
                if rear_frame_data is None:
                    # If HTTP also fails, mark rear camera as unavailable and stop trying
                    if not hasattr(self, '_rear_http_fail_count'):
                        self._rear_http_fail_count = 0
                    self._rear_http_fail_count += 1
                    # After 5 consecutive failures, disable rear camera fetching
                    if self._rear_http_fail_count >= 5:
                        self._rear_camera_disabled = True
                        self.node.get_logger().info("Rear camera not available - disabling rear camera frame fetching")
                else:
                    # Reset failure count on success
                    self._rear_http_fail_count = 0
                    if not hasattr(self, '_http_fallback_logged_rear'):
                        self.node.get_logger().info("Using HTTP fallback for rear camera frames")
                        self._http_fallback_logged_rear = True
        
        if rear_frame_data is not None:
            frame, timestamp = rear_frame_data
            self.publish_video(frame, "rear")

        # Publish telemetry if available
        if self._latest_telemetry:
            self.publish_telemetry(self._latest_telemetry)

    def _control_callback(self, msg: String):
        """Handle control commands from ROS2."""
        try:
            command = json.loads(msg.data)
            self.handle_control_command(command)
        except json.JSONDecodeError as e:
            self.node.get_logger().error(f"Invalid JSON in control command: {e}")
        except Exception as e:
            self.node.get_logger().error(f"Error processing control command: {e}")

    def get_webrtc_config(self):
        """
        Get WebRTC configuration for Frodobots SDK.
        
        Returns:
            Dictionary with track mapping configuration
        """
        # Try to get from ROS2 parameters first
        try:
            self.node.declare_parameter("webrtc.track_mappings.front.track_id_contains", "1000")
            self.node.declare_parameter("webrtc.track_mappings.rear.track_id_contains", "1001")
            
            front_id = self.node.get_parameter("webrtc.track_mappings.front.track_id_contains").get_parameter_value().string_value
            rear_id = self.node.get_parameter("webrtc.track_mappings.rear.track_id_contains").get_parameter_value().string_value
            
            return {
                "track_mappings": {
                    "front": {"track_id_contains": front_id},
                    "rear": {"track_id_contains": rear_id},
                }
            }
        except:
            # Default Frodobots mapping
            return {
                "track_mappings": {
                    "front": {"track_id_contains": "1000"},
                    "rear": {"track_id_contains": "1001"},
                }
            }

    def _on_video_frame_received(self, frame: np.ndarray, camera_type: str, timestamp: float):
        """
        Callback for video frames received from WebRTC.
        
        This is called directly by the WebRTC receiver when frames arrive.
        We can use this for real-time processing if needed, but frames are
        also available via get_frame() for timer-based publishing.
        """
        # Frames are already queued, this callback is optional
        # Can be used for immediate processing if needed
        pass

    def _on_telemetry_received(self, telemetry_data: dict):
        """Callback for telemetry received from WebRTC data channel."""
        self._latest_telemetry = telemetry_data

    def _fetch_frame_via_http(self, camera_type: str) -> Optional[Tuple[np.ndarray, float]]:
        """
        Fetch video frame from Frodobots SDK via HTTP endpoint.
        
        Args:
            camera_type: "front" or "rear"
            
        Returns:
            Tuple of (frame_array, timestamp) or None on error
        """
        if not self.use_http_fallback or not self.sdk_url:
            return None
        
        # Skip rear camera if it's been disabled due to consistent failures
        if camera_type == "rear" and hasattr(self, '_rear_camera_disabled') and self._rear_camera_disabled:
            return None
            
        try:
            # Construct endpoint URL
            if camera_type == "front":
                endpoint = self.http_fallback_endpoint
            elif camera_type == "rear":
                # Use /v2/rear for rear camera
                endpoint = "/v2/rear" if self.http_fallback_endpoint == "/v2/front" else "/v1/rear"
            else:
                self.node.get_logger().warn(f"Unknown camera type for HTTP fetch: {camera_type}")
                return None
            
            url = f"{self.sdk_url.rstrip('/')}{endpoint}"
            
            # Make HTTP GET request with timeout
            response = requests.get(url, timeout=2.0)
            response.raise_for_status()
            
            # Parse JSON response
            data = response.json()
            
            # Extract base64 image and timestamp
            frame_key = f"{camera_type}_frame"
            if frame_key not in data:
                self.node.get_logger().warn(f"HTTP response missing '{frame_key}' field")
                return None
            
            base64_image = data[frame_key]
            timestamp = data.get("timestamp", 0.0)
            
            # Decode base64 to image bytes
            image_bytes = base64.b64decode(base64_image)
            
            # Convert to numpy array using cv2
            image_array = np.frombuffer(image_bytes, dtype=np.uint8)
            frame = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
            
            if frame is None:
                self.node.get_logger().warn(f"Failed to decode image from HTTP response for {camera_type} camera")
                return None
            
            # Log successful fetch (every Nth frame to reduce spam)
            if camera_type == "front":
                self._http_fetch_count_front += 1
                if self._http_fetch_count_front == 1 or self._http_fetch_count_front % 30 == 0:
                    self.node.get_logger().info(f"Fetched front camera frame #{self._http_fetch_count_front} via HTTP (size: {frame.shape})")
            else:
                self._http_fetch_count_rear += 1
                if self._http_fetch_count_rear == 1 or self._http_fetch_count_rear % 30 == 0:
                    self.node.get_logger().info(f"Fetched rear camera frame #{self._http_fetch_count_rear} via HTTP (size: {frame.shape})")
            
            # Reset error counts on success
            self._http_error_count = 0
            if camera_type == "front" and hasattr(self, '_front_404_count'):
                self._front_404_count = 0

            return (frame, timestamp)
            
        except requests.exceptions.Timeout:
            self._http_error_count += 1
            if self._http_error_count == 1 or self._http_error_count % 30 == 0:
                self.node.get_logger().warn(f"HTTP timeout fetching {camera_type} frame from {url}")
            return None
        except requests.exceptions.RequestException as e:
            self._http_error_count += 1
            # For rear camera, check if it's a 404 (not available) and disable after first few attempts
            if camera_type == "rear" and "404" in str(e):
                if not hasattr(self, '_rear_404_count'):
                    self._rear_404_count = 0
                self._rear_404_count += 1
                # Only log first 404, then disable
                if self._rear_404_count == 1:
                    self.node.get_logger().info("Rear camera endpoint not available (404) - will skip rear camera fetching")
                elif self._rear_404_count >= 3:
                    self._rear_camera_disabled = True
                return None
            # For front camera 404: frame not ready yet (browser/session starting). Log once, then rarely.
            if camera_type == "front" and "404" in str(e):
                if not hasattr(self, '_front_404_count'):
                    self._front_404_count = 0
                self._front_404_count += 1
                if self._front_404_count == 1:
                    self.node.get_logger().info(
                        "Front frame not available yet (404) - will retry (browser/session may still be starting)"
                    )
                elif self._front_404_count % 120 == 0:
                    self.node.get_logger().info(
                        f"Front frame still unavailable after {self._front_404_count} attempts - continuing to retry"
                    )
                return None
            # For other errors or front non-404, log normally (throttled)
            if self._http_error_count == 1 or self._http_error_count % 30 == 0:
                self.node.get_logger().warn(f"HTTP error fetching {camera_type} frame: {e}")
            return None
        except Exception as e:
            self._http_error_count += 1
            if self._http_error_count == 1 or self._http_error_count % 30 == 0:
                self.node.get_logger().error(f"Unexpected error fetching {camera_type} frame via HTTP: {e}")
            return None


def main(args=None):
    """Main entry point for Frodobots bridge node."""
    rclpy.init(args=args)

    node = Node("frodobots_bridge")
    adapter = FrodobotsAdapter(node)

    # Start streams
    if not adapter.start_streams():
        node.get_logger().error("Failed to start Frodobots adapter")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        adapter.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
