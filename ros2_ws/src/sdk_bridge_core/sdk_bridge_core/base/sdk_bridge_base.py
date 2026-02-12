"""
Abstract base class for SDK bridge implementations.

All SDK adapters must implement this interface to ensure standardized
ROS2 topic publishing and control command handling.
"""

from abc import ABC, abstractmethod
from typing import Optional, Dict, Any
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header

# Standardized camera types
STANDARD_CAMERA_TYPES = ["front", "rear", "left", "right", "top", "bottom"]


class SDKBridgeBase(ABC):
    """
    Abstract base class defining the standard interface for SDK bridges.
    
    All SDK-specific adapters must implement these methods to ensure
    consistent ROS2 integration across different robotic SDKs.
    """

    def __init__(self, node: Node):
        """
        Initialize the bridge adapter.
        
        Args:
            node: ROS2 node instance for creating publishers/subscribers
        """
        self.node = node
        self._is_running = False
        self._webrtc_receiver = None

    @abstractmethod
    def start_streams(self) -> bool:
        """
        Initialize and start receiving streams from the SDK.
        
        This should establish WebRTC connections, HTTP connections, or
        whatever mechanism the SDK uses for streaming.
        
        Returns:
            True if streams started successfully, False otherwise
        """
        pass

    @abstractmethod
    def publish_video(self, frame_data: any, camera_type: str = "front") -> None:
        """
        Publish a video frame to ROS2.
        
        Args:
            frame_data: Video frame data (format depends on SDK, adapter converts)
            camera_type: Standardized camera type (one of: front, rear, left, right, top, bottom)
        """
        pass

    @abstractmethod
    def publish_audio(self, audio_data: any) -> None:
        """
        Publish audio stream data to ROS2.
        
        Args:
            audio_data: Audio data (format depends on SDK, adapter converts)
        """
        pass

    @abstractmethod
    def publish_telemetry(self, telemetry_data: dict) -> None:
        """
        Publish robot telemetry to ROS2.
        
        Args:
            telemetry_data: Dictionary containing telemetry data (lat, lon, heading, etc.)
        """
        pass

    @abstractmethod
    def handle_control_command(self, command: dict) -> bool:
        """
        Process a control command from ROS2 and send it to the SDK.
        
        Args:
            command: Control command dictionary with standardized format
            
        Returns:
            True if command was sent successfully, False otherwise
        """
        pass

    @abstractmethod
    def stop(self) -> None:
        """
        Stop all streams and cleanup resources.
        """
        pass

    @property
    def is_running(self) -> bool:
        """Check if the bridge is currently running."""
        return self._is_running

    def _set_running(self, running: bool) -> None:
        """Internal method to update running state."""
        self._is_running = running

    def get_webrtc_config(self) -> Optional[Dict[str, Any]]:
        """
        Get WebRTC configuration for track mapping.
        
        Override this method to provide SDK-specific track mapping configuration.
        
        Returns:
            Dictionary with WebRTC configuration:
            {
                "track_mappings": {
                    "front": {"track_id_contains": "1000"},
                    "rear": {"track_id_contains": "1001"}
                }
            }
            or None if WebRTC is not used
        """
        return None

    @property
    def webrtc_receiver(self):
        """
        Get the WebRTC receiver instance if available.
        
        Returns:
            WebRTCReceiver instance or None
        """
        return self._webrtc_receiver

    def _set_webrtc_receiver(self, receiver):
        """Internal method to set WebRTC receiver."""
        self._webrtc_receiver = receiver
