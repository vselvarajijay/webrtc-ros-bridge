#!/usr/bin/env python3
"""
Main ROS2 bridge service node.

Manages SDK bridge instances and handles standardized ROS2 topic
subscriptions/publications for streams and control commands.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import logging

logger = logging.getLogger(__name__)


class BridgeService(Node):
    """
    Main bridge service that manages SDK bridge instances.
    
    This service subscribes to standardized control topics and manages
    the lifecycle of SDK bridge adapters.
    """

    def __init__(self):
        super().__init__("bridge_service")
        
        # Standardized ROS2 topics (same for all SDKs)
        self.declare_parameter("video_topic_front", "/robot/video/front")
        self.declare_parameter("video_topic_rear", "/robot/video/rear")
        self.declare_parameter("telemetry_topic", "/robot/telemetry")
        self.declare_parameter("control_topic", "/robot/control")
        
        video_topic_front = self.get_parameter("video_topic_front").get_parameter_value().string_value
        video_topic_rear = self.get_parameter("video_topic_rear").get_parameter_value().string_value
        telemetry_topic = self.get_parameter("telemetry_topic").get_parameter_value().string_value
        control_topic = self.get_parameter("control_topic").get_parameter_value().string_value
        
        # Store active bridge instances
        self.bridges = {}
        
        # Subscriber for control commands
        self.control_subscriber = self.create_subscription(
            String,
            control_topic,
            self.control_callback,
            10
        )
        
        self.get_logger().info(f"Bridge service initialized")
        self.get_logger().info(f"Control topic: {control_topic}")
        self.get_logger().info(f"Video topics: {video_topic_front}, {video_topic_rear}")
        self.get_logger().info(f"Telemetry topic: {telemetry_topic}")

    def register_bridge(self, bridge_id: str, bridge_instance) -> None:
        """
        Register an SDK bridge instance.
        
        Args:
            bridge_id: Unique identifier for the bridge
            bridge_instance: Instance of SDKBridgeBase implementation
        """
        self.bridges[bridge_id] = bridge_instance
        self.get_logger().info(f"Registered bridge: {bridge_id}")

    def unregister_bridge(self, bridge_id: str) -> None:
        """
        Unregister an SDK bridge instance.
        
        Args:
            bridge_id: Unique identifier for the bridge
        """
        if bridge_id in self.bridges:
            bridge = self.bridges[bridge_id]
            bridge.stop()
            del self.bridges[bridge_id]
            self.get_logger().info(f"Unregistered bridge: {bridge_id}")

    def control_callback(self, msg: String) -> None:
        """
        Handle incoming control commands from ROS2.
        
        Args:
            msg: String message containing JSON control command
        """
        try:
            command = json.loads(msg.data)
            self.get_logger().debug(f"Received control command: {command}")
            
            # Forward to all registered bridges
            # In a multi-SDK scenario, you might want to route to specific bridges
            for bridge_id, bridge in self.bridges.items():
                try:
                    success = bridge.handle_control_command(command)
                    if success:
                        self.get_logger().debug(f"Command sent successfully to bridge: {bridge_id}")
                    else:
                        self.get_logger().warn(f"Failed to send command to bridge: {bridge_id}")
                except Exception as e:
                    self.get_logger().error(f"Error handling command in bridge {bridge_id}: {e}")
                    
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid JSON in control command: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing control command: {e}")

    def shutdown(self) -> None:
        """Shutdown all bridges."""
        for bridge_id, bridge in list(self.bridges.items()):
            self.unregister_bridge(bridge_id)


def main(args=None):
    rclpy.init(args=args)
    
    bridge_service = BridgeService()
    
    try:
        rclpy.spin(bridge_service)
    except KeyboardInterrupt:
        pass
    finally:
        bridge_service.shutdown()
        bridge_service.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
