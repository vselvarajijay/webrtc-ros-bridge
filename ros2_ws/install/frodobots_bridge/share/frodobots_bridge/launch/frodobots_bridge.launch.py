from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "sdk_url",
            default_value="http://localhost:8000",
            description="Frodobots SDK base URL"
        ),
        DeclareLaunchArgument(
            "webrtc_signaling_port",
            default_value="8080",
            description="WebRTC signaling server port"
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("frodobots_bridge"),
                "config",
                "frodobots_config.yaml"
            ]),
            description="Path to configuration file"
        ),
        DeclareLaunchArgument(
            "video_topic_front",
            default_value="/robot/video/front",
            description="ROS2 topic for front camera video"
        ),
        DeclareLaunchArgument(
            "video_topic_rear",
            default_value="/robot/video/rear",
            description="ROS2 topic for rear camera video"
        ),
        DeclareLaunchArgument(
            "telemetry_topic",
            default_value="/robot/telemetry",
            description="ROS2 topic for robot telemetry"
        ),
        DeclareLaunchArgument(
            "control_topic",
            default_value="/robot/control",
            description="ROS2 topic for control commands"
        ),
        Node(
            package="frodobots_bridge",
            executable="frodobots_bridge",
            name="frodobots_bridge",
            parameters=[
                LaunchConfiguration("config_file"),
                {
                    "sdk_url": LaunchConfiguration("sdk_url"),
                    "webrtc_signaling_port": LaunchConfiguration("webrtc_signaling_port"),
                    "video_topic_front": LaunchConfiguration("video_topic_front"),
                    "video_topic_rear": LaunchConfiguration("video_topic_rear"),
                    "telemetry_topic": LaunchConfiguration("telemetry_topic"),
                    "control_topic": LaunchConfiguration("control_topic"),
                }
            ],
            output="screen",
        ),
    ])
