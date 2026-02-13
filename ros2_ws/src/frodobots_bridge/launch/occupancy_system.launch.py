"""Launch Frodobots bridge and occupancy grid node together."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "sdk_url",
            default_value="http://localhost:8000",
            description="Frodobots SDK base URL",
        ),
        DeclareLaunchArgument(
            "webrtc_signaling_port",
            default_value="8080",
            description="WebRTC signaling server port",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("frodobots_bridge"),
                "config",
                "frodobots_config.yaml",
            ]),
            description="Path to configuration file",
        ),
        DeclareLaunchArgument(
            "video_topic_front",
            default_value="/robot/video/front",
            description="ROS2 topic for front camera video",
        ),
        DeclareLaunchArgument(
            "video_topic_rear",
            default_value="/robot/video/rear",
            description="ROS2 topic for rear camera video",
        ),
        DeclareLaunchArgument(
            "telemetry_topic",
            default_value="/robot/telemetry",
            description="ROS2 topic for robot telemetry",
        ),
        DeclareLaunchArgument(
            "control_topic",
            default_value="/robot/control",
            description="ROS2 topic for control commands",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("frodobots_bridge"),
                    "launch",
                    "frodobots_bridge.launch.py",
                ]),
            ]),
            launch_arguments=[
                ("sdk_url", LaunchConfiguration("sdk_url")),
                ("webrtc_signaling_port", LaunchConfiguration("webrtc_signaling_port")),
                ("config_file", LaunchConfiguration("config_file")),
                ("video_topic_front", LaunchConfiguration("video_topic_front")),
                ("video_topic_rear", LaunchConfiguration("video_topic_rear")),
                ("telemetry_topic", LaunchConfiguration("telemetry_topic")),
                ("control_topic", LaunchConfiguration("control_topic")),
            ],
        ),
        Node(
            package="occupancy_grid",
            executable="occupancy_node",
            name="occupancy_node",
            parameters=[{
                "input_image_topic": LaunchConfiguration("video_topic_front"),
                "annotated_image_topic": "/occupancy/annotated_image",
                "grid_topic": "/occupancy/grid",
                "bev_width_m": 4.0,
                "bev_depth_m": 6.0,
                "grid_resolution": 0.01,
                "overlay_transparency": 0.3,
                "enable_bev_overlay": True,
                "frame_id": "camera_link",
            }],
            output="screen",
        ),
    ])
