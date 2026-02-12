from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
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
            package="sdk_bridge_core",
            executable="bridge_service",
            name="bridge_service",
            parameters=[{
                "video_topic_front": LaunchConfiguration("video_topic_front"),
                "video_topic_rear": LaunchConfiguration("video_topic_rear"),
                "telemetry_topic": LaunchConfiguration("telemetry_topic"),
                "control_topic": LaunchConfiguration("control_topic"),
            }],
            output="screen",
        ),
    ])
