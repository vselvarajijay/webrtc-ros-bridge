from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "source",
            default_value="device",
            description="Video source: device or webrtc",
        ),
        DeclareLaunchArgument("device", default_value="0", description="Camera device index (e.g. 0 for /dev/video0)"),
        DeclareLaunchArgument("topic", default_value="camera/image_raw", description="Output image topic name"),
        DeclareLaunchArgument("width", default_value="640", description="Frame width"),
        DeclareLaunchArgument("height", default_value="480", description="Frame height"),
        DeclareLaunchArgument("fps", default_value="30.0", description="Target FPS"),
        DeclareLaunchArgument("frame_id", default_value="camera_optical_frame", description="Frame ID for message header"),
        DeclareLaunchArgument(
            "webrtc_signaling_port",
            default_value="8080",
            description="Port for WebRTC HTTP signaling (POST /offer)",
        ),
        Node(
            package="camera_node",
            executable="camera_node",
            name="camera_node",
            parameters=[{
                "source": LaunchConfiguration("source"),
                "device_index": LaunchConfiguration("device"),
                "topic": LaunchConfiguration("topic"),
                "width": LaunchConfiguration("width"),
                "height": LaunchConfiguration("height"),
                "fps": LaunchConfiguration("fps"),
                "frame_id": LaunchConfiguration("frame_id"),
                "webrtc_signaling_port": LaunchConfiguration("webrtc_signaling_port"),
            }],
        ),
    ])
