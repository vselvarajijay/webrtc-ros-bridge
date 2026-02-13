from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "input_image_topic",
            default_value="/robot/video/front",
            description="Input camera image topic",
        ),
        DeclareLaunchArgument(
            "annotated_image_topic",
            default_value="/occupancy/annotated_image",
            description="Output annotated image topic",
        ),
        DeclareLaunchArgument(
            "grid_topic",
            default_value="/occupancy/grid",
            description="Output occupancy grid topic",
        ),
        DeclareLaunchArgument(
            "floor_hsv_lower",
            default_value="[0, 0, 100]",
            description="HSV lower bound for floor (H, S, V)",
        ),
        DeclareLaunchArgument(
            "floor_hsv_upper",
            default_value="[180, 50, 255]",
            description="HSV upper bound for floor (H, S, V)",
        ),
        DeclareLaunchArgument(
            "bev_width_m",
            default_value="4.0",
            description="BEV width in meters",
        ),
        DeclareLaunchArgument(
            "bev_depth_m",
            default_value="6.0",
            description="BEV depth in meters",
        ),
        DeclareLaunchArgument(
            "grid_resolution",
            default_value="0.01",
            description="Grid resolution in meters per pixel",
        ),
        DeclareLaunchArgument(
            "overlay_transparency",
            default_value="0.3",
            description="Floor overlay transparency (0-1)",
        ),
        DeclareLaunchArgument(
            "enable_bev_overlay",
            default_value="true",
            description="Show BEV mini-map in corner",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="camera_link",
            description="Frame ID for grid and images",
        ),
        Node(
            package="occupancy_grid",
            executable="occupancy_node",
            name="occupancy_node",
            parameters=[{
                "input_image_topic": LaunchConfiguration("input_image_topic"),
                "annotated_image_topic": LaunchConfiguration("annotated_image_topic"),
                "grid_topic": LaunchConfiguration("grid_topic"),
                "bev_width_m": LaunchConfiguration("bev_width_m"),
                "bev_depth_m": LaunchConfiguration("bev_depth_m"),
                "grid_resolution": LaunchConfiguration("grid_resolution"),
                "overlay_transparency": LaunchConfiguration("overlay_transparency"),
                "enable_bev_overlay": LaunchConfiguration("enable_bev_overlay"),
                "frame_id": LaunchConfiguration("frame_id"),
            }],
        ),
    ])
