#!/usr/bin/env python3
"""
Occupancy grid node: subscribe to camera, segment floor, build BEV grid, publish
annotated image and nav_msgs/OccupancyGrid.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import ParameterType
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False


def _default_bev_src_points(width: int, height: int) -> np.ndarray:
    """Default trapezoid in image: floor region (wider at bottom)."""
    w, h = float(width), float(height)
    return np.float32([
        [w * 0.20, h * 0.85],  # bottom-left
        [w * 0.80, h * 0.85],  # bottom-right
        [w * 0.65, h * 0.35],  # top-right
        [w * 0.35, h * 0.35],  # top-left
    ])


def _bev_dst_shape(bev_width_m: float, bev_depth_m: float, resolution: float) -> tuple:
    """(cols, rows) in pixels for BEV."""
    cols = int(round(bev_width_m / resolution))
    rows = int(round(bev_depth_m / resolution))
    return max(1, cols), max(1, rows)


class OccupancyNode(Node):
    """Subscribes to camera image, publishes occupancy grid and annotated image."""

    def __init__(self):
        super().__init__("occupancy_node")
        if not HAS_CV2:
            self.get_logger().error("opencv-python not available")
            raise RuntimeError("opencv-python required")

        self.declare_parameter("input_image_topic", "/robot/video/front")
        self.declare_parameter("floor_hsv_lower", [0, 0, 100])
        self.declare_parameter("floor_hsv_upper", [180, 50, 255])
        self.declare_parameter("bev_width_m", 4.0)
        self.declare_parameter("bev_depth_m", 6.0)
        self.declare_parameter("grid_resolution", 0.01)
        self.declare_parameter("overlay_transparency", 0.3)
        self.declare_parameter("enable_bev_overlay", True)
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("annotated_image_topic", "/occupancy/annotated_image")
        self.declare_parameter("grid_topic", "/occupancy/grid")
        self.declare_parameter("bev_overlay_width", 200)
        self.declare_parameter("bev_overlay_height", 300)

        self._bridge = CvBridge()
        self._homography = None
        self._bev_shape = None
        self._last_header = None
        self._first_frame_logged = False

        topic_in = self.get_parameter("input_image_topic").get_parameter_value().string_value
        topic_annotated = self.get_parameter("annotated_image_topic").get_parameter_value().string_value
        topic_grid = self.get_parameter("grid_topic").get_parameter_value().string_value

        # Use sensor_data QoS so we match video publisher (BEST_EFFORT) and bridge can receive our output
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._sub = self.create_subscription(Image, topic_in, self._image_callback, sensor_qos)
        self._pub_annotated = self.create_publisher(Image, topic_annotated, sensor_qos)
        self._pub_grid = self.create_publisher(OccupancyGrid, topic_grid, 10)

        self.get_logger().info(
            "Occupancy node: %s -> %s, %s" % (topic_in, topic_annotated, topic_grid)
        )

    def _get_param_ints(self, name: str, default: list) -> list:
        p = self.get_parameter(name).get_parameter_value()
        if p.type == ParameterType.PARAMETER_INTEGER_ARRAY and p.integer_array_value:
            return list(p.integer_array_value)
        if p.type == ParameterType.PARAMETER_STRING_ARRAY and p.string_array_value:
            return [int(x) for x in p.string_array_value]
        return default

    def _get_param_doubles(self, name: str, default: list) -> list:
        p = self.get_parameter(name).get_parameter_value()
        if p.type == ParameterType.PARAMETER_DOUBLE_ARRAY and p.double_array_value:
            return list(p.double_array_value)
        if p.type == ParameterType.PARAMETER_STRING_ARRAY and p.string_array_value:
            return [float(x) for x in p.string_array_value]
        return default

    def _ensure_homography(self, width: int, height: int) -> np.ndarray:
        if self._homography is not None and self._bev_shape is not None:
            return self._homography
        bev_width_m = self.get_parameter("bev_width_m").get_parameter_value().double_value
        bev_depth_m = self.get_parameter("bev_depth_m").get_parameter_value().double_value
        resolution = self.get_parameter("grid_resolution").get_parameter_value().double_value
        cols, rows = _bev_dst_shape(bev_width_m, bev_depth_m, resolution)
        self._bev_shape = (cols, rows)
        src = _default_bev_src_points(width, height)
        dst = np.float32([
            [0, rows - 1],
            [cols - 1, rows - 1],
            [cols - 1, 0],
            [0, 0],
        ])
        self._homography = cv2.getPerspectiveTransform(src, dst)
        return self._homography

    def _segment_floor(self, bgr: np.ndarray) -> np.ndarray:
        """Binary mask: 255 = floor, 0 = non-floor."""
        lower = self._get_param_ints("floor_hsv_lower", [0, 0, 100])
        upper = self._get_param_ints("floor_hsv_upper", [180, 50, 255])
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        lower_arr = np.array(lower, dtype=np.uint8)
        upper_arr = np.array(upper, dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_arr, upper_arr)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return mask

    def _floor_mask_to_bev(self, mask: np.ndarray, width: int, height: int) -> np.ndarray:
        """BEV mask: 255 = free, 0 = occupied/unknown."""
        H = self._ensure_homography(width, height)
        cols, rows = self._bev_shape
        bev = cv2.warpPerspective(mask, H, (cols, rows), flags=cv2.INTER_LINEAR)
        return bev

    def _bev_mask_to_occupancy_grid(self, bev_mask: np.ndarray, header: Header) -> OccupancyGrid:
        """nav_msgs/OccupancyGrid: 0=free, 100=occupied, -1=unknown."""
        cols, rows = self._bev_shape
        resolution = self.get_parameter("grid_resolution").get_parameter_value().double_value
        frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        # Threshold BEV: high value -> free (0), low -> occupied (100), outside view -> -1
        free_thresh = 128
        data = np.full((rows * cols,), -1, dtype=np.int8)
        for i in range(rows):
            for j in range(cols):
                idx = i * cols + j
                v = bev_mask[i, j]
                if v >= free_thresh:
                    data[idx] = 0
                else:
                    data[idx] = 100

        msg = OccupancyGrid()
        msg.header = header
        msg.info.resolution = resolution
        msg.info.width = cols
        msg.info.height = rows
        bev_width_m = self.get_parameter("bev_width_m").get_parameter_value().double_value
        msg.info.origin = Pose()
        msg.info.origin.position.x = -bev_width_m / 2.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = data.tolist()
        return msg

    def _build_annotated_image(
        self,
        bgr: np.ndarray,
        floor_mask: np.ndarray,
        bev_mask: np.ndarray,
    ) -> np.ndarray:
        """Overlay floor (green tint) and optional BEV mini-map."""
        overlay_transparency = self.get_parameter("overlay_transparency").get_parameter_value().double_value
        enable_bev = self.get_parameter("enable_bev_overlay").get_parameter_value().bool_value
        ow = self.get_parameter("bev_overlay_width").get_parameter_value().integer_value
        oh = self.get_parameter("bev_overlay_height").get_parameter_value().integer_value

        out = bgr.copy()
        # Green floor overlay
        green = np.zeros_like(bgr)
        green[:, :, 1] = 255
        mask_3 = np.stack([floor_mask, floor_mask, floor_mask], axis=-1).astype(np.uint8)
        out = np.where(mask_3 > 0, cv2.addWeighted(out, 1.0, green, overlay_transparency, 0), out).astype(np.uint8)

        if enable_bev and self._bev_shape is not None:
            cols, rows = self._bev_shape
            bev_rgb = np.zeros((rows, cols, 3), dtype=np.uint8)
            bev_rgb[bev_mask >= 128] = [0, 255, 0]   # free = green
            bev_rgb[bev_mask < 128] = [128, 128, 128]  # occupied/unknown = gray
            bev_small = cv2.resize(bev_rgb, (ow, oh), interpolation=cv2.INTER_NEAREST)
            # Grid lines every ~0.5m in overlay (scale from BEV: 0.5m/resolution px, then scale to ow/oh)
            res = self.get_parameter("grid_resolution").get_parameter_value().double_value
            step_bev = max(1, int(0.5 / res))
            step_x = max(1, int(ow * step_bev / cols))
            step_y = max(1, int(oh * step_bev / rows))
            for x in range(0, ow, step_x):
                bev_small[:, x] = [255, 255, 255]
            for y in range(0, oh, step_y):
                bev_small[y, :] = [255, 255, 255]
            h, w = out.shape[:2]
            x1, y1 = w - ow - 10, 10
            if x1 >= 0 and y1 + oh <= h:
                out[y1 : y1 + oh, x1 : x1 + ow] = bev_small
        return out

    def _image_callback(self, msg: Image) -> None:
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn("cv_bridge conversion failed: %s" % e)
            return
        try:
            height, width = bgr.shape[:2]
            header = msg.header
            if header.frame_id == "":
                header.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
            self._last_header = header

            floor_mask = self._segment_floor(bgr)
            self._ensure_homography(width, height)
            bev_mask = self._floor_mask_to_bev(floor_mask, width, height)

            # Occupancy grid
            grid_msg = self._bev_mask_to_occupancy_grid(bev_mask, header)
            self._pub_grid.publish(grid_msg)

            # Annotated image
            annotated = self._build_annotated_image(bgr, floor_mask, bev_mask)
            out_msg = self._bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            out_msg.header = header
            self._pub_annotated.publish(out_msg)
            if not self._first_frame_logged:
                self._first_frame_logged = True
                self.get_logger().info(
                    "First occupancy frame received and published (input=%s)"
                    % self.get_parameter("input_image_topic").get_parameter_value().string_value
                )
        except Exception as e:
            self.get_logger().error("Occupancy pipeline failed: %s" % e, throttle_duration_sec=5.0)


def main(args=None):
    import sys
    if not HAS_CV2:
        print("opencv-python required for occupancy_node", file=sys.stderr)
        sys.exit(1)
    rclpy.init(args=args)
    node = OccupancyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
