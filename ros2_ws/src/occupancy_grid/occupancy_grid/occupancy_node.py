#!/usr/bin/env python3
"""
Occupancy grid node: adaptive floor segmentation from monocular RGB camera.
Learns floor appearance from bottom region, handles multi-room navigation.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False


class OccupancyNode(Node):
    """Adaptive floor segmentation -> BEV occupancy grid from monocular camera."""

    def __init__(self):
        super().__init__("occupancy_node")
        if not HAS_CV2:
            self.get_logger().error("opencv-python required")
            raise RuntimeError("opencv-python required")

        # Parameters
        self.declare_parameter("input_image_topic", "/robot/video/front")
        self.declare_parameter("annotated_image_topic", "/occupancy/annotated_image")
        self.declare_parameter("grid_topic", "/occupancy/grid")
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("bev_width_m", 4.0)
        self.declare_parameter("bev_depth_m", 6.0)
        self.declare_parameter("grid_resolution", 0.05)  # 5cm cells
        self.declare_parameter("recalibrate_interval", 30)  # frames
        self.declare_parameter("overlay_alpha", 0.3)

        # State
        self._bridge = CvBridge()
        self._floor_model = None
        self._frame_count = 0
        self._homography = None
        self._bev_shape = None
        self._prev_bev = None
        self._prev_gray = None

        # QoS: best effort to match typical camera publishers
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscriptions & Publishers
        topic_in = self.get_parameter("input_image_topic").value
        topic_annotated = self.get_parameter("annotated_image_topic").value
        topic_grid = self.get_parameter("grid_topic").value

        self._sub = self.create_subscription(Image, topic_in, self._on_image, sensor_qos)
        self._pub_annotated = self.create_publisher(Image, topic_annotated, sensor_qos)
        self._pub_grid = self.create_publisher(OccupancyGrid, topic_grid, 10)

        self.get_logger().info(f"Occupancy node: {topic_in} -> {topic_annotated}, {topic_grid}")

    def _compute_homography(self, width: int, height: int):
        """Compute perspective transform: image trapezoid -> BEV rectangle."""
        if self._homography is not None:
            return

        # Trapezoid in image (floor region, wider at bottom)
        w, h = float(width), float(height)
        src = np.float32([
            [w * 0.20, h * 0.85],  # bottom-left
            [w * 0.80, h * 0.85],  # bottom-right
            [w * 0.65, h * 0.35],  # top-right
            [w * 0.35, h * 0.35],  # top-left
        ])

        # BEV rectangle in meters -> pixels
        bev_width_m = self.get_parameter("bev_width_m").value
        bev_depth_m = self.get_parameter("bev_depth_m").value
        resolution = self.get_parameter("grid_resolution").value

        cols = max(1, int(round(bev_width_m / resolution)))
        rows = max(1, int(round(bev_depth_m / resolution)))
        self._bev_shape = (cols, rows)

        dst = np.float32([
            [0, rows - 1],         # bottom-left
            [cols - 1, rows - 1],  # bottom-right
            [cols - 1, 0],         # top-right
            [0, 0],                # top-left
        ])

        self._homography = cv2.getPerspectiveTransform(src, dst)
        self.get_logger().info(f"BEV: {cols}x{rows} px, {bev_width_m}x{bev_depth_m}m @ {resolution}m/px")

    def _floor_roi_mask(self, width: int, height: int) -> np.ndarray:
        """Binary mask for the floor trapezoid ROI (255 inside). Same shape as homography."""
        w, h = float(width), float(height)
        poly = np.array([[
            [w * 0.20, h * 0.85],
            [w * 0.80, h * 0.85],
            [w * 0.65, h * 0.35],
            [w * 0.35, h * 0.35],
        ]], dtype=np.int32)
        mask = np.zeros((height, width), dtype=np.uint8)
        cv2.fillPoly(mask, poly, 255)
        return mask

    def _update_floor_model(self, bgr: np.ndarray):
        """Learn floor color distribution from 75-90% image height (avoid bottom reflections)."""
        h, w = bgr.shape[:2]
        floor_region = bgr[int(h * 0.75):int(h * 0.90), :]

        # LAB: use only chroma (A, B) so glare/shadows (L) don't corrupt floor model
        lab = cv2.cvtColor(floor_region, cv2.COLOR_BGR2LAB).astype(np.float32)
        chroma = lab[:, :, 1:3]
        pixels = chroma.reshape(-1, 2)

        # Robust statistics: median + MAD (resistant to outliers)
        median = np.median(pixels, axis=0)
        mad = np.median(np.abs(pixels - median), axis=0)

        self._floor_model = {"median": median, "mad": mad}

    def _detect_glare(self, bgr: np.ndarray, floor_roi: np.ndarray) -> np.ndarray:
        """
        Glare = very bright AND low chroma (near white), restricted to floor ROI.
        Adaptive thresholds based on ROI percentiles.
        Returns mask 255=glare.
        """
        lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB).astype(np.float32)
        L = lab[:, :, 0]
        A = lab[:, :, 1] - 128.0
        B = lab[:, :, 2] - 128.0
        chroma = np.sqrt(A * A + B * B)

        roi_idx = floor_roi > 0
        if np.count_nonzero(roi_idx) < 100:
            return np.zeros((bgr.shape[0], bgr.shape[1]), dtype=np.uint8)

        L_hi = np.percentile(L[roi_idx], 95)
        C_lo = np.percentile(chroma[roi_idx], 20)
        glare = (L >= L_hi) & (chroma <= C_lo) & roi_idx
        glare = glare.astype(np.uint8) * 255

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))
        glare = cv2.morphologyEx(glare, cv2.MORPH_CLOSE, kernel)
        return glare

    def _detect_specular_gradient(self, bgr: np.ndarray, floor_roi: np.ndarray) -> np.ndarray:
        """
        Specular blobs: bright but low structural gradient (within floor ROI).
        Adaptive bright threshold via ROI percentile.
        """
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY).astype(np.float32)
        gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
        gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
        mag = np.sqrt(gx * gx + gy * gy)

        roi_idx = floor_roi > 0
        if np.count_nonzero(roi_idx) < 100:
            return np.zeros((bgr.shape[0], bgr.shape[1]), dtype=np.uint8)

        bright_thr = np.percentile(gray[roi_idx], 97)
        lowgrad_thr = np.percentile(mag[roi_idx], 35)
        bright = gray >= bright_thr
        low_grad = mag <= lowgrad_thr
        specular = (bright & low_grad & roi_idx).astype(np.uint8) * 255

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        specular = cv2.morphologyEx(specular, cv2.MORPH_CLOSE, kernel)
        return specular

    def _segment_floor(self, bgr: np.ndarray, uncertain: np.ndarray) -> np.ndarray:
        """Adaptive color-based floor segmentation with spatial prior. Excludes precomputed uncertain mask."""
        h, w = bgr.shape[:2]

        # Recalibrate floor model periodically
        recal_interval = self.get_parameter("recalibrate_interval").value
        if self._frame_count % recal_interval == 0:
            self._update_floor_model(bgr)

        if self._floor_model is None:
            return np.zeros((h, w), dtype=np.uint8)

        # Segment via chroma only (A, B); L ignored so glare/shadows don't dominate
        lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB).astype(np.float32)
        chroma = lab[:, :, 1:3]
        median = self._floor_model["median"]
        mad = self._floor_model["mad"]

        # Mahalanobis-ish distance (normalized by MAD) on chroma
        diff = np.abs(chroma - median)
        distance = np.max(diff / (mad + 1e-6), axis=2)

        # Threshold: pixels within 3 MADs are floor candidates
        mask = (distance < 3.0).astype(np.float32)

        # Spatial prior: weight bottom of image higher
        y_weight = np.linspace(0.3, 1.0, h).reshape(-1, 1)
        mask = mask * np.tile(y_weight, (1, w))
        mask = (mask * 255).astype(np.uint8)

        # Morphological cleanup
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        mask = cv2.bitwise_and(mask, cv2.bitwise_not(uncertain))
        return mask

    def _mask_to_bev(self, mask: np.ndarray) -> np.ndarray:
        """Warp floor mask to bird's-eye view with temporal stabilization."""
        cols, rows = self._bev_shape
        bev = cv2.warpPerspective(mask, self._homography, (cols, rows), flags=cv2.INTER_LINEAR)
        if self._prev_bev is not None:
            bev = cv2.addWeighted(bev, 0.6, self._prev_bev, 0.4, 0)
        self._prev_bev = bev.copy()
        return bev

    def _bev_to_occupancy_grid(self, bev: np.ndarray, header) -> OccupancyGrid:
        """Convert BEV mask to ROS OccupancyGrid (0=free, 100=occupied, -1=unknown)."""
        cols, rows = self._bev_shape
        resolution = self.get_parameter("grid_resolution").value
        frame_id = self.get_parameter("frame_id").value

        flat = bev.flatten()
        data = np.full_like(flat, -1, dtype=np.int8)  # default UNKNOWN
        data[flat >= 200] = 0   # strong floor -> free
        data[flat <= 50] = 100  # strong non-floor -> occupied

        msg = OccupancyGrid()
        msg.header = header
        msg.header.frame_id = frame_id
        msg.info.resolution = resolution
        msg.info.width = cols
        msg.info.height = rows

        # Origin: center of grid at y=0 (front of robot), x=0 (robot centerline)
        bev_width_m = self.get_parameter("bev_width_m").value
        msg.info.origin = Pose()
        msg.info.origin.position.x = -bev_width_m / 2.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = data.tolist()

        return msg

    def _create_debug_image(self, bgr: np.ndarray, floor_mask: np.ndarray, bev: np.ndarray) -> np.ndarray:
        """Annotated image: green floor overlay + BEV mini-map."""
        alpha = self.get_parameter("overlay_alpha").value
        out = bgr.copy()

        # Green tint on detected floor
        green_overlay = np.zeros_like(bgr)
        green_overlay[:, :, 1] = 255
        mask_3ch = cv2.cvtColor(floor_mask, cv2.COLOR_GRAY2BGR)
        out = np.where(mask_3ch > 0, cv2.addWeighted(out, 1.0, green_overlay, alpha, 0), out)

        # BEV mini-map in top-right corner (free=green, unknown=yellow, occupied=gray)
        cols, rows = self._bev_shape
        bev_rgb = np.zeros((rows, cols, 3), dtype=np.uint8)
        bev_rgb[bev >= 200] = [0, 255, 0]     # free = green
        bev_rgb[(bev > 50) & (bev < 200)] = [0, 255, 255]  # unknown = yellow
        bev_rgb[bev <= 50] = [100, 100, 100]  # occupied = gray

        # Resize to 200x300 for display
        bev_small = cv2.resize(bev_rgb, (200, 300), interpolation=cv2.INTER_NEAREST)

        # Grid lines every 0.5m
        resolution = self.get_parameter("grid_resolution").value
        step_bev = max(1, int(0.5 / resolution))
        step_x = max(1, int(200 * step_bev / cols))
        step_y = max(1, int(300 * step_bev / rows))
        for x in range(0, 200, step_x):
            bev_small[:, x] = [255, 255, 255]
        for y in range(0, 300, step_y):
            bev_small[y, :] = [255, 255, 255]

        # Overlay in top-right
        h, w = out.shape[:2]
        if w > 210 and h > 310:
            out[10:310, w-210:w-10] = bev_small

        return out

    def _on_image(self, msg: Image):
        """Main callback: segment floor, build occupancy grid, publish debug viz."""
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge failed: {e}")
            return

        try:
            h, w = bgr.shape[:2]
            self._frame_count += 1

            # Optional color constancy (reduce lighting variation; skip if xphoto unavailable)
            try:
                bgr = cv2.xphoto.createSimpleWB().balanceWhite(bgr)
            except Exception:
                pass

            floor_roi = self._floor_roi_mask(w, h)

            # Temporal flicker: reflections change frame-to-frame (adaptive + ROI)
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            if self._prev_gray is not None:
                diff = cv2.absdiff(gray, self._prev_gray).astype(np.float32)
                roi_idx = floor_roi > 0
                if np.count_nonzero(roi_idx) >= 100:
                    thr = np.percentile(diff[roi_idx], 90)
                    flicker_mask = ((diff >= max(10.0, thr)) & roi_idx).astype(np.uint8) * 255
                else:
                    flicker_mask = np.zeros((h, w), dtype=np.uint8)
            else:
                flicker_mask = np.zeros((h, w), dtype=np.uint8)
            self._prev_gray = gray.copy()

            # Compute homography once
            if self._homography is None:
                self._compute_homography(w, h)

            glare_mask = self._detect_glare(bgr, floor_roi)
            specular_mask = self._detect_specular_gradient(bgr, floor_roi)
            uncertain = cv2.bitwise_or(glare_mask, cv2.bitwise_or(specular_mask, flicker_mask))
            floor_mask = self._segment_floor(bgr, uncertain)

            # BEV transform
            bev = self._mask_to_bev(floor_mask)

            # Publish occupancy grid
            header = msg.header
            if not header.frame_id:
                header.frame_id = self.get_parameter("frame_id").value
            grid = self._bev_to_occupancy_grid(bev, header)
            self._pub_grid.publish(grid)

            # Publish debug image
            annotated = self._create_debug_image(bgr, floor_mask, bev)
            img_msg = self._bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            img_msg.header = header
            self._pub_annotated.publish(img_msg)

        except Exception as e:
            self.get_logger().error(f"Pipeline failed: {e}", throttle_duration_sec=5.0)


def main(args=None):
    if not HAS_CV2:
        print("ERROR: opencv-python required", file=__import__("sys").stderr)
        return 1

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