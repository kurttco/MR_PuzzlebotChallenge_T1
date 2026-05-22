#!/usr/bin/env python3
"""
line_detector.py
----------------
Line detector node for MC5 (line following).

Subscribes:
  /video_source/raw          sensor_msgs/Image (from ros_deep_learning)

Publishes:
  /line_error                std_msgs/Float32   normalized error in [-1, +1]
  /line_detected             std_msgs/Bool      True if at least one valid blob
  /line_debug                sensor_msgs/Image  annotated frame (optional)

Pipeline per frame:
  1. Crop the image to a ROI (vertical band, full width). The line is
     looked for near the bottom of the frame so we ignore ceiling,
     walls, and far-away noise.
  2. Convert to grayscale.
  3. Gaussian blur to attenuate texture noise.
  4. Adaptive threshold (THRESH_BINARY_INV) so DARK lines on a lighter
     floor become WHITE blobs on a black background. Adaptive (vs a
     fixed threshold) means uneven lighting across the floor is handled
     locally instead of needing a perfectly flat illumination.
  5. Morphological open + close (elliptical kernel) to clean small
     specks and fill internal holes in the line.
  6. cv2.connectedComponentsWithStats over the binary mask. This is
     the moments / connected components approach (no findContours):
     each connected white blob gets a label, and we get bounding box,
     area, and centroid for free.
  7. Filter labels by:
        - area in [min_blob_area, max_blob_area]    (reject noise and whole-floor blobs)
        - bbox aspect ratio h/w >= min_aspect_ratio (reject horizontal dashes)
  8. If no valid blob remains, publish (error=0.0, detected=False).
     Otherwise compute the average centroid x and normalize the
     horizontal error to [-1, +1]:
        error = (cx_avg - W/2) / (W/2)
     where W is the FULL image width (not ROI width). Because the ROI
     uses full width, the x coordinate of a centroid in the ROI is the
     same x in the full frame, so this is consistent.

The detector publishes a normalized scalar -- it has NO memory of past
frames and NO control logic. The controller is the one that handles
"line lost" timeouts, speed control, and the FSM.
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool


class LineDetector(Node):

    def __init__(self):
        super().__init__('line_detector')

        # ================ parameters ================
        self.declare_parameter('image_topic', '/video_source/raw')
        self.declare_parameter('roi_top_ratio', 0.55)
        self.declare_parameter('roi_bottom_ratio', 0.85)
        self.declare_parameter('blur_kernel_size', 5)
        self.declare_parameter('threshold_block_size', 31)
        self.declare_parameter('threshold_c', 8)
        self.declare_parameter('morph_kernel_size', 5)
        self.declare_parameter('min_blob_area', 300)
        self.declare_parameter('max_blob_area', 50000)
        self.declare_parameter('min_aspect_ratio', 0.8)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('publish_rate_hz', 20.0)

        image_topic = self.get_parameter('image_topic').value
        self.roi_top_ratio = float(self.get_parameter('roi_top_ratio').value)
        self.roi_bottom_ratio = float(
            self.get_parameter('roi_bottom_ratio').value)

        # Kernel sizes must be odd for OpenCV; coerce if needed.
        self.blur_k = self._odd(
            int(self.get_parameter('blur_kernel_size').value))
        self.thr_block = self._odd(
            int(self.get_parameter('threshold_block_size').value))
        # block_size for adaptiveThreshold must be >= 3
        if self.thr_block < 3:
            self.thr_block = 3

        self.thr_c = int(self.get_parameter('threshold_c').value)

        self.morph_k = int(self.get_parameter('morph_kernel_size').value)
        if self.morph_k < 1:
            self.morph_k = 1
        self.morph_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (self.morph_k, self.morph_k))

        self.min_area = int(self.get_parameter('min_blob_area').value)
        self.max_area = int(self.get_parameter('max_blob_area').value)
        self.min_ar = float(self.get_parameter('min_aspect_ratio').value)

        self.publish_debug = bool(
            self.get_parameter('publish_debug_image').value)

        # ================ state ================
        self.bridge = CvBridge()
        self.last_frame = None
        self.frame_h = 0
        self.frame_w = 0

        # ================ pubs/subs ================
        self.create_subscription(
            Image, image_topic, self._cb_image, qos_profile_sensor_data)

        self.pub_error = self.create_publisher(Float32, '/line_error', 10)
        self.pub_detected = self.create_publisher(Bool, '/line_detected', 10)
        self.pub_debug = None
        if self.publish_debug:
            self.pub_debug = self.create_publisher(
                Image, '/line_debug', 10)

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / rate, self._process)

        self.get_logger().info(
            f'Line detector up | image={image_topic} | '
            f'ROI=[{self.roi_top_ratio:.2f}-{self.roi_bottom_ratio:.2f}] | '
            f'min_area={self.min_area}px ar>={self.min_ar} | '
            f'rate={rate} Hz'
        )

    @staticmethod
    def _odd(n: int) -> int:
        """Coerce n to the nearest odd integer >= 1."""
        if n < 1:
            return 1
        return n if (n % 2 == 1) else n + 1

    # ============================================================

    def _cb_image(self, msg: Image):
        try:
            self.last_frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
            self.frame_h, self.frame_w = self.last_frame.shape[:2]
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')

    # ============================================================

    def _process(self):
        if self.last_frame is None:
            return

        img = self.last_frame.copy()
        H, W = img.shape[:2]
        if H == 0 or W == 0:
            return

        # ---- ROI ----
        roi_top = int(H * self.roi_top_ratio)
        roi_bottom = int(H * self.roi_bottom_ratio)
        if roi_bottom <= roi_top:
            return
        roi = img[roi_top:roi_bottom, :]

        # ---- preprocessing ----
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (self.blur_k, self.blur_k), 0)

        binary = cv2.adaptiveThreshold(
            blurred,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            self.thr_block,
            self.thr_c
        )

        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, self.morph_kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, self.morph_kernel)

        # ---- connected components ----
        num_labels, _labels, stats, centroids = \
            cv2.connectedComponentsWithStats(binary, connectivity=8)

        valid_cx = []
        valid_boxes = []  # (x, y, w, h) in ROI coords, for debug

        # label 0 is the background, skip it
        for i in range(1, num_labels):
            area = int(stats[i, cv2.CC_STAT_AREA])
            if area < self.min_area or area > self.max_area:
                continue

            x = int(stats[i, cv2.CC_STAT_LEFT])
            y = int(stats[i, cv2.CC_STAT_TOP])
            w = int(stats[i, cv2.CC_STAT_WIDTH])
            h = int(stats[i, cv2.CC_STAT_HEIGHT])

            aspect = h / max(w, 1)
            if aspect < self.min_ar:
                continue

            # centroid x in ROI coords. Because ROI is full width, this
            # x is also the x in the full frame.
            cx = float(centroids[i, 0])
            valid_cx.append(cx)
            valid_boxes.append((x, y, w, h))

        # ---- compute error and publish ----
        if not valid_cx:
            line_error = 0.0
            line_detected = False
            cx_avg = None
        else:
            cx_avg = float(np.mean(valid_cx))
            line_error = (cx_avg - (W / 2.0)) / (W / 2.0)
            line_error = max(-1.0, min(1.0, line_error))
            line_detected = True

        err_msg = Float32()
        err_msg.data = float(line_error)
        self.pub_error.publish(err_msg)

        det_msg = Bool()
        det_msg.data = bool(line_detected)
        self.pub_detected.publish(det_msg)

        # ---- debug image ----
        if self.pub_debug is not None:
            self._publish_debug_image(
                img, roi_top, roi_bottom,
                valid_boxes, cx_avg, line_error, line_detected
            )

    # ============================================================

    def _publish_debug_image(self, img, roi_top, roi_bottom,
                             valid_boxes, cx_avg, line_error, line_detected):
        dbg = img.copy()
        H, W = dbg.shape[:2]

        # ROI rectangle (green)
        cv2.rectangle(
            dbg, (0, roi_top), (W - 1, roi_bottom), (0, 255, 0), 2)

        # Blob bounding boxes (yellow), shifted by roi_top into full frame
        for (x, y, w, h) in valid_boxes:
            x1 = x
            y1 = y + roi_top
            x2 = x + w
            y2 = y + h + roi_top
            cv2.rectangle(dbg, (x1, y1), (x2, y2), (0, 255, 255), 1)

        # Average centroid: vertical line through full ROI height (magenta)
        if cx_avg is not None:
            xi = int(round(cx_avg))
            cv2.line(dbg, (xi, roi_top), (xi, roi_bottom),
                     (255, 0, 255), 2)

        # Image center line (white) -- reference
        cx_ref = int(W / 2)
        cv2.line(dbg, (cx_ref, roi_top), (cx_ref, roi_bottom),
                 (255, 255, 255), 1)

        # Text overlay (white text, black shadow for legibility)
        n = len(valid_boxes)
        text = f'err: {line_error:+.3f} | blobs: {n} | det: {int(line_detected)}'
        # shadow
        cv2.putText(dbg, text, (11, 26), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 0, 0), 3, cv2.LINE_AA)
        # foreground
        cv2.putText(dbg, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (255, 255, 255), 1, cv2.LINE_AA)

        try:
            self.pub_debug.publish(
                self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8'))
        except Exception as e:
            self.get_logger().warn(f'cv_bridge debug: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LineDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
