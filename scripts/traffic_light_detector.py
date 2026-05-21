#!/usr/bin/env python3
"""
traffic_light_detector.py
-------------------------
Detector de semaforo (rojo / amarillo / verde) para el PuzzleBot.

Subscribe:
  /video_source/raw       sensor_msgs/Image (de ros_deep_learning)

Publishes:
  /semaphore_state        puzzlebot_mc2/SemaphoreState
  /traffic_light_debug    sensor_msgs/Image  (anotada, opcional)

Pipeline por frame:
  1. BGR -> HSV
  2. Para cada color (R, Y, G): inRange con rangos del YAML.
     Rojo usa DOS rangos por wrap-around de H.
  3. Morphology open + close (kernel eliptico) para limpiar ruido.
  4. findContours sobre la mascara.
  5. Filtrar cada contorno por:
        - area > min_area_px              (descarta puntitos de ruido)
        - circularidad > min_circularity   (4*pi*area / perimetro^2)
        - aspect ratio cuadrado-ish        (filtra rectangulos largos)
  6. Para cada color, queda el blob valido de mayor area (o ninguno).
  7. Entre los tres colores, gana el de mayor area absoluta.
  8. Publicar SemaphoreState con (state, confidence, area).

NO se hace memoria del estado anterior aqui; eso le toca al controller
(es donde la logica del 'red latch' tiene sentido conocer la trayectoria).
"""

import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from puzzlebot_mc2.msg import SemaphoreState


# Constantes locales (espejo del .msg)
ST_UNKNOWN = 0
ST_GREEN = 1
ST_YELLOW = 2
ST_RED = 3
ST_NAMES = {0: 'UNKNOWN', 1: 'GREEN', 2: 'YELLOW', 3: 'RED'}
ST_BGR = {1: (0, 255, 0), 2: (0, 255, 255), 3: (0, 0, 255), 0: (180, 180, 180)}


class TrafficLightDetector(Node):

    def __init__(self):
        super().__init__('traffic_light_detector')

        # ================ parametros HSV ================
        self.declare_parameter('red1_h_min', 0)
        self.declare_parameter('red1_h_max', 10)
        self.declare_parameter('red2_h_min', 170)
        self.declare_parameter('red2_h_max', 180)
        self.declare_parameter('red_s_min', 100)
        self.declare_parameter('red_s_max', 255)
        self.declare_parameter('red_v_min', 100)
        self.declare_parameter('red_v_max', 255)

        self.declare_parameter('yellow_h_min', 20)
        self.declare_parameter('yellow_h_max', 35)
        self.declare_parameter('yellow_s_min', 100)
        self.declare_parameter('yellow_s_max', 255)
        self.declare_parameter('yellow_v_min', 100)
        self.declare_parameter('yellow_v_max', 255)

        self.declare_parameter('green_h_min', 40)
        self.declare_parameter('green_h_max', 85)
        self.declare_parameter('green_s_min', 80)
        self.declare_parameter('green_s_max', 255)
        self.declare_parameter('green_v_min', 80)
        self.declare_parameter('green_v_max', 255)

        # ================ filtros geometricos ================
        self.declare_parameter('min_area_px', 500)
        self.declare_parameter('min_circularity', 0.65)
        self.declare_parameter('min_aspect_ratio', 0.7)
        self.declare_parameter('max_aspect_ratio', 1.4)
        self.declare_parameter('morph_kernel_size', 5)

        # ================ I/O ================
        self.declare_parameter('image_topic', '/video_source/raw')
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('publish_rate_hz', 15.0)

        # ---- leer rangos HSV en arrays numpy ----
        self.red_ranges = [
            self._mk_range('red1_h_min', 'red1_h_max',
                           'red_s_min', 'red_s_max',
                           'red_v_min', 'red_v_max'),
            self._mk_range('red2_h_min', 'red2_h_max',
                           'red_s_min', 'red_s_max',
                           'red_v_min', 'red_v_max'),
        ]
        self.yellow_ranges = [
            self._mk_range('yellow_h_min', 'yellow_h_max',
                           'yellow_s_min', 'yellow_s_max',
                           'yellow_v_min', 'yellow_v_max'),
        ]
        self.green_ranges = [
            self._mk_range('green_h_min', 'green_h_max',
                           'green_s_min', 'green_s_max',
                           'green_v_min', 'green_v_max'),
        ]

        self.min_area = int(self.get_parameter('min_area_px').value)
        self.min_circ = float(self.get_parameter('min_circularity').value)
        self.min_ar = float(self.get_parameter('min_aspect_ratio').value)
        self.max_ar = float(self.get_parameter('max_aspect_ratio').value)
        ks = int(self.get_parameter('morph_kernel_size').value)
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ks, ks))

        self.publish_debug = bool(
            self.get_parameter('publish_debug_image').value)

        # ================ estado interno ================
        self.bridge = CvBridge()
        self.last_image = None
        self.frame_w = 0
        self.frame_h = 0

        # ================ pubs/subs ================
        image_topic = self.get_parameter('image_topic').value
        self.create_subscription(Image, image_topic, self._cb_image,
                                 qos_profile_sensor_data)
        self.pub_state = self.create_publisher(
            SemaphoreState, '/semaphore_state', 10)
        self.pub_debug = None
        if self.publish_debug:
            self.pub_debug = self.create_publisher(
                Image, '/traffic_light_debug', 10)

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / rate, self._process)

        self.get_logger().info(
            f'Traffic light detector up | image={image_topic} | '
            f'min_area={self.min_area}px circ>{self.min_circ} '
            f'ar=[{self.min_ar},{self.max_ar}]'
        )

    def _mk_range(self, h_lo_p, h_hi_p, s_lo_p, s_hi_p, v_lo_p, v_hi_p):
        lower = np.array([
            int(self.get_parameter(h_lo_p).value),
            int(self.get_parameter(s_lo_p).value),
            int(self.get_parameter(v_lo_p).value),
        ], dtype=np.uint8)
        upper = np.array([
            int(self.get_parameter(h_hi_p).value),
            int(self.get_parameter(s_hi_p).value),
            int(self.get_parameter(v_hi_p).value),
        ], dtype=np.uint8)
        return (lower, upper)

    # ============================================================

    def _cb_image(self, msg: Image):
        try:
            self.last_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
            self.frame_h, self.frame_w = self.last_image.shape[:2]
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')

    def _detect_color(self, hsv, ranges_list):
        """Devuelve (best_contour_or_None, area_px). Aplica todos los rangos
        del mismo color (rojo necesita 2) y selecciona el blob valido mas
        grande tras filtros geometricos."""
        # construir mascara combinando todos los sub-rangos del color
        mask = None
        for lo, hi in ranges_list:
            m = cv2.inRange(hsv, lo, hi)
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        # limpieza morfologica
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        best_area = 0
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.min_area:
                continue
            perim = cv2.arcLength(c, True)
            if perim <= 1e-3:
                continue
            circ = 4.0 * math.pi * area / (perim * perim)
            if circ < self.min_circ:
                continue
            x, y, w, h = cv2.boundingRect(c)
            ar = w / max(h, 1)
            if ar < self.min_ar or ar > self.max_ar:
                continue
            if area > best_area:
                best_area = area
                best = c
        return best, best_area

    # ============================================================

    def _process(self):
        if self.last_image is None:
            return
        img = self.last_image
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        red_c, red_a = self._detect_color(hsv, self.red_ranges)
        yel_c, yel_a = self._detect_color(hsv, self.yellow_ranges)
        grn_c, grn_a = self._detect_color(hsv, self.green_ranges)

        candidates = []
        if red_a > 0:
            candidates.append((ST_RED, red_c, red_a))
        if yel_a > 0:
            candidates.append((ST_YELLOW, yel_c, yel_a))
        if grn_a > 0:
            candidates.append((ST_GREEN, grn_c, grn_a))

        if not candidates:
            state_id, contour, area = ST_UNKNOWN, None, 0
        else:
            candidates.sort(key=lambda x: x[2], reverse=True)
            state_id, contour, area = candidates[0]

        # publish state
        msg = SemaphoreState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.state = int(state_id)
        total = self.frame_w * self.frame_h
        msg.confidence = float(area) / total if total > 0 else 0.0
        msg.blob_pixel_area = int(area)
        self.pub_state.publish(msg)

        # debug image
        if self.pub_debug is not None:
            self._publish_debug(img, candidates, state_id)

    def _publish_debug(self, img, candidates, winner_state):
        dbg = img.copy()
        for sid, c, area in candidates:
            if c is None:
                continue
            color = ST_BGR[sid]
            cv2.drawContours(dbg, [c], -1, color, 2)
            x, y, w, h = cv2.boundingRect(c)
            cv2.rectangle(dbg, (x, y), (x + w, y + h), color, 1)
            cv2.putText(dbg, f'{ST_NAMES[sid]} a={int(area)}',
                        (x, max(y - 5, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # banner del estado actual
        banner_color = ST_BGR[winner_state]
        cv2.rectangle(dbg, (0, 0), (self.frame_w, 40), banner_color, -1)
        cv2.putText(dbg, f'STATE: {ST_NAMES[winner_state]}',
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2)
        try:
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8'))
        except Exception as e:
            self.get_logger().warn(f'cv_bridge debug: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDetector()
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
