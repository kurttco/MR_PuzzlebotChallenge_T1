#!/usr/bin/env python3
"""
hsv_calibrator.py
-----------------
Herramienta interactiva para calibrar los rangos HSV del semaforo segun la
iluminacion real del cuarto. Se suscribe al mismo topico que el detector,
asi que solo necesita que ros_deep_learning este publicando.

Modo de uso:
  1. Levantar ros_deep_learning (publica /video_source/raw):
       ros2 launch ros_deep_learning video_source.ros2.launch
  2. Lanzar este nodo:
       ros2 run puzzlebot_mc2 hsv_calibrator.py
  3. Aparece una ventana con la imagen, mascara y resultado, mas 6 sliders.
  4. Cargar preset del color a calibrar:
       'r' = RED      'y' = YELLOW      'g' = GREEN
  5. Poner la cartulina del color frente a la camara.
  6. Mover los sliders hasta que SOLO ese color quede blanco en la mascara.
  7. Imprimir los valores actuales con 'p' y copiarlos al
     config/vision_params.yaml.
  8. Repetir para cada color (los 3).
  9. 'q' para salir.

NOTAS:
  - El rojo tiene wrap-around en H: si el rojo abarca ambos extremos
    (cerca de 0 y cerca de 180), calibrar UNO de los dos rangos primero
    poniendo el slider H_min cerca de 0, y luego el otro rango con H_min
    cerca de 170. Cada lado se mete en {red1_*, red2_*} del YAML.
  - Si la imagen tarda en aparecer, verificar que el topico exista:
       ros2 topic hz /video_source/raw
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


WINDOW_NAME = 'HSV Calibrator'
TRACKBARS = ['H_min', 'H_max', 'S_min', 'S_max', 'V_min', 'V_max']

# Defaults razonables al cargar cada preset
PRESETS = {
    'r': ('RED   ', [0,   10, 100, 255, 100, 255]),
    'y': ('YELLOW', [20,  35, 100, 255, 100, 255]),
    'g': ('GREEN ', [40,  85,  80, 255,  80, 255]),
}


class HsvCalibrator(Node):
    def __init__(self):
        super().__init__('hsv_calibrator')
        self.declare_parameter('image_topic', '/video_source/raw')
        topic = self.get_parameter('image_topic').value

        self.bridge = CvBridge()
        self.last_image = None
        self.current_color = 'r'

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, 1100, 700)
        for name in TRACKBARS:
            max_val = 180 if name.startswith('H_') else 255
            cv2.createTrackbar(name, WINDOW_NAME, 0, max_val, lambda v: None)
        self._apply_preset(self.current_color)

        self.create_subscription(
            Image, topic, self._cb_image, qos_profile_sensor_data)
        self.create_timer(1.0 / 30.0, self._tick)

        print('============ HSV Calibrator ============')
        print(f'  Suscrito a: {topic}')
        print('  p  -> imprime valores actuales para el YAML')
        print('  r  -> preset RED')
        print('  y  -> preset YELLOW')
        print('  g  -> preset GREEN')
        print('  q  -> salir')
        print('========================================')

    def _apply_preset(self, key):
        name, vals = PRESETS[key]
        for tn, v in zip(TRACKBARS, vals):
            cv2.setTrackbarPos(tn, WINDOW_NAME, int(v))
        self.current_color = key
        print(f'>> Preset cargado: {name.strip()}')

    def _cb_image(self, msg):
        try:
            self.last_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge: {e}')

    def _read_trackbars(self):
        return [cv2.getTrackbarPos(n, WINDOW_NAME) for n in TRACKBARS]

    def _tick(self):
        if self.last_image is None:
            return
        h_lo, h_hi, s_lo, s_hi, v_lo, v_hi = self._read_trackbars()

        img = self.last_image.copy()
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower = np.array([h_lo, s_lo, v_lo], dtype=np.uint8)
        upper = np.array([h_hi, s_hi, v_hi], dtype=np.uint8)

        mask = cv2.inRange(hsv, lower, upper)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        result = cv2.bitwise_and(img, img, mask=mask)

        # Layout 2x2
        H, W = img.shape[:2]
        cell_w = 540
        cell_h = int(H * cell_w / W)
        img_s = cv2.resize(img, (cell_w, cell_h))
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        mask_s = cv2.resize(mask_bgr, (cell_w, cell_h))
        result_s = cv2.resize(result, (cell_w, cell_h))
        empty = np.zeros_like(result_s)

        # etiquetas
        for label, panel in [('original', img_s), ('mask', mask_s),
                             ('result', result_s)]:
            cv2.putText(panel, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(panel, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 0), 1, cv2.LINE_AA)

        top = np.hstack([img_s, mask_s])
        bottom = np.hstack([result_s, empty])
        canvas = np.vstack([top, bottom])

        info = (f'{PRESETS[self.current_color][0].strip()}  '
                f'H[{h_lo}-{h_hi}]  S[{s_lo}-{s_hi}]  V[{v_lo}-{v_hi}]')
        cv2.putText(canvas, info, (10, canvas.shape[0] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        cv2.imshow(WINDOW_NAME, canvas)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print('>> Saliendo')
            rclpy.shutdown()
        elif key == ord('p'):
            color_name = PRESETS[self.current_color][0].strip().lower()
            print(f'\n=== Valores para {color_name.upper()} (copia al YAML) ===')
            print(f'  {color_name}_h_min: {h_lo}')
            print(f'  {color_name}_h_max: {h_hi}')
            print(f'  {color_name}_s_min: {s_lo}')
            print(f'  {color_name}_s_max: {s_hi}')
            print(f'  {color_name}_v_min: {v_lo}')
            print(f'  {color_name}_v_max: {v_hi}\n')
        elif key == ord('r'):
            self._apply_preset('r')
        elif key == ord('y'):
            self._apply_preset('y')
        elif key == ord('g'):
            self._apply_preset('g')


def main():
    rclpy.init()
    node = HsvCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
