#!/usr/bin/env python3
"""
path_generator.py
-----------------
Nodo generador de trayectoria para MC2.

Flujo:
  1. Al arrancar, carga los waypoints del YAML (parámetros).
  2. Valida alcanzabilidad ANTES de empezar, reportando cada punto.
     - Geométrico: dentro del bounding box del workspace.
     - Cinemático: N/A para diferencial (todo (x,y) es alcanzable).
     - Dinámico:   t_estimado a v_max <= timeout configurado.
  3. Publica el primer goal en /goals (puzzlebot_mc2/Goal).
  4. Escucha /goal_reached (std_msgs/UInt32). Cuando el id coincide
     con el goal actual, espera `inter_goal_pause_s` segundos y publica
     el siguiente. Cuando se acabaron, termina.
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt32
from puzzlebot_mc2.msg import Goal


class PathGenerator(Node):

    def __init__(self):
        super().__init__('path_generator')

        # ---- parámetros ----
        self.declare_parameter('waypoints_x', [2.0, 2.0, 0.0, 0.0])
        self.declare_parameter('waypoints_y', [0.0, 2.0, 2.0, 0.0])
        self.declare_parameter('workspace_min_x', -0.5)
        self.declare_parameter('workspace_max_x',  3.0)
        self.declare_parameter('workspace_min_y', -0.5)
        self.declare_parameter('workspace_max_y',  3.0)
        self.declare_parameter('v_max_for_check', 0.15)
        self.declare_parameter('goal_timeout_s',  60.0)
        self.declare_parameter('publish_delay_s',  3.0)
        self.declare_parameter('inter_goal_pause_s', 1.0)

        xs = list(self.get_parameter('waypoints_x').value)
        ys = list(self.get_parameter('waypoints_y').value)
        if len(xs) != len(ys) or len(xs) == 0:
            raise RuntimeError(
                f'waypoints mal configurados: len(x)={len(xs)}, len(y)={len(ys)}'
            )
        self.waypoints = [(float(x), float(y)) for x, y in zip(xs, ys)]

        self.ws_x = (float(self.get_parameter('workspace_min_x').value),
                     float(self.get_parameter('workspace_max_x').value))
        self.ws_y = (float(self.get_parameter('workspace_min_y').value),
                     float(self.get_parameter('workspace_max_y').value))
        self.v_max = float(self.get_parameter('v_max_for_check').value)
        self.timeout = float(self.get_parameter('goal_timeout_s').value)
        self.delay = float(self.get_parameter('publish_delay_s').value)
        self.pause = float(self.get_parameter('inter_goal_pause_s').value)

        # ---- estado ----
        self.current_idx = 0
        self._next_timer = None
        self._start_timer = None

        # ---- validación previa ----
        self._validate_all_or_die()

        # ---- pubs/subs ----
        self.pub = self.create_publisher(Goal, '/goals', 10)
        self.create_subscription(UInt32, '/goal_reached', self._cb_reached, 10)

        self.get_logger().info(
            f'Path generator listo | {len(self.waypoints)} waypoints | '
            f'primer goal en {self.delay:.1f}s'
        )
        self._start_timer = self.create_timer(self.delay, self._send_first)

    # -------------------- validación ------------------------------

    def _check_reachability(self, x: float, y: float, idx: int):
        """Retorna (reachable: bool, reason: str)."""
        # 1. Workspace bounds
        if not (self.ws_x[0] <= x <= self.ws_x[1] and
                self.ws_y[0] <= y <= self.ws_y[1]):
            return False, (
                f'fuera del workspace '
                f'x∈[{self.ws_x[0]:.2f},{self.ws_x[1]:.2f}] '
                f'y∈[{self.ws_y[0]:.2f},{self.ws_y[1]:.2f}]'
            )
        # 2. Kinemático: diferencial alcanza cualquier (x,y). OK.
        # 3. Dinámico: tiempo estimado a v_max
        px, py = self.waypoints[idx - 1] if idx > 0 else (0.0, 0.0)
        dist = math.hypot(x - px, y - py)
        t_est = dist / max(self.v_max, 1e-3)
        if t_est > self.timeout:
            return False, (
                f't estimado {t_est:.1f}s > timeout {self.timeout:.1f}s '
                f'(dist={dist:.2f}m, v_max={self.v_max})'
            )
        return True, 'ok'

    def _validate_all_or_die(self):
        self.get_logger().info('--- Validación de alcanzabilidad ---')
        all_ok = True
        for i, (x, y) in enumerate(self.waypoints):
            ok, reason = self._check_reachability(x, y, i)
            tag = 'OK    ' if ok else 'NO OK '
            self.get_logger().info(
                f'  {tag} wp{i} ({x:+.2f}, {y:+.2f}) -> {reason}'
            )
            if not ok:
                all_ok = False
        if not all_ok:
            raise RuntimeError('uno o más waypoints NO alcanzables, abortando')
        self.get_logger().info('--- Todos los waypoints alcanzables ---')

    # -------------------- control de flujo ------------------------

    def _send_first(self):
        if self._start_timer is not None:
            self._start_timer.cancel()
            self._start_timer = None
        self._publish_current()

    def _publish_current(self):
        if self.current_idx >= len(self.waypoints):
            self.get_logger().info('Todos los goals completados. Shutting down.')
            # shutdown controlado
            self.create_timer(1.0, self._self_shutdown)
            return
        x, y = self.waypoints[self.current_idx]
        msg = Goal()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0   # orientación no usada por el controller
        msg.id = int(self.current_idx)
        msg.is_last = bool(self.current_idx == len(self.waypoints) - 1)
        # publicar varias veces para robustez
        for _ in range(3):
            self.pub.publish(msg)
        self.get_logger().info(
            f'→ enviado goal {self.current_idx}: ({x:.2f}, {y:.2f}) '
            f'{"[LAST]" if msg.is_last else ""}'
        )

    def _cb_reached(self, msg: UInt32):
        if int(msg.data) != self.current_idx:
            return
        self.get_logger().info(f'✓ goal {self.current_idx} confirmado por controller')
        self.current_idx += 1
        # programar la publicación del siguiente con pausa
        if self._next_timer is not None:
            self._next_timer.cancel()
        self._next_timer = self.create_timer(self.pause, self._delayed_publish)

    def _delayed_publish(self):
        if self._next_timer is not None:
            self._next_timer.cancel()
            self._next_timer = None
        self._publish_current()

    def _self_shutdown(self):
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PathGenerator()
    except RuntimeError as e:
        print(f'[path_generator] fallo de inicialización: {e}')
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
