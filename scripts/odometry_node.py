#!/usr/bin/env python3
"""
odometry_node.py
----------------
Integra las lecturas de encoders (/VelocityEncL, /VelocityEncR) a una pose 2D
del robot y la publica en /robot_pose (geometry_msgs/PoseStamped).

Supuestos:
- Los encoders publican std_msgs/Float32 con velocidad angular de rueda en rad/s.
  (Verificar con: ros2 topic echo /VelocityEncL — ver nota al pie si no cuadra.)

Cinemática:
    v      = r * (w_R + w_L) / 2
    omega  = r * (w_R - w_L) / L
Integración:
    x_{k+1}     = x_k + v * cos(theta_k) * dt
    y_{k+1}     = y_k + v * sin(theta_k) * dt
    theta_{k+1} = theta_k + omega * dt
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        self.declare_parameter('wheel_radius', 0.045)
        self.declare_parameter('wheel_base', 0.18)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('left_encoder_topic', '/VelocityEncL')
        self.declare_parameter('right_encoder_topic', '/VelocityEncR')
        self.declare_parameter('pose_topic', '/robot_pose')
        # NUEVO: factor de escala aplicado a las lecturas de encoder.
        # Si los encoders reportan en rad/s -> dejar en 1.0 (default).
        # Si tras calibración se detecta que la odometría subestima la
        # distancia real en un 14%, poner encoder_scale = 1.14.
        # Si reportan en RPM: encoder_scale = 2*pi/60 ≈ 0.1047.
        self.declare_parameter('encoder_scale', 1.0)
        # NUEVO: cada N segundos loguea las últimas lecturas de encoder
        # (útil para verificar unidades al calibrar). 0 = desactivado.
        self.declare_parameter('diag_log_period_s', 2.0)

        self.r = float(self.get_parameter('wheel_radius').value)
        self.L = float(self.get_parameter('wheel_base').value)
        self.encoder_scale = float(self.get_parameter('encoder_scale').value)
        self.diag_period = float(self.get_parameter('diag_log_period_s').value)
        rate = float(self.get_parameter('publish_rate_hz').value)
        left_topic = self.get_parameter('left_encoder_topic').value
        right_topic = self.get_parameter('right_encoder_topic').value
        pose_topic = self.get_parameter('pose_topic').value

        # estado
        self.wl = 0.0
        self.wr = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        self.last_diag_wall = time.time()

        # subs/pubs
        # Los encoders vienen de micro-ROS (firmware ESP32), que publica con
        # BEST_EFFORT. Si nos suscribimos con el QoS default (RELIABLE) el
        # subscriber rechaza los mensajes y la odometría se queda congelada.
        # qos_profile_sensor_data es BEST_EFFORT + KEEP_LAST(5), correcto
        # para streams de sensores.
        self.create_subscription(Float32, left_topic, self._cb_wl,
                                 qos_profile_sensor_data)
        self.create_subscription(Float32, right_topic, self._cb_wr,
                                 qos_profile_sensor_data)
        self.pub = self.create_publisher(PoseStamped, pose_topic, 10)

        self.create_timer(1.0 / rate, self._update)

        self.get_logger().info(
            f'Odometry up | r={self.r} m, L={self.L} m | '
            f'subs: {left_topic}, {right_topic} | pub: {pose_topic}'
        )

    def _cb_wl(self, msg: Float32):
        self.wl = float(msg.data) * self.encoder_scale

    def _cb_wr(self, msg: Float32):
        self.wr = float(msg.data) * self.encoder_scale

    def _update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0 or dt > 0.5:
            return

        v = self.r * (self.wr + self.wl) / 2.0
        omega = self.r * (self.wr - self.wl) / self.L

        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += omega * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Log de diagnóstico periódico (útil para verificar encoders)
        if self.diag_period > 0.0:
            wall = time.time()
            if wall - self.last_diag_wall >= self.diag_period:
                self.last_diag_wall = wall
                self.get_logger().info(
                    f'[diag] wl={self.wl:+.3f} wr={self.wr:+.3f} rad/s '
                    f'(post-scale) | v={v:+.3f} m/s w={omega:+.3f} rad/s | '
                    f'pose=({self.x:+.3f},{self.y:+.3f},'
                    f'{math.degrees(self.theta):+.1f}°)'
                )

        msg = PoseStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = 0.0
        # quaternion de yaw puro
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(self.theta / 2.0)
        msg.pose.orientation.w = math.cos(self.theta / 2.0)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

# NOTA SOBRE UNIDADES DE ENCODER
# ------------------------------
# Si al correr el robot la odometría avanza mucho más rápido/lento que lo real,
# probablemente los encoders NO reportan rad/s sino otra escala. Opciones:
#   (a) Escalar con un factor (parámetro `encoder_scale` — añadir si hace falta).
#   (b) Revisar el tipo exacto y unidades con `ros2 topic echo /VelocityEncL`
#       cuando el robot esté moviéndose a velocidad conocida.
# k_lin de MC2 dice que cmd=1 -> v=0.074 m/s -> w_rueda = 0.074/0.045 ≈ 1.644 rad/s.
# Rodando en línea recta con cmd=1 en ambas ruedas deberías ver ~1.644 en el encoder.
