#!/usr/bin/env python3
"""
cmd_vel_bridge.py
-----------------
Bridge entre /cmd_vel (geometry_msgs/Twist) y los setpoints por rueda del
firmware del PuzzleBot (/VelocitySetL, /VelocitySetR como std_msgs/Float32).

Esto existe porque el firmware actual de la Hackerboard no responde
directamente a /cmd_vel; en MC1/MC2 se comprobó que sólo mueve a través de
los tópicos por rueda.

Modelo inverso (empírico, de MC2):
    cmd_straight = v / k_lin
    cmd_turn     = w / k_ang
    cmd_L = cmd_straight - cmd_turn
    cmd_R = cmd_straight + cmd_turn

Este modelo captura *directamente* la respuesta real del motor (incluye
deadzones y no-linealidades de orden cero), y por encima el PID cierra el
lazo en pose.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')

        self.declare_parameter('wheel_radius', 0.045)   # no se usa aquí, pero queda para completitud
        self.declare_parameter('wheel_base',   0.18)
        self.declare_parameter('k_lin',        0.074)
        self.declare_parameter('k_ang',        0.561)
        self.declare_parameter('max_cmd',      3.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('left_cmd_topic', '/VelocitySetL')
        self.declare_parameter('right_cmd_topic', '/VelocitySetR')

        self.k_lin = float(self.get_parameter('k_lin').value)
        self.k_ang = float(self.get_parameter('k_ang').value)
        self.max_cmd = float(self.get_parameter('max_cmd').value)

        sub_topic = self.get_parameter('cmd_vel_topic').value
        pub_l = self.get_parameter('left_cmd_topic').value
        pub_r = self.get_parameter('right_cmd_topic').value

        self.create_subscription(Twist, sub_topic, self._cb, 10)
        self.pub_l = self.create_publisher(Float32, pub_l, 10)
        self.pub_r = self.create_publisher(Float32, pub_r, 10)

        self.get_logger().info(
            f'Bridge up | k_lin={self.k_lin}, k_ang={self.k_ang} | '
            f'{sub_topic} -> ({pub_l}, {pub_r})'
        )

    def _cb(self, msg: Twist):
        v = float(msg.linear.x)
        w = float(msg.angular.z)

        cmd_straight = v / self.k_lin if self.k_lin != 0.0 else 0.0
        cmd_turn = w / self.k_ang if self.k_ang != 0.0 else 0.0

        cmd_l = cmd_straight - cmd_turn
        cmd_r = cmd_straight + cmd_turn

        cmd_l = max(-self.max_cmd, min(self.max_cmd, cmd_l))
        cmd_r = max(-self.max_cmd, min(self.max_cmd, cmd_r))

        ml = Float32(); ml.data = float(cmd_l)
        mr = Float32(); mr.data = float(cmd_r)
        self.pub_l.publish(ml)
        self.pub_r.publish(mr)

    def publish_stop(self):
        zero = Float32(); zero.data = 0.0
        for _ in range(10):
            self.pub_l.publish(zero)
            self.pub_r.publish(zero)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
