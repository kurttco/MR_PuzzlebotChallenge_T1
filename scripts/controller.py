#!/usr/bin/env python3

import csv
import math
import os
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import UInt32
from puzzlebot_mc2.msg import Goal, SemaphoreState


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat(qx, qy, qz, qw) -> float:
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz)
    )


def clip(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


SEM_UNKNOWN = 0
SEM_GREEN = 1
SEM_YELLOW = 2
SEM_RED = 3
SEM_NAMES = {
    0: 'UNKNOWN',
    1: 'GREEN',
    2: 'YELLOW',
    3: 'RED'
}


class PID:
    def __init__(self, kp, ki, kd, integral_max=1e6, deriv_alpha=0.2):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_max = integral_max
        self.deriv_alpha = deriv_alpha
        self.integral = 0.0
        self.prev_error = None
        self.filtered_deriv = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = None
        self.filtered_deriv = 0.0

    def update(self, error, dt):
        if dt <= 0.0:
            return 0.0

        self.integral += error * dt
        self.integral = clip(self.integral, -self.integral_max, self.integral_max)

        if self.prev_error is None:
            raw_deriv = 0.0
        else:
            raw_deriv = (error - self.prev_error) / dt

        self.filtered_deriv = (
            self.deriv_alpha * raw_deriv
            + (1.0 - self.deriv_alpha) * self.filtered_deriv
        )

        self.prev_error = error

        return (
            self.kp * error
            + self.ki * self.integral
            + self.kd * self.filtered_deriv
        )


class State(Enum):
    IDLE = 0
    ROTATE_TO_GOAL = 1
    MOVE_TO_GOAL = 2
    HOLD = 3


class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        # ================= PARAMETROS =================
        self.declare_parameter('control_mode', 'sequential')

        self.declare_parameter('kp_ang', 0.6)
        self.declare_parameter('ki_ang', 0.0)
        self.declare_parameter('kd_ang', 0.0)

        self.declare_parameter('kp_lin', 0.4)
        self.declare_parameter('ki_lin', 0.0)
        self.declare_parameter('kd_lin', 0.0)

        self.declare_parameter('sim_kp_lin', 0.3)
        self.declare_parameter('sim_ki_lin', 0.0)
        self.declare_parameter('sim_kp_ang', 0.6)
        self.declare_parameter('sim_ki_ang', 0.0)

        self.declare_parameter('v_max', 0.20)
        self.declare_parameter('omega_max', 0.35)

        self.declare_parameter('epsilon_pos', 0.05)
        self.declare_parameter('epsilon_ang', 0.05)
        self.declare_parameter('hold_time', 0.0)

        self.declare_parameter('integral_max', 0.5)
        self.declare_parameter('derivative_filter_alpha', 0.2)

        self.declare_parameter('v_slew_rate', 0.18)
        self.declare_parameter('w_slew_rate', 0.8)

        self.declare_parameter('approach_zone', 0.25)
        self.declare_parameter('approach_zone_min_scale', 0.20)

        # NUEVO: si durante avance se desorienta mucho, vuelve a girar
        self.declare_parameter('reorient_threshold', 0.25)

        # NUEVO: ganancia angular usada mientras avanza
        self.declare_parameter('move_heading_gain', 0.30)

        self.declare_parameter('log_csv', True)
        self.declare_parameter('log_dir', '/tmp/puzzlebot_logs')
        self.declare_parameter('control_rate_hz', 20.0)

        self.declare_parameter('enable_semaphore', False)
        self.declare_parameter('semaphore_topic', '/semaphore_state')
        self.declare_parameter('yellow_speed_scale', 0.4)
        self.declare_parameter('green_frames_to_release', 5)
        self.declare_parameter('semaphore_timeout_s', 1.0)

        # ================= LECTURA PARAMETROS =================
        self.mode = self.get_parameter('control_mode').value

        self.v_max = float(self.get_parameter('v_max').value)
        self.omega_max = float(self.get_parameter('omega_max').value)

        self.eps_pos = float(self.get_parameter('epsilon_pos').value)
        self.eps_ang = float(self.get_parameter('epsilon_ang').value)
        self.hold_time = float(self.get_parameter('hold_time').value)

        integral_max = float(self.get_parameter('integral_max').value)
        deriv_alpha = float(self.get_parameter('derivative_filter_alpha').value)

        self.v_slew = float(self.get_parameter('v_slew_rate').value)
        self.w_slew = float(self.get_parameter('w_slew_rate').value)

        self.approach_zone = float(self.get_parameter('approach_zone').value)
        self.approach_min_scale = float(
            self.get_parameter('approach_zone_min_scale').value
        )

        self.reorient_threshold = float(
            self.get_parameter('reorient_threshold').value
        )
        self.move_heading_gain = float(
            self.get_parameter('move_heading_gain').value
        )

        self.enable_sem = bool(self.get_parameter('enable_semaphore').value)
        self.yellow_scale = float(self.get_parameter('yellow_speed_scale').value)
        self.green_to_release = int(
            self.get_parameter('green_frames_to_release').value
        )
        self.sem_timeout = float(self.get_parameter('semaphore_timeout_s').value)

        # ================= PID =================
        self.pid_ang = PID(
            float(self.get_parameter('kp_ang').value),
            float(self.get_parameter('ki_ang').value),
            float(self.get_parameter('kd_ang').value),
            integral_max,
            deriv_alpha
        )

        self.pid_lin = PID(
            float(self.get_parameter('kp_lin').value),
            float(self.get_parameter('ki_lin').value),
            float(self.get_parameter('kd_lin').value),
            integral_max,
            deriv_alpha
        )

        self.pid_sim_lin = PID(
            float(self.get_parameter('sim_kp_lin').value),
            float(self.get_parameter('sim_ki_lin').value),
            0.0,
            integral_max,
            deriv_alpha
        )

        self.pid_sim_ang = PID(
            float(self.get_parameter('sim_kp_ang').value),
            float(self.get_parameter('sim_ki_ang').value),
            0.0,
            integral_max,
            deriv_alpha
        )

        # ================= ESTADO =================
        self.state = State.IDLE

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.have_pose = False

        self.goal_x = None
        self.goal_y = None
        self.goal_id = None

        self.in_tolerance_since = None
        self.last_time = None

        self.last_v_pub = 0.0
        self.last_w_pub = 0.0

        # Semaforo
        self.sem_state = SEM_UNKNOWN
        self.last_sem_time = None
        self.red_latched = False
        self.green_streak = 0

        self.last_v_pre_sem = 0.0
        self.last_w_pre_sem = 0.0

        # Logging
        self.log_csv = bool(self.get_parameter('log_csv').value)
        self.log_dir = self.get_parameter('log_dir').value
        self.log_rows = []

        if self.log_csv:
            os.makedirs(self.log_dir, exist_ok=True)

        self.start_wall = time.time()
        sem_tag = '_sem' if self.enable_sem else ''
        self.run_tag = f"{self.mode}{sem_tag}_{int(self.start_wall)}"

        # ================= ROS PUB/SUB =================
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_reached = self.create_publisher(UInt32, '/goal_reached', 10)

        self.create_subscription(PoseStamped, '/robot_pose', self._cb_pose, 10)
        self.create_subscription(Goal, '/goals', self._cb_goal, 10)

        if self.enable_sem:
            sem_topic = self.get_parameter('semaphore_topic').value
            self.create_subscription(
                SemaphoreState,
                sem_topic,
                self._cb_semaphore,
                10
            )

        rate = float(self.get_parameter('control_rate_hz').value)
        self.create_timer(1.0 / rate, self._step)

        sem_str = 'ON' if self.enable_sem else 'OFF'
        self.get_logger().info(
            f'Controller up | mode={self.mode} | sem={sem_str} | '
            f'v_max={self.v_max:.2f} | omega_max={self.omega_max:.2f} | '
            f'reorient_threshold={self.reorient_threshold:.2f}'
        )

    # ================= CALLBACKS =================

    def _cb_pose(self, msg: PoseStamped):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.theta = wrap_pi(yaw_from_quat(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ))
        self.have_pose = True

    def _cb_goal(self, msg: Goal):
        self.goal_x = float(msg.pose.position.x)
        self.goal_y = float(msg.pose.position.y)
        self.goal_id = int(msg.id)

        self.pid_ang.reset()
        self.pid_lin.reset()
        self.pid_sim_lin.reset()
        self.pid_sim_ang.reset()

        self.in_tolerance_since = None

        if self.mode == 'sequential':
            self.state = State.ROTATE_TO_GOAL
        else:
            self.state = State.MOVE_TO_GOAL

        self.get_logger().info(
            f'[goal {self.goal_id}] nuevo goal '
            f'({self.goal_x:.2f}, {self.goal_y:.2f}) desde '
            f'({self.x:.2f}, {self.y:.2f}, {math.degrees(self.theta):.1f} deg)'
        )

    def _cb_semaphore(self, msg: SemaphoreState):
        self.sem_state = int(msg.state)
        self.last_sem_time = self.get_clock().now()

        if self.sem_state == SEM_RED:
            if not self.red_latched:
                self.get_logger().info('SEMAPHORE: RED detected -> latch ON')
            self.red_latched = True
            self.green_streak = 0

        elif self.sem_state == SEM_GREEN:
            self.green_streak += 1
            if self.red_latched and self.green_streak >= self.green_to_release:
                self.red_latched = False
                self.get_logger().info(
                    f'SEMAPHORE: GREEN x{self.green_streak} -> latch OFF'
                )

        else:
            self.green_streak = 0

    # ================= SEMAFORO =================

    def _effective_sem_state(self) -> int:
        if self.last_sem_time is None:
            return SEM_RED if self.red_latched else SEM_UNKNOWN

        elapsed = (
            self.get_clock().now() - self.last_sem_time
        ).nanoseconds * 1e-9

        if elapsed > self.sem_timeout:
            return SEM_RED if self.red_latched else SEM_UNKNOWN

        if self.red_latched:
            return SEM_RED

        return self.sem_state

    def _apply_semaphore_override(self, v_cmd, w_cmd):
        if not self.enable_sem:
            return v_cmd, w_cmd

        eff = self._effective_sem_state()

        if eff == SEM_RED:
            return 0.0, 0.0

        if eff == SEM_YELLOW:
            return v_cmd * self.yellow_scale, w_cmd

        return v_cmd, w_cmd

    # ================= PUBLICACION =================

    def _apply_approach_slowdown(self, v_cmd, dist):
        if dist >= self.approach_zone:
            return v_cmd

        scale = max(self.approach_min_scale, dist / self.approach_zone)
        cap = self.v_max * scale
        sign = 1.0 if v_cmd >= 0.0 else -1.0

        return sign * min(abs(v_cmd), cap)

    def _publish_cmd(self, v_desired, w_desired, dt):
        v_sat = clip(v_desired, -self.v_max, self.v_max)
        w_sat = clip(w_desired, -self.omega_max, self.omega_max)

        max_dv = self.v_slew * dt
        max_dw = self.w_slew * dt

        v = clip(v_sat, self.last_v_pub - max_dv, self.last_v_pub + max_dv)
        w = clip(w_sat, self.last_w_pub - max_dw, self.last_w_pub + max_dw)

        self.last_v_pub = v
        self.last_w_pub = w

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)

        self.pub_cmd.publish(cmd)

        return v, w

    def _publish_stop(self, dt):
        return self._publish_cmd(0.0, 0.0, dt)

    def _declare_reached(self):
        if self.goal_id is None:
            return

        msg = UInt32()
        msg.data = int(self.goal_id)

        for _ in range(3):
            self.pub_reached.publish(msg)

        err = math.hypot(self.x - self.goal_x, self.y - self.goal_y)

        self.get_logger().info(
            f'[goal {self.goal_id}] ALCANZADO | error final = {err:.3f} m'
        )

        self.state = State.IDLE

        if self.log_csv:
            self._flush_log()

    # ================= LOGGING =================

    def _flush_log(self):
        if not self.log_rows:
            return

        path = os.path.join(self.log_dir, f'run_{self.run_tag}.csv')
        write_header = not os.path.exists(path)

        with open(path, 'a', newline='') as f:
            writer = csv.writer(f)

            if write_header:
                writer.writerow([
                    't',
                    'x',
                    'y',
                    'theta',
                    'goal_x',
                    'goal_y',
                    'error_pos',
                    'error_ang',
                    'v_cmd_pre_sem',
                    'w_cmd_pre_sem',
                    'v_cmd',
                    'w_cmd',
                    'state',
                    'mode',
                    'goal_id',
                    'sem_state',
                    'red_latched'
                ])

            writer.writerows(self.log_rows)

        self.log_rows.clear()

    def _maybe_log(self, v_pre, w_pre, v_out, w_out, dist=0.0, ang_error=0.0):
        if not self.log_csv:
            return

        t_rel = time.time() - self.start_wall

        row = [
            f'{t_rel:.3f}',
            f'{self.x:.4f}',
            f'{self.y:.4f}',
            f'{self.theta:.4f}',
            f'{self.goal_x:.4f}' if self.goal_x is not None else '0.0000',
            f'{self.goal_y:.4f}' if self.goal_y is not None else '0.0000',
            f'{dist:.4f}',
            f'{ang_error:.4f}',
            f'{self.last_v_pre_sem:.4f}',
            f'{self.last_w_pre_sem:.4f}',
            f'{v_out:.4f}',
            f'{w_out:.4f}',
            self.state.name,
            self.mode,
            self.goal_id if self.goal_id is not None else -1,
            SEM_NAMES[self._effective_sem_state()],
            int(self.red_latched),
        ]

        self.log_rows.append(row)

        if len(self.log_rows) > 200:
            self._flush_log()

    # ================= LOOP PRINCIPAL =================

    def _step(self):
        now = self.get_clock().now()

        if self.last_time is None:
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0 or dt > 0.5:
            return

        if not self.have_pose:
            return

        if self.state == State.IDLE:
            v_out, w_out = self._publish_stop(dt)
            self._maybe_log(0.0, 0.0, v_out, w_out)
            return

        if self.goal_x is None or self.goal_y is None:
            return

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y

        dist = math.hypot(dx, dy)
        heading_goal = math.atan2(dy, dx)
        ang_error = wrap_pi(heading_goal - self.theta)

        v_cmd = 0.0
        w_cmd = 0.0

        # ================= MODO SECUENCIAL =================
        if self.mode == 'sequential':

            if self.state == State.ROTATE_TO_GOAL:
                v_cmd = 0.0
                w_cmd = self.pid_ang.update(ang_error, dt)

                if abs(ang_error) < self.eps_ang:
                    self.get_logger().info(
                        f'[goal {self.goal_id}] orientado -> MOVE_TO_GOAL'
                    )
                    self.state = State.MOVE_TO_GOAL
                    self.pid_ang.reset()
                    self.pid_lin.reset()
                    v_cmd = 0.0
                    w_cmd = 0.0

            elif self.state == State.MOVE_TO_GOAL:

                # CORRECCION IMPORTANTE:
                # Si el robot se desorienta mucho durante el avance,
                # no sigue avanzando. Regresa a rotar primero.
                if abs(ang_error) > self.reorient_threshold:
                    self.get_logger().warn(
                        f'[goal {self.goal_id}] reorientando: '
                        f'ang_error={math.degrees(ang_error):.1f} deg'
                    )
                    self.state = State.ROTATE_TO_GOAL
                    self.pid_ang.reset()
                    self.pid_lin.reset()
                    v_out, w_out = self._publish_stop(dt)
                    self._maybe_log(0.0, 0.0, v_out, w_out, dist, ang_error)
                    return

                v_cmd = self.pid_lin.update(dist, dt)
                v_cmd = self._apply_approach_slowdown(v_cmd, dist)

                # Correccion angular suave mientras avanza
                w_cmd = self.move_heading_gain * self.pid_ang.update(
                    ang_error,
                    dt
                )

                # Si ya casi llegó, entra a HOLD
                if dist < self.eps_pos:
                    self.state = State.HOLD
                    self.in_tolerance_since = now
                    self.pid_ang.reset()
                    self.pid_lin.reset()
                    v_cmd = 0.0
                    w_cmd = 0.0

            elif self.state == State.HOLD:
                v_cmd = 0.0
                w_cmd = 0.0

                if dist > self.eps_pos:
                    self.state = State.MOVE_TO_GOAL
                    self.in_tolerance_since = None

                elif self.in_tolerance_since is not None:
                    elapsed_hold = (
                        now - self.in_tolerance_since
                    ).nanoseconds * 1e-9

                    if elapsed_hold >= self.hold_time:
                        self._publish_stop(dt)
                        self._declare_reached()
                        return

        # ================= MODO SIMULTANEO =================
        else:

            if self.state == State.MOVE_TO_GOAL:
                scale = max(0.0, math.cos(ang_error))

                v_cmd = scale * self.pid_sim_lin.update(dist, dt)
                v_cmd = self._apply_approach_slowdown(v_cmd, dist)

                w_cmd = self.pid_sim_ang.update(ang_error, dt)

                if dist < self.eps_pos:
                    self.state = State.HOLD
                    self.in_tolerance_since = now
                    v_cmd = 0.0
                    w_cmd = 0.0

            elif self.state == State.HOLD:
                v_cmd = 0.0
                w_cmd = 0.0

                if dist > self.eps_pos:
                    self.state = State.MOVE_TO_GOAL
                    self.in_tolerance_since = None

                elif self.in_tolerance_since is not None:
                    elapsed_hold = (
                        now - self.in_tolerance_since
                    ).nanoseconds * 1e-9

                    if elapsed_hold >= self.hold_time:
                        self._publish_stop(dt)
                        self._declare_reached()
                        return

        # Guardar output antes del semaforo
        self.last_v_pre_sem = v_cmd
        self.last_w_pre_sem = w_cmd

        # Semaforo
        v_cmd, w_cmd = self._apply_semaphore_override(v_cmd, w_cmd)

        # Publicar
        v_out, w_out = self._publish_cmd(v_cmd, w_cmd, dt)

        # Log
        self._maybe_log(v_cmd, w_cmd, v_out, w_out, dist, ang_error)


def main(args=None):
    rclpy.init(args=args)
    node = Controller()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        stop = Twist()

        try:
            for _ in range(10):
                node.pub_cmd.publish(stop)
        except Exception:
            pass

        if node.log_csv:
            node._flush_log()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
