#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist


class XboxCmdVel(Node):
    def __init__(self):
        super().__init__('xbox_cmdvel')

        # ===== Topics =====
        self.declare_parameter('analog_topic', '/xbox/analog')
        self.declare_parameter('event_topic', '/xbox/event')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # ===== Publish/watchdog =====
        self.declare_parameter('cmd_vel_publish_rate_hz', 20.0)
        self.declare_parameter('joy_timeout_sec', 0.5)

        # ===== Control mapping from analog array =====
        self.declare_parameter('IDX_LX', 0)
        self.declare_parameter('IDX_LY', 1)
        self.declare_parameter('IDX_RX', 2)
        self.declare_parameter('IDX_RY', 3)
        self.declare_parameter('IDX_LT', 4)
        self.declare_parameter('IDX_RT', 5)

        self.declare_parameter('drive_v_axis', 'RY')   # RY or LY
        self.declare_parameter('drive_w_axis', 'LX')   # LX or RX

        # Gate (your trigger: 1 -> -1, pressed when value < threshold)
        self.declare_parameter('rt_threshold', 0.5)
        self.declare_parameter('require_rt_hold', True)

        # Scaling / invert
        self.declare_parameter('scale_linear', 0.5)
        self.declare_parameter('scale_angular', 1.0)
        self.declare_parameter('invert_linear', False)
        self.declare_parameter('invert_angular', False)

        self.declare_parameter('axis_active_threshold', 0.02)

        # Trim
        self.declare_parameter('trim_step', 0.01)
        self.declare_parameter('trim_linear_min', -1.0)
        self.declare_parameter('trim_linear_max', 1.0)
        self.declare_parameter('trim_angular_min', -2.0)
        self.declare_parameter('trim_angular_max', 2.0)

        gp = self.get_parameter
        self.analog_topic = str(gp('analog_topic').value)
        self.event_topic = str(gp('event_topic').value)
        self.cmd_vel_topic = str(gp('cmd_vel_topic').value)

        self.pub_hz = float(gp('cmd_vel_publish_rate_hz').value)
        self.timeout_sec = float(gp('joy_timeout_sec').value)

        self.IDX = {
            'LX': int(gp('IDX_LX').value),
            'LY': int(gp('IDX_LY').value),
            'RX': int(gp('IDX_RX').value),
            'RY': int(gp('IDX_RY').value),
            'LT': int(gp('IDX_LT').value),
            'RT': int(gp('IDX_RT').value),
        }
        self.drive_v_axis = str(gp('drive_v_axis').value).strip().upper()
        self.drive_w_axis = str(gp('drive_w_axis').value).strip().upper()

        self.rt_threshold = float(gp('rt_threshold').value)
        self.require_rt_hold = bool(gp('require_rt_hold').value)

        self.scale_linear = float(gp('scale_linear').value)
        self.scale_angular = float(gp('scale_angular').value)
        self.invert_linear = bool(gp('invert_linear').value)
        self.invert_angular = bool(gp('invert_angular').value)
        self.axis_active_th = float(gp('axis_active_threshold').value)

        self.trim_step = float(gp('trim_step').value)
        self.trim_linear_min = float(gp('trim_linear_min').value)
        self.trim_linear_max = float(gp('trim_linear_max').value)
        self.trim_angular_min = float(gp('trim_angular_min').value)
        self.trim_angular_max = float(gp('trim_angular_max').value)

        self.get_logger().info("===== xbox_cmdvel parameters loaded =====")
        self.get_logger().info(f"analog_topic         = {self.analog_topic}")
        self.get_logger().info(f"event_topic          = {self.event_topic}")
        self.get_logger().info(f"cmd_vel_topic        = {self.cmd_vel_topic}")
        self.get_logger().info(f"publish_rate         = {self.pub_hz} Hz")
        self.get_logger().info(f"joy_timeout_sec      = {self.timeout_sec}")

        self.get_logger().info(
            f"analog IDX: LX={self.IDX['LX']} LY={self.IDX['LY']} "
            f"RX={self.IDX['RX']} RY={self.IDX['RY']} "
            f"LT={self.IDX['LT']} RT={self.IDX['RT']}"
        )

        self.get_logger().info(
            f"drive mapping: v_axis={self.drive_v_axis} w_axis={self.drive_w_axis}"
        )

        self.get_logger().info(
            f"RT gate: threshold={self.rt_threshold} require_rt_hold={self.require_rt_hold}"
        )

        self.get_logger().info(
            f"scale: linear={self.scale_linear} angular={self.scale_angular} "
            f"invert_linear={self.invert_linear} invert_angular={self.invert_angular}"
        )

        self.get_logger().info(
            f"axis_active_threshold={self.axis_active_th}"
        )

        self.get_logger().info(
            f"trim: step={self.trim_step} "
            f"lin[{self.trim_linear_min},{self.trim_linear_max}] "
            f"ang[{self.trim_angular_min},{self.trim_angular_max}]"
        )
        self.get_logger().info("========================================")


        # Pub/Sub
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(Float32MultiArray, self.analog_topic, self.cb_analog, 10)
        self.create_subscription(String, self.event_topic, self.cb_event, 10)

        # Timer publish fixed rate
        self.timer = self.create_timer(1.0 / max(1e-3, self.pub_hz), self._on_timer)

        # State
        self._last_analog = [0.0] * 8
        self._last_analog_time = 0.0

        self._lb_down = False
        self.linear_trim = 0.0
        self.angular_trim = 0.0

        self._last_cmd = Twist()

        self.get_logger().info(
            f'🚗 xbox_cmdvel started: analog={self.analog_topic} event={self.event_topic} -> {self.cmd_vel_topic} '
            f'@{self.pub_hz:.1f}Hz timeout={self.timeout_sec:.2f}s'
        )

    def _clamp(self, v, vmin, vmax):
        return max(vmin, min(vmax, v))

    def cb_analog(self, msg: Float32MultiArray):
        data = list(msg.data)
        if len(data) < 8:
            data = data + [0.0] * (8 - len(data))

        self._last_analog = [float(x) for x in data[:8]]
        self._last_analog_time = time.time()
        self._last_cmd = self._compute_cmd()

    def cb_event(self, msg: String):
        ev = (msg.data or "").strip().upper()

        if ev == "LB":
            self._lb_down = True
            return
        if ev == "LB_RELEASE":
            self._lb_down = False
            return

        if self._lb_down and ev == "Y":
            self.linear_trim = self._clamp(self.linear_trim + self.trim_step,
                                        self.trim_linear_min, self.trim_linear_max)
            self.get_logger().info(f"[trim] LB+Y step=+{self.trim_step:.3f} -> {self._status_line()}")

        elif self._lb_down and ev == "A":
            self.linear_trim = self._clamp(self.linear_trim - self.trim_step,
                                        self.trim_linear_min, self.trim_linear_max)
            self.get_logger().info(f"[trim] LB+A step=-{self.trim_step:.3f} -> {self._status_line()}")

        elif self._lb_down and ev == "B":
            self.angular_trim = self._clamp(self.angular_trim + self.trim_step,
                                            self.trim_angular_min, self.trim_angular_max)
            self.get_logger().info(f"[trim] LB+B step=+{self.trim_step:.3f} -> {self._status_line()}")

        elif self._lb_down and ev == "X":
            self.angular_trim = self._clamp(self.angular_trim - self.trim_step,
                                            self.trim_angular_min, self.trim_angular_max)
            self.get_logger().info(f"[trim] LB+X step=-{self.trim_step:.3f} -> {self._status_line()}")

        self._last_cmd = self._compute_cmd()

    def _compute_cmd(self) -> Twist:
        a = self._last_analog

        v_axis = self.drive_v_axis if self.drive_v_axis in self.IDX else 'RY'
        w_axis = self.drive_w_axis if self.drive_w_axis in self.IDX else 'LX'

        v = float(a[self.IDX[v_axis]])
        w = float(a[self.IDX[w_axis]])

        if abs(v) < self.axis_active_th:
            v = 0.0
        if abs(w) < self.axis_active_th:
            w = 0.0

        rt_val = float(a[self.IDX['RT']])
        rt_pressed = (rt_val < self.rt_threshold)  # scale 1..-1

        if self.require_rt_hold and (not rt_pressed):
            v = 0.0
            w = 0.0

        lin_gain = max(0.0, self.scale_linear + self.linear_trim)
        ang_gain = max(0.0, self.scale_angular + self.angular_trim)

        v = lin_gain * v
        w = ang_gain * w

        if self.invert_linear:
            v = -v
        if self.invert_angular:
            w = -w

        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        return t

    def _fmt(self, x: float) -> str:
        return f"{x:+.3f}"

    def _status_line(self) -> str:

        lin_gain = max(0.0, self.scale_linear + self.linear_trim)
        ang_gain = max(0.0, self.scale_angular + self.angular_trim)

        return (
            f"trim lin={self._fmt(self.linear_trim)} ang={self._fmt(self.angular_trim)} | "
            f"gain lin={self._fmt(lin_gain)} ang={self._fmt(ang_gain)} "
        )

    def _on_timer(self):
        now = time.time()
        stale = (self._last_analog_time <= 0.0) or ((now - self._last_analog_time) > self.timeout_sec)
        if stale:
            # reset stored cmd too
            self._last_cmd = Twist()
            self.cmd_pub.publish(self._last_cmd)
            return

        self.cmd_pub.publish(self._last_cmd)


def main():
    rclpy.init()
    rclpy.spin(XboxCmdVel())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
