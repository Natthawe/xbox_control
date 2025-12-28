#!/usr/bin/env python3
import time
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String


def deadzone(v: float, dz: float) -> float:
    if abs(v) < dz:
        return 0.0
    return math.copysign((abs(v) - dz) / (1.0 - dz), v)


class XboxButtonsDebug(Node):
    def __init__(self):
        super().__init__('xbox_buttons_debug')

        # ===== Parameters =====
        self.declare_parameter('debug_topic', '/xbox_control/debug/buttons')
        self.declare_parameter('throttle_sec', 0.01)

        # Axes indices
        self.declare_parameter('AX_LX', 0)
        self.declare_parameter('AX_LY', 1)
        self.declare_parameter('AX_RX', 2)
        self.declare_parameter('AX_RY', 3)
        self.declare_parameter('AX_RT', 4)
        self.declare_parameter('AX_LT', 5)
        self.declare_parameter('AX_DPAD_X', 6)
        self.declare_parameter('AX_DPAD_Y', 7)

        # Trigger thresholds
        self.declare_parameter('lt_threshold', 0.5)
        self.declare_parameter('rt_threshold', 0.5)

        # Stick direction
        self.declare_parameter('stick_deadzone', 0.08)
        self.declare_parameter('stick_dir_threshold', 0.35)

        # Invert options
        self.declare_parameter('invert_lx', False)
        self.declare_parameter('invert_ly', False)
        self.declare_parameter('invert_rx', False)
        self.declare_parameter('invert_ry', False)
        self.declare_parameter('invert_dpad_x', False)
        self.declare_parameter('invert_dpad_y', False)

        # Button indices
        self.declare_parameter('BTN_A', 0)
        self.declare_parameter('BTN_B', 1)
        self.declare_parameter('BTN_X', 3)
        self.declare_parameter('BTN_Y', 4)
        self.declare_parameter('BTN_LB', 6)
        self.declare_parameter('BTN_RB', 7)
        self.declare_parameter('BTN_VIEW', 10)
        self.declare_parameter('BTN_MENU', 11)
        self.declare_parameter('BTN_LS', 13)
        self.declare_parameter('BTN_RS', 14)
        self.declare_parameter('BTN_SHARE', 15)

        gp = self.get_parameter
        self.topic = gp('debug_topic').value
        self.throttle_sec = gp('throttle_sec').value

        self.AX_LX = gp('AX_LX').value
        self.AX_LY = gp('AX_LY').value
        self.AX_RX = gp('AX_RX').value
        self.AX_RY = gp('AX_RY').value
        self.AX_RT = gp('AX_RT').value
        self.AX_LT = gp('AX_LT').value
        self.AX_DPAD_X = gp('AX_DPAD_X').value
        self.AX_DPAD_Y = gp('AX_DPAD_Y').value

        self.lt_threshold = gp('lt_threshold').value
        self.rt_threshold = gp('rt_threshold').value

        self.stick_dz = gp('stick_deadzone').value
        self.stick_dir_th = gp('stick_dir_threshold').value

        self.invert_lx = gp('invert_lx').value
        self.invert_ly = gp('invert_ly').value
        self.invert_rx = gp('invert_rx').value
        self.invert_ry = gp('invert_ry').value
        self.invert_dpad_x = gp('invert_dpad_x').value
        self.invert_dpad_y = gp('invert_dpad_y').value

        self.BTN_A = gp('BTN_A').value
        self.BTN_B = gp('BTN_B').value
        self.BTN_X = gp('BTN_X').value
        self.BTN_Y = gp('BTN_Y').value
        self.BTN_LB = gp('BTN_LB').value
        self.BTN_RB = gp('BTN_RB').value
        self.BTN_VIEW = gp('BTN_VIEW').value
        self.BTN_MENU = gp('BTN_MENU').value
        self.BTN_LS = gp('BTN_LS').value
        self.BTN_RS = gp('BTN_RS').value
        self.BTN_SHARE = gp('BTN_SHARE').value

        self.pub = self.create_publisher(String, self.topic, 10)
        self.create_subscription(Joy, '/joy', self.cb, 10)

        self.prev_buttons = None
        self._last_emit_t = 0.0

        self._prev_lt_pressed = False
        self._prev_rt_pressed = False
        self._prev_dpad_x = 0
        self._prev_dpad_y = 0
        self._prev_ls_dir = "CENTER"
        self._prev_rs_dir = "CENTER"

        self.get_logger().info(f'🎮 xbox_buttons_debug started -> {self.topic}')

    # ---------- helpers ----------
    def _axis(self, axes, idx, default=0.0):
        return float(axes[idx]) if 0 <= idx < len(axes) else default

    def _rose(self, prev, curr, idx):
        try:
            return prev[idx] == 0 and curr[idx] == 1
        except Exception:
            return False

    def _emit(self, text: str):
        now = time.time()
        if now - self._last_emit_t < self.throttle_sec:
            return
        self._last_emit_t = now
        self.pub.publish(String(data=text))

    def _dir4(self, x: float, y: float):
        if abs(x) < self.stick_dir_th and abs(y) < self.stick_dir_th:
            return "CENTER"
        return "UP" if abs(y) >= abs(x) and y > 0 else \
               "DOWN" if abs(y) >= abs(x) else \
               "RIGHT" if x > 0 else "LEFT"

    # ---------- callback ----------
    def cb(self, msg: Joy):
        if self.prev_buttons is None:
            self.prev_buttons = list(msg.buttons)

        # ===== Trigger =====
        raw_lt = self._axis(msg.axes, self.AX_LT, 1.0)
        raw_rt = self._axis(msg.axes, self.AX_RT, 1.0)
        lt_pressed = raw_lt < self.lt_threshold
        rt_pressed = raw_rt < self.rt_threshold

        if lt_pressed != self._prev_lt_pressed:
            self._emit("LT" if lt_pressed else "LT_RELEASE")
            self._prev_lt_pressed = lt_pressed

        if rt_pressed != self._prev_rt_pressed:
            self._emit("RT" if rt_pressed else "RT_RELEASE")
            self._prev_rt_pressed = rt_pressed

        # ===== D-pad =====
        dpad_x = int(round(self._axis(msg.axes, self.AX_DPAD_X, 0.0)))
        dpad_y = int(round(self._axis(msg.axes, self.AX_DPAD_Y, 0.0)))

        if self.invert_dpad_x:
            dpad_x = -dpad_x
        if self.invert_dpad_y:
            dpad_y = -dpad_y

        if dpad_x != self._prev_dpad_x or dpad_y != self._prev_dpad_y:
            if dpad_x == 1: self._emit("DPAD_RIGHT")
            if dpad_x == -1: self._emit("DPAD_LEFT")
            if dpad_y == 1: self._emit("DPAD_UP")
            if dpad_y == -1: self._emit("DPAD_DOWN")
            if dpad_x == 0 and dpad_y == 0:
                self._emit("DPAD_RELEASE")
            self._prev_dpad_x = dpad_x
            self._prev_dpad_y = dpad_y

        # ===== Buttons =====
        for name, idx in [
            ("A", self.BTN_A), ("B", self.BTN_B), ("X", self.BTN_X), ("Y", self.BTN_Y),
            ("LB", self.BTN_LB), ("RB", self.BTN_RB),
            ("VIEW", self.BTN_VIEW), ("MENU", self.BTN_MENU),
            ("LS_CLICK", self.BTN_LS), ("RS_CLICK", self.BTN_RS),
            ("SHARE", self.BTN_SHARE),
        ]:
            if self._rose(self.prev_buttons, msg.buttons, idx):
                self._emit(name)

        # ===== Stick directions =====
        lx = deadzone(self._axis(msg.axes, self.AX_LX, 0.0), self.stick_dz)
        ly = deadzone(self._axis(msg.axes, self.AX_LY, 0.0), self.stick_dz)
        rx = deadzone(self._axis(msg.axes, self.AX_RX, 0.0), self.stick_dz)
        ry = deadzone(self._axis(msg.axes, self.AX_RY, 0.0), self.stick_dz)

        if self.invert_lx: lx = -lx
        if self.invert_ly: ly = -ly
        if self.invert_rx: rx = -rx
        if self.invert_ry: ry = -ry

        ls_dir = self._dir4(lx, ly)
        rs_dir = self._dir4(rx, ry)

        if ls_dir != self._prev_ls_dir:
            self._emit("LS_RELEASE" if ls_dir == "CENTER" else f"LS_{ls_dir}")
            self._prev_ls_dir = ls_dir

        if rs_dir != self._prev_rs_dir:
            self._emit("RS_RELEASE" if rs_dir == "CENTER" else f"RS_{rs_dir}")
            self._prev_rs_dir = rs_dir

        self.prev_buttons = list(msg.buttons)


def main():
    rclpy.init()
    rclpy.spin(XboxButtonsDebug())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
