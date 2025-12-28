#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String, Float32MultiArray


def deadzone(v: float, dz: float) -> float:
    if abs(v) < dz:
        return 0.0
    return math.copysign((abs(v) - dz) / (1.0 - dz), v)


class XboxInput(Node):
    """
    Convert /joy ->:
      - /xbox/analog (Float32MultiArray): [lx, ly, rx, ry, lt, rt, dpad_x, dpad_y]
      - /xbox/event  (String): edge events + direction changes (sticks/dpad) + LT/RT press/release
    """

    def __init__(self):
        super().__init__('xbox_input')

        # ===== Topics / rates =====
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('analog_topic', '/xbox/analog')
        self.declare_parameter('event_topic', '/xbox/event')
        self.declare_parameter('analog_publish_rate_hz', 50.0)  # publish analog at fixed rate
        self.declare_parameter('event_throttle_sec', 0.01)       # minimal gap between event publishes

        # ===== Axis indices =====
        self.declare_parameter('AX_LX', 0)
        self.declare_parameter('AX_LY', 1)
        self.declare_parameter('AX_RX', 2)
        self.declare_parameter('AX_RY', 3)
        self.declare_parameter('AX_RT', 4)
        self.declare_parameter('AX_LT', 5)
        self.declare_parameter('AX_DPAD_X', 6)
        self.declare_parameter('AX_DPAD_Y', 7)

        # ===== Thresholds =====
        self.declare_parameter('stick_deadzone', 0.08)
        self.declare_parameter('stick_dir_threshold', 0.35)  # for 8-dir
        self.declare_parameter('dpad_dir_threshold', 0.5)     # -1/0/1

        # Trigger pressed thresholds (your controller: 1.0 -> -1.0, pressed when value < threshold)
        self.declare_parameter('lt_threshold', 0.5)
        self.declare_parameter('rt_threshold', 0.5)

        # Direction mode: threshold|angle
        self.declare_parameter('stick_dir_mode', 'angle')
        self.declare_parameter('dpad_dir_mode', 'threshold')

        # ===== Invert options =====
        self.declare_parameter('invert_lx', False)
        self.declare_parameter('invert_ly', False)
        self.declare_parameter('invert_rx', False)
        self.declare_parameter('invert_ry', False)
        self.declare_parameter('invert_dpad_x', False)
        self.declare_parameter('invert_dpad_y', False)

        # ===== Button indices =====
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

        self.joy_topic = str(gp('joy_topic').value)
        self.analog_topic = str(gp('analog_topic').value)
        self.event_topic = str(gp('event_topic').value)
        self.analog_hz = float(gp('analog_publish_rate_hz').value)
        self.event_throttle = float(gp('event_throttle_sec').value)

        self.AX_LX = int(gp('AX_LX').value)
        self.AX_LY = int(gp('AX_LY').value)
        self.AX_RX = int(gp('AX_RX').value)
        self.AX_RY = int(gp('AX_RY').value)
        self.AX_RT = int(gp('AX_RT').value)
        self.AX_LT = int(gp('AX_LT').value)
        self.AX_DPAD_X = int(gp('AX_DPAD_X').value)
        self.AX_DPAD_Y = int(gp('AX_DPAD_Y').value)

        self.stick_dz = float(gp('stick_deadzone').value)
        self.stick_dir_th = float(gp('stick_dir_threshold').value)
        self.dpad_dir_th = float(gp('dpad_dir_threshold').value)

        self.lt_threshold = float(gp('lt_threshold').value)
        self.rt_threshold = float(gp('rt_threshold').value)

        self.stick_dir_mode = str(gp('stick_dir_mode').value).strip().lower()
        self.dpad_dir_mode = str(gp('dpad_dir_mode').value).strip().lower()
        if self.stick_dir_mode not in ('threshold', 'angle'):
            self.stick_dir_mode = 'angle'
        if self.dpad_dir_mode not in ('threshold', 'angle'):
            self.dpad_dir_mode = 'threshold'

        self.inv_lx = bool(gp('invert_lx').value)
        self.inv_ly = bool(gp('invert_ly').value)
        self.inv_rx = bool(gp('invert_rx').value)
        self.inv_ry = bool(gp('invert_ry').value)
        self.inv_dx = bool(gp('invert_dpad_x').value)
        self.inv_dy = bool(gp('invert_dpad_y').value)

        self.BTN_A = int(gp('BTN_A').value)
        self.BTN_B = int(gp('BTN_B').value)
        self.BTN_X = int(gp('BTN_X').value)
        self.BTN_Y = int(gp('BTN_Y').value)
        self.BTN_LB = int(gp('BTN_LB').value)
        self.BTN_RB = int(gp('BTN_RB').value)
        self.BTN_VIEW = int(gp('BTN_VIEW').value)
        self.BTN_MENU = int(gp('BTN_MENU').value)
        self.BTN_LS = int(gp('BTN_LS').value)
        self.BTN_RS = int(gp('BTN_RS').value)
        self.BTN_SHARE = int(gp('BTN_SHARE').value)

        # Pub/Sub
        self.pub_analog = self.create_publisher(Float32MultiArray, self.analog_topic, 10)
        self.pub_event = self.create_publisher(String, self.event_topic, 10)
        self.create_subscription(Joy, self.joy_topic, self.cb_joy, 10)

        # State
        self.prev_buttons = None
        self._last_event_t = 0.0
        self._prev_btn_state = {}

        self._prev_ls_dir = "CENTER"
        self._prev_rs_dir = "CENTER"
        self._prev_dpad_dir = "CENTER"

        self._prev_lt_pressed = False
        self._prev_rt_pressed = False

        # Latest analog values (published by timer)
        self._analog = [0.0] * 8  # lx,ly,rx,ry,lt,rt,dpad_x,dpad_y

        # publish analog at fixed rate
        self.timer = self.create_timer(1.0 / max(1e-3, self.analog_hz), self._pub_analog_timer)

        self.get_logger().info(
            f'🎮 xbox_input started: joy={self.joy_topic} -> analog={self.analog_topic} event={self.event_topic} '
            f'| stick_mode={self.stick_dir_mode} dpad_mode={self.dpad_dir_mode} '
            f'| trig_threshold lt/rt=({self.lt_threshold:.2f},{self.rt_threshold:.2f})'
        )

    # -------- helpers --------
    def _axis(self, axes, idx, default=0.0) -> float:
        return float(axes[idx]) if 0 <= idx < len(axes) else default

    def _rose(self, prev, curr, idx) -> bool:
        try:
            return prev[idx] == 0 and curr[idx] == 1
        except Exception:
            return False

    def _emit_event(self, text: str):
        # events ที่ห้ามโดน throttle
        critical = (
            "LB", "LB_RELEASE", "RB", "RB_RELEASE",
            "LT", "LT_RELEASE", "RT", "RT_RELEASE",
        )
        if text in critical:
            self.pub_event.publish(String(data=text))
            return

        now = time.time()
        if now - self._last_event_t < self.event_throttle:
            return
        self._last_event_t = now
        self.pub_event.publish(String(data=text))

    def _dir8_threshold(self, x: float, y: float, th: float) -> str:
        ax = abs(x)
        ay = abs(y)
        if ax < th and ay < th:
            return "CENTER"
        x_on = ax >= th
        y_on = ay >= th

        if x_on and y_on:
            if y > 0 and x > 0: return "UP_RIGHT"
            if y > 0 and x < 0: return "UP_LEFT"
            if y < 0 and x > 0: return "DOWN_RIGHT"
            return "DOWN_LEFT"

        if y_on:
            return "UP" if y > 0 else "DOWN"
        return "RIGHT" if x > 0 else "LEFT"

    def _dir8_angle(self, x: float, y: float, th: float) -> str:
        if abs(x) < th and abs(y) < th:
            return "CENTER"
        ang = math.degrees(math.atan2(y, x))  # 0=RIGHT, 90=UP
        if -22.5 <= ang < 22.5: return "RIGHT"
        if 22.5 <= ang < 67.5: return "UP_RIGHT"
        if 67.5 <= ang < 112.5: return "UP"
        if 112.5 <= ang < 157.5: return "UP_LEFT"
        if ang >= 157.5 or ang < -157.5: return "LEFT"
        if -157.5 <= ang < -112.5: return "DOWN_LEFT"
        if -112.5 <= ang < -67.5: return "DOWN"
        return "DOWN_RIGHT"

    def _dir8(self, x: float, y: float, th: float, mode: str) -> str:
        return self._dir8_angle(x, y, th) if mode == 'angle' else self._dir8_threshold(x, y, th)

    def _pub_analog_timer(self):
        msg = Float32MultiArray()
        msg.data = [float(v) for v in self._analog]
        self.pub_analog.publish(msg)

    def _btn_down(self, buttons, idx) -> bool:
        try:
            return buttons[idx] == 1
        except Exception:
            return False

    # -------- callback --------
    def cb_joy(self, msg: Joy):
        if self.prev_buttons is None:
            self.prev_buttons = list(msg.buttons)

        # --- read axes ---
        lx = deadzone(self._axis(msg.axes, self.AX_LX, 0.0), self.stick_dz)
        ly = deadzone(self._axis(msg.axes, self.AX_LY, 0.0), self.stick_dz)
        rx = deadzone(self._axis(msg.axes, self.AX_RX, 0.0), self.stick_dz)
        ry = deadzone(self._axis(msg.axes, self.AX_RY, 0.0), self.stick_dz)

        lt = self._axis(msg.axes, self.AX_LT, 1.0)
        rt = self._axis(msg.axes, self.AX_RT, 1.0)

        # D-pad often -1/0/1; round for stability
        dpad_x = float(int(round(self._axis(msg.axes, self.AX_DPAD_X, 0.0))))
        dpad_y = float(int(round(self._axis(msg.axes, self.AX_DPAD_Y, 0.0))))

        # --- invert axes if needed ---
        if self.inv_lx: lx = -lx
        if self.inv_ly: ly = -ly
        if self.inv_rx: rx = -rx
        if self.inv_ry: ry = -ry
        if self.inv_dx: dpad_x = -dpad_x
        if self.inv_dy: dpad_y = -dpad_y

        # store for timer publishing
        self._analog = [lx, ly, rx, ry, lt, rt, dpad_x, dpad_y]

        # --- LT/RT pressed/release events (axis based) ---
        lt_pressed = (lt < self.lt_threshold)
        rt_pressed = (rt < self.rt_threshold)

        if lt_pressed != self._prev_lt_pressed:
            self._emit_event("LT" if lt_pressed else "LT_RELEASE")
            self._prev_lt_pressed = lt_pressed

        if rt_pressed != self._prev_rt_pressed:
            self._emit_event("RT" if rt_pressed else "RT_RELEASE")
            self._prev_rt_pressed = rt_pressed

        # --- stick direction events (8-dir) ---
        ls_dir = self._dir8(lx, ly, self.stick_dir_th, self.stick_dir_mode)
        rs_dir = self._dir8(rx, ry, self.stick_dir_th, self.stick_dir_mode)

        if ls_dir != self._prev_ls_dir:
            self._emit_event("LS_RELEASE" if ls_dir == "CENTER" else f"LS_{ls_dir}")
            self._prev_ls_dir = ls_dir

        if rs_dir != self._prev_rs_dir:
            self._emit_event("RS_RELEASE" if rs_dir == "CENTER" else f"RS_{rs_dir}")
            self._prev_rs_dir = rs_dir

        # --- dpad direction events (8-dir) ---
        dpad_dir = self._dir8(dpad_x, dpad_y, self.dpad_dir_th, self.dpad_dir_mode)
        if dpad_dir != self._prev_dpad_dir:
            self._emit_event("DPAD_RELEASE" if dpad_dir == "CENTER" else f"DPAD_{dpad_dir}")
            self._prev_dpad_dir = dpad_dir

        # --- button edge events ---
        for name, idx in [
            ("A", self.BTN_A), ("B", self.BTN_B), ("X", self.BTN_X), ("Y", self.BTN_Y),
            ("LB", self.BTN_LB), ("RB", self.BTN_RB),
            ("VIEW", self.BTN_VIEW), ("MENU", self.BTN_MENU),
            ("LS_CLICK", self.BTN_LS), ("RS_CLICK", self.BTN_RS),
            ("SHARE", self.BTN_SHARE),
        ]:
            if self._rose(self.prev_buttons, msg.buttons, idx):
                self._emit_event(name)

        # --- hold/release events for modifier buttons (LB/RB) ---
        for name, idx in [("LB", self.BTN_LB), ("RB", self.BTN_RB)]:
            cur = self._btn_down(msg.buttons, idx)
            prev = bool(self._prev_btn_state.get(name, False))
            if cur != prev:
                self._emit_event(name if cur else f"{name}_RELEASE")
                self._prev_btn_state[name] = cur

        self.prev_buttons = list(msg.buttons)


def main():
    rclpy.init()
    rclpy.spin(XboxInput())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
