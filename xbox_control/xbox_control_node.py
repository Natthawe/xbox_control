#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Int8, Float32MultiArray, Int32MultiArray

def deadzone(v, dz=0.08):
    return 0.0 if abs(v) < dz else math.copysign((abs(v) - dz) / (1.0 - dz), v)


class XboxControl(Node):
    def __init__(self):
        super().__init__('xbox_control')

        # ===== Parameters =====
        # General settings
        self.declare_parameter('scale_linear', 0.5)
        self.declare_parameter('scale_angular', 1.0)
        self.declare_parameter('rt_threshold', 0.5)
        self.declare_parameter('deadzone', 0.08)
        self.declare_parameter('invert_linear', False)
        self.declare_parameter('invert_angular', False)
        self.declare_parameter('axis_active_threshold', 0.02)
        self.declare_parameter('lt_threshold', 0.5)
        self.declare_parameter('cmd_vel_publish_rate_hz', 20.0)

        # Mapping
        self.declare_parameter('AX_LX', 0)
        self.declare_parameter('AX_RY', 3)
        self.declare_parameter('AX_RT', 4)
        self.declare_parameter('AX_LT', 5)
        self.declare_parameter('BTN_A', 0)
        self.declare_parameter('BTN_B', 1)
        self.declare_parameter('BTN_X', 3)
        self.declare_parameter('BTN_Y', 4)
        self.declare_parameter('BTN_LB', 6)

        # Trim (gain mode)
        self.declare_parameter('trim_step', 0.01)
        self.declare_parameter('trim_linear_min', -1.0)
        self.declare_parameter('trim_linear_max', 1.0)
        self.declare_parameter('trim_angular_min', -2.0)
        self.declare_parameter('trim_angular_max', 2.0)

        # Debug
        self.declare_parameter('debug_enable', False)
        self.declare_parameter('debug_publish_rate_hz', 10.0)
        self.declare_parameter('debug_log_throttle_sec', 2.0)

        gp = self.get_parameter
        self.scale_linear = float(gp('scale_linear').value)
        self.scale_angular = float(gp('scale_angular').value)
        self.rt_threshold = float(gp('rt_threshold').value)
        self.dz = float(gp('deadzone').value)
        self.invert_linear = bool(gp('invert_linear').value)
        self.invert_angular = bool(gp('invert_angular').value)
        self.axis_active_th = float(gp('axis_active_threshold').value)

        self.AX_LX = int(gp('AX_LX').value)
        self.AX_RY = int(gp('AX_RY').value)
        self.AX_RT = int(gp('AX_RT').value)
        self.AX_LT = int(gp('AX_LT').value)
        self.BTN_A = int(gp('BTN_A').value)
        self.BTN_B = int(gp('BTN_B').value)
        self.BTN_X = int(gp('BTN_X').value)
        self.BTN_Y = int(gp('BTN_Y').value)
        self.BTN_LB = int(gp('BTN_LB').value)

        self.trim_step = float(gp('trim_step').value)
        self.trim_linear_min = float(gp('trim_linear_min').value)
        self.trim_linear_max = float(gp('trim_linear_max').value)
        self.trim_angular_min = float(gp('trim_angular_min').value)
        self.trim_angular_max = float(gp('trim_angular_max').value)
        self.lt_threshold = float(gp('lt_threshold').value)
        self.cmd_vel_pub_hz = float(gp('cmd_vel_publish_rate_hz').value)

        # Debug params
        self.debug_enable = bool(gp('debug_enable').value)
        self.debug_pub_hz = float(gp('debug_publish_rate_hz').value)
        self.debug_log_th = float(gp('debug_log_throttle_sec').value)

        # Publishers / Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tray_pub = self.create_publisher(Int32, '/tray_cmd', 10)
        # Publisher Test Mode
        self.test_mode_pub = self.create_publisher(Int8, '/test_mode', 10)
        
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # เก็บคำสั่งล่าสุดไว้ส่งซ้ำ
        self._last_twist = Twist()

        # สถานะ Test Mode
        self.is_test_mode = False 

        # ตั้ง timer ให้ publish /cmd_vel คงที่
        self.cmd_timer = self.create_timer(1.0 / max(1e-3, self.cmd_vel_pub_hz), self._on_cmd_timer)

        # Debug publishers
        if self.debug_enable:
            self.pub_dbg_axes = self.create_publisher(Float32MultiArray, '/xbox_control/debug/axes', 10)
            self.pub_dbg_proc = self.create_publisher(Float32MultiArray, '/xbox_control/debug/processed', 10)
            self.pub_dbg_flags = self.create_publisher(Int32MultiArray, '/xbox_control/debug/flags', 10)
            self.pub_dbg_cmd = self.create_publisher(Twist, '/xbox_control/debug/cmd', 10)
            self.dbg_timer = self.create_timer(1.0 / max(1e-3, self.debug_pub_hz), self._on_debug_timer)
            self._last_log_t = 0.0

        # Edge states & trims
        self.prev_buttons = None
        self.linear_trim = 0.0
        self.angular_trim = 0.0

        # store last state for debug timer
        self._dbg_state = {
            'raw_lx': 0.0, 'raw_ry': 0.0, 'raw_rt': 1.0,
            'rs_v': 0.0, 'ls_h': 0.0,
            'lin_gain': self.scale_linear, 'ang_gain': self.scale_angular,
            'rt_pressed': 0, 'v_active': 0, 'w_active': 0,
            'btnY_edge': 0, 'btnA_edge': 0, 'lb_down': 0,
            'cmd_v': 0.0, 'cmd_w': 0.0
        }

        self.get_logger().info('🎮 Xbox Control ready.')
        self.get_logger().info(f'DEBUG: {self.debug_enable}')
        self.get_logger().info(f'Axis Active_Threshold : {self.axis_active_th:.2f}')

    # -------- Helpers --------
    def _axis(self, axes, idx, default=0.0):
        return float(axes[idx]) if 0 <= idx < len(axes) else default

    def _rose(self, prev, curr, idx):
        try:
            return prev[idx] == 0 and curr[idx] == 1
        except IndexError:
            return False

    def _clamp(self, v, vmin, vmax):
        return max(vmin, min(vmax, v))

    # -------- Debug timer --------
    def _on_debug_timer(self):
        # publish arrays/flags/cmd
        fa = Float32MultiArray()
        fa.data = [self._dbg_state['raw_lx'], self._dbg_state['raw_ry'], self._dbg_state['raw_rt']]
        self.pub_dbg_axes.publish(fa)

        fp = Float32MultiArray()
        fp.data = [self._dbg_state['ls_h'], self._dbg_state['rs_v'], self._dbg_state['lin_gain'], self._dbg_state['ang_gain']]
        self.pub_dbg_proc.publish(fp)

        fi = Int32MultiArray()
        fi.data = [
            int(self._dbg_state['rt_pressed']),
            int(self._dbg_state['v_active']),
            int(self._dbg_state['w_active']),
            int(self._dbg_state['btnY_edge']),
            int(self._dbg_state['btnA_edge']),
            int(self._dbg_state['lb_down']),
        ]
        self.pub_dbg_flags.publish(fi)

        tw = Twist()
        tw.linear.x = float(self._dbg_state['cmd_v'])
        tw.angular.z = float(self._dbg_state['cmd_w'])
        self.pub_dbg_cmd.publish(tw)

        # throttled log
        now = time.time()
        if now - self._last_log_t >= self.debug_log_th:
            self._last_log_t = now
            self.get_logger().info(
                f"[DBG] raw(lx,ry,rt)=({self._dbg_state['raw_lx']:+.3f}, {self._dbg_state['raw_ry']:+.3f}, {self._dbg_state['raw_rt']:+.3f}) "
                f"proc(ls_h,rs_v)=({self._dbg_state['ls_h']:+.3f}, {self._dbg_state['rs_v']:+.3f}) "
                f"gain(L,A)=({self._dbg_state['lin_gain']:.3f}, {self._dbg_state['ang_gain']:.3f}) "
                f"flags(RT,v,w,Y,A,LB)=({fi.data[0]}, {fi.data[1]}, {fi.data[2]}, {fi.data[3]}, {fi.data[4]}, {fi.data[5]}) "
                f"cmd(v,w)=({self._dbg_state['cmd_v']:+.3f}, {self._dbg_state['cmd_w']:+.3f})"
            )

    # -------- Cmd timer --------
    def _on_cmd_timer(self):
        # *** ถ้าอยู่ในโหมด Test ให้หยุดส่ง cmd_vel ***
        if self.is_test_mode:
            return 
        
        # ส่งคำสั่งล่าสุดออกทุก ๆ รอบ แม้ไม่มี joy เข้ามา (Normal Mode)
        self.cmd_pub.publish(self._last_twist)

    # -------- Callback --------

    def joy_callback(self, msg: Joy):
        if self.prev_buttons is None:
            self.prev_buttons = list(msg.buttons)

        # ===== LT state =====
        raw_lt = self._axis(msg.axes, self.AX_LT, 1.0)
        lt_pressed = (raw_lt < self.lt_threshold)

        # ===== Button Edges =====
        btnA_edge = self._rose(self.prev_buttons, msg.buttons, self.BTN_A)
        btnB_edge = self._rose(self.prev_buttons, msg.buttons, self.BTN_B)
        btnX_edge = self._rose(self.prev_buttons, msg.buttons, self.BTN_X)
        btnY_edge = self._rose(self.prev_buttons, msg.buttons, self.BTN_Y)

        # ===== Tray commands: LT + (Y/A) =====
        if lt_pressed and btnY_edge:
            self.tray_pub.publish(Int32(data=1))     # LT + Y → เปิด Tray (1)

        if lt_pressed and btnA_edge:
            self.tray_pub.publish(Int32(data=-1))    # LT + A → ปิด Tray (-1)

        # ===== TEST MODE Control: LT + (B/X) =====
        # LT + B = ENABLE Test Mode (1)
        if lt_pressed and btnB_edge:
            self.is_test_mode = True
            self.test_mode_pub.publish(Int8(data=1))
            self.get_logger().info('>>> TEST MODE ENABLED (Joy cmd_vel BLOCKED) <<<')

        # LT + X = DISABLE Test Mode (0)
        if lt_pressed and btnX_edge:
            self.is_test_mode = False
            self.test_mode_pub.publish(Int8(data=0))
            self.get_logger().info('<<< TEST MODE DISABLED (Joy cmd_vel RESUMED) >>>')


        # ===== Trim adjust: LB + (Y/A/B/X) edge =====
        lb_down = (0 <= self.BTN_LB < len(msg.buttons) and msg.buttons[self.BTN_LB] == 1)

        if lb_down:
            if btnY_edge:
                self.linear_trim = self._clamp(self.linear_trim + self.trim_step,
                                               self.trim_linear_min, self.trim_linear_max)
                self.get_logger().info(f'🔧 linear_trim = {self.linear_trim:+.3f}')
            if btnA_edge:
                self.linear_trim = self._clamp(self.linear_trim - self.trim_step,
                                               self.trim_linear_min, self.trim_linear_max)
                self.get_logger().info(f'🔧 linear_trim = {self.linear_trim:+.3f}')
            if btnB_edge:
                self.angular_trim = self._clamp(self.angular_trim + self.trim_step,
                                                self.trim_angular_min, self.trim_angular_max)
                self.get_logger().info(f'🔧 angular_trim = {self.angular_trim:+.3f}')
            if btnX_edge:
                self.angular_trim = self._clamp(self.angular_trim - self.trim_step,
                                                self.trim_angular_min, self.trim_angular_max)
                self.get_logger().info(f'🔧 angular_trim = {self.angular_trim:+.3f}')

        # ===== Driving (RT hold) =====
        raw_lx = self._axis(msg.axes, self.AX_LX, 0.0)
        raw_ry = self._axis(msg.axes, self.AX_RY, 0.0)
        raw_rt = self._axis(msg.axes, self.AX_RT, 1.0)
        rt_pressed = (raw_rt < self.rt_threshold)

        twist = Twist()
        if rt_pressed:
            # RS↑ -> +linear.x ; LS← -> +angular.z
            rs_v = deadzone(raw_ry, self.dz)
            ls_h = -deadzone(raw_lx, self.dz)

            if self.invert_linear:
                rs_v = -rs_v
            if self.invert_angular:
                ls_h = -ls_h

            linear_gain = max(0.0, self.scale_linear + self.linear_trim)
            angular_gain = max(0.0, self.scale_angular + self.angular_trim)

            v_active = abs(rs_v) > self.axis_active_th
            w_active = abs(ls_h) > self.axis_active_th

            v = float(linear_gain * rs_v) if v_active else 0.0
            w = float(angular_gain * ls_h) if w_active else 0.0

            twist.linear.x = v
            twist.angular.z = w
        else:
            rs_v = 0.0
            ls_h = 0.0
            v_active = 0
            w_active = 0
            v = w = 0.0
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # เก็บไว้ให้ timer ส่งออกตามรอบ (ถ้าไม่ได้อยู่ใน test mode timer จะเอาค่านี้ไปส่ง)
        self._last_twist = twist

        # keep debug state (for timer)
        if self.debug_enable:
            self._dbg_state.update({
                'raw_lx': raw_lx, 'raw_ry': raw_ry, 'raw_rt': raw_rt,
                'rs_v': rs_v, 'ls_h': ls_h,
                'lin_gain': (self.scale_linear + self.linear_trim),
                'ang_gain': (self.scale_angular + self.angular_trim),
                'rt_pressed': int(rt_pressed),
                'v_active': int(v_active), 'w_active': int(w_active),
                'btnY_edge': int(btnY_edge), 'btnA_edge': int(btnA_edge),
                'lb_down': int(lb_down),
                'cmd_v': v, 'cmd_w': w
            })

        # update edge state
        self.prev_buttons = list(msg.buttons)


def main():
    rclpy.init()
    node = XboxControl()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
