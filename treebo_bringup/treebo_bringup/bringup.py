#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import threading
import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray


class TreeboBringup(Node):
    """
    - /cmd_vel 구독 → PWM 명령으로 변환 → UART로 "m m1 m2 m3 m4\n" 송신
    - UART 수신에서 "ENC ms e1 e2 e3 e4" 파싱 → /encoder_raw(Int32MultiArray) 퍼블리시
      data = [ms, e1, e2, e3, e4]  (e1~e4는 누적 tick)
    """

    def __init__(self):
        super().__init__("treebo_bringup")

        # ---- params ----
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("serial_timeout", 0.05)

        self.declare_parameter("use_pwm", True)
        self.declare_parameter("pwm_min", 1800)
        self.declare_parameter("pwm_max", 3500)

        self.declare_parameter("max_lin_vel", 0.5)   # m/s, 정규화 기준
        self.declare_parameter("max_ang_vel", 1.0)   # rad/s

        self.declare_parameter("vx_deadzone", 0.02)  # m/s
        self.declare_parameter("wz_deadzone", 0.05)  # rad/s
        self.declare_parameter("cmd_timeout", 0.4)   # sec

        self.declare_parameter("track_width", 0.12)  # m (좌우 바퀴 거리, 차동근사에 사용)

        self.port = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baudrate").value)
        self.ser_timeout = float(self.get_parameter("serial_timeout").value)

        self.use_pwm = bool(self.get_parameter("use_pwm").value)
        self.pwm_min = int(self.get_parameter("pwm_min").value)
        self.pwm_max = int(self.get_parameter("pwm_max").value)
        if self.pwm_max < self.pwm_min:
            self.pwm_max = self.pwm_min

        self.max_lin = float(self.get_parameter("max_lin_vel").value)
        self.max_ang = float(self.get_parameter("max_ang_vel").value)

        self.vx_deadzone = float(self.get_parameter("vx_deadzone").value)
        self.wz_deadzone = float(self.get_parameter("wz_deadzone").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)

        self.track = float(self.get_parameter("track_width").value)

        # ---- serial ----
        self.ser = serial.Serial(self.port, self.baud, timeout=self.ser_timeout)
        time.sleep(2.0)

        # ---- ros i/o ----
        self.create_subscription(Twist, "cmd_vel", self.cb_cmd, 10)
        self.pub_enc = self.create_publisher(Int32MultiArray, "encoder_raw", 50)

        # ---- state ----
        self.last_cmd_time = self.get_clock().now()

        # ---- rx thread ----
        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        # ---- watchdog timer ----
        self.create_timer(0.05, self._watchdog)

        # stop at start
        self._send_motor(0, 0, 0, 0)

    @staticmethod
    def _clamp(x, lo, hi):
        return max(lo, min(hi, x))

    def _to_pwm(self, x_norm: float) -> int:
        """-1~1 → 0 또는 ±[pwm_min..pwm_max]"""
        if abs(x_norm) < 1e-6:
            return 0
        s = 1 if x_norm > 0 else -1
        a = self._clamp(abs(x_norm), 0.0, 1.0)
        pwm = int(self.pwm_min + a * (self.pwm_max - self.pwm_min))
        return s * pwm

    def _send_motor(self, m1: int, m2: int, m3: int, m4: int):
        cmd = f"m {int(m1)} {int(m2)} {int(m3)} {int(m4)}\n"
        self.ser.write(cmd.encode("ascii"))
        self.ser.flush()

    def cb_cmd(self, msg: Twist):
        vx = float(msg.linear.x)
        wz = float(msg.angular.z)

        vx = self._clamp(vx, -self.max_lin, self.max_lin)
        wz = self._clamp(wz, -self.max_ang, self.max_ang)

        if abs(vx) < self.vx_deadzone:
            vx = 0.0
        if abs(wz) < self.wz_deadzone:
            wz = 0.0

        # 차동근사: 좌/우 선속도
        v_left = vx - wz * (self.track / 2.0)
        v_right = vx + wz * (self.track / 2.0)

        left_norm = self._clamp(v_left / max(self.max_lin, 1e-3), -1.0, 1.0)
        right_norm = self._clamp(v_right / max(self.max_lin, 1e-3), -1.0, 1.0)

        if self.use_pwm:
            m_left = self._to_pwm(left_norm)
            m_right = self._to_pwm(right_norm)
        else:
            m_left = int(left_norm * 1000)
            m_right = int(right_norm * 1000)

        # 4륜 매핑 (사용자 확인): m1=FL, m2=RL, m3=RR, m4=FR
        self._send_motor(m_left, m_left, m_right, m_right)

        self.last_cmd_time = self.get_clock().now()

    def _watchdog(self):
        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds * 1e-9
        if dt > self.cmd_timeout:
            self._send_motor(0, 0, 0, 0)

    def _rx_loop(self):
        buf = b""
        while self._running:
            try:
                data = self.ser.read(128)
                if not data:
                    continue
                buf += data
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    line = raw.strip().decode(errors="ignore")
                    if not line:
                        continue
                    if line.startswith("ENC "):
                        # ENC <ms> <e1> <e2> <e3> <e4>
                        parts = line.split()
                        if len(parts) >= 6:
                            try:
                                ms = int(parts[1])
                                e1 = int(parts[2])
                                e2 = int(parts[3])
                                e3 = int(parts[4])
                                e4 = int(parts[5])
                                msg = Int32MultiArray()
                                msg.data = [ms, e1, e2, e3, e4]
                                self.pub_enc.publish(msg)
                            except ValueError:
                                pass
            except Exception:
                time.sleep(0.01)

    def destroy_node(self):
        self._running = False
        try:
            self._send_motor(0, 0, 0, 0)
        except Exception:
            pass
        try:
            if self._rx_thread is not None:
                self._rx_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TreeboBringup()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
