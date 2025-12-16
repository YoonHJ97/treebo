#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""treebo_bringup (calibrated)

이 파일은 사용자가 진행한 보정 결과를 기본값으로 반영한 bringup입니다.

주요 변경점
1) PWM 모드에서 gain을 PWM에 곱하지 않고, norm(-1~1)에 곱한 뒤 _to_pwm()로 변환
   - pwm_min(예: 1800) 아래로 내려가 바퀴가 멈추는 현상 방지
2) cmd_vel의 angular.z에 turn_scale을 적용(회전만 증폭/감쇠 가능)
3) 사용자가 튜닝한 gain / max_ang_vel / use_pwm_deadzone 기본값 반영

UART protocol
  TX: "m <m1> <m2> <m3> <m4>\n"
  RX: "ENC <ms> <e1> <e2> <e3> <e4>\n"  (누적 tick)

Motor mapping (사용자 확인)
  m1=FL, m2=RL, m3=RR, m4=FR
"""

import time
import threading
import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, Float32MultiArray


class TreeboBringup(Node):
    def __init__(self):
        super().__init__("treebo_bringup")

        # ---- Serial params ----
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("serial_timeout", 0.05)

        # ---- Command/PWM params ----
        self.declare_parameter("use_pwm", True)
        self.declare_parameter("pwm_min", 1800)
        self.declare_parameter("pwm_max", 3500)

        # 정규화 기준
        self.declare_parameter("max_lin_vel", 0.5)      # m/s
        self.declare_parameter("max_ang_vel", 6.0)      # rad/s  (사용자 실험값 반영)
        self.declare_parameter("turn_scale", 1.5)       # angular.z 스케일(회전만 키우고 싶을 때)
        self.declare_parameter("track_width", 0.12)     # m

        self.declare_parameter("vx_deadzone", 0.02)
        self.declare_parameter("wz_deadzone", 0.05)
        self.declare_parameter("cmd_timeout", 0.4)

        # ---- Calibrated per-motor gain defaults (cmd_vel ~ 0.2 기준) ----
        self.declare_parameter("gain_m1", 0.79)
        self.declare_parameter("gain_m2", 0.48)
        self.declare_parameter("gain_m3", 0.59)
        self.declare_parameter("gain_m4", 0.98)

        self.declare_parameter("invert_m1", False)
        self.declare_parameter("invert_m2", False)
        self.declare_parameter("invert_m3", False)
        self.declare_parameter("invert_m4", False)

        # ※ calibrated 방식에서는 deadzone을 끄는 쪽이 튜닝/재현성이 좋았음
        self.declare_parameter("use_pwm_deadzone", False)

        # ---- Debug / telemetry ----
        self.declare_parameter("debug_tx", False)
        self.declare_parameter("publish_wheel_speed", True)
        self.declare_parameter("publish_wheel_delta", False)
        self.declare_parameter("debug_enc_speed", True)
        self.declare_parameter("debug_enc_period", 0.5)

        # ---- Read params ----
        self.port = str(self.get_parameter("port").value)
        self.baud = int(self.get_parameter("baudrate").value)
        self.ser_timeout = float(self.get_parameter("serial_timeout").value)

        self.use_pwm = bool(self.get_parameter("use_pwm").value)
        self.pwm_min = int(self.get_parameter("pwm_min").value)
        self.pwm_max = int(self.get_parameter("pwm_max").value)
        if self.pwm_max < self.pwm_min:
            self.pwm_max = self.pwm_min

        self.max_lin = float(self.get_parameter("max_lin_vel").value)
        self.max_ang = float(self.get_parameter("max_ang_vel").value)
        self.turn_scale = float(self.get_parameter("turn_scale").value)
        self.track = float(self.get_parameter("track_width").value)

        self.vx_deadzone = float(self.get_parameter("vx_deadzone").value)
        self.wz_deadzone = float(self.get_parameter("wz_deadzone").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)

        self.gain = {
            "m1": float(self.get_parameter("gain_m1").value),
            "m2": float(self.get_parameter("gain_m2").value),
            "m3": float(self.get_parameter("gain_m3").value),
            "m4": float(self.get_parameter("gain_m4").value),
        }
        self.invert = {
            "m1": bool(self.get_parameter("invert_m1").value),
            "m2": bool(self.get_parameter("invert_m2").value),
            "m3": bool(self.get_parameter("invert_m3").value),
            "m4": bool(self.get_parameter("invert_m4").value),
        }
        self.use_pwm_deadzone = bool(self.get_parameter("use_pwm_deadzone").value)

        self.debug_tx = bool(self.get_parameter("debug_tx").value)
        self.publish_wheel_speed = bool(self.get_parameter("publish_wheel_speed").value)
        self.publish_wheel_delta = bool(self.get_parameter("publish_wheel_delta").value)
        self.debug_enc_speed = bool(self.get_parameter("debug_enc_speed").value)
        self.debug_enc_period = float(self.get_parameter("debug_enc_period").value)

        # ---- Serial open ----
        self.ser = serial.Serial(self.port, self.baud, timeout=self.ser_timeout)
        time.sleep(2.0)

        # ---- ROS I/O ----
        self.create_subscription(Twist, "cmd_vel", self.cb_cmd, 10)
        self.pub_enc = self.create_publisher(Int32MultiArray, "encoder_raw", 50)

        self.pub_speed = None
        if self.publish_wheel_speed:
            self.pub_speed = self.create_publisher(Float32MultiArray, "wheel_ticks_per_sec", 50)

        self.pub_delta = None
        if self.publish_wheel_delta:
            self.pub_delta = self.create_publisher(Int32MultiArray, "wheel_delta_ticks", 50)

        # ---- State ----
        self.last_cmd_time = self.get_clock().now()
        self._last_enc = None
        self._last_speed_log_t = time.time()

        # ---- RX thread ----
        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        # ---- Watchdog timer ----
        self.create_timer(0.05, self._watchdog)

        # Stop at start
        self._send_motor(0, 0, 0, 0)
        self.get_logger().info(
            f"Started(calib). UART={self.port}@{self.baud} "
            f"gain=({self.gain['m1']:.2f},{self.gain['m2']:.2f},{self.gain['m3']:.2f},{self.gain['m4']:.2f}) "
            f"max_ang={self.max_ang:.2f} turn_scale={self.turn_scale:.2f}"
        )

    # ---------- helpers ----------
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

    def apply_gain_norm(self, x_norm: float, g: float, invert: bool) -> float:
        """정규화(-1~1) 명령에 gain을 적용한 뒤, -1~1로 클램프해서 반환."""
        y = self._clamp(x_norm * g, -1.0, 1.0)
        if invert:
            y = -y
        return y

    def _send_motor(self, m1: int, m2: int, m3: int, m4: int):
        cmd = f"m {int(m1)} {int(m2)} {int(m3)} {int(m4)}\n"
        self.ser.write(cmd.encode("ascii"))
        self.ser.flush()
        if self.debug_tx:
            self.get_logger().info(f"TX m {m1} {m2} {m3} {m4}")

    # ---------- ROS callbacks ----------
    def cb_cmd(self, msg: Twist):
        vx = float(msg.linear.x)
        wz = float(msg.angular.z)

        # 회전 스케일(조이스틱 turn이 약할 때만 키우기 용도)
        wz *= self.turn_scale

        # safety clamp
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
            # ★ 중요: gain은 norm에 적용 후 PWM 변환 (pwm_min 아래로 떨어져 멈추는 문제 방지)
            n1 = self.apply_gain_norm(left_norm, self.gain["m1"], self.invert["m1"])
            n2 = self.apply_gain_norm(left_norm, self.gain["m2"], self.invert["m2"])
            n3 = self.apply_gain_norm(right_norm, self.gain["m3"], self.invert["m3"])
            n4 = self.apply_gain_norm(right_norm, self.gain["m4"], self.invert["m4"])
            m1 = self._to_pwm(n1)
            m2 = self._to_pwm(n2)
            m3 = self._to_pwm(n3)
            m4 = self._to_pwm(n4)
        else:
            # speed mode placeholder (0~1000)
            n1 = self.apply_gain_norm(left_norm, self.gain["m1"], self.invert["m1"])
            n2 = self.apply_gain_norm(left_norm, self.gain["m2"], self.invert["m2"])
            n3 = self.apply_gain_norm(right_norm, self.gain["m3"], self.invert["m3"])
            n4 = self.apply_gain_norm(right_norm, self.gain["m4"], self.invert["m4"])
            m1 = int(n1 * 1000)
            m2 = int(n2 * 1000)
            m3 = int(n3 * 1000)
            m4 = int(n4 * 1000)

        self._send_motor(m1, m2, m3, m4)
        self.last_cmd_time = self.get_clock().now()

    def _watchdog(self):
        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds * 1e-9
        if dt > self.cmd_timeout:
            self._send_motor(0, 0, 0, 0)

    # ---------- UART RX ----------
    def _rx_loop(self):
        buf = b""
        while self._running:
            try:
                data = self.ser.read(256)
                if not data:
                    continue

                buf += data
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    line = raw.strip().decode(errors="ignore")
                    if not line:
                        continue
                    if not line.startswith("ENC "):
                        continue

                    parts = line.split()
                    if len(parts) < 6:
                        continue

                    try:
                        ms = int(parts[1])
                        e1 = int(parts[2]); e2 = int(parts[3]); e3 = int(parts[4]); e4 = int(parts[5])
                    except ValueError:
                        continue

                    # 1) encoder_raw publish
                    msg = Int32MultiArray()
                    msg.data = [ms, e1, e2, e3, e4]
                    self.pub_enc.publish(msg)

                    # 2) speed / delta ticks
                    now_t = time.time()
                    if self._last_enc is not None:
                        last_t, last_ms, le1, le2, le3, le4 = self._last_enc
                        dt = now_t - last_t
                        if dt > 1e-3:
                            de1 = e1 - le1
                            de2 = e2 - le2
                            de3 = e3 - le3
                            de4 = e4 - le4

                            if self.pub_delta is not None:
                                dmsg = Int32MultiArray()
                                dmsg.data = [ms, de1, de2, de3, de4]
                                self.pub_delta.publish(dmsg)

                            if self.pub_speed is not None:
                                s1 = de1 / dt
                                s2 = de2 / dt
                                s3 = de3 / dt
                                s4 = de4 / dt
                                smsg = Float32MultiArray()
                                smsg.data = [float(s1), float(s2), float(s3), float(s4)]
                                self.pub_speed.publish(smsg)

                                if self.debug_enc_speed and (now_t - self._last_speed_log_t) >= self.debug_enc_period:
                                    self._last_speed_log_t = now_t
                                    self.get_logger().info(
                                        f"ENC_SPEED ticks/s m1={s1:.1f} m2={s2:.1f} m3={s3:.1f} m4={s4:.1f}"
                                    )

                    self._last_enc = (now_t, ms, e1, e2, e3, e4)

            except Exception:
                time.sleep(0.01)

    # ---------- shutdown ----------
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
