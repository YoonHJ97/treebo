#!/usr/bin/env python3
# coding: utf-8
"""
treebolib.py (새 버전)

- STM32 펌웨어(텍스트 기반 프로토콜)에 맞춘 경량 라이브러리
- 모터 제어 + 엔코더 수신에만 집중
- ROS2 SLAM & Navigation에서 쓰기 좋게 cmd_vel 스타일 함수 제공

프로토콜 요약 (STM32 펌웨어 기준):

PC -> STM32 (UART2)
  - "m1 m2 m3 m4\\n"   : 각 바퀴 속도 명령 (예: "-500 0 0 0\\n")
  - "enc_on\\n"        : $upload:1,0,0# (누적 엔코더 업로드 ON)
  - "enc_off\\n"       : $upload:0,0,0# (업로드 OFF)

STM32 -> PC
  - "READY\\r\\n"
  - "SPD OK\\r\\n" / "SPD ERR\\r\\n"
  - "ENC <total> <v1> <v2> <v3>\\r\\n"

  예) ENC 188901 0 1 0

사용 예:
    from treebolib import TreeboLib
    bot = TreeboLib(port="/dev/ttyACM0", debug=True)
    bot.create_receive_threading()

    # 엔코더 업로드 켜기
    bot.set_encoder_upload(True)

    # 전진 0.2 m/s
    bot.set_car_motion(0.2, 0.0, 0.0)

    time.sleep(1.0)
    bot.set_car_motion(0.0, 0.0, 0.0)

    total, v1, v2, v3 = bot.get_encoder()
"""

import math
import threading
import time

import serial


class TreeboLib:
    """
    Treebo용 직렬 통신 라이브러리 (모터 + 엔코더 전용)
    """

    def __init__(self,
                 port="/dev/ttyACM0",
                 baudrate=115200,
                 wheel_radius=0.04,
                 base_length=0.095,
                 base_width=0.12,
                 ticks_per_rev=4320,
                 max_lin_vel=0.5,
                 debug=False):

        """
        port         : STM32가 연결된 직렬 포트 (예: /dev/ttyACM0)
        baudrate     : UART 속도 (펌웨어와 동일하게 115200)
        wheel_radius : 바퀴 반지름 [m] (R = 0.04)
        base_length  : 로봇 길이 [m] (L = 0.095) - 현재는 사용 X (확장용)
        base_width   : 로봇 폭(좌우 바퀴 중심 거리 근사) [m] (W = 0.12)
        ticks_per_rev: 엔코더 1바퀴당 tick 수 (4320)
        max_lin_vel  : 최대 선속도 [m/s] (이 값에 해당하는 명령이 ±1000으로 맵핑)
        debug        : True면 자세한 로그 출력
        """
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        self.debug = debug

        # 로봇 파라미터
        self.R = float(wheel_radius)
        self.L = float(base_length)
        self.W = float(base_width)     # 좌/우 바퀴 중심 거리 근사
        self.ticks_per_rev = int(ticks_per_rev)
        self.max_lin_vel = float(max_lin_vel)

        # 수신 상태
        self._enc_total = 0
        self._enc_v1 = 0
        self._enc_v2 = 0
        self._enc_v3 = 0

        self._lock = threading.Lock()
        self._rx_thread = None
        self._running = False

        if self.debug:
            print("[TreeboLib] OPEN",
                  f"port={port}, baud={baudrate}, R={self.R}, W={self.W}, ticks_per_rev={self.ticks_per_rev}")

    # ----------------------------------------------------------------------
    # 내부: 수신 스레드
    # ----------------------------------------------------------------------
    def _rx_loop(self):
        buf = b""
        while self._running:
            try:
                data = self.ser.read(64)
                if not data:
                    continue

                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.strip().decode(errors="ignore")
                    if not line:
                        continue

                    if self.debug:
                        # 원시 수신 라인 보고 싶으면 아래 주석 해제
                        # print("[RX RAW]", repr(line))
                        pass

                    # "ENC total v1 v2 v3"
                    if line.startswith("ENC "):
                        parts = line.split()
                        if len(parts) >= 5:
                            try:
                                total = int(parts[1])
                                v1 = int(parts[2])
                                v2 = int(parts[3])
                                v3 = int(parts[4])
                                with self._lock:
                                    self._enc_total = total
                                    self._enc_v1 = v1
                                    self._enc_v2 = v2
                                    self._enc_v3 = v3
                                if self.debug:
                                    print(f"[ENC] total={total}, v=({v1},{v2},{v3})")
                            except ValueError:
                                if self.debug:
                                    print("[PARSE ERR]", line)
                    else:
                        # READY, SPD OK, SPD ERR 등은 필요시 참고용
                        if self.debug:
                            print("[INFO]", line)
            except Exception as e:
                if self.debug:
                    print("[RX LOOP ERR]", e)
                time.sleep(0.05)

    # ----------------------------------------------------------------------
    # 공개 API: 수신 스레드 제어 (기존 create_receive_threading 형식 모사)
    # ----------------------------------------------------------------------
    def create_receive_threading(self):
        """백그라운드 수신 스레드 시작"""
        if self._rx_thread is not None:
            return
        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()
        if self.debug:
            print("[TreeboLib] receive thread started")

    def stop_receive_threading(self):
        """수신 스레드 중지"""
        self._running = False
        if self._rx_thread is not None:
            self._rx_thread.join(timeout=1.0)
            self._rx_thread = None

    def close(self):
        """포트/스레드 정리"""
        self.stop_receive_threading()
        try:
            self.ser.close()
        except Exception:
            pass

    # ----------------------------------------------------------------------
    # 공개 API: 엔코더 제어/읽기
    # ----------------------------------------------------------------------
    def set_encoder_upload(self, enable: bool):
        """
        STM32 펌웨어에 enc_on / enc_off 명령 전송.
        enable=True  -> 누적 엔코더 업로드 ON
        enable=False -> 업로드 OFF
        """
        cmd = "enc_on\n" if enable else "enc_off\n"
        if self.debug:
            print("[TX]", cmd.strip())
        self.ser.write(cmd.encode("utf-8"))
        self.ser.flush()

    def get_encoder(self):
        """
        마지막으로 수신한 엔코더 값 반환.

        return: (total, v1, v2, v3)
          - total : 누적 tick 값 (해석은 펌웨어/모터드라이버 정의에 따름)
          - v1,v2,v3 : 모터드라이버에서 보내는 추가 속도/상태 값
        """
        with self._lock:
            return self._enc_total, self._enc_v1, self._enc_v2, self._enc_v3

    def ticks_to_distance(self, delta_ticks: int) -> float:
        """
        엔코더 tick 변화량 -> 거리 [m] 변환.
        ticks_per_rev, wheel_radius를 사용.
        """
        return (delta_ticks / float(self.ticks_per_rev)) * (2.0 * math.pi * self.R)

    # ----------------------------------------------------------------------
    # 공개 API: 저수준 모터 명령
    # ----------------------------------------------------------------------
    def set_wheel_speed(self, m1, m2, m3, m4):
        """
        바퀴별 속도 명령을 그대로 STM32에 전송.
        펌웨어에서 "m1 m2 m3 m4"를 받아 모터드라이버에 $spd:...# 로 보내는 구조.

        보통 m1..m4 범위는 [-1000, 1000] 근처를 사용.
        """
        cmd = f"{int(m1)} {int(m2)} {int(m3)} {int(m4)}\n"
        if self.debug:
            print("[TX]", cmd.strip())
        self.ser.write(cmd.encode("utf-8"))
        self.ser.flush()

    # ----------------------------------------------------------------------
    # 공개 API: cmd_vel 스타일 고수준 주행 명령
    # ----------------------------------------------------------------------
    def set_car_motion(self, v_x, v_y, v_z):
        """
        ROS2의 geometry_msgs/Twist 와 비슷한 형식:

        v_x : 전/후진 선속도 [m/s] (앞 방향 +)
        v_y : (현재 하드웨어에서는 측면 이동 없음, 무시)
        v_z : yaw 각속도 [rad/s] (반시계 방향 +)

        내부에서는 4륜 스키드-스티어를 단순 차동구동 모델로 보고
        좌/우 바퀴 선속도로 변환한 뒤, [-1000,1000] 범위의 모터 명령으로 매핑.
        """
        # 입력 정리
        vx = float(v_x)
        wz = float(v_z)      # 회전

        # 좌/우 바퀴 선속도 [m/s]
        # D = W 를 좌/우 바퀴 중심 거리로 근사
        D = self.W
        v_left  = vx - wz * (D / 2.0)
        v_right = vx + wz * (D / 2.0)

        # 선속도 -> 모터 드라이버 명령 단위
        # max_lin_vel [m/s] 에서 명령 1000이 되도록 스케일링
        scale = 1000.0 / max(self.max_lin_vel, 1e-3)

        m_left  = v_left  * scale
        m_right = v_right * scale

        # 포화
        def clamp(x, limit=1000.0):
            if x > limit:
                return limit
            if x < -limit:
                return -limit
            return x

        m1 = clamp(m_left)
        m2 = clamp(m_left)
        m3 = clamp(m_right)
        m4 = clamp(m_right)

        if self.debug:
            print(f"[CMD_VEL] vx={vx:.3f}, wz={wz:.3f} -> "
                  f"m_left={m_left:.1f}, m_right={m_right:.1f}")

        self.set_wheel_speed(m1, m2, m3, m4)
