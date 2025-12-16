#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import threading
import serial


class TreeboLib:
    """
    STM32 UART 프로토콜
      - 모터 명령:  "m m1 m2 m3 m4\n"
      - 엔코더 출력: "ENC <ms> <e1> <e2> <e3> <e4>\r\n"  (STM32 printf로 송신)

    e1~e4는 "누적 tick"을 권장 (odometry가 훨씬 안정적)
    """

    def __init__(
        self,
        port="/dev/ttyACM0",
        baudrate=115200,
        timeout=0.05,
        debug=False,
        use_pwm=True,
        pwm_min=1800,
        pwm_max=3500,
    ):
        self.debug = bool(debug)

        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2.0)  # 보드 리셋/부팅 안정화

        # PWM 매핑(0이면 정지, 0이 아니면 최소 pwm 이상)
        self.use_pwm = bool(use_pwm)
        self.pwm_min = int(abs(pwm_min))
        self.pwm_max = int(abs(pwm_max))
        if self.pwm_max < self.pwm_min:
            self.pwm_max = self.pwm_min

        # 엔코더 최신값(누적 tick)
        self._enc_ms = 0
        self._enc = [0, 0, 0, 0]
        self._enc_valid = False
        self._lock = threading.Lock()

        # 수신 스레드
        self._running = False
        self._rx_thread = None

    # ----------------- 기본 유틸 -----------------
    @staticmethod
    def _clamp(x, lo, hi):
        return max(lo, min(hi, x))

    def _to_pwm(self, x_norm: float) -> int:
        """
        x_norm: -1.0~+1.0
        return: 0 또는 ±[pwm_min..pwm_max]
        """
        x = float(x_norm)
        if abs(x) < 1e-6:
            return 0
        s = 1 if x > 0 else -1
        a = self._clamp(abs(x), 0.0, 1.0)
        pwm = int(self.pwm_min + a * (self.pwm_max - self.pwm_min))
        return s * pwm

    # ----------------- UART 송신 -----------------
    def send_motor(self, m1: int, m2: int, m3: int, m4: int):
        cmd = f"m {int(m1)} {int(m2)} {int(m3)} {int(m4)}\n"
        if self.debug:
            print("[TX]", cmd.strip())
        self.ser.write(cmd.encode("ascii"))
        self.ser.flush()

    def stop(self):
        self.send_motor(0, 0, 0, 0)

    # ----------------- UART 수신 (ENC) -----------------
    def start_rx(self):
        if self._rx_thread is not None:
            return
        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def close(self):
        self._running = False
        if self._rx_thread is not None:
            self._rx_thread.join(timeout=1.0)
            self._rx_thread = None
        try:
            self.ser.close()
        except Exception:
            pass

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
                                e = [int(parts[2]), int(parts[3]), int(parts[4]), int(parts[5])]
                                with self._lock:
                                    self._enc_ms = ms
                                    self._enc = e
                                    self._enc_valid = True
                            except ValueError:
                                if self.debug:
                                    print("[ENC PARSE ERR]", line)
                        else:
                            if self.debug:
                                print("[ENC FORMAT?]", line)
                    else:
                        if self.debug:
                            print("[RX]", line)

            except Exception as e:
                if self.debug:
                    print("[RX ERR]", e)
                time.sleep(0.01)

    def get_encoder(self):
        """
        return: (valid, ms, [e1,e2,e3,e4])
        """
        with self._lock:
            return self._enc_valid, self._enc_ms, list(self._enc)
