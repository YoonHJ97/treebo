#!/usr/bin/env python3
# coding: utf-8

"""
motor_calib_bidirectional.py

목적:
  - 동일한 PWM 크기(|pwm|)에 대해
      +방향, -방향 각각에서 ticks/s를 측정하고
  - 두 방향을 모두 고려한 보정 gain 후보를 계산한다.

주의:
  - 실행 전에 treebo_bringup 노드는 종료 (시리얼 포트 충돌 방지).
  - 로봇은 반드시 바닥에서 띄운 상태에서 테스트.
"""

import time
from statistics import mean

from treebolib import TreeboBase


def measure_one(bot: TreeboBase, pwm: int,
                duration: float = 3.0,
                sample_dt: float = 0.2):
    """
    주어진 pwm(%)로 duration 동안 돌리면서,
    sample_dt 간격으로 ticks/s를 측정해서 (m1~m4) 평균값을 반환.
    """
    print("\n==============================")
    print(f"[SEGMENT] PWM = {pwm}% (모든 바퀴 동일)")
    print("  ※ 바퀴가 공중에 떠 있는지 다시 확인하세요.")

    e1, e2, e3, e4 = bot.get_encoders()
    print(f"  START ENC = ({e1}, {e2}, {e3}, {e4})")

    bot.set_wheel_pwm(pwm, pwm, pwm, pwm)

    t0 = time.time()
    last_t = t0
    last_e = (e1, e2, e3, e4)

    s1_list, s2_list, s3_list, s4_list = [], [], [], []

    while True:
        now = time.time()
        if now - t0 >= duration:
            break

        if now - last_t >= sample_dt:
            ne1, ne2, ne3, ne4 = bot.get_encoders()
            dt = now - last_t
            de1 = ne1 - last_e[0]
            de2 = ne2 - last_e[1]
            de3 = ne3 - last_e[2]
            de4 = ne4 - last_e[3]

            s1 = de1 / dt
            s2 = de2 / dt
            s3 = de3 / dt
            s4 = de4 / dt

            s1_list.append(s1)
            s2_list.append(s2)
            s3_list.append(s3)
            s4_list.append(s4)

            print(
                f"  t={now - t0:4.1f}s  "
                f"ENC=({ne1:7d}, {ne2:7d}, {ne3:7d}, {ne4:7d})  "
                f"ticks/s≈({s1:7.1f}, {s2:7.1f}, {s3:7.1f}, {s4:7.1f})"
            )

            last_t = now
            last_e = (ne1, ne2, ne3, ne4)

        time.sleep(0.01)

    bot.set_wheel_pwm(0, 0, 0, 0)
    time.sleep(0.3)
    e1e, e2e, e3e, e4e = bot.get_encoders()
    print(f"  END   ENC = ({e1e}, {e2e}, {e3e}, {e4e})")
    print(f"  TOTAL dENC = ({e1e - e1}, {e2e - e2}, {e3e - e3}, {e4e - e4})")

    if not s1_list:
        print("  ⚠ 샘플이 없습니다 (duration/sample_dt 확인).")
        return None

    avg_s1 = mean(s1_list)
    avg_s2 = mean(s2_list)
    avg_s3 = mean(s3_list)
    avg_s4 = mean(s4_list)

    print(f"  AVG ticks/s = (m1={avg_s1:.1f}, m2={avg_s2:.1f}, m3={avg_s3:.1f}, m4={avg_s4:.1f})")
    return avg_s1, avg_s2, avg_s3, avg_s4


def main():
    port = "/dev/ttyUSB0"  # 필요시 수정
    car_type = TreeboBase.CARTYPE_X3

    print("=== Treebo Motor Calibration (Bidirectional) ===")
    print(f"포트: {port}, car_type: {car_type}")
    print("※ treebo_bringup 노드는 꺼진 상태여야 합니다.")
    print("※ 로봇은 반드시 공중에 띄워놓고 테스트하세요.")

    bot = TreeboBase(port=port, car_type=car_type, debug=False)
    bot.start_background_reader()
    time.sleep(0.1)
    bot.set_auto_report(True, persist=False)
    time.sleep(0.2)

    fw = bot.get_firmware_version()
    print(f"[INFO] Firmware version = {fw:.2f}")
    bot.beep(80)
    time.sleep(0.3)

    pwm_mag = 30  # 기준으로 쓸 PWM 크기 (%). 필요하면 40, 50 등으로 바꿔도 됨.

    # +방향 측정
    print("\n>>> + 방향 측정 시작")
    fwd = measure_one(bot, +pwm_mag)

    # -방향 측정 (속도는 abs()로 쓸 예정)
    print("\n>>> - 방향 측정 시작")
    rev = measure_one(bot, -pwm_mag)

    if fwd is None or rev is None:
        print("\n⚠ 측정 실패: 결과가 없습니다.")
        bot.set_wheel_pwm(0, 0, 0, 0)
        bot.close()
        return

    f1, f2, f3, f4 = fwd
    r1, r2, r3, r4 = rev

    # 부호를 제거하고 속도 크기만 사용
    af1, af2, af3, af4 = abs(f1), abs(f2), abs(f3), abs(f4)
    ar1, ar2, ar3, ar4 = abs(r1), abs(r2), abs(r3), abs(r4)

    print("\n=== 요약: 양방향 평균 속도 ===")
    print(f"  FWD ticks/s = (m1={af1:.1f}, m2={af2:.1f}, m3={af3:.1f}, m4={af4:.1f})")
    print(f"  REV ticks/s = (m1={ar1:.1f}, m2={ar2:.1f}, m3={ar3:.1f}, m4={ar4:.1f})")

    # 기준은 m1의 양방향 평균 속도
    ref1 = (af1 + ar1) / 2.0
    avg2 = (af2 + ar2) / 2.0
    avg3 = (af3 + ar3) / 2.0
    avg4 = (af4 + ar4) / 2.0

    # gain = ref / avg_i  (m1을 1.0 기준으로 맞추는 형태)
    g1 = 1.0
    g2 = ref1 / avg2 if avg2 != 0 else 1.0
    g3 = ref1 / avg3 if avg3 != 0 else 1.0
    g4 = ref1 / avg4 if avg4 != 0 else 1.0

    print("\n=== 보정 gain 추천 (양방향 타협값, 기준: m1=1.0) ===")
    print(f"  기준 PWM 크기 = {pwm_mag}%")
    print(f"  gain_m1 ≈ {g1:.2f}")
    print(f"  gain_m2 ≈ {g2:.2f}")
    print(f"  gain_m3 ≈ {g3:.2f}")
    print(f"  gain_m4 ≈ {g4:.2f}")
    print("\n이 값을 bringup 파라미터(gain_m1~4)에 넣고 전진/후진 모두 테스트하면서 소폭 조정하면 됩니다.")

    bot.set_wheel_pwm(0, 0, 0, 0)
    time.sleep(0.2)
    bot.close()
    print("\n=== Motor Calibration (Bidirectional) 종료 ===")


if __name__ == "__main__":
    main()
