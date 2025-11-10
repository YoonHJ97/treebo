import serial
import time

# ===== 설정 부분 =====
PORT = "/dev/ttyUSB0"      # 상황에 맞게 수정 (예: "/dev/ttyACM0")
BAUDRATE = 115200
UPLOAD_MODE = 3            # 3: 모터 현재 속도(mm/s) 리포트
TEST_SPEED = 200           # 모터 테스트할 때 쓸 기본 속도
PRINT_RAW = False          # True로 바꾸면 수신 원본 문자열도 같이 출력

# =====================

recv_buffer = ""  # 수신 데이터 누적 버퍼


def open_serial():
    print(f"[INFO] 시리얼 포트 여는 중... ({PORT}, {BAUDRATE}bps)")
    ser = serial.Serial(
        port=PORT,
        baudrate=BAUDRATE,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=0.05,
    )
    time.sleep(2.0)  # 보드 리셋/안정화 대기
    print("[INFO] 시리얼 포트 오픈 완료")
    return ser


def send_data(ser, text: str):
    """문자열을 그대로 보내되, # 포함 여부는 호출하는 쪽에서 관리"""
    if PRINT_RAW:
        print(f"[TX] {text}")
    ser.write(text.encode("utf-8"))
    ser.flush()
    time.sleep(0.01)


def read_message(ser):
    """
    시리얼 버퍼에서 데이터를 읽어 '#' 단위로 잘라
    한 번에 한 메시지씩 반환.
    없으면 None.
    """
    global recv_buffer

    if ser.in_waiting > 0:
        try:
            data = ser.read(ser.in_waiting).decode("utf-8", errors="ignore")
        except UnicodeDecodeError:
            return None

        recv_buffer += data

        if "#" in recv_buffer:
            msg, recv_buffer = recv_buffer.split("#", 1)
            msg = msg.strip() + "#"
            if PRINT_RAW:
                print(f"[RX_RAW] {msg}")
            return msg

    return None 
    


def send_upload_command(ser, mode: int):
    """
    모터드라이버에 어떤 데이터를 올려달라고 요청하는 명령
    0: 없음, 1: 총 엔코더, 2: 실시간 엔코더, 3: 속도
    """
    if mode == 0:
        cmd = "$upload:0,0,0#"
    elif mode == 1:
        cmd = "$upload:1,0,0#"
    elif mode == 2:
        cmd = "$upload:0,1,0#"
    elif mode == 3:
        cmd = "$upload:0,0,1#"
    else:
        cmd = "$upload:0,0,0#"

    send_data(ser, cmd)
    print(f"[INFO] upload 모드 설정: {mode} (cmd: {cmd.strip()})")


def parse_mspd(msg: str):
    """
    $MSPD: 값들을 파싱해서 [M1, M2, M3, M4] 리스트로 반환.
    파싱 실패 시 None
    """
    if not msg.startswith("$MSPD:"):
        return None

    body = msg[len("$MSPD:"):-1]  # "$MSPD:"와 맨 끝 "#" 제거
    parts = body.split(",")

    values = []
    for v in parts:
        v = v.strip()
        if not v:
            continue
        if "." in v:
            try:
                values.append(float(v))
            except ValueError:
                values.append(0.0)
        else:
            try:
                values.append(int(v))
            except ValueError:
                values.append(0)

    if len(values) == 0:
        return None

    # 모터가 4채널이 아니면 길이가 다를 수도 있지만, 기본은 4로 가정
    return values


def print_mspd(values):
    """M1: v1, M2: v2 ... 형식으로 출력"""
    text = ", ".join(f"M{i+1}: {v}" for i, v in enumerate(values))
    print(f"[MSPD] {text}")


def control_speed(ser, m1, m2, m3, m4):
    """속도 제어 명령 전송"""
    cmd = f"$spd:{m1},{m2},{m3},{m4}#"
    send_data(ser, cmd)


def stop_all(ser):
    """모든 모터 정지"""
    print("[INFO] 모든 모터 정지")
    control_speed(ser, 0, 0, 0, 0)


# ===================== 테스트 시퀀스 =====================

def comm_test(ser, duration=2.0):
    """
    1단계: 통신 및 $MSPD 수신 테스트
    duration 동안 $MSPD를 읽어보며 정상 수신되는지 확인
    """
    print("\n[TEST 1] 통신 + $MSPD 수신 테스트")
    print("  - 몇 초 동안 수신되는 속도 데이터를 출력합니다.")
    print("  - 아직 모터에는 속도 명령을 보내지 않습니다.\n")

    start = time.time()
    while time.time() - start < duration:
        msg = read_message(ser)
        if msg is None:
            continue

        if msg.startswith("$MSPD:"):
            values = parse_mspd(msg)
            if values is not None:
                print_mspd(values)

    print("[TEST 1] 종료\n")


def encoder_manual_test(ser):
    """
    2단계: 엔코더 수동 테스트
    - 모터를 돌리지 않고 사용자가 손으로 바퀴를 돌리면서 M1~M4 값 변화를 확인
    """
    print("\n[TEST 2] 엔코더 수동 테스트")
    print("  1) 지금은 모터에 명령을 보내지 않습니다.")
    print("  2) 각 바퀴를 손으로 천천히 돌려보세요.")
    print("  3) 어떤 바퀴를 돌릴 때 콘솔의 M1~M4 값이 변하는지 확인해서")
    print("     '어느 바퀴 ↔ 어느 채널(엔코더)' 연결인지 체크하세요.")
    input("준비되면 Enter 키를 누르면 시작합니다...")

    print("[INFO] 엔코더 수동 테스트 시작 (종료하려면 Ctrl+C)...\n")

    try:
        while True:
            msg = read_message(ser)
            if msg is None:
                continue
            if msg.startswith("$MSPD:"):
                values = parse_mspd(msg)
                if values is not None:
                    print_mspd(values)
    except KeyboardInterrupt:
        print("\n[TEST 2] 사용자 종료\n")


def motor_channel_test(ser, speed=TEST_SPEED, duration=10.0):
    """
    3단계: 모터 채널별 동작 테스트
    - M1 → M2 → M3 → M4 순서로 하나씩만 돌려봄
    - 각 단계별로 엔코더 값이 해당 채널에서 변화하는지 확인
    """
    print("\n[TEST 3] 모터 채널별 동작 테스트")
    print("  - 순서대로 M1, M2, M3, M4만 각각 돌려봅니다.")
    print("  - 각 단계에서 '지금 어느 바퀴가 돌아가는지' + '어느 채널 값이 변하는지'")
    print("    눈으로 확인해서 M+/M-, 엔코더 채널이 올바른지 체크하세요.\n")

    for idx in range(4):
        input(f"[단계 {idx+1}] M{idx+1}만 테스트합니다. 준비되면 Enter를 누르세요...")

        vals = [0, 0, 0, 0]
        vals[idx] = speed

        print(f"[INFO] M{idx+1}에 속도 {speed} 명령 전송")
        control_speed(ser, vals[0], vals[1], vals[2], vals[3])

        start = time.time()
        try:
            while time.time() - start < duration:
                msg = read_message(ser)
                if msg is None:
                    continue
                if msg.startswith("$MSPD:"):
                    values = parse_mspd(msg)
                    if values is not None:
                        print_mspd(values)
        except KeyboardInterrupt:
            print("\n[INFO] 사용자에 의해 중단됨")
            break

        print(f"[INFO] M{idx+1} 테스트 종료, 모터 정지")
        stop_all(ser)
        time.sleep(1.0)

    print("\n[TEST 3] 전체 모터 채널 테스트 종료\n")


# ===================== 메인 =====================

def main():
    ser = None
    try:
        ser = open_serial()

        # 1) 어떤 데이터 받을지 설정 (여기서는 속도만)
        send_upload_command(ser, UPLOAD_MODE)

        # 2) 통신 테스트
        comm_test(ser, duration=3.0)

        # 3) 엔코더 수동 테스트
        encoder_manual_test(ser)

        # 4) 모터 채널별 테스트
        motor_channel_test(ser, speed=TEST_SPEED, duration=10.0)

    except KeyboardInterrupt:
        print("\n[MAIN] 사용자 종료 요청")
    except Exception as e:
        print(f"[ERROR] 예외 발생: {e}")
    finally:
        if ser is not None and ser.is_open:
            stop_all(ser)
            ser.close()
            print("[INFO] 시리얼 포트 닫힘")


if __name__ == "__main__":
    main()
