import serial
import time

PORT = "/dev/ttyACM0"   # Windows면 "COM5" 같은 형태
BAUD = 115200

def send(ser, v1, v2, v3, v4):
    cmd = f"m {v1} {v2} {v3} {v4}\n"
    ser.write(cmd.encode("ascii"))
    # print(cmd.strip())  # 필요하면 주석 해제

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.2)
    time.sleep(2.0)  # 보드 리셋/시리얼 안정화 대기

    # 1) 정지
    send(ser, 0, 0, 0, 0)
    time.sleep(1.0)

    # 2) M1만 천천히 램프업/다운
    for v in range(0, 2001, 200):   # 0 -> 400
        send(ser, v, 0, 0, 0)
        time.sleep(2.5)

    for v in range(2000, -2001, -200):  # 400 -> -400
        send(ser, v, 0, 0, 0)
        time.sleep(2.5)

    for v in range(-2000, 1, 200):  # -400 -> 0
        send(ser, v, 0, 0, 0)
        time.sleep(2.5)

    # 3) 4개 동시에
    for v in [100, 200, 300, 0, -200, 0]:
        send(ser, v, v, v, v)
        time.sleep(1.0)

    # 4) 정지
    send(ser, 0, 0, 0, 0)
    time.sleep(0.5)
    ser.close()

if __name__ == "__main__":
    main()
