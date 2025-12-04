import serial
import time

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)

def send_cmd(line: str):
    msg = (line + '\n').encode('utf-8')
    ser.write(msg)
    ser.flush()
    print(f"TX: {line}")
    time.sleep(0.05)

try:
    while True:
        # 예: 뒤로 0.5 초 정도 굴리기
        send_cmd("-500 0 0 0")
        time.sleep(0.5)
        send_cmd("0 0 0 0")
        time.sleep(0.5)

        # 수신된 로그 프린트 (STM32가 보내는 SPD OK, PARSED 등)
        data = ser.read(1024)
        if data:
            print("RX:", data.decode(errors='ignore'), end='')

except KeyboardInterrupt:
    ser.close()
