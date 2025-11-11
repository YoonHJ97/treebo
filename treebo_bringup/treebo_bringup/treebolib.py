#!/usr/bin/env python3
# coding: utf-8
"""
treebolib.py

Yahboom Rosmaster_Lib(V3.3.9)를 기반으로,
- 주행/IMU/엔코더/배터리 등 자율주행에 필요한 부분만 남기고
- 팔/서보/LED/버저/아크만 등은 제거한 경량 버전입니다.

기본 사용 예:
    from treebolib import TreeboLib

    bot = TreeboLib(car_type=1, com="/dev/ttyUSB0", debug=True)
    bot.create_receive_threading()
    bot.set_auto_report_state(True, forever=False)
    bot.set_car_motion(0.2, 0.0, 0.0)  # 전진

"""

import struct
import time
import serial
import threading


class TreeboLib(object):
    __uart_state = 0

    def __init__(self, car_type=1, com="/dev/myserial", delay=0.002, debug=False):
        """
        :param car_type: X3=1, X3_PLUS=2, X1=4, R2=5 (원본 펌웨어 기준)
        :param com: 시리얼 포트 (예: "/dev/ttyUSB0" 또는 udev 심볼릭 링크)
        :param delay: 명령 간 지연 시간 (초)
        :param debug: True이면 보낸 패킷을 출력
        """
        self.ser = serial.Serial(com, 115200)

        self.__delay_time = delay
        self.__debug = debug

        # 프로토콜 기본 값
        self.__HEAD = 0xFF
        self.__DEVICE_ID = 0xFC
        self.__COMPLEMENT = 257 - self.__DEVICE_ID
        self.__CAR_TYPE = car_type
        self.__CAR_ADJUST = 0x80

        # 기능 코드(주행/센서 관련만 남김)
        self.FUNC_AUTO_REPORT = 0x01

        self.FUNC_REPORT_SPEED = 0x0A
        self.FUNC_REPORT_MPU_RAW = 0x0B
        self.FUNC_REPORT_IMU_ATT = 0x0C
        self.FUNC_REPORT_ENCODER = 0x0D
        self.FUNC_REPORT_ICM_RAW = 0x0E

        self.FUNC_RESET_STATE = 0x0F

        self.FUNC_MOTOR = 0x10
        self.FUNC_CAR_RUN = 0x11
        self.FUNC_MOTION = 0x12
        self.FUNC_SET_MOTOR_PID = 0x13
        self.FUNC_SET_CAR_TYPE = 0x15

        self.FUNC_REQUEST_DATA = 0x50
        self.FUNC_VERSION = 0x51

        self.FUNC_RESET_FLASH = 0xA0

        # 카 타입 상수 (참고용)
        self.CARTYPE_X3 = 0x01
        self.CARTYPE_X3_PLUS = 0x02
        self.CARTYPE_X1 = 0x04
        self.CARTYPE_R2 = 0x05

        # 센서 및 상태 변수
        self.__ax = 0.0
        self.__ay = 0.0
        self.__az = 0.0
        self.__gx = 0.0
        self.__gy = 0.0
        self.__gz = 0.0
        self.__mx = 0.0
        self.__my = 0.0
        self.__mz = 0.0
        self.__vx = 0.0
        self.__vy = 0.0
        self.__vz = 0.0

        self.__yaw = 0.0
        self.__roll = 0.0
        self.__pitch = 0.0

        self.__encoder_m1 = 0
        self.__encoder_m2 = 0
        self.__encoder_m3 = 0
        self.__encoder_m4 = 0

        self.__version_H = 0
        self.__version_L = 0
        self.__version = 0

        self.__pid_index = 0
        self.__kp1 = 0
        self.__ki1 = 0
        self.__kd1 = 0

        self.__battery_voltage = 0

        self.__read_car_type = 0

        if self.__debug:
            print("cmd_delay=" + str(self.__delay_time) + "s")

        if self.ser.isOpen():
            print("TreeboLib Serial Opened! Baudrate=115200")
        else:
            print("Serial Open Failed!")

    def __del__(self):
        try:
            self.ser.close()
        except Exception:
            pass
        self.__uart_state = 0
        if self.__debug:
            print("serial Close!")

    # ------------------------------------------------------------
    # 내부: 데이터 파싱
    # ------------------------------------------------------------
    def __parse_data(self, ext_type, ext_data):
        """수신 패킷의 기능 코드에 따라 내부 상태 업데이트"""
        if ext_type == self.FUNC_REPORT_SPEED:
            # vx, vy, vz [m/s], 배터리 전압(1/10 V)
            self.__vx = int(struct.unpack('h', bytearray(ext_data[0:2]))[0]) / 1000.0
            self.__vy = int(struct.unpack('h', bytearray(ext_data[2:4]))[0]) / 1000.0
            self.__vz = int(struct.unpack('h', bytearray(ext_data[4:6]))[0]) / 1000.0
            self.__battery_voltage = struct.unpack('B', bytearray(ext_data[6:7]))[0]

        elif ext_type == self.FUNC_REPORT_MPU_RAW:
            # MPU9250 원시 데이터
            gyro_ratio = 1 / 3754.9  # ±500dps
            self.__gx = struct.unpack('h', bytearray(ext_data[0:2]))[0] * gyro_ratio
            self.__gy = struct.unpack('h', bytearray(ext_data[2:4]))[0] * -gyro_ratio
            self.__gz = struct.unpack('h', bytearray(ext_data[4:6]))[0] * -gyro_ratio

            accel_ratio = 1 / 1671.84
            self.__ax = struct.unpack('h', bytearray(ext_data[6:8]))[0] * accel_ratio
            self.__ay = struct.unpack('h', bytearray(ext_data[8:10]))[0] * accel_ratio
            self.__az = struct.unpack('h', bytearray(ext_data[10:12]))[0] * accel_ratio

            mag_ratio = 1.0
            self.__mx = struct.unpack('h', bytearray(ext_data[12:14]))[0] * mag_ratio
            self.__my = struct.unpack('h', bytearray(ext_data[14:16]))[0] * mag_ratio
            self.__mz = struct.unpack('h', bytearray(ext_data[16:18]))[0] * mag_ratio

        elif ext_type == self.FUNC_REPORT_ICM_RAW:
            # ICM20948 원시 데이터
            gyro_ratio = 1 / 1000.0
            self.__gx = struct.unpack('h', bytearray(ext_data[0:2]))[0] * gyro_ratio
            self.__gy = struct.unpack('h', bytearray(ext_data[2:4]))[0] * gyro_ratio
            self.__gz = struct.unpack('h', bytearray(ext_data[4:6]))[0] * gyro_ratio

            accel_ratio = 1 / 1000.0
            self.__ax = struct.unpack('h', bytearray(ext_data[6:8]))[0] * accel_ratio
            self.__ay = struct.unpack('h', bytearray(ext_data[8:10]))[0] * accel_ratio
            self.__az = struct.unpack('h', bytearray(ext_data[10:12]))[0] * accel_ratio

            mag_ratio = 1 / 1000.0
            self.__mx = struct.unpack('h', bytearray(ext_data[12:14]))[0] * mag_ratio
            self.__my = struct.unpack('h', bytearray(ext_data[14:16]))[0] * mag_ratio
            self.__mz = struct.unpack('h', bytearray(ext_data[16:18]))[0] * mag_ratio

        elif ext_type == self.FUNC_REPORT_IMU_ATT:
            # 보드 자세각(라디안 단위)
            self.__roll = struct.unpack('h', bytearray(ext_data[0:2]))[0] / 10000.0
            self.__pitch = struct.unpack('h', bytearray(ext_data[2:4]))[0] / 10000.0
            self.__yaw = struct.unpack('h', bytearray(ext_data[4:6]))[0] / 10000.0

        elif ext_type == self.FUNC_REPORT_ENCODER:
            # 4개 바퀴 엔코더
            self.__encoder_m1 = struct.unpack('i', bytearray(ext_data[0:4]))[0]
            self.__encoder_m2 = struct.unpack('i', bytearray(ext_data[4:8]))[0]
            self.__encoder_m3 = struct.unpack('i', bytearray(ext_data[8:12]))[0]
            self.__encoder_m4 = struct.unpack('i', bytearray(ext_data[12:16]))[0]

        elif ext_type == self.FUNC_SET_MOTOR_PID:
            self.__pid_index = struct.unpack('B', bytearray(ext_data[0:1]))[0]
            self.__kp1 = struct.unpack('h', bytearray(ext_data[1:3]))[0]
            self.__ki1 = struct.unpack('h', bytearray(ext_data[3:5]))[0]
            self.__kd1 = struct.unpack('h', bytearray(ext_data[5:7]))[0]

        elif ext_type == self.FUNC_SET_CAR_TYPE:
            car_type = struct.unpack('B', bytearray(ext_data[0:1]))[0]
            self.__read_car_type = car_type

        elif ext_type == self.FUNC_VERSION:
            self.__version_H = struct.unpack('B', bytearray(ext_data[0:1]))[0]
            self.__version_L = struct.unpack('B', bytearray(ext_data[1:2]))[0]

    # ------------------------------------------------------------
    # 내부: 수신 스레드
    # ------------------------------------------------------------
    def __receive_data(self):
        """시리얼에서 계속 데이터를 읽어서 __parse_data에 넘김"""
        self.ser.flushInput()
        while True:
            head1 = bytearray(self.ser.read())[0]
            if head1 == self.__HEAD:
                head2 = bytearray(self.ser.read())[0]
                check_sum = 0
                rx_check_num = 0
                if head2 == self.__DEVICE_ID - 1:
                    ext_len = bytearray(self.ser.read())[0]
                    ext_type = bytearray(self.ser.read())[0]
                    ext_data = []
                    check_sum = ext_len + ext_type
                    data_len = ext_len - 2
                    while len(ext_data) < data_len:
                        value = bytearray(self.ser.read())[0]
                        ext_data.append(value)
                        if len(ext_data) == data_len:
                            rx_check_num = value
                        else:
                            check_sum = check_sum + value
                    if check_sum % 256 == rx_check_num:
                        self.__parse_data(ext_type, ext_data)
                    else:
                        if self.__debug:
                            print("check sum error:", ext_len, ext_type, ext_data)

    def __request_data(self, function, param=0):
        """MCU에 특정 기능의 데이터 요청"""
        cmd = [self.__HEAD, self.__DEVICE_ID, 0x05,
               self.FUNC_REQUEST_DATA, int(function) & 0xff, int(param) & 0xff]
        checksum = sum(cmd, self.__COMPLEMENT) & 0xff
        cmd.append(checksum)
        self.ser.write(cmd)
        if self.__debug:
            print("request:", cmd)
        time.sleep(0.002)

    def __limit_motor_value(self, value):
        """모터 PWM 값 제한 [-100, 100], 127은 유지 명령"""
        if value == 127:
            return 127
        elif value > 100:
            return 100
        elif value < -100:
            return -100
        else:
            return int(value)

    # ------------------------------------------------------------
    # 외부 API: 스레드/리포트 설정
    # ------------------------------------------------------------
    def create_receive_threading(self):
        """수신 스레드 시작 (자동 리포트 데이터 처리용)"""
        try:
            if self.__uart_state == 0:
                name1 = "task_serial_receive"
                task_receive = threading.Thread(target=self.__receive_data, name=name1)
                task_receive.setDaemon(True)
                task_receive.start()
                print("----------------create receive threading--------------")
                self.__uart_state = 1
                time.sleep(0.05)
        except Exception:
            print('---create_receive_threading error!---')

    def set_auto_report_state(self, enable, forever=False):
        """
        자동 리포트(속도/IMU/엔코더 등) ON/OFF

        enable=True  : 보드가 주기적으로 데이터를 보냄
        enable=False : 자동 리포트 중지
        forever=True : 플래시 저장(지원하는 펌웨어에서만)
        """
        try:
            state1 = 1 if enable else 0
            state2 = 0x5F if forever else 0
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x05,
                   self.FUNC_AUTO_REPORT, state1, state2]
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("report:", cmd)
            time.sleep(self.__delay_time)
        except Exception:
            print('---set_auto_report_state error!---')

    # ------------------------------------------------------------
    # 외부 API: 주행 제어
    # ------------------------------------------------------------
    def set_motor(self, speed_1, speed_2, speed_3, speed_4):
        """
        PWM 직접 제어 (인코더 사용 X)
        speed_n: [-100, 100]
        """
        try:
            t_speed_a = bytearray(struct.pack('b', self.__limit_motor_value(speed_1)))
            t_speed_b = bytearray(struct.pack('b', self.__limit_motor_value(speed_2)))
            t_speed_c = bytearray(struct.pack('b', self.__limit_motor_value(speed_3)))
            t_speed_d = bytearray(struct.pack('b', self.__limit_motor_value(speed_4)))
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_MOTOR,
                   t_speed_a[0], t_speed_b[0], t_speed_c[0], t_speed_d[0]]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("motor:", cmd)
            time.sleep(self.__delay_time)
        except Exception:
            print('---set_motor error!---')

    def set_car_run(self, state, speed, adjust=False):
        """
        단순 상태 기반 주행 (잘 쓰지는 않음)
        state: 0=정지, 1=전진, 2=후진, 3=좌, 4=우, 5=좌회전, 6=우회전
        speed: [-100, 100]
        """
        try:
            car_type = self.__CAR_TYPE
            if adjust:
                car_type = car_type | self.__CAR_ADJUST
            t_speed = bytearray(struct.pack('h', int(speed)))
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_CAR_RUN,
                   car_type, int(state & 0xff), t_speed[0], t_speed[1]]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("car_run:", cmd)
            time.sleep(self.__delay_time)
        except Exception:
            print('---set_car_run error!---')

    def set_car_motion(self, v_x, v_y, v_z):
        '''
        输入范围 input range: 
        X3: v_x=[-1.0, 1.0], v_y=[-1.0, 1.0], v_z=[-5, 5]
        X3PLUS: v_x=[-0.7, 0.7], v_y=[-0.7, 0.7], v_z=[-3.2, 3.2]
        R2/R2L: v_x=[-1.8, 1.8], v_y=[-0.045, 0.045], v_z=[-3, 3]

        - ROS에서 들어오는 v_x 를 "전후진"으로 사용하고,
          보드에서는 v_y 축에 매핑한다.
        - 원래 v_y 입력은 사용하지 않는다(측면 이동 X).
        - 회전(v_z)은 방향이 반대로 동작하고 있으므로 부호를 뒤집어서 보낸다.
        '''
        try:
            # ---- Treebo용 축 재매핑 ----
            # ROS 입력
            ros_vx = v_x       # 전/후진으로 쓰고 싶은 축
            ros_vy = v_y       # (사용 안 함)
            ros_vz = v_z       # 회전

            # 보드에 보낼 값
            board_vx = 0.0          # x축은 아예 사용하지 않음
            board_vy = ros_vx       # x 입력을 y축 속도로 사용
            board_vz = -ros_vz      # 회전 방향 반전

            # 패킷 변환
            vx_parms = bytearray(struct.pack('h', int(board_vx * 1000)))
            vy_parms = bytearray(struct.pack('h', int(board_vy * 1000)))
            vz_parms = bytearray(struct.pack('h', int(board_vz * 1000)))

            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_MOTION, self.__CAR_TYPE,
                   vx_parms[0], vx_parms[1],
                   vy_parms[0], vy_parms[1],
                   vz_parms[0], vz_parms[1]]

            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)

            if self.__debug:
                print("motion:", cmd)

            time.sleep(self.__delay_time)

        except Exception:
            print('---set_car_motion error!---')
            pass

    def set_pid_param(self, kp, ki, kd, forever=False):
        """
        모션 PID 설정 (set_car_motion의 속도 응답에 영향)
        kp, ki, kd: [0, 10.0]
        forever=True: flash 저장
        """
        try:
            state = 0x5F if forever else 0
            if kp > 10 or ki > 10 or kd > 10 or kp < 0 or ki < 0 or kd < 0:
                print("PID value must be:[0, 10.00]")
                return
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x0A, self.FUNC_SET_MOTOR_PID]
            kp_params = bytearray(struct.pack('h', int(kp * 1000)))
            ki_params = bytearray(struct.pack('h', int(ki * 1000)))
            kd_params = bytearray(struct.pack('h', int(kd * 1000)))
            cmd.extend([kp_params[0], kp_params[1],
                        ki_params[0], ki_params[1],
                        kd_params[0], kd_params[1],
                        state])
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("pid:", cmd)
            time.sleep(self.__delay_time)
            if forever:
                time.sleep(0.1)
        except Exception:
            print('---set_pid_param error!---')

    def set_car_type(self, car_type):
        """보드에 카 타입 설정 (X3/X1/R2 등)"""
        if str(car_type).isdigit():
            self.__CAR_TYPE = int(car_type) & 0xff
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00,
                   self.FUNC_SET_CAR_TYPE, self.__CAR_TYPE, 0x5F]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("car_type:", cmd)
            time.sleep(0.1)
        else:
            print("set_car_type input invalid")

    # ------------------------------------------------------------
    # 외부 API: 상태 리셋/클리어
    # ------------------------------------------------------------
    def reset_flash_value(self):
        """보드에 저장된 설정(플래시) 초기화 (공장 출하시 값으로)"""
        try:
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x04,
                   self.FUNC_RESET_FLASH, 0x5F]
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("flash:", cmd)
            time.sleep(self.__delay_time)
            time.sleep(0.1)
        except Exception:
            print('---reset_flash_value error!---')

    def reset_car_state(self):
        """차 상태 리셋 (정지/LED/버저 OFF 등)"""
        try:
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x04,
                   self.FUNC_RESET_STATE, 0x5F]
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("reset_car_state:", cmd)
            time.sleep(self.__delay_time)
        except Exception:
            print('---reset_car_state error!---')

    def clear_auto_report_data(self):
        """내부 캐시된 센서 데이터 초기화 (변수만 0으로)"""
        self.__battery_voltage = 0
        self.__ax = 0.0
        self.__ay = 0.0
        self.__az = 0.0
        self.__gx = 0.0
        self.__gy = 0.0
        self.__gz = 0.0
        self.__mx = 0.0
        self.__my = 0.0
        self.__mz = 0.0
        self.__vx = 0.0
        self.__vy = 0.0
        self.__vz = 0.0
        self.__yaw = 0.0
        self.__roll = 0.0
        self.__pitch = 0.0

    # ------------------------------------------------------------
    # 외부 API: 센서/상태 읽기
    # ------------------------------------------------------------
    def get_accelerometer_data(self):
        """가속도계 (ax, ay, az) [m/s^2]"""
        return self.__ax, self.__ay, self.__az

    def get_gyroscope_data(self):
        """자이로 (gx, gy, gz) [rad/s]"""
        return self.__gx, self.__gy, self.__gz

    def get_magnetometer_data(self):
        """자력계 (mx, my, mz)"""
        return self.__mx, self.__my, self.__mz

    def get_imu_attitude_data(self, ToAngle=True):
        """
        보드 자세 (roll, pitch, yaw)
        ToAngle=True : degree 단위
        ToAngle=False: radian 단위
        """
        if ToAngle:
            RtA = 57.2957795
            roll = self.__roll * RtA
            pitch = self.__pitch * RtA
            yaw = self.__yaw * RtA
        else:
            roll, pitch, yaw = self.__roll, self.__pitch, self.__yaw
        return roll, pitch, yaw

    def get_motion_data(self):
        """속도 (vx, vy, vz) [m/s, m/s, rad/s]"""
        return self.__vx, self.__vy, self.__vz

    def get_battery_voltage(self):
        """배터리 전압 [V]"""
        return self.__battery_voltage / 10.0

    def get_motor_encoder(self):
        """4바퀴 엔코더 카운트 (m1, m2, m3, m4)"""
        return self.__encoder_m1, self.__encoder_m2, self.__encoder_m3, self.__encoder_m4

    def get_motion_pid(self):
        """모션 PID [kp, ki, kd] 읽기"""
        self.__kp1 = 0
        self.__ki1 = 0
        self.__kd1 = 0
        self.__pid_index = 0
        self.__request_data(self.FUNC_SET_MOTOR_PID, int(1))
        for _ in range(20):
            if self.__pid_index > 0:
                kp = float(self.__kp1 / 1000.0)
                ki = float(self.__ki1 / 1000.0)
                kd = float(self.__kd1 / 1000.0)
                if self.__debug:
                    print("get_motion_pid:", self.__pid_index, [kp, ki, kd])
                return [kp, ki, kd]
            time.sleep(0.001)
        return [-1, -1, -1]

    def get_car_type_from_machine(self):
        """보드에 저장된 현재 car_type 읽기"""
        self.__request_data(self.FUNC_SET_CAR_TYPE)
        for _ in range(20):
            if self.__read_car_type != 0:
                car_type = self.__read_car_type
                self.__read_car_type = 0
                return car_type
            time.sleep(0.001)
        return -1

    def get_version(self):
        """펌웨어 버전 (예: 3.3)"""
        if self.__version_H == 0:
            self.__request_data(self.FUNC_VERSION)
            for _ in range(20):
                if self.__version_H != 0:
                    val = self.__version_H * 1.0
                    self.__version = val + self.__version_L / 10.0
                    if self.__debug:
                        print("get_version: V", self.__version)
                    return self.__version
                time.sleep(0.001)
        else:
            return self.__version
        return -1
