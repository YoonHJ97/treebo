#!/usr/bin/env python3
# encoding: utf-8

"""
treebo_bringup / bringup.py

- /cmd_vel 구독 → Rosmaster(또는 Treebo 보드)에 속도 명령 전달
- /vel_raw, /imu/data_raw, /imu/mag, /voltage, /edition, /joint_states, /encoder_raw 퍼블리시

Yahboom X3용 Mcnamu_driver_X3.py를 기반으로
Treebo 전용으로 간소화 + encoder_raw 추가한 버전입니다.
"""

from math import pi

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, MagneticField, JointState
from std_msgs.msg import Float32, Int32MultiArray

from treebo_bringup.treebolib import TreeboLib  # 네가 수정한 라이브러리 import


CAR_TYPE_DICT = {
    "R2": 5,
    "X3": 1,
    "NONE": -1,
}


class TreeboBringup(Node):
    def __init__(self) -> None:
        super().__init__("treebo_bringup")

        self.RA2DE = 180.0 / pi

        # ---------------- Rosmaster / Treebo 보드 연결 ----------------
        self.car = TreeboLib()

        # ---------------- 파라미터 ----------------
        # 기본 차종: X3 (메카넘)
        self.declare_parameter("car_type", "X3")
        self.car_type = (
            self.get_parameter("car_type").get_parameter_value().string_value
        )
        self.get_logger().info(f"car_type = {self.car_type}")

        # car_type 문자열을 보드 타입 ID로 매핑
        car_type_key = self.car_type.upper()
        car_type_id = CAR_TYPE_DICT.get(car_type_key, -1)
        if car_type_id > 0:
            self.car.set_car_type(car_type_id)
            self.get_logger().info(f"Set car_type ID = {car_type_id}")
        else:
            # 유효하지 않은 값이면 기본값(X3)로 사용
            self.car.set_car_type(CAR_TYPE_DICT["X3"])
            self.get_logger().warn(
                f"Unknown car_type '{self.car_type}', fallback to X3 (ID={CAR_TYPE_DICT['X3']})"
            )

        self.declare_parameter("imu_link", "imu_link")
        self.imu_link = (
            self.get_parameter("imu_link").get_parameter_value().string_value
        )
        self.get_logger().info(f"imu_link = {self.imu_link}")

        self.declare_parameter("Prefix", "")
        self.prefix = self.get_parameter("Prefix").get_parameter_value().string_value
        self.get_logger().info(f"Prefix = '{self.prefix}'")

        self.declare_parameter("xlinear_limit", 1.0)
        self.xlinear_limit = (
            self.get_parameter("xlinear_limit").get_parameter_value().double_value
        )

        self.declare_parameter("ylinear_limit", 1.0)
        self.ylinear_limit = (
            self.get_parameter("ylinear_limit").get_parameter_value().double_value
        )

        self.declare_parameter("angular_limit", 5.0)
        self.angular_limit = (
            self.get_parameter("angular_limit").get_parameter_value().double_value
        )

        # ---------------- 통신 설정 ----------------
        # /cmd_vel: 키보드 텔레옵, 네비게이션 스택 등에서 들어오는 속도 명령
        self.cmd_vel_sub = self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, 10
        )

        # 퍼블리셔들
        self.edition_pub = self.create_publisher(Float32, "edition", 10)
        self.voltage_pub = self.create_publisher(Float32, "voltage", 10)
        self.joint_state_pub = self.create_publisher(JointState, "joint_states", 10)
        self.vel_pub = self.create_publisher(Twist, "vel_raw", 50)
        self.imu_pub = self.create_publisher(Imu, "/imu/data_raw", 50)
        self.mag_pub = self.create_publisher(MagneticField, "/imu/mag", 50)

        # ✅ 인코더 raw 퍼블리셔 추가 (m1, m2, m3, m4)
        self.encoder_pub = self.create_publisher(
            Int32MultiArray, "encoder_raw", 10
        )

        # 10Hz로 센서/속도 퍼블리시
        self.timer = self.create_timer(0.1, self.pub_data)

        # 보드 수신 스레드 시작 (MCU → PC 데이터 수신)
        self.car.create_receive_threading()

        self.get_logger().info("TreeboBringup node started.")

    # --------------------------------------------------
    # /cmd_vel 콜백: ROS 속도 명령 → 보드로 전달
    # --------------------------------------------------
    def cmd_vel_callback(self, msg: Twist) -> None:
        if not isinstance(msg, Twist):
            return

        # 입력 속도 제한 (파라미터 기준)
        vx = max(min(msg.linear.x, self.xlinear_limit), -self.xlinear_limit)
        vy = max(min(msg.linear.y, self.ylinear_limit), -self.ylinear_limit)
        wz = max(min(msg.angular.z, self.angular_limit), -self.angular_limit)

        # 여기서는 "표준 ROS 축" 그대로 라이브러리에 넘긴다.
        # (treebolib.TreeboLib.set_car_motion 안에서
        #  board_vx = 0.0, board_vy = vx, board_vz = -wz 등으로 재매핑해 둔 상태라고 가정)
        self.car.set_car_motion(vx, vy, wz)

    # --------------------------------------------------
    # 주기적으로 센서/속도/인코더 데이터를 퍼블리시
    # --------------------------------------------------
    def pub_data(self) -> None:
        time_stamp = Clock().now()

        imu_msg = Imu()
        twist_msg = Twist()
        battery_msg = Float32()
        edition_msg = Float32()
        mag_msg = MagneticField()
        state_msg = JointState()
        enc_msg = Int32MultiArray()

        state_msg.header.stamp = time_stamp.to_msg()
        state_msg.header.frame_id = "joint_states"

        if len(self.prefix) == 0:
            state_msg.name = [
                "back_right_joint",
                "back_left_joint",
                "front_left_steer_joint",
                "front_left_wheel_joint",
                "front_right_steer_joint",
                "front_right_wheel_joint",
            ]
        else:
            p = self.prefix
            state_msg.name = [
                p + "back_right_joint",
                p + "back_left_joint",
                p + "front_left_steer_joint",
                p + "front_left_wheel_joint",
                p + "front_right_steer_joint",
                p + "front_right_wheel_joint",
            ]

        # 보드에서 버전 / 배터리 / IMU / 자기장 / 속도 / 인코더 읽기
        edition_msg.data = float(self.car.get_version())
        battery_msg.data = float(self.car.get_battery_voltage())

        ax, ay, az = self.car.get_accelerometer_data()
        gx, gy, gz = self.car.get_gyroscope_data()
        mx, my, mz = self.car.get_magnetometer_data()
        vx, vy, angular = self.car.get_motion_data()

        # ✅ 인코더 raw 값 (4개 모터)
        try:
            m1, m2, m3, m4 = self.car.get_motor_encoder()
            enc_msg.data = [int(m1), int(m2), int(m3), int(m4)]
        except Exception as e:
            # 인코더가 아직 준비 안 되었거나 에러일 경우, 로그만 찍고 건너뜀
            self.get_logger().debug(f"Failed to read motor encoder: {e}")
            enc_msg.data = []

        # IMU 메시지
        imu_msg.header.stamp = time_stamp.to_msg()
        imu_msg.header.frame_id = self.imu_link
        imu_msg.linear_acceleration.x = float(ax)
        imu_msg.linear_acceleration.y = float(ay)
        imu_msg.linear_acceleration.z = float(az)
        imu_msg.angular_velocity.x = float(gx)
        imu_msg.angular_velocity.y = float(gy)
        imu_msg.angular_velocity.z = float(gz)

        # 자기장 메시지
        mag_msg.header.stamp = time_stamp.to_msg()
        mag_msg.header.frame_id = self.imu_link
        mag_msg.magnetic_field.x = float(mx)
        mag_msg.magnetic_field.y = float(my)
        mag_msg.magnetic_field.z = float(mz)

        # 속도 메시지 (보드가 보고하는 실제 속도)
        # 여기서는 보드 좌표계를 ROS 좌표계로 맞추기 위해
        # vx, vy를 스왑해서 사용 (기존 Mcnamu_driver_X3.py와 동일한 형태)
        twist_msg.linear.x = float(vy)
        twist_msg.linear.y = float(vx)
        twist_msg.angular.z = float(angular)

        # 퍼블리시
        self.imu_pub.publish(imu_msg)
        self.mag_pub.publish(mag_msg)
        self.vel_pub.publish(twist_msg)
        self.voltage_pub.publish(battery_msg)
        self.edition_pub.publish(edition_msg)
        self.joint_state_pub.publish(state_msg)

        # ✅ 인코더 카운트 퍼블리시
        if enc_msg.data:
            self.encoder_pub.publish(enc_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TreeboBringup()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
