#!/usr/bin/env python3
# encoding: utf-8

"""
odom_publisher.py

- 구독:
  - encoder_raw (std_msgs/Int32MultiArray) : m1, m2, m3, m4 인코더 누적 카운트

- 발행:
  - /odom (nav_msgs/Odometry)
  - TF: odom -> base_link

전제:
- 4륜 메카넘 베이스
- 바퀴 반지름 R, 베이스 길이 L, 너비 W, 인코더 ticks_per_rev 를 파라미터로 받음
- encoder_raw 에 들어오는 m1~m4는 바퀴 순서 [FL, FR, RL, RR] 라고 가정
  (필요하면 mapping/부호는 encoder_signs 파라미터로 조정)
"""

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster


class EncoderOdomNode(Node):
    def __init__(self):
        super().__init__('encoder_odom_node')

        # ---------------- 파라미터 ----------------
        # 바퀴 반지름 (m)
        self.declare_parameter('wheel_radius', 0.04)   # 40mm
        # 베이스 길이/너비 (m)
        self.declare_parameter('base_length', 0.095)   # 95mm
        self.declare_parameter('base_width', 0.12)     # 120mm
        # 인코더 해상도 (counts / 바퀴 1rev)
        self.declare_parameter('ticks_per_rev', 4320)

        # 인코더 부호 보정용 (전방 +방향으로 맞추기 위함)
        # 기본값은 [1, 1, 1, 1] 이고,
        # 실험 결과에 따라 예: [-1, 1, 1, -1] 로 설정해서 사용
        self.declare_parameter('encoder_signs', [1, 1, 1, 1])

        # rclpy 에서는 .value 를 쓰는게 가장 단순/안전
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.base_length = float(self.get_parameter('base_length').value)
        self.base_width = float(self.get_parameter('base_width').value)
        self.ticks_per_rev = int(self.get_parameter('ticks_per_rev').value)

        enc_signs_param = self.get_parameter('encoder_signs').value
        if isinstance(enc_signs_param, (list, tuple)) and len(enc_signs_param) == 4:
            self.encoder_signs = [int(s) for s in enc_signs_param]
        else:
            self.get_logger().warn(
                f"encoder_signs 파라미터가 이상합니다: {enc_signs_param}. "
                "길이 4의 리스트로 기대합니다. [1, 1, 1, 1] 로 대체합니다."
            )
            self.encoder_signs = [1, 1, 1, 1]

        self.get_logger().info(
            f"wheel_radius={self.wheel_radius:.4f} m, "
            f"L={self.base_length:.4f} m, W={self.base_width:.4f} m, "
            f"ticks_per_rev={self.ticks_per_rev}, "
            f"encoder_signs={self.encoder_signs}"
        )

        # ---------------- 상태 변수 ----------------
        # 로봇 위치 (odom 좌표계)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0  # rad

        # 마지막 인코더 값 / 시간
        self.last_encoders = None
        self.last_time = self.get_clock().now()

        # 퍼블리셔 & TF 브로드캐스터
        self.odom_pub = self.create_publisher(Odometry, 'odom', 50)
        self.tf_broadcaster = TransformBroadcaster(self)

        # 구독: encoder_raw
        self.enc_sub = self.create_subscription(
            Int32MultiArray,
            'encoder_raw',
            self.encoder_callback,
            50
        )

        self.get_logger().info("EncoderOdomNode started. Subscribing to encoder_raw.")

    # --------------------------------------------------
    # encoder_raw 콜백
    # --------------------------------------------------
    def encoder_callback(self, msg: Int32MultiArray):
        now = self.get_clock().now()

        if len(msg.data) < 4:
            self.get_logger().warn("encoder_raw has less than 4 elements.")
            return

        enc = list(msg.data[:4])

        # 첫 콜백이면 기준만 잡고 리턴
        if self.last_encoders is None:
            self.last_encoders = enc
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            # 시간 차가 0 이하이면 적분 불가
            self.last_encoders = enc
            self.last_time = now
            return

        # 인코더 변화량 (부호 보정 포함)
        delta_counts = [
            (enc[i] - self.last_encoders[i]) * self.encoder_signs[i]
            for i in range(4)
        ]

        self.last_encoders = enc
        self.last_time = now

        # 바퀴 1rev 당 거리
        wheel_circum = 2.0 * math.pi * self.wheel_radius

        # 각 바퀴 이동 거리 (m)
        d_fl = (delta_counts[0] / self.ticks_per_rev) * wheel_circum  # front-left
        d_fr = (delta_counts[1] / self.ticks_per_rev) * wheel_circum  # front-right
        d_rl = (delta_counts[2] / self.ticks_per_rev) * wheel_circum  # rear-left
        d_rr = (delta_counts[3] / self.ticks_per_rev) * wheel_circum  # rear-right

        # 각 바퀴 선속도 (m/s)
        v_fl = d_fl / dt
        v_fr = d_fr / dt
        v_rl = d_rl / dt
        v_rr = d_rr / dt

        # ---------------- 메카넘 역기구학 ----------------
        # L: 로봇 중심에서 앞/뒤 바퀴까지의 거리
        # W: 로봇 중심에서 좌/우 바퀴까지의 거리
        #
        # 바퀴 순서 [FL, FR, RL, RR] 기준
        # vx = (v_fl + v_fr + v_rl + v_rr) / 4
        # vy = (-v_fl + v_fr + v_rl - v_rr) / 4
        # wz = (-v_fl + v_fr - v_rl + v_rr) / (4 * (L + W))
        L = self.base_length
        W = self.base_width

        vx = (v_fl + v_fr + v_rl + v_rr) / 4.0
        vy = (-v_fl + v_fr + v_rl - v_rr) / 4.0
        wz = (-v_fl + v_fr - v_rl + v_rr) / (4.0 * (L + W))

        # ---------------- 좌표 적분 (odom 프레임) ----------------
        dx = (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * dt
        dy = (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * dt
        dyaw = wz * dt

        self.x += dx
        self.y += dy
        self.yaw += dyaw

        # yaw를 -pi~pi로 정규화
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # ---------------- odom 메시지 & TF 발행 ----------------
        self.publish_odom_and_tf(now, vx, vy, wz)

    # --------------------------------------------------
    # Odometry + TF 발행
    # --------------------------------------------------
    def publish_odom_and_tf(self, stamp, vx, vy, wz):
        # 쿼터니언 변환 (roll=0, pitch=0, yaw=self.yaw)
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, self.yaw)

        # Odometry 메시지
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # 속도는 base_link 기준
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        self.odom_pub.publish(odom)

        # TF (odom -> base_link)
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp.to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
