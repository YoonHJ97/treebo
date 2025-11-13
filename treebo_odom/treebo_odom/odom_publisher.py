#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
base_node_x3 / Odom Publisher (ROS 2 Jazzy, Ubuntu 24.04)
- 차동/스키드-스티어 전제(메카넘 X)
- 우선순위: /wheel_omega(각속도) → /vel_raw(Twist) → /cmd_vel(Twist)
- yaw: /imu/data_raw 자이로(z) 적분 + /imu/mag 느린 보정(옵션)
- /odom 발행 + TF(odom -> base_link) 브로드캐스트
- /reset_odom (std_srvs/Empty) 지원
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster


class OdomPublisher(Node):
    def __init__(self):
        super().__init__('base_node_x3')

        # ---------------- Parameters ----------------
        # Frames & behavior
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)

        # Topics
        self.declare_parameter('wheel_omega_topic', '/wheel_omega')  # std_msgs/Float32MultiArray
        self.declare_parameter('vel_raw_topic', '/vel_raw')          # geometry_msgs/Twist
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')          # geometry_msgs/Twist
        self.declare_parameter('imu_raw_topic', '/imu/data_raw')     # sensor_msgs/Imu
        self.declare_parameter('mag_topic', '/imu/mag')              # sensor_msgs/MagneticField

        # Kinematics (차동/스키드-스티어)
        self.declare_parameter('wheel_radius', 0.04)  # R (m) 40mm
        self.declare_parameter('track_width', 0.12)   # T (m) 120mm (좌/우 바퀴 중심 간 거리)

        # Behaviors
        self.declare_parameter('use_cmd_vel_fallback', True)

        # IMU 보정/필터
        self.declare_parameter('gyro_bias_z', 0.0)      # 고정 바이어스 보정
        self.declare_parameter('mag_declination', 0.0)  # 자기편차(rad), 한국 ~ +0.3~0.4
        self.declare_parameter('mag_alpha', 0.02)       # 마그 yaw 보정율(0~1, 작게)
        self.declare_parameter('mag_flip_y', False)     # 축 정의 다르면 True로 바꿔보세요
        self.declare_parameter('mag_flip_x', False)

        # Covariance
        self.declare_parameter('cov_linear', 0.3)
        self.declare_parameter('cov_angular', 0.5)

        # Read params
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.R = float(self.get_parameter('wheel_radius').value)
        self.T = float(self.get_parameter('track_width').value)

        self.gyro_bias_z = float(self.get_parameter('gyro_bias_z').value)
        self.mag_decl = float(self.get_parameter('mag_declination').value)
        self.mag_alpha = float(self.get_parameter('mag_alpha').value)
        self.mag_flip_y = bool(self.get_parameter('mag_flip_y').value)
        self.mag_flip_x = bool(self.get_parameter('mag_flip_x').value)

        self.cov_lin = float(self.get_parameter('cov_linear').value)
        self.cov_ang = float(self.get_parameter('cov_angular').value)

        self.use_cmd_vel_fallback = bool(self.get_parameter('use_cmd_vel_fallback').value)

        # ---------------- QoS ----------------
        qos = QoSProfile(depth=20)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.history = QoSHistoryPolicy.KEEP_LAST

        # ---------------- Publishers & TF ----------------
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.br = TransformBroadcaster(self)

        # ---------------- Subscriptions ----------------
        self.sub_w_omega = self.create_subscription(
            Float32MultiArray,
            self.get_parameter('wheel_omega_topic').value,
            self.wheel_omega_cb,
            qos
        )
        self.sub_vel_raw = self.create_subscription(
            Twist,
            self.get_parameter('vel_raw_topic').value,
            self.vel_raw_cb,
            qos
        )
        self.sub_cmd_vel = self.create_subscription(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            self.cmd_vel_cb,
            qos
        )
        self.sub_imu_raw = self.create_subscription(
            Imu,
            self.get_parameter('imu_raw_topic').value,
            self.imu_raw_cb,
            qos
        )
        self.sub_mag = self.create_subscription(
            MagneticField,
            self.get_parameter('mag_topic').value,
            self.mag_cb,
            qos
        )

        # ---------------- Service ----------------
        self.create_service(Empty, 'reset_odom', self.on_reset)

        # ---------------- State ----------------
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.vx = 0.0      # m/s
        self.wz = 0.0      # rad/s

        self.have_wheel_omega = False
        self.have_vel_raw = False

        self.last_stamp = self.get_clock().now()
        self.last_mag_yaw = None

        # 50 Hz loop
        self.timer = self.create_timer(0.02, self.on_timer)

        self.get_logger().info(
            f'base_node_x3 started: R={self.R:.3f} m, T={self.T:.3f} m, '
            f'frames=({self.odom_frame}->{self.base_frame})'
        )

    # ---------------- Callbacks ----------------
    def wheel_omega_cb(self, msg: Float32MultiArray):
        """각속도 입력(우선순위 1): 길이 2([w_l,w_r]) 또는 길이>=4([fl,fr,rl,rr])"""
        d = list(msg.data)
        if len(d) == 2:
            w_l, w_r = d[0], d[1]
        elif len(d) >= 4:
            # 스키드-스티어 4륜: 좌/우 평균
            w_l = 0.5 * (d[0] + d[2])  # fl, rl
            w_r = 0.5 * (d[1] + d[3])  # fr, rr
        else:
            return

        # 차동 모델
        self.vx = 0.5 * self.R * (w_r + w_l)
        self.wz = (self.R / self.T) * (w_r - w_l)
        self.have_wheel_omega = True

    def vel_raw_cb(self, msg: Twist):
        """실속도 입력(우선순위 2): geometry_msgs/Twist"""
        if self.have_wheel_omega:
            return
        self.vx = float(msg.linear.x)
        # vy는 비홀로노믹이라 사용하지 않음
        self.wz = float(msg.angular.z)
        self.have_vel_raw = True

    def cmd_vel_cb(self, msg: Twist):
        """명령속도 폴백(우선순위 3)"""
        if (self.have_wheel_omega or self.have_vel_raw) and self.use_cmd_vel_fallback:
            return
        if self.use_cmd_vel_fallback:
            self.vx = float(msg.linear.x)
            self.wz = float(msg.angular.z)

    def imu_raw_cb(self, msg: Imu):
        """자이로 z 적분으로 yaw 갱신 (orientation은 비어있을 수 있음)"""
        now = self.get_clock().now()
        dt = (now - self.last_stamp).nanoseconds * 1e-9
        if 0.0 < dt < 0.5:
            gz = float(msg.angular_velocity.z) - self.gyro_bias_z
            self.yaw = self._wrap(self.yaw + gz * dt)
        self.last_stamp = now

    def mag_cb(self, msg: MagneticField):
        """자력계 기반 yaw를 아주 느리게 보정(콤플리멘터리)"""
        mx = float(msg.magnetic_field.x)
        my = float(msg.magnetic_field.y)

        if self.mag_flip_x:
            mx = -mx
        if self.mag_flip_y:
            my = -my

        if abs(mx) < 1e-12 and abs(my) < 1e-12:
            return

        yaw_mag = math.atan2(my, mx) + self.mag_decl
        dyaw = self._angle_diff(yaw_mag, self.yaw)
        self.yaw = self._wrap(self.yaw + self.mag_alpha * dyaw)
        self.last_mag_yaw = yaw_mag

    # ---------------- Helpers ----------------
    @staticmethod
    def _wrap(a):
        """[-pi, pi)"""
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    def _angle_diff(self, a, b):
        return self._wrap(a - b)

    # ---------------- Service ----------------
    def on_reset(self, req, res):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.wz = 0.0
        self.last_stamp = self.get_clock().now()
        self.get_logger().info('Odom reset to (0,0,0).')
        return res

    # ---------------- Timer loop ----------------
    def on_timer(self):
        now = self.get_clock().now()
        dt = (now - self.last_stamp).nanoseconds * 1e-9
        if not (0.0 < dt < 0.5):
            self.last_stamp = now
            return

        # 위치 적분 (비홀로노믹: vy=0)
        x_dot = self.vx * math.cos(self.yaw)
        y_dot = self.vx * math.sin(self.yaw)
        self.x += x_dot * dt
        self.y += y_dot * dt

        self.last_stamp = now

        # Odometry 메시지
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        cz = math.cos(self.yaw * 0.5)
        sz = math.sin(self.yaw * 0.5)
        odom.pose.pose.orientation.z = sz
        odom.pose.pose.orientation.w = cz

        # Covariances (dead-reckoning 이므로 여유있게)
        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0] = self.cov_lin
        odom.pose.covariance[7] = self.cov_lin
        odom.pose.covariance[35] = self.cov_ang

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.wz

        self.odom_pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.z = sz
            t.transform.rotation.w = cz
            self.br.sendTransform(t)


def main():
    rclpy.init()
    node = OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
