#!/usr/bin/env python3
# encoding: utf-8

"""
odom_publisher.py (single-total version)

- 구독:
  - encoder_raw (std_msgs/Int32MultiArray)
    data[0] = total encoder count (Yahboom $MAll:total,...# 에서 온 누적값)

- 발행:
  - /odom (nav_msgs/Odometry)
  - TF: odom -> base_link

특징:
- 총 tick 값(total)만을 사용해서 "전후(x) 이동"만 계산.
- 회전 속도(wz)는 계산할 수 없으므로 0으로 둔다.
  (추후 IMU나 다른 센서와 결합해서 yaw를 추정하는 쪽으로 확장 가능)
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
        super().__init__("encoder_odom_node")

        # 바퀴 반지름 (m)
        self.declare_parameter("wheel_radius", 0.04)
        # 인코더 해상도 (counts / rev)
        self.declare_parameter("ticks_per_rev", 4320)
        # total tick의 부호 (앞으로 갔을 때 total이 감소하던가? 등에 맞춰 -1/1 설정)
        self.declare_parameter("total_sign", -1)

        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.ticks_per_rev = int(self.get_parameter("ticks_per_rev").value)
        self.total_sign = int(self.get_parameter("total_sign").value)

        # 로봇 상태 (odom 좌표계)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # 마지막 total, 시간
        self.last_total = None
        self.last_time = self.get_clock().now()

        # 퍼블리셔 & TF
        self.odom_pub = self.create_publisher(Odometry, "odom", 50)
        self.tf_broadcaster = TransformBroadcaster(self)

        # 구독: encoder_raw (data[0] = total)
        self.enc_sub = self.create_subscription(
            Int32MultiArray,
            "encoder_raw",
            self.encoder_callback,
            50,
        )

    def encoder_callback(self, msg: Int32MultiArray):
        now = self.get_clock().now()

        if len(msg.data) < 1:
            return

        total = int(msg.data[0])

        # 첫 콜백이면 기준만 잡고 리턴
        if self.last_total is None:
            self.last_total = total
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            self.last_total = total
            self.last_time = now
            return

        # total 변화량 (부호 조정)
        dticks = (total - self.last_total) * self.total_sign
        self.last_total = total
        self.last_time = now

        # 바퀴 1rev 당 거리
        wheel_circum = 2.0 * math.pi * self.wheel_radius

        # 이동 거리 (m)
        d = (dticks / float(self.ticks_per_rev)) * wheel_circum

        # 선속도 v, 각속도 wz(=0)
        v = d / dt
        wz = 0.0

        # odom 좌표계에서 적분
        dx = d * math.cos(self.yaw)
        dy = d * math.sin(self.yaw)
        dyaw = wz * dt

        self.x += dx
        self.y += dy
        self.yaw += dyaw
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        self.publish_odom_and_tf(now, v, wz)

    def publish_odom_and_tf(self, stamp, v, wz):
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, self.yaw)

        # Odometry 메시지
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = wz

        self.odom_pub.publish(odom)

        # TF (odom -> base_link)
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp.to_msg()
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_link"
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


if __name__ == "__main__":
    main()
