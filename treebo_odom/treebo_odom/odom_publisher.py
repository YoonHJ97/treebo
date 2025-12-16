#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class TreeboOdom(Node):
    """
    /encoder_raw: Int32MultiArray data=[ms, e1, e2, e3, e4] (누적 tick)
      - e1=m1(FL), e2=m2(RL) -> LEFT
      - e3=m3(RR), e4=m4(FR) -> RIGHT

    출력:
      - /odom (nav_msgs/Odometry)
      - TF: odom -> base_link
    """

    def __init__(self):
        super().__init__("treebo_odom")

        self.declare_parameter("wheel_radius", 0.04)   # m
        self.declare_parameter("ticks_per_rev", 4320)
        self.declare_parameter("track_width", 0.12)    # m

        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)

        self.R = float(self.get_parameter("wheel_radius").value)
        self.tpr = int(self.get_parameter("ticks_per_rev").value)
        self.track = float(self.get_parameter("track_width").value)

        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        self.sub = self.create_subscription(Int32MultiArray, "encoder_raw", self.cb_enc, 50)
        self.pub = self.create_publisher(Odometry, "odom", 50)
        self.tfbr = TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.prev = None
        self.prev_stamp = None

    @staticmethod
    def _wrap(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def _ticks_to_dist(self, dticks: float) -> float:
        return (dticks / float(self.tpr)) * (2.0 * math.pi * self.R)

    def _yaw_to_quat(self, yaw: float):
        return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

    def cb_enc(self, msg: Int32MultiArray):
        if len(msg.data) < 5:
            return

        # data=[ms,e1,e2,e3,e4]  (누적 tick)
        ms = int(msg.data[0])
        e1, e2, e3, e4 = map(int, msg.data[1:5])

        left_now = (e1 + e2) / 2.0   # FL, RL
        right_now = (e3 + e4) / 2.0  # RR, FR

        stamp = self.get_clock().now()

        if self.prev is None:
            self.prev = (left_now, right_now)
            self.prev_stamp = stamp
            return

        dt = (stamp - self.prev_stamp).nanoseconds * 1e-9
        if dt <= 1e-4:
            return

        left_prev, right_prev = self.prev
        dL_ticks = left_now - left_prev
        dR_ticks = right_now - right_prev

        self.prev = (left_now, right_now)
        self.prev_stamp = stamp

        dL = self._ticks_to_dist(dL_ticks)
        dR = self._ticks_to_dist(dR_ticks)

        ds = (dR + dL) / 2.0
        dyaw = (dR - dL) / max(self.track, 1e-6)

        yaw_mid = self.yaw + dyaw * 0.5
        self.x += ds * math.cos(yaw_mid)
        self.y += ds * math.sin(yaw_mid)
        self.yaw = self._wrap(self.yaw + dyaw)

        vx = ds / dt
        wz = dyaw / dt

        qx, qy, qz, qw = self._yaw_to_quat(self.yaw)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = float(vx)
        odom.twist.twist.angular.z = float(wz)

        self.pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = odom.header.stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = float(self.x)
            t.transform.translation.y = float(self.y)
            t.transform.translation.z = 0.0
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tfbr.sendTransform(t)


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = TreeboOdom()   # ← 파일 안 클래스 이름과 동일해야 함
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()