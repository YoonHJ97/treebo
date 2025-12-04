#!/usr/bin/env python3
# encoding: utf-8

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray

from treebo_bringup.treebolib import TreeboLib


class TreeboBringup(Node):
    def __init__(self) -> None:
        super().__init__("treebo_bringup")

        # ---- 실제로 쓰는 파라미터만 ----
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("wheel_radius", 0.04)   # [m] ticks -> 거리 변환용
        self.declare_parameter("ticks_per_rev", 4320)
        self.declare_parameter("xlinear_limit", 0.5)   # /cmd_vel 제한
        self.declare_parameter("angular_limit", 1.0)   # /cmd_vel 제한
        self.declare_parameter("Prefix", "")           # joint 이름 prefix

        port = self.get_parameter("port").value
        wheel_radius = float(self.get_parameter("wheel_radius").value)
        ticks_per_rev = int(self.get_parameter("ticks_per_rev").value)
        self.xlinear_limit = float(self.get_parameter("xlinear_limit").value)
        self.angular_limit = float(self.get_parameter("angular_limit").value)
        self.prefix = self.get_parameter("Prefix").value

        # ---- Treebo 보드 연결 (모터+엔코더) ----
        self.car = TreeboLib(
            port=port,
            baudrate=115200,
            wheel_radius=wheel_radius,
            base_length=0.095,
            base_width=0.12,
            ticks_per_rev=ticks_per_rev,
            max_lin_vel=0.5,
            debug=False,      # ← 로그 안 찍게
        )

        # ENC total v1 v2 v3 수신용 스레드
        self.car.create_receive_threading()

        # enc_on/enc_off는 여기서 켜도 되고, 따로 스크립트에서 해도 됨
        # 필요하면 주석 해제
        # self.car.set_encoder_upload(True)

        # ---- ROS 통신 ----
        self.cmd_vel_sub = self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, 10
        )

        self.encoder_pub = self.create_publisher(Int32MultiArray, "encoder_raw", 10)
        self.joint_state_pub = self.create_publisher(JointState, "joint_states", 10)

        # encoder 기반 속도 추정은 odom 노드에서 할 거면 여기선 안 해도 됨
        # (지금은 encoder_raw만 퍼블리시)
        self.timer = self.create_timer(0.1, self.pub_data)

    # /cmd_vel → 보드로 속도 명령
    def cmd_vel_callback(self, msg: Twist) -> None:
        vx = max(min(msg.linear.x, self.xlinear_limit), -self.xlinear_limit)
        vz = max(min(msg.angular.z, self.angular_limit), -self.angular_limit)
        vy = 0.0

        self.car.set_car_motion(vx, vy, vz)

    # 주기적으로 encoder_raw + joint_states 퍼블리시
    def pub_data(self) -> None:
        now = Clock().now().to_msg()

        # 1) encoder_raw: [total, v1, v2, v3]
        total, v1, v2, v3 = self.car.get_encoder()
        enc_msg = Int32MultiArray()
        enc_msg.data = [int(total), int(v1), int(v2), int(v3)]
        self.encoder_pub.publish(enc_msg)

        # 2) joint_states: URDF/RViz용 0값
        js = JointState()
        js.header.stamp = now
        js.header.frame_id = "joint_states"

        if len(self.prefix) == 0:
            js.name = [
                "rear_right_wheel_joint",
                "rear_left_wheel_joint",
                "front_left_wheel_joint",
                "front_right_wheel_joint",
            ]
        else:
            p = self.prefix
            js.name = [
                p + "rear_right_wheel_joint",
                p + "rear_left_wheel_joint",
                p + "front_left_wheel_joint",
                p + "front_right_wheel_joint",
            ]

        n = len(js.name)
        js.position = [0.0] * n
        js.velocity = [0.0] * n
        js.effort = [0.0] * n

        self.joint_state_pub.publish(js)

    def destroy_node(self):
        try:
            self.car.set_car_motion(0.0, 0.0, 0.0)
            # self.car.set_encoder_upload(False)  # enc_on 여기서 켰다면 끌 때 사용
            self.car.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TreeboBringup()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
