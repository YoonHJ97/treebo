#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ----- Launch Arguments -----
    wheel_radius   = LaunchConfiguration("wheel_radius")
    ticks_per_rev  = LaunchConfiguration("ticks_per_rev")
    track_width    = LaunchConfiguration("track_width")
    encoder_topic  = LaunchConfiguration("encoder_topic")
    invert_left    = LaunchConfiguration("invert_left")
    invert_right   = LaunchConfiguration("invert_right")
    invert_translation = LaunchConfiguration("invert_translation")
    invert_rotation = LaunchConfiguration("invert_rotation")
    yaw_offset     = LaunchConfiguration("yaw_offset")
    publish_tf     = LaunchConfiguration("publish_tf")
    odom_frame     = LaunchConfiguration("odom_frame")
    base_frame     = LaunchConfiguration("base_frame")

    return LaunchDescription(
        [
            # 기본값은 적당히 세팅해두고,
            # 나중에 필요하면 bringup.launch.py에서 launch_arguments로 덮어쓸 수 있음
            DeclareLaunchArgument("wheel_radius", default_value="0.04"),
            DeclareLaunchArgument("ticks_per_rev", default_value="4320"),
            DeclareLaunchArgument("track_width", default_value="0.391"),

            DeclareLaunchArgument("encoder_topic", default_value="encoder_raw"),

            DeclareLaunchArgument("invert_left", default_value="true"),
            DeclareLaunchArgument("invert_right", default_value="true"),
            DeclareLaunchArgument(
                "invert_translation",
                default_value="true",
                description="Invert linear odometry direction",
            ),
            DeclareLaunchArgument(
                "invert_rotation",
                default_value="false",
                description="Invert angular odometry direction",
            ),
            DeclareLaunchArgument(
                "yaw_offset",
                default_value="0.0",
                description="Yaw offset in radians",
            ),
            DeclareLaunchArgument("publish_tf", default_value="true"),

            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),

            # ----- Odometry Node -----
            Node(
                package="treebo_odom",
                executable="odom_publisher",
                name="treebo_odom",
                output="screen",
                parameters=[
                            {
                                "wheel_radius": wheel_radius,
                                "ticks_per_rev": ticks_per_rev,
                                "track_width": track_width,
                                "encoder_topic": encoder_topic,
                                "invert_left": invert_left,
                                "invert_right": invert_right,
                                "invert_translation": invert_translation,
                                "invert_rotation": invert_rotation,
                                "yaw_offset": yaw_offset,
                                "publish_tf": publish_tf,
                                "odom_frame": odom_frame,
                                "base_frame": base_frame,
                            }
                ],
            ),
        ]
    )
