#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # ---------- 1) Treebo bringup 노드 ----------
    bringup_node = Node(
        package="treebo_bringup",
        executable="bringup",
        name="treebo_bringup",
        output="screen",
    )

    # ---------- 2) URDF upload (robot_state_publisher 등) ----------
    description_share = get_package_share_directory("treebo_description")
    upload_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(description_share, "launch", "upload.launch.xml")
        )
    )

    # ---------- 3) Odometry publisher 실행 ----------
    odom_share = get_package_share_directory("treebo_odom")
    odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(odom_share, "launch", "odom.launch.py")
        )
    )

    # ---------- 4) C1 LiDAR 실행 ----------
    sllidar_share = get_package_share_directory("sllidar_ros2")
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sllidar_share, "launch", "sllidar_c1_launch.py")
        ),
        launch_arguments={
            "frame_id": "laser_link",
        }.items(),
    )

    # bringup + description + odom + lidar
    return LaunchDescription(
        [
            bringup_node,
            upload_launch,
            odom_launch,
            lidar_launch,
        ]
    )
