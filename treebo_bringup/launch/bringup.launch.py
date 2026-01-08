#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    FindExecutable,
)
from launch_ros.substitutions import FindPackageShare

xacro_file = PathJoinSubstitution([
    FindPackageShare("treebo_description"),
    "urdf",
    "robot.urdf.xacro"
])

def generate_launch_description():
    invert_left = LaunchConfiguration("invert_left")
    invert_right = LaunchConfiguration("invert_right")
    invert_translation = LaunchConfiguration("invert_translation")
    invert_rotation = LaunchConfiguration("invert_rotation")
    yaw_offset = LaunchConfiguration("yaw_offset")
    lidar_inverted = LaunchConfiguration("lidar_inverted")
    invert_cmd_vel = LaunchConfiguration("invert_cmd_vel")
    invert_cmd_vel_angular = LaunchConfiguration("invert_cmd_vel_angular")
    # ---------- 1) Treebo bringup 노드 ----------
    bringup_node = Node(
        package="treebo_bringup",
        executable="bringup",
        name="treebo_bringup",
        output="screen",
        parameters=[{
            "track_width": 0.397,        # m (좌우 바퀴 사이 거리)
            "invert_cmd_vel": invert_cmd_vel,
            "invert_cmd_vel_angular": invert_cmd_vel_angular,
        }]
    )
    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": Command([
                FindExecutable(name="xacro"),
                " ",
                xacro_file,
            ]),
        }],
    )
    # # ---------- 2) URDF upload (robot_state_publisher 등) ----------
    # description_share = get_package_share_directory("treebo_description")
    # upload_launch = IncludeLaunchDescription(
    #     AnyLaunchDescriptionSource(
    #         os.path.join(description_share, "launch", "upload.launch.xml")
    #     )
    # )

    # ---------- 3) Odometry publisher 실행 ----------
    odom_share = get_package_share_directory("treebo_odom")
    odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(odom_share, "launch", "odom.launch.py")
        ),
        launch_arguments={
            "track_width": "0.397",
            "wheel_radius": "0.04",
            "ticks_per_rev": "4320",
            "invert_left": invert_left,
            "invert_right": invert_right,
            "invert_translation": invert_translation,
            "invert_rotation": invert_rotation,
            "yaw_offset": yaw_offset,
        }.items(),
    )

    # ---------- 4) C1 LiDAR 실행 ----------
    sllidar_share = get_package_share_directory("sllidar_ros2")
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sllidar_share, "launch", "sllidar_c1_launch.py")
        ),
        launch_arguments={
            "frame_id": "laser_link",
            "inverted": lidar_inverted,
        }.items(),
    )

    # bringup + description + odom + lidar
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "invert_left",
                default_value="true",
                description="Invert left encoder tick direction",
            ),
            DeclareLaunchArgument(
                "invert_right",
                default_value="true",
                description="Invert right encoder tick direction",
            ),
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
            DeclareLaunchArgument(
                "lidar_inverted",
                default_value="false",
                description="Invert lidar scan data",
            ),
            DeclareLaunchArgument(
                "invert_cmd_vel",
                default_value="true",
                description="Invert cmd_vel linear and angular directions",
            ),
            DeclareLaunchArgument(
                "invert_cmd_vel_angular",
                default_value="false",
                description="Invert cmd_vel angular direction",
            ),
            bringup_node,
            # upload_launch,
            odom_launch,
            lidar_launch,
            joint_state_pub,
            robot_state_publisher,
        ]
    )
