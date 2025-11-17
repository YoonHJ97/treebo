#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
)
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState, matches_node_name
from launch_ros.event_handlers import OnStateTransition

from lifecycle_msgs.msg import Transition


def generate_launch_description():
    # ---------- Launch Arguments ----------
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params_file = LaunchConfiguration("slam_params_file")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    nav_share_dir = get_package_share_directory("treebo_navigation")
    default_slam_params = os.path.join(
        nav_share_dir, "config", "slam_toolbox_mapping.yaml"
    )
    declare_slam_params_file = DeclareLaunchArgument(
        "slam_params_file",
        default_value=default_slam_params,
        description="Full path to the slam_toolbox mapping parameter file",
    )

    # ---------- Lifecycle Node: slam_toolbox ----------
    slam_toolbox_node = LifecycleNode(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",            # ★ 이 줄이 없어서 에러 났던 거
        output="screen",
        parameters=[
            slam_params_file,
            {"use_sim_time": use_sim_time},
        ],
    )

    # 1) 노드가 뜨면 CONFIGURE 전이 보내기
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_node_name("/slam_toolbox"),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    # 2) CONFIGURE → INACTIVE 로 넘어가면 ACTIVATE 전이 보내기
    activate_event_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_node_name("/slam_toolbox"),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_slam_params_file,
            slam_toolbox_node,
            configure_event,
            activate_event_handler,
        ]
    )
