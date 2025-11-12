#!/usr/bin/env python3
"""
Launch Treebo bringup together with slam_toolbox for live map building.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("treebo_navigation")

    use_sim_time = LaunchConfiguration("use_sim_time")
    car_type = LaunchConfiguration("car_type")
    slam_params_file = LaunchConfiguration("slam_params_file")
    start_bringup = LaunchConfiguration("start_bringup")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use Gazebo/Sim time if true",
    )

    declare_car_type = DeclareLaunchArgument(
        "car_type",
        default_value="X3",
        description="Treebo car type to pass to treebo_bringup",
    )

    declare_slam_params = DeclareLaunchArgument(
        "slam_params_file",
        default_value=PathJoinSubstitution(
            [pkg_share, "config", "slam_toolbox_mapping.yaml"]
        ),
        description="Full path to the slam_toolbox parameter file",
    )

    declare_start_bringup = DeclareLaunchArgument(
        "start_bringup",
        default_value="true",
        description="If true run treebo_bringup in this launch file",
    )

    bringup_node = Node(
        package="treebo_bringup",
        executable="bringup",
        name="treebo_bringup",
        output="screen",
        parameters=[{"car_type": car_type}],
        condition=IfCondition(start_bringup),
    )

    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params_file, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_car_type,
            declare_slam_params,
            declare_start_bringup,
            bringup_node,
            slam_node,
        ]
    )
