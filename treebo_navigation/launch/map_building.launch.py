from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_nav = get_package_share_directory('treebo_navigation')
    pkg_desc = get_package_share_directory('treebo_description')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    ekf_yaml = os.path.join(pkg_nav, 'config', 'ekf_localization.yaml')
    slam_yaml = os.path.join(pkg_nav, 'config', 'slam_toolbox_mapping.yaml')
    urdf_xacro = os.path.join(pkg_desc, 'urdf', 'robot.urdf.xacro')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # 로봇 모델 → TF(base_link 등) 출력을 위해 필수
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf_xacro],
        ),

        # EKF (odom→base_link + /odometry/filtered)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_yaml, {'use_sim_time': use_sim_time}],
        ),

        # SLAM (map↔odom 제공)
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_yaml, {'use_sim_time': use_sim_time}],
        ),

        # RViz는 선택
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', os.path.join(pkg_nav, 'config', 'your_nav.rviz')],
        #     parameters=[{'use_sim_time': use_sim_time}],
        # ),
    ])
