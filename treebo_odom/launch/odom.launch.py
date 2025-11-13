from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='treebo_odom',
            executable='odom_publisher',
            name='treebo_odom',
            output='screen',
            parameters=[{
                # Frames
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'publish_tf': True,

                # Topics
                'wheel_omega_topic': '/wheel_omega',
                'vel_raw_topic': '/vel_raw',
                'cmd_vel_topic': '/cmd_vel',
                'imu_raw_topic': '/imu/data_raw',
                'mag_topic': '/imu/mag',

                # Vehicle params (차동/스키드-스티어)
                'wheel_radius': 0.04,  # 40 mm
                'track_width':  0.12,  # 120 mm

                # Behavior
                'use_cmd_vel_fallback': True,

                # IMU / Magnetometer
                'gyro_bias_z': 0.0,
                'mag_declination': 0.0,  # 필요 시 +0.35 등으로 조정
                'mag_alpha': 0.02,       # 보정 강도 (작게)
                'mag_flip_y': False,     # 축 반전 필요 시 True
                'mag_flip_x': False,

                # Covariances
                'cov_linear': 0.3,
                'cov_angular': 0.5,
            }],
        ),
    ])
