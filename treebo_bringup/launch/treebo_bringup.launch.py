from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    # ===== Arguments =====
    use_sim_time      = LaunchConfiguration('use_sim_time')
    use_ekf           = LaunchConfiguration('use_ekf')
    use_imu_filter    = LaunchConfiguration('use_imu_filter')
    urdf_xacro        = LaunchConfiguration('urdf_xacro')        # 절대/상대 경로 모두 허용
    odom_topic        = LaunchConfiguration('odom_topic')        # raw wheel odom
    imu_topic         = LaunchConfiguration('imu_topic')         # imu_filter 출력 or 생 IMU
    base_frame        = LaunchConfiguration('base_frame')        # base_link 프레임명
    odom_frame        = LaunchConfiguration('odom_frame')        # odom 프레임명
    map_frame         = LaunchConfiguration('map_frame')         # map 프레임명(2D면 world=odom 사용 권장)

    # ===== Robot Description (xacro) =====
    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        urdf_xacro
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content
        }]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        # 실제 하드웨어가 joint_states를 내보내면 비활성화해도 됩니다.
    )

    # ===== IMU Filter (optional) =====
    imu_filter_node = Node(
        condition=None,  # 런치 arg로 On/Off
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[Path(get_package_share_directory('treebo_bringup')) / 'param' / 'imu_filter.yaml'],
        # 실제 생 IMU 토픽을 imu_filter.yaml에서 지정하세요.
    )

    # ===== EKF (robot_localization) =====
    ekf_params_path = Path(get_package_share_directory('treebo_bringup')) / 'param' / 'ekf.yaml'
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            # 동적으로 바꾸고 싶은 핵심 파라미터는 런치 args에서 주입
            'base_link_frame': base_frame,
            'odom_frame': odom_frame,
            'map_frame': map_frame,
            # 센서 토픽명은 ekf.yaml에 기본값이 있으나, 런치 arg로 오버라이드 가능
            'odom0': odom_topic,
            'imu0': imu_topic,
        }, str(ekf_params_path)]
    )

    # ===== Assemble =====
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('use_sim_time',  default_value='false'))
    ld.add_action(DeclareLaunchArgument('use_ekf',       default_value='true'))
    ld.add_action(DeclareLaunchArgument('use_imu_filter',default_value='true'))

    # 기본 Xacro 경로(필요시 수정) : treebo_description/urdf/treebo_X3.urdf.xacro
    default_xacro = str(Path(get_package_share_directory('treebo_description')) / 'urdf' / 'treebo_X3.urdf.xacro')
    ld.add_action(DeclareLaunchArgument('urdf_xacro',    default_value=default_xacro))

    # 기본 토픽/프레임(현재 Treebo 실제 명칭으로 맞춰주세요)
    ld.add_action(DeclareLaunchArgument('odom_topic',    default_value='/odom_raw'))
    ld.add_action(DeclareLaunchArgument('imu_topic',     default_value='/imu/data'))  # imu_filter 출력이면 그 토픽
    ld.add_action(DeclareLaunchArgument('base_frame',    default_value='base_link'))
    ld.add_action(DeclareLaunchArgument('odom_frame',    default_value='odom'))
    ld.add_action(DeclareLaunchArgument('map_frame',     default_value='map'))

    # 필수 노드
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)

    # 선택 노드 (use_imu_filter/use_ekf로 제어)
    from launch.conditions import IfCondition
    imu_filter_node.condition = IfCondition(use_imu_filter)
    ld.add_action(imu_filter_node)

    ekf_node.condition = IfCondition(use_ekf)
    ld.add_action(ekf_node)

    return ld
