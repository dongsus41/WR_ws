from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    웨어러블 로봇 시스템의 모든 구성 요소를 시작하는 메인 런치 파일입니다.
    1. CAN 초기화 명령어 자동 실행
    2. ros2_socketcan 런치 파일 실행
    3. data_processing 패키지의 모든 노드 실행
    """

    # 패키지 경로 설정
    bringup_pkg_share = get_package_share_directory('wearable_robot_bringup')
    socketcan_pkg_share = get_package_share_directory('ros2_socketcan')

    # 런치 파일 경로
    data_processing_launch = os.path.join(
        bringup_pkg_share,
        'launch',
        'data_processing_all.launch.py'
    )

    socketcan_launch = os.path.join(
        socketcan_pkg_share,
        'launch',
        'socket_can_bridge.launch.py'  # ros2_socketcan 패키지의 런치 파일명 (실제 파일명에 맞게 수정 필요)
    )

    # CAN 인터페이스 설정
    can_interface = LaunchConfiguration('can_interface', default='can0')
    bitrate = LaunchConfiguration('bitrate', default='1000000')  # 1Mbps

    # 1. CAN 초기화 명령어 실행
    # can0 인터페이스 다운 (만약 이미 실행 중이라면)
    can_down = ExecuteProcess(
        cmd=['sudo', 'ifconfig', can_interface, 'down'],
        name='can_down',
        shell=True,
        output='screen'
    )

    # can0 인터페이스 설정 (bitrate 지정)
    can_setup = ExecuteProcess(
        cmd=[
            'sudo', 'ip', 'link', 'set',
            can_interface, 'type', 'can',
            'bitrate', bitrate
        ],
        name='can_setup',
        shell=True,
        output='screen'
    )

    # can0 인터페이스 활성화
    can_up = ExecuteProcess(
        cmd=['sudo', 'ifconfig', can_interface, 'up'],
        name='can_up',
        shell=True,
        output='screen'
    )

    # 2. ros2_socketcan 런치 파일 실행 (CAN 설정 완료 후)
    socketcan_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([socketcan_launch]),
        launch_arguments={
            'interface': can_interface,
        }.items()
    )

    # CAN 설정 완료 후 socketcan 실행을 위한 이벤트 핸들러
    socketcan_event = RegisterEventHandler(
        OnProcessStart(
            target_action=can_up,
            on_start=[
                LogInfo(msg='CAN interface is up. Starting socketcan bridge...'),
                socketcan_launch_include
            ]
        )
    )

    # 3. data_processing 패키지의 모든 노드 실행
    data_processing_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([data_processing_launch])
    )

    # socketcan 실행 후 data_processing 실행을 위한 이벤트 핸들러
    # 이 부분은 실제 구현에서 더 정교하게 수정이 필요할 수 있습니다.
    # 현재는 socketcan 이벤트 핸들러를 사용하는 대신 직접 등록만 합니다.

    # 런치 설명 구성
    return LaunchDescription([
        # CAN 초기화 명령어 실행
        can_down,
        can_setup,
        can_up,
        socketcan_event,

        # data_processing 런치 (CAN 설정과 독립적으로 실행)
        data_processing_include
    ])
