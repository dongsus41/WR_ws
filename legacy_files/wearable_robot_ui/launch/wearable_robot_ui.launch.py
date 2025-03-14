from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition  # 이 줄 추가
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    웨어러블 로봇 UI 및 관련 시스템을 실행하는 런치 파일
    """
    # 패키지 경로 가져오기
    control_pkg_share = get_package_share_directory('wearable_robot_control')
    ui_pkg_share = get_package_share_directory('wearable_robot_ui')

    # 런치 파라미터 정의
    use_test_mode = LaunchConfiguration('use_test_mode')
    can_interface = LaunchConfiguration('can_interface')

    # 파라미터 선언
    declare_use_test_mode = DeclareLaunchArgument(
        'use_test_mode',
        default_value='false',
        description='테스트 모드 사용 여부 (CAN 인터페이스 없이 실행)'
    )

    declare_can_interface = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='CAN 인터페이스 이름'
    )

    # config 파일 경로
    temp_control_config = os.path.join(control_pkg_share, 'config', 'temperature_control_params.yaml')

    # Node 선언

    # CAN 데이터 처리 노드
    can_processor_node = Node(
        package='wearable_robot_data_processing',
        executable='can_data_processor',
        name='can_data_processor',
        output='screen',
        condition=UnlessCondition(use_test_mode)  # 수정된 부분
    )

    # 데이터 파싱 노드
    parser_node = Node(
        package='wearable_robot_data_processing',
        executable='data_parser_node',
        name='data_parser_node',
        output='screen'
    )

    # 온도 제어 노드
    actuator_control_node = Node(
        package='wearable_robot_control',
        executable='actuator_control_node',
        name='actuator_control_node',
        parameters=[temp_control_config],
        output='screen'
    )

    # 명령 송신 노드
    command_send_node = Node(
        package='wearable_robot_control',
        executable='command_send_node',
        name='command_send_node',
        parameters=[temp_control_config],
        output='screen',
        condition=UnlessCondition(use_test_mode)  # 수정된 부분
    )

    # RQT UI 실행
    rqt_with_plugin = ExecuteProcess(
        cmd=['rqt', '--force-discover', '--standalone', 'TemperatureControlPlugin'],
        output='screen'
    )

    return LaunchDescription([
        declare_use_test_mode,
        declare_can_interface,
        can_processor_node,
        parser_node,
        actuator_control_node,
        command_send_node,
        rqt_with_plugin
    ])
