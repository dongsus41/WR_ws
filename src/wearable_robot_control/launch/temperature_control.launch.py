from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 패키지 경로 가져오기
    control_pkg_share = get_package_share_directory('wearable_robot_control')

    # config 파일 경로
    temp_control_config = os.path.join(control_pkg_share, 'config', 'temperature_control_params.yaml')

    # CAN FD 설정 명령어 실행
    can_setup = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'sudo ip link set can0 down && '
            'sudo ip link set can0 type can bitrate 1000000 dbitrate 1000000 berr-reporting on fd on && '
            'sudo ip link set can0 up'
        ],
        name='can_setup',
        shell=True,
        output='screen'
    )

    # 데이터 파싱 노드
    parser_node = Node(
        package='wearable_robot_data_processing',
        executable='data_parser_node',
        name='data_parser_node',
        output='screen'
    )

    # 온도 제어 노드
    temp_control_node = Node(
        package='wearable_robot_control',
        executable='actuator_temp_control_node',
        name='actuator_temp_control_node',
        parameters=[temp_control_config],
        output='screen'
    )

    # 온도 로깅 노드
    temp_logger_node = Node(
        package='wearable_robot_control',
        executable='temperature_logger_node',
        name='temperature_logger_node',
        parameters=[temp_control_config],
        output='screen'
    )

    return LaunchDescription([
        can_setup,
        parser_node,
        temp_control_node,
        temp_logger_node
    ])
