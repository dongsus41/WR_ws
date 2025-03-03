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

    # can data processing 노드
    can_processor_node = Node(
        package='wearable_robot_data_processing',
        executable='can_data_processor',
        name='can_data_processor',
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
    actuator_control_node = Node(
        package='wearable_robot_control',
        executable='actuator_control_node',
        name='actuator_control_node',
        parameters=[temp_control_config],
        output='screen'
    )


    # command 노드
    command_send_node = Node(
        package='wearable_robot_control',
        executable='command_send_node',
        name='command_send_node',
        parameters=[temp_control_config],
        output='screen'
    )

    return LaunchDescription([
        can_processor_node,
        parser_node,
        actuator_control_node,
        command_send_node
    ])
