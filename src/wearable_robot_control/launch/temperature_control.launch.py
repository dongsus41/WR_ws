
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # config 파일 경로
    control_pkg_share = get_package_share_directory('wearable_robot_control')
    temp_control_config = os.path.join(control_pkg_share, 'config', 'temperature_control_params.yaml')

    # CAN 데이터 처리 노드
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
        can_processor_node,
        parser_node,
        temp_control_node,
        temp_logger_node
    ])
