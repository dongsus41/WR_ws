from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # YAML 파일 경로 설정
    config = os.path.join(
        get_package_share_directory('wearable_robot_bringup'),
        'config',
        'serial_params.yaml'
    )


    return LaunchDescription([
        Node(
            package='wearable_robot_hw',
            executable='USART_actuator_node',
            name='USART_actuator_node',
            parameters=[config],
            output='screen'
        ),
        Node(
            package='wearable_robot_hw',
            executable='USART_displacement_node',  # 다른 노드의 실행 파일 이름
            name='USART_displacement_node',
            parameters=[config],
            output='screen'
        )
    ])
