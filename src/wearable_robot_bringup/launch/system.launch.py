import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # CAN 데이터 추출 노드
        Node(
            package='wearable_robot_hw',
            executable='CAN_data_extractor_node',
            name='CAN_data_extractor',
            output='screen'
        ),

        # 데이터 파싱 노드
        Node(
            package='wearable_robot_data_processing',
            executable='data_parser_node',
            name='data_parser',
            output='screen'
        ),

        # IMU 데이터 처리 노드
        Node(
            package='wearable_robot_data_processing',
            executable='IMU_processing_node',
            name='IMU_processing',
            output='screen'
        ),

        # 변위 데이터 처리 노드
        Node(
            package='wearable_robot_data_processing',
            executable='displacement_processing_node',
            name='displacement_processing',
            output='screen'
        ),

        # CAN 명령 전송 노드
        Node(
            package='wearable_robot_hw',
            executable='CAN_command_transmit_node',
            name='CAN_command_transmit',
            output='screen'
        ),
    ])
