from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 패키지 경로 가져오기
    pkg_share = get_package_share_directory('wearable_robot_bringup')

    # config 파일 경로
    config_file = os.path.join(pkg_share, 'config', 'calibration_params.yaml')

    # config 파일을 위한 launch argument
    config_arg = DeclareLaunchArgument(
        'config_file_path',
        default_value=config_file,
        description='Path to the calibration parameters YAML file'
    )

    # Data Parser Node
    parser_node = Node(
        package='wearable_robot_data_processing',
        executable='data_parser_node',
        name='data_parser_node',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # Displacement Processing Node
    displacement_node = Node(
        package='wearable_robot_data_processing',
        executable='displacement_processing_node',
        name='displacement_processing_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file_path'),
            {'use_sim_time': False}
        ]
    )

    # IMU Processing Node
    imu_node = Node(
        package='wearable_robot_data_processing',
        executable='imu_processing_node',
        name='imu_processing_node',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        config_arg,
        parser_node,
        displacement_node,
        imu_node
    ])
