#!/usr/bin/env python3
import os
import csv
import datetime
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String, Float64
from std_srvs.srv import Trigger, SetBool
from wearable_robot_interfaces.msg import TemperatureData, ActuatorCommand
from collections import deque

class TemperatureLoggerNode(Node):
    """
    온도, PWM 및 목표 온도 데이터를 수집하고 CSV 파일로 저장하는 로거 노드
    """
    def __init__(self):
        super().__init__('temperature_logger_node')

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT  # 빠른 데이터 전송 우선
        )

        # 파라미터 선언
        self.declare_parameter('log_directory', '~/temp_logs')
        self.declare_parameter('active_actuator', 5) # 0~5중 5번 구동기 & 5번 온도센서
        self.declare_parameter('logging_frequency', 250.0)  # 250Hz 로깅 기본값
        self.declare_parameter('max_buffer_size', 150000)  # 10분(250Hz * 60초 * 4) 데이터 저장 가능

        # 경로 설정
        self.log_dir = os.path.expanduser(self.get_parameter('log_directory').value)
        os.makedirs(self.log_dir, exist_ok=True)

        # 로깅 데이터 초기화
        self.reset_data()

        # ROS 구독자 설정
        self.temperature_subscription = self.create_subscription(
            TemperatureData,
            'temperature_data',
            self.temperature_callback,
            qos
        )

        self.actuator_subscription = self.create_subscription(
            ActuatorCommand,
            'actuator_command',
            self.actuator_callback,
            qos
        )

        self.target_temp_subscription = self.create_subscription(
            TemperatureData,
            'target_temperature',
            self.target_temp_callback,
            qos
        )

        # 로깅 타이머 설정
        self.logging_timer = self.create_timer(
            1.0 / self.get_parameter('logging_frequency').value,
            self.log_data
        )

        # 현재 로그 파일 이름 발행
        self.filename_publisher = self.create_publisher(
            String,
            'current_log_filename',
            10
        )

        user_filename = os.environ.get('TEMPERATURE_LOG_FILENAME', '')
        if user_filename:
            self.user_filename_prefix = user_filename
        else:
            self.user_filename_prefix = 'temperature_log'

        # 서비스 서버 설정
        self.reset_service = self.create_service(
            Trigger,
            'reset_temperature_log',
            self.reset_log_callback
        )

        self.save_service = self.create_service(
            SetBool,
            'save_temperature_log',
            self.save_log_callback
        )

        # 로깅 상태
        self.is_logging = False
        self.csv_writer = None
        self.csv_file = None

        self.get_logger().info('온도 로거 노드가 시작되었습니다.')
        self.get_logger().info(f'로그 디렉토리: {self.log_dir}')

    def reset_data(self):
        """데이터 배열 초기화 - deque 사용으로 성능 개선"""
        self.max_buffer_size = self.get_parameter('max_buffer_size').value
        self.temperatures = deque(maxlen=self.max_buffer_size)
        self.pwm_values = deque(maxlen=self.max_buffer_size)
        self.target_temperatures = deque(maxlen=self.max_buffer_size)
        self.timestamps = deque(maxlen=self.max_buffer_size)

        # 마지막 수신 데이터 저장 변수 초기화
        self.current_temperature = 0.0
        self.current_pwm = 0
        self.current_target_temp = 0.0
        self.active_actuator = self.get_parameter('active_actuator').value

        # 새 로그 파일 이름 초기화
        self.current_log_filename = None

    def temperature_callback(self, msg):
        """온도 데이터 콜백"""
        if len(msg.temperature) > self.active_actuator:
            self.current_temperature = msg.temperature[self.active_actuator]


    def actuator_callback(self, msg):
        """액추에이터 PWM 데이터 콜백"""
        if len(msg.pwm) > self.active_actuator:
            self.current_pwm = msg.pwm[self.active_actuator]

    def target_temp_callback(self, msg):
        """목표 온도 데이터 콜백"""
        if len(msg.temperature) > self.active_actuator:
            self.current_target_temp = msg.temperature[self.active_actuator]

    def log_data(self):
        """주기적으로 데이터를 메모리에 기록"""
        # 데이터를 배열에 추가
        self.timestamps.append(self.get_clock().now().to_msg())
        self.temperatures.append(self.current_temperature)
        self.pwm_values.append(self.current_pwm)
        self.target_temperatures.append(self.current_target_temp)


        # 활성 로깅 중이라면 CSV에 바로 저장
        if self.is_logging and self.csv_writer is not None:
            timestamp = self.get_clock().now()
            datetime_str = timestamp.to_msg().sec + timestamp.to_msg().nanosec / 1e9

            self.csv_writer.writerow([
                datetime_str,
                self.current_temperature,
                self.current_pwm,
                self.current_target_temp
            ])
            if len(self.timestamps) % 100 == 0:
                self.csv_file.flush()

    def start_logging(self):
        """CSV 로깅 시작"""
        if self.is_logging:
            self.get_logger().info('이미 로깅 중입니다.')
            return

        # 로그 파일 생성
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.current_log_filename = f"temp_log_{timestamp}.csv"
        log_path = os.path.join(self.log_dir, self.current_log_filename)

        try:
            self.csv_file = open(log_path, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)

            # 헤더 작성
            self.csv_writer.writerow([
                'Timestamp',
                'Temperature (C)',
                'PWM',
                'Target Temperature (C)'
            ])

            self.is_logging = True
            self.get_logger().info(f'로깅 시작: {log_path}')

            # 현재 파일 이름 발행
            msg = String()
            msg.data = self.current_log_filename
            self.filename_publisher.publish(msg)

            return True
        except Exception as e:
            self.get_logger().error(f'로그 파일 생성 실패: {str(e)}')
            return False

    def stop_logging(self):
        """CSV 로깅 중지"""
        if not self.is_logging:
            self.get_logger().info('로깅 중이 아닙니다.')
            return False

        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None

        self.is_logging = False
        self.get_logger().info('로깅 중지됨')
        return True

    def reset_log_callback(self, request, response):
        """로그 데이터 초기화 서비스 콜백"""
        self.stop_logging()
        self.reset_data()
        response.success = True
        response.message = "로그 데이터가 초기화되었습니다."
        self.get_logger().info('로그 데이터 초기화')
        return response

    def save_log_callback(self, request, response):
        """로그 데이터 저장 서비스 콜백"""
        if request.data:
            # 로깅 시작
            result = self.start_logging()
            response.success = result
            response.message = "로깅 시작됨" if result else "로깅 시작 실패"
        else:
            # 로깅 중지
            result = self.stop_logging()
            response.success = result
            response.message = "로깅 중지됨" if result else "로깅 중이 아님"

        return response

    def save_current_data(self, custom_filename=None):
        """현재까지 메모리에 있는 데이터를 CSV로 저장"""
        # 파일 이름 생성
        if custom_filename is None:
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"{self.user_filename_prefix}_{timestamp}.csv"
        else:
            filename = custom_filename

        log_path = os.path.join(self.log_dir, filename)

        try:
            with open(log_path, 'w', newline='') as csv_file:
                csv_writer = csv.writer(csv_file)

                # 헤더 작성
                csv_writer.writerow([
                    'Timestamp',
                    'Temperature (C)',
                    'PWM',
                    'Target Temperature (C)'
                ])

                # 데이터 작성
                for i in range(len(self.timestamps)):
                    timestamp = self.timestamps[i]
                    datetime_str = timestamp.sec + timestamp.nanosec / 1e9

                    csv_writer.writerow([
                        datetime_str,
                        self.temperatures[i],
                        self.pwm_values[i],
                        self.target_temperatures[i]
                    ])

            self.get_logger().info(f'데이터가 저장되었습니다: {log_path}')
            return True, log_path
        except Exception as e:
            self.get_logger().error(f'데이터 저장 실패: {str(e)}')
            return False, None

def main(args=None):
    rclpy.init(args=args)
    node = TemperatureLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 로깅 중지 확인
        if node.is_logging:
            node.stop_logging()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
