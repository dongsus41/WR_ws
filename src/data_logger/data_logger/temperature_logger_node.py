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
        self.declare_parameter('max_buffer_size', 150000)  # 10분(250Hz * 60초 * 10) 데이터 저장 가능
        self.declare_parameter('user_filename_prefix', 'temperature_log')  # 사용자 지정 파일 이름 접두사

        # 사용자 지정 파일명 파라미터 처리
        user_filename = os.environ.get('TEMPERATURE_LOG_FILENAME', '')
        if user_filename:
            self.set_parameters([Parameter('user_filename_prefix', value=user_filename)])
            self.get_logger().info(f'환경 변수에서 파일명 설정: {user_filename}')

        # 경로 설정
        self.log_dir = os.path.expanduser(self.get_parameter('log_directory').value)
        os.makedirs(self.log_dir, exist_ok=True)
        self.get_logger().info(f'로그 디렉토리 생성: {self.log_dir}')

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

        # Control Mode 구독 추가
        self.control_mode_subscription = self.create_subscription(
            Bool,
            'control_mode',
            self.control_mode_callback,
            qos  # 제어 모드는 빠른 업데이트가 필요하지 않으므로 기본 QoS 사용
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

        # 로깅 상태 발행
        self.logging_status_publisher = self.create_publisher(
            Bool,
            'logging_status',
            10
        )

        # 서비스 파라미터 구독 설정
        self.filename_subscription = self.create_subscription(
            String,
            'set_log_filename',
            self.update_filename_callback,
            10
        )

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

        # 상태 점검 타이머 (2초마다 상태 발행)
        self.status_timer = self.create_timer(
            2.0,
            self.publish_status
        )

        # 로깅 상태
        self.is_logging = False
        self.csv_writer = None
        self.csv_file = None
        self.last_data_count = 0  # 데이터 수집 확인용

        # 마지막 수신 시간 초기화
        self.last_temperature_time = None
        self.last_pwm_time = None
        self.last_target_time = None
        self.last_control_mode_time = None

        self.get_logger().info('온도 로거 노드가 시작되었습니다.')
        self.get_logger().info(f'로그 디렉토리: {self.log_dir}')
        self.get_logger().info(f'활성 구동기: {self.active_actuator}')
        self.get_logger().info(f'로깅 주파수: {self.get_parameter("logging_frequency").value}Hz')
        self.get_logger().info(f'사용자 파일명 접두사: {self.get_parameter("user_filename_prefix").value}')
        self.get_logger().info('제어 모드(Control Mode) 로깅 기능이 추가되었습니다')

    def control_mode_callback(self, msg):
        """제어 모드 데이터 콜백 (true: 자동, false: 수동)"""
        try:
            self.current_control_mode = msg.data
            self.last_control_mode_time = self.get_clock().now()
            mode_str = "자동 제어" if msg.data else "수동 제어"
            self.get_logger().debug(f'제어 모드 데이터 수신: {mode_str}')
        except Exception as e:
            self.get_logger().error(f'제어 모드 데이터 처리 오류: {str(e)}')

    def publish_status(self):
        """주기적으로 로깅 상태 발행"""
        try:
            # 로깅 상태 발행
            status_msg = Bool()
            status_msg.data = self.is_logging
            self.logging_status_publisher.publish(status_msg)

            # 수집된 데이터 수 확인 및 로깅
            current_count = len(self.timestamps)
            if current_count != self.last_data_count:
                mode_str = "자동 제어" if self.current_control_mode else "수동 제어"
                self.get_logger().debug(f'수집된 데이터: {current_count}개, 마지막 온도: {self.current_temperature:.2f}°C, 제어 모드: {mode_str}')
                self.last_data_count = current_count

            # 현재 파일명 발행
            if self.is_logging and self.current_log_filename:
                filename_msg = String()
                filename_msg.data = self.current_log_filename
                self.filename_publisher.publish(filename_msg)

            # 데이터 수신 타임아웃 확인
            now = self.get_clock().now()
            if self.last_temperature_time:
                time_diff = (now - self.last_temperature_time).nanoseconds / 1e9
                if time_diff > 5.0:  # 5초 이상 데이터 없음
                    self.get_logger().warn(f'온도 데이터 수신 없음: {time_diff:.1f}초')

            if self.last_pwm_time:
                time_diff = (now - self.last_pwm_time).nanoseconds / 1e9
                if time_diff > 5.0:
                    self.get_logger().warn(f'PWM 데이터 수신 없음: {time_diff:.1f}초')

            if self.last_control_mode_time:
                time_diff = (now - self.last_control_mode_time).nanoseconds / 1e9
                if time_diff > 10.0:  # 제어 모드는 자주 변경되지 않으므로 타임아웃을 10초로 설정
                    mode_str = "자동 제어" if self.current_control_mode else "수동 제어"
                    self.get_logger().warn(f'제어 모드 데이터 수신 없음: {time_diff:.1f}초 (현재: {mode_str})')
        except Exception as e:
            self.get_logger().error(f'상태 발행 오류: {str(e)}')

    def update_filename_callback(self, msg):
        """사용자 지정 파일명 업데이트 콜백"""
        try:
            if msg.data:
                # ROS 2 Foxy에서는 set_parameters() 메서드 사용
                self.set_parameters([Parameter('user_filename_prefix', value=msg.data)])
                self.user_filename_prefix = msg.data  # 내부 변수도 업데이트
                self.get_logger().info(f'파일명이 업데이트 되었습니다: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'파일명 업데이트 오류: {str(e)}')

    def reset_data(self):
        """데이터 배열 초기화 - deque 사용으로 성능 개선"""
        try:
            self.max_buffer_size = self.get_parameter('max_buffer_size').value
            self.temperatures = deque(maxlen=self.max_buffer_size)
            self.pwm_values = deque(maxlen=self.max_buffer_size)
            self.target_temperatures = deque(maxlen=self.max_buffer_size)
            self.control_modes = deque(maxlen=self.max_buffer_size)  # 제어 모드 데이터 추가
            self.timestamps = deque(maxlen=self.max_buffer_size)

            # 마지막 수신 데이터 저장 변수 초기화
            self.current_temperature = 0.0
            self.current_pwm = 0
            self.current_target_temp = 0.0
            self.current_control_mode = False  # 기본값은 수동 모드(False)
            self.active_actuator = self.get_parameter('active_actuator').value
            self.user_filename_prefix = self.get_parameter('user_filename_prefix').value

            # 새 로그 파일 이름 초기화
            self.current_log_filename = None

            self.get_logger().info('로그 데이터 초기화 완료')
        except Exception as e:
            self.get_logger().error(f'데이터 초기화 오류: {str(e)}')

    def temperature_callback(self, msg):
        """온도 데이터 콜백"""
        try:
            if len(msg.temperature) > self.active_actuator:
                self.current_temperature = msg.temperature[self.active_actuator]
                self.last_temperature_time = self.get_clock().now()
                self.get_logger().debug(f'온도 데이터 수신: {self.current_temperature:.2f}°C')
            else:
                self.get_logger().warn(f'유효하지 않은 온도 데이터: 인덱스 {self.active_actuator} 범위 초과')
        except Exception as e:
            self.get_logger().error(f'온도 데이터 처리 오류: {str(e)}')

    def actuator_callback(self, msg):
        """액추에이터 PWM 데이터 콜백"""
        try:
            if len(msg.pwm) > self.active_actuator:
                self.current_pwm = msg.pwm[self.active_actuator]
                self.last_pwm_time = self.get_clock().now()
                self.get_logger().debug(f'PWM 데이터 수신: {self.current_pwm}')
            else:
                self.get_logger().warn(f'유효하지 않은 PWM 데이터: 인덱스 {self.active_actuator} 범위 초과')
        except Exception as e:
            self.get_logger().error(f'PWM 데이터 처리 오류: {str(e)}')

    def target_temp_callback(self, msg):
        """목표 온도 데이터 콜백"""
        try:
            if len(msg.temperature) > self.active_actuator:
                self.current_target_temp = msg.temperature[self.active_actuator]
                self.last_target_time = self.get_clock().now()
                self.get_logger().debug(f'목표 온도 데이터 수신: {self.current_target_temp:.2f}°C')
            else:
                self.get_logger().warn(f'유효하지 않은 목표 온도 데이터: 인덱스 {self.active_actuator} 범위 초과')
        except Exception as e:
            self.get_logger().error(f'목표 온도 데이터 처리 오류: {str(e)}')

    def log_data(self):
        """주기적으로 데이터를 메모리에 기록"""
        try:
            # 데이터를 배열에 추가
            self.timestamps.append(self.get_clock().now().to_msg())
            self.temperatures.append(self.current_temperature)
            self.pwm_values.append(self.current_pwm)
            self.target_temperatures.append(self.current_target_temp)
            self.control_modes.append(self.current_control_mode)  # 제어 모드 추가

            # 활성 로깅 중이라면 CSV에 바로 저장
            if self.is_logging and self.csv_writer is not None:
                try:
                    timestamp = self.get_clock().now()
                    datetime_str = timestamp.to_msg().sec + timestamp.to_msg().nanosec / 1e9

                    # 제어 모드 문자열 변환 (데이터 분석 시 가독성을 위해)
                    control_mode_str = "Auto" if self.current_control_mode else "Manual"

                    self.csv_writer.writerow([
                        datetime_str,
                        self.current_temperature,
                        self.current_pwm,
                        self.current_target_temp,
                        control_mode_str  # 제어 모드 추가
                    ])

                    # 주기적으로 파일을 디스크에 플러시
                    if len(self.timestamps) % 100 == 0:
                        if self.csv_file:
                            self.csv_file.flush()
                            self.get_logger().debug(f'로그 파일 플러시 완료: {len(self.timestamps)}개 데이터')
                except Exception as e:
                    self.get_logger().error(f'CSV 작성 중 오류: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'데이터 로깅 중 오류: {str(e)}')

    def start_logging(self, custom_filename=None):
        """CSV 로깅 시작"""
        if self.is_logging:
            self.get_logger().info('이미 로깅 중입니다.')
            return True  # 이미 로깅 중이므로 성공 반환

        # 로그 파일 생성
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

        # 사용자 지정 파일명 사용 (있는 경우)
        prefix = custom_filename if custom_filename else self.user_filename_prefix
        self.current_log_filename = f"{prefix}_{timestamp}.csv"
        log_path = os.path.join(self.log_dir, self.current_log_filename)

        try:
            self.csv_file = open(log_path, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)

            # 헤더 작성 (Control Mode 칼럼 추가)
            self.csv_writer.writerow([
                'Timestamp',
                'Temperature (C)',
                'PWM',
                'Target Temperature (C)',
                'Control Mode'  # Auto 또는 Manual
            ])

            self.is_logging = True
            self.get_logger().info(f'로깅 시작: {log_path}')

            # 현재 파일 이름 발행
            msg = String()
            msg.data = self.current_log_filename
            self.filename_publisher.publish(msg)

            # 로깅 상태 발행
            status_msg = Bool()
            status_msg.data = True
            self.logging_status_publisher.publish(status_msg)

            return True
        except Exception as e:
            self.get_logger().error(f'로그 파일 생성 실패: {str(e)}')
            return False

    def stop_logging(self):
        """CSV 로깅 중지"""
        if not self.is_logging:
            self.get_logger().info('로깅 중이 아닙니다.')
            return True  # 이미 로깅 중이 아니므로 성공 반환

        try:
            if self.csv_file is not None:
                # 파일에 남은 데이터를 모두 기록
                self.csv_file.flush()
                self.csv_file.close()
                self.csv_file = None
                self.csv_writer = None

            self.is_logging = False
            self.get_logger().info('로깅 중지됨')

            # 로깅 상태 발행
            status_msg = Bool()
            status_msg.data = False
            self.logging_status_publisher.publish(status_msg)

            return True
        except Exception as e:
            self.get_logger().error(f'로깅 중지 중 오류: {str(e)}')
            return False

    def reset_log_callback(self, request, response):
        """로그 데이터 초기화 서비스 콜백"""
        try:
            # 로깅 중지
            if self.is_logging:
                self.stop_logging()

            # 데이터 초기화
            self.reset_data()

            response.success = True
            response.message = "로그 데이터가 초기화되었습니다."
            self.get_logger().info('로그 데이터 초기화 완료')
            return response
        except Exception as e:
            response.success = False
            response.message = f"로그 데이터 초기화 오류: {str(e)}"
            self.get_logger().error(f'로그 데이터 초기화 오류: {str(e)}')
            return response

    def save_log_callback(self, request, response):
        """로그 데이터 저장 서비스 콜백"""
        try:
            # ROS 2 Foxy에서는 request.data 속성을 직접 확인
            if hasattr(request, 'data'):
                if request.data:
                    # 로깅 시작
                    custom_filename = None

                    # 현재 파라미터에서 사용자 지정 파일명 가져오기
                    custom_filename = self.user_filename_prefix
                    self.get_logger().info(f'사용자 지정 파일명으로 로깅 시작: {custom_filename}')

                    result = self.start_logging(custom_filename)
                    response.success = result
                    response.message = "로깅 시작됨" if result else "로깅 시작 실패"
                else:
                    # 로깅 중지
                    result = self.stop_logging()
                    response.success = result
                    response.message = "로깅 중지됨" if result else "로깅 중지 실패"
            else:
                # 예상치 못한 요청 타입
                response.success = False
                response.message = "잘못된, 또는 지원되지 않는 요청 타입입니다."
                self.get_logger().error('잘못된 요청 타입입니다')

            return response
        except Exception as e:
            response.success = False
            response.message = f"서비스 처리 중 오류: {str(e)}"
            self.get_logger().error(f'로그 저장 서비스 처리 오류: {str(e)}')
            return response

    def save_current_data(self, custom_filename=None):
        """현재까지 메모리에 있는 데이터를 CSV로 저장"""
        # 데이터가 없으면 저장하지 않음
        if not self.timestamps or len(self.timestamps) == 0:
            self.get_logger().warn('저장할 데이터가 없습니다.')
            return False, "저장할 데이터가 없습니다."

        try:
            # 파일 이름 생성
            if custom_filename is None:
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"{self.user_filename_prefix}_{timestamp}.csv"
            else:
                filename = custom_filename

            log_path = os.path.join(self.log_dir, filename)

            with open(log_path, 'w', newline='') as csv_file:
                csv_writer = csv.writer(csv_file)

                # 헤더 작성 (Control Mode 칼럼 추가)
                csv_writer.writerow([
                    'Timestamp',
                    'Temperature (C)',
                    'PWM',
                    'Target Temperature (C)',
                    'Control Mode'  # Auto 또는 Manual
                ])

                # 데이터 작성
                for i in range(len(self.timestamps)):
                    timestamp = self.timestamps[i]
                    datetime_str = timestamp.sec + timestamp.nanosec / 1e9

                    # 제어 모드를 문자열로 변환
                    control_mode_str = "Auto" if self.control_modes[i] else "Manual"

                    csv_writer.writerow([
                        datetime_str,
                        self.temperatures[i],
                        self.pwm_values[i],
                        self.target_temperatures[i],
                        control_mode_str
                    ])

            self.get_logger().info(f'데이터가 저장되었습니다: {log_path} (총 {len(self.timestamps)}개 데이터)')
            return True, log_path
        except Exception as e:
            error_msg = f'데이터 저장 실패: {str(e)}'
            self.get_logger().error(error_msg)
            return False, error_msg

def main(args=None):
    rclpy.init(args=args)
    try:
        node = TemperatureLoggerNode()
        rclpy.spin(node)
    except Exception as e:
        print(f"노드 실행 중 오류 발생: {str(e)}")
    finally:
        try:
            # 노드 정리
            if 'node' in locals() and node.is_logging:
                node.stop_logging()
            if 'node' in locals():
                node.destroy_node()
        except Exception as e:
            print(f"노드 정리 중 오류 발생: {str(e)}")
        rclpy.shutdown()

if __name__ == '__main__':
    main()

