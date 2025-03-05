#!/usr/bin/env python3

import os
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from wearable_robot_interfaces.msg import ActuatorCommand, TemperatureData
from std_msgs.msg import Bool, Float64
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QMessageBox
from python_qt_binding.QtCore import Qt, QTimer



class ActuatorControlPlugin(Plugin):
    """
    웨어러블 로봇의 구동기 제어 및 온도 모니터링을 위한 rqt 플러그인
    """
    def __init__(self, context):
        super(ActuatorControlPlugin, self).__init__(context)
        # 플러그인 제목 설정
        self.setObjectName('ActuatorControlPlugin')

        try:
            import rclpy
            if not rclpy.ok():
                rclpy.init()

            import time
            node_name = f'actuator_control_plugin_{int(time.time())}'
            self.node = Node(node_name)

            self.node.get_logger().info(f"노드 생성 성공: {node_name}")

            # 여기서 두 번째 노드 생성 코드를 제거하고, 바로 QoS 설정으로 넘어갑니다

            # QoS 설정
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )

        except Exception as e:
            print(f"[ERROR]: 노드 생성 실패: {str(e)}")


        # 발행자 생성
        self.actuator_pub = self.node.create_publisher(
            ActuatorCommand, 'direct_pwm_command', qos_profile)  # 수동 제어용 직접 PWM 명령
        self.target_temp_pub = self.node.create_publisher(
            TemperatureData, 'target_temperature', qos_profile)  # 자동 제어용 목표 온도
        self.kp_pub = self.node.create_publisher(
            Float64, 'kp_param', qos_profile)  # PI 제어기 Kp 게인
        self.ki_pub = self.node.create_publisher(
            Float64, 'ki_param', qos_profile)  # PI 제어기 Ki 게인
        self.emergency_pub = self.node.create_publisher(
            Bool, 'emergency_stop', qos_profile)  # 비상 정지 명령
        self.control_mode_publisher = self.node.create_publisher(
            Bool, 'control_mode', qos_profile)  # 제어 모드 (자동/수동)

        # 구독자 생성
        self.temp_sub = self.node.create_subscription(
            TemperatureData, 'temperature_data', self.temp_callback, qos_profile)  # 온도 데이터
        self.pwm_sub = self.node.create_subscription(
            ActuatorCommand, 'pwm_state', self.pwm_callback, qos_profile)  # 현재 PWM 상태

        # 주기적인 작업을 위한 타이머 (1초마다 안전 검사)
        self.timer = self.node.create_timer(1.0, self.timer_callback)

        # ROS 2 스핀을 위한 스레드 생성
        self.spin_thread = threading.Thread(target=self.spin_ros)
        self.spin_thread.daemon = True
        self.spin_thread.start()

        # UI 설정
        self._widget = QWidget()

        # UI 파일 경로를 찾기 위한 여러 가능한 위치를 시도
        ui_file = None
        candidate_paths = [
            # 1. 패키지 설치 경로 내 resource 디렉토리
            os.path.join(os.path.dirname(os.path.realpath(__file__)), 'resource', 'actuator_control.ui'),
            # 2. 현재 디렉토리의 상위 디렉토리의 resource
            os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'resource', 'actuator_control.ui'),
            # 3. 공유 리소스 디렉토리 (ROS 2 표준)
            os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), 'share',
                        'wearable_robot_rqt_plugins', 'resource', 'actuator_control.ui'),
            # 4. 패키지 리소스 디렉토리
            os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), 'resource', 'actuator_control.ui'),
        ]

        # 각 경로를 시도하여 존재하는 첫 번째 경로 사용
        for path in candidate_paths:
            if os.path.exists(path):
                ui_file = path
                self.node.get_logger().info(f"UI 파일을 찾았습니다: {ui_file}")
                break

        if ui_file is None:
            raise FileNotFoundError("actuator_control.ui 파일을 찾을 수 없습니다. 설치가 올바르게 되었는지 확인하세요.")

        loadUi(ui_file, self._widget)
        self._widget.setObjectName('ActuatorControlPluginUI')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        # UI 이벤트 연결
        self._widget.manual_mode_radio.toggled.connect(self.mode_changed)
        self._widget.auto_mode_radio.toggled.connect(self.mode_changed)
        self._widget.apply_button.clicked.connect(self.apply_settings)
        self._widget.emergency_stop_button.clicked.connect(self.emergency_stop)

        # PWM 슬라이더와 스핀박스 연결 (값 동기화)
        self._widget.pwm_slider.valueChanged.connect(self._widget.pwm_spin.setValue)
        self._widget.pwm_spin.valueChanged.connect(self._widget.pwm_slider.setValue)

        # 내부 상태 변수 초기화
        self.temperature = 0.0              # 현재 온도
        self.current_pwm = 0                # 현재 PWM 값
        self.auto_mode = False              # 자동 제어 모드 여부
        self.is_emergency_stop = False      # 비상 정지 상태
        self.safety_temp_threshold = 80.0   # 안전 온도 임계값 (°C)

        # Qt 타이머 추가 (UI 업데이트용)
        self.qt_timer = QTimer()
        self.qt_timer.timeout.connect(self.update_ui)
        self.qt_timer.start(250)  # 250ms 마다 UI 업데이트

        # 초기화를 완료했음을 표시
        self._widget.status_label.setText('플러그인이 초기화되었습니다')
        self.node.get_logger().info('구동기 제어 플러그인이 초기화되었습니다')

        # 초기 모드 발행 (기본: 수동 모드)
        self.publish_control_mode(False)

    def publish_control_mode(self, auto_mode):
        """
        제어 모드(자동/수동)를 발행합니다.

        Args:
            auto_mode (bool): True이면 자동 온도 제어 모드, False이면 수동 PWM 제어 모드
        """
        if not hasattr(self, 'control_mode_publisher'):
            self.control_mode_publisher = self.node.create_publisher(
                Bool, 'control_mode', 10)

        msg = Bool()
        msg.data = auto_mode
        self.control_mode_publisher.publish(msg)

        mode_str = "자동 온도 제어" if auto_mode else "수동 PWM 제어"
        self.node.get_logger().info(f"제어 모드 변경: {mode_str}")

        # UI 상태 업데이트 (모드에 따라 활성화/비활성화할 위젯 설정)
        if hasattr(self, '_widget'):
            # 자동 모드일 때는 온도 설정 활성화, PWM 설정 비활성화
            self._widget.target_temp_spin.setEnabled(auto_mode)
            self._widget.pwm_slider.setEnabled(not auto_mode)
            self._widget.pwm_spin.setEnabled(not auto_mode)

    def shutdown_plugin(self):
        """플러그인이 종료될 때 호출되는 메서드"""
        # Qt 타이머 중지
        if hasattr(self, 'qt_timer') and self.qt_timer.isActive():
            self.qt_timer.stop()

        # 비상 정지 해제 (안전을 위해)
        if hasattr(self, 'emergency_pub'):
            msg = Bool()
            msg.data = False
            self.emergency_pub.publish(msg)

        # ROS 노드 정리
        if hasattr(self, 'node') and self.node:
            self.node.get_logger().info("플러그인 종료 중...")
            self.node.destroy_node()

        # 스핀 스레드 종료 대기
        if hasattr(self, 'spin_thread') and self.spin_thread.is_alive():
            self.spin_thread.join(timeout=1.0)  # 최대 1초 대기

        print("[INFO]: 플러그인 자원이 정상적으로 해제되었습니다")

    def spin_ros(self):
        """ROS 2 노드 스핀을 위한 메서드"""
        try:
            # SingleThreadedExecutor 사용
            from rclpy.executors import SingleThreadedExecutor
            executor = SingleThreadedExecutor()
            executor.add_node(self.node)

            # executor 스핀
            while rclpy.ok():
                executor.spin_once(timeout_sec=0.1)  # 100ms 타임아웃으로 반응성 유지
        except Exception as e:
            self.node.get_logger().error(f"ROS 스핀 스레드 오류: {str(e)}")
        finally:
            if hasattr(self, 'node') and self.node:
                self.node.get_logger().info("스핀 스레드가 종료됩니다")

    def timer_callback(self):
        """
        주기적인 안전 검사 및 상태 모니터링을 위한 타이머 콜백
        """
        # 온도 안전 검사
        self.check_temperature_safety()

    def update_ui(self):
        """
        UI를 업데이트하는 함수 (Qt 타이머에서 호출)
        """
        if not hasattr(self, '_widget') or self._widget is None:
            return

        try:
        # UI 위젯이 유효한지 확인

            # 온도와 PWM 값 업데이트
            self._widget.temp_label.setText(f"{self.temperature:.1f}°C")
            self._widget.pwm_label.setText(f"{self.current_pwm}")

            # 비상 정지 상태 표시
            if self.is_emergency_stop:
                self._widget.status_label.setText("비상 정지 상태")
                self._widget.status_label.setStyleSheet("color: red; font-weight: bold;")

                # 비상 정지 중에는 버튼 상태 조정
                self._widget.apply_button.setEnabled(False)
                self._widget.emergency_stop_button.setText("비상 정지 해제")
                # 모드 변경 비활성화
                self._widget.manual_mode_radio.setEnabled(False)
                self._widget.auto_mode_radio.setEnabled(False)
                # 설정 그룹 비활성화
                self._widget.manual_settings_group.setEnabled(False)
                self._widget.auto_settings_group.setEnabled(False)
            else:
                # 비상 정지가 아닌 경우 버튼 상태 복원
                self._widget.apply_button.setEnabled(True)
                self._widget.emergency_stop_button.setText("비상 정지")
                # 모드 변경 활성화
                self._widget.manual_mode_radio.setEnabled(True)
                self._widget.auto_mode_radio.setEnabled(True)
                # 현재 모드에 따른 설정 그룹 활성화
                if self.auto_mode:
                    self._widget.auto_settings_group.setEnabled(True)
                    self._widget.manual_settings_group.setEnabled(False)
                else:
                    self._widget.auto_settings_group.setEnabled(False)
                    self._widget.manual_settings_group.setEnabled(True)
        except RuntimeError as e:
            if "deleted" in str(e):
                # 위젯이 삭제된 경우 타이머 중지
                if hasattr(self, 'qt_timer'):
                    self.qt_timer.stop()

    def temp_callback(self, msg):
        """
        온도 데이터 수신 콜백
        """
        try:
            # 구동기 5번(인덱스 5) 온도 데이터 저장
            if len(msg.temperature) > 5:
                self.temperature = msg.temperature[5]
                self.node.get_logger().debug(f'구동기 5번 온도: {self.temperature:.1f}°C')
        except Exception as e:
            self.node.get_logger().error(f'온도 데이터 처리 오류: {str(e)}')

    def pwm_callback(self, msg):
        """
        현재 PWM 상태 수신 콜백
        """
        try:
            # 구동기 5번(인덱스 5) PWM 값 저장
            if len(msg.pwm) > 5:
                self.current_pwm = msg.pwm[5]
                self.node.get_logger().debug(f'구동기 5번 현재 PWM: {self.current_pwm}')
        except Exception as e:
            self.node.get_logger().error(f'PWM 데이터 처리 오류: {str(e)}')

    def check_temperature_safety(self):
        """
        온도가 안전 범위 내에 있는지 확인하고 필요시 비상 정지 활성화
        """
        if self.temperature >= self.safety_temp_threshold and not self.is_emergency_stop:
            self.node.get_logger().error(
                f'온도 임계값 초과! 현재: {self.temperature:.1f}°C, 임계값: {self.safety_temp_threshold:.1f}°C')

            # 비상 정지 활성화
            self.is_emergency_stop = True
            self.publish_emergency_stop(True)
            self.publish_actuator_command(0)  # PWM 0으로 설정

            # UI 업데이트 (비동기적으로 실행되므로 메인 스레드에서 처리 필요)
            # Qt의 스레드 안전성을 위해 QTimer.singleShot 사용
            QTimer.singleShot(0, lambda: self._show_temperature_warning())

    def _show_temperature_warning(self):
        """
        온도 경고 메시지 표시 (메인 스레드에서 실행)
        """
        warning_msg = QMessageBox()
        warning_msg.setIcon(QMessageBox.Warning)
        warning_msg.setWindowTitle("온도 경고")
        warning_msg.setText(f"구동기 5번의 온도가 안전 임계값을 초과했습니다!")
        warning_msg.setInformativeText(f"현재 온도: {self.temperature:.1f}°C\n임계값: {self.safety_temp_threshold:.1f}°C\n\n비상 정지가 활성화되었습니다.")
        warning_msg.setStandardButtons(QMessageBox.Ok)
        warning_msg.exec_()

    def mode_changed(self, checked):
        """
        제어 모드 변경 이벤트 처리
        """
        if self._widget.auto_mode_radio.isChecked():
            self.auto_mode = True
            self._widget.auto_settings_group.setEnabled(True)
            self._widget.manual_settings_group.setEnabled(False)
            self._widget.status_label.setText('자동 모드 활성화')
            self._widget.status_label.setStyleSheet("")

            # 자동 모드로 전환 시 현재 설정된 파라미터 즉시 발행
            self.publish_control_parameters()
            self.node.get_logger().info('자동 제어 모드로 전환합니다')
        else:
            self.auto_mode = False
            self._widget.auto_settings_group.setEnabled(False)
            self._widget.manual_settings_group.setEnabled(True)
            self._widget.status_label.setText('수동 모드 활성화')
            self._widget.status_label.setStyleSheet("")

            self.node.get_logger().info('수동 제어 모드로 전환합니다')

    def apply_settings(self):
        """
        현재 UI 설정에 따라 제어 명령 발행
        """
        if self.is_emergency_stop:
            self.node.get_logger().warn('비상 정지 중입니다. 제어 명령이 무시됩니다.')
            return  # 비상 정지 중에는 명령 무시

        if self.auto_mode:
            # 자동 모드: 목표 온도와 PI 제어 파라미터 발행
            self.publish_control_parameters()
            self._widget.status_label.setText('자동 제어 파라미터가 적용되었습니다')
            self.node.get_logger().info(f'자동 제어 파라미터 적용: 목표 온도={self._widget.target_temp_spin.value():.1f}°C, Kp={self._widget.kp_spin.value():.2f}, Ki={self._widget.ki_spin.value():.3f}')
        else:
            # 수동 모드: PWM 값 직접 설정
            pwm_value = self._widget.pwm_spin.value()
            self.publish_actuator_command(pwm_value)
            self._widget.status_label.setText(f'구동기 5번 PWM이 {pwm_value}으로 설정되었습니다')
            self.node.get_logger().info(f'수동 PWM 제어 적용: 구동기 5번, PWM={pwm_value}')

    def publish_control_parameters(self):
        """
        자동 제어를 위한 파라미터 발행
        """
        try:
            # 목표 온도 발행
            target_temp_msg = TemperatureData()
            target_temp_msg.header.stamp = self.node.get_clock().now().to_msg()

            target_temp_msg.temperature = [0.0] * 6

            target_temp_msg.temperature[5] = self._widget.target_temp_spin.value()
            self.target_temp_pub.publish(target_temp_msg)

            # Kp 값 발행
            kp_msg = Float64()
            kp_msg.data = self._widget.kp_spin.value()
            self.kp_pub.publish(kp_msg)

            # Ki 값 발행
            ki_msg = Float64()
            ki_msg.data = self._widget.ki_spin.value()
            self.ki_pub.publish(ki_msg)

            self.node.get_logger().debug('제어 파라미터 발행 완료')
        except Exception as e:
            self.node.get_logger().error(f'제어 파라미터 발행 오류: {str(e)}')

    def emergency_stop(self):
        """
        비상 정지 버튼 이벤트 처리
        """
        self.is_emergency_stop = not self.is_emergency_stop

        # 비상 정지 메시지 발행
        emergency_msg = Bool()
        emergency_msg.data = self.is_emergency_stop
        self.emergency_pub.publish(emergency_msg)

        if self.is_emergency_stop:
            # 비상 정지 활성화: 모든 구동기 정지
            self.publish_actuator_command(0)
            self._widget.status_label.setText("비상 정지가 활성화되었습니다")
            self._widget.status_label.setStyleSheet("color: red; font-weight: bold;")
            self.node.get_logger().error('비상 정지가 활성화되었습니다!')
        else:
            # 비상 정지 해제
            self._widget.status_label.setText("비상 정지가 해제되었습니다")
            self._widget.status_label.setStyleSheet("")
            self.node.get_logger().info('비상 정지가 해제되었습니다')

    def publish_emergency_stop(self, activate):
        """
        비상 정지 메시지 발행
        """
        emergency_msg = Bool()
        emergency_msg.data = activate
        self.emergency_pub.publish(emergency_msg)
        self.node.get_logger().debug(f'비상 정지 메시지 발행: {activate}')

    def publish_actuator_command(self, pwm_value):
        """
        구동기 제어 명령 발행 (수동 모드용 직접 PWM 명령)
        """
        msg = ActuatorCommand()
        msg.header.stamp = self.node.get_clock().now().to_msg()

        # 모든 채널 0으로 초기화
        msg.pwm = [0, 0, 0, 0, 0, 0]

        # 구동기 5번만 설정 (인덱스는 0부터 시작하므로 인덱스 5)
        msg.pwm[5] = pwm_value

        self.actuator_pub.publish(msg)
        self.node.get_logger().debug(f'구동기 명령 발행: 구동기 5번 PWM = {pwm_value}')

def main(args=None):
    """
    rqt 플러그인 메인 함수 (독립 실행 모드에서 사용)
    """
    # 독립 실행 모드일 때만 rclpy.init을 호출합니다
    # rqt 내에서 실행될 때는 이미 초기화되어 있으므로 호출하지 않습니다

    from rqt_gui.main import Main
    main = Main()
    sys.exit(main.main(sys.argv, standalone='wearable_robot_rqt_plugins.actuator_control_plugin.ActuatorControlPlugin'))

if __name__ == '__main__':
    import sys

    # 독립 실행 모드일 때는 rclpy를 직접 초기화합니다
    rclpy.init(args=sys.argv)
    main()



