#!/usr/bin/env python3

import os
import rclpy
import rclpy.node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from wearable_robot_interfaces.msg import FanCommand, TemperatureData, ActuatorCommand
from std_msgs.msg import Float64
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QRadioButton, QDoubleSpinBox, QCheckBox, QLineEdit
from python_qt_binding.QtCore import Qt, QTimer
import threading
import pyqtgraph as pg
import numpy as np
from datetime import datetime, timedelta
import csv

class TempControlPlugin(Plugin):
    """
    웨어러블 로봇의 온도 모니터링 및 팬 제어를 위한 rqt 플러그인

    주요 기능:
    - 실시간 온도 그래프 모니터링
    - 자동/수동 팬 제어 모드
    - PID 제어 파라미터 설정
    - 데이터 로깅 및 저장
    """
    def __init__(self, context):
        super(FanControlPlugin, self).__init__(context)
        # 플러그인 제목 설정
        self.setObjectName('TempControlPlugin')

        # ROS 2 노드 생성
        self.node = rclpy.create_node('temp_control_plugin')

        # QoS 설정
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 발행자 및 구독자 생성
        self.pwm_pub = self.node.create_publisher(
            ActuatorCommand, 'actuator_command', qos_profile)
        self.kp_pub = self.node.create_publisher(
            Float64, 'kp_param', qos_profile)
        self.ki_pub = self.node.create_publisher(
            Float64, 'ki_param', qos_profile)
        self.target_temp_pub = self.node.create_publisher(
            TemperatureData, 'target_temperature', qos_profile)
        self.temp_sub = self.node.create_subscription(
            TemperatureData, 'temperature_data', self.temp_callback, qos_profile)

        # UI 설정
        self._widget = QWidget()

        # UI 파일 경로를 찾기 위한 여러 가능한 위치를 시도
        ui_file = None
        candidate_paths = [
            # 1. 패키지 설치 경로 내 resource 디렉토리
            os.path.join(os.path.dirname(os.path.realpath(__file__)), 'resource', 'temp_control.ui'),
            # 2. 현재 디렉토리의 상위 디렉토리의 resource
            os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'resource', 'temp_control.ui'),
            # 3. 공유 리소스 디렉토리 (ROS 2 표준)
            os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), 'share',
                        'wearable_robot_rqt_plugins', 'resource', 'temp_control.ui'),
            # 4. 패키지 리소스 디렉토리
            os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), 'resource', 'temp_control.ui'),
        ]

        # 각 경로를 시도하여 존재하는 첫 번째 경로 사용
        for path in candidate_paths:
            if os.path.exists(path):
                ui_file = path
                self.node.get_logger().info(f"UI 파일을 찾았습니다: {ui_file}")
                break

        if ui_file is None:
            raise FileNotFoundError("temp_control.ui 파일을 찾을 수 없습니다. 설치가 올바르게 되었는지 확인하세요.")

        loadUi(ui_file, self._widget)
        self._widget.setObjectName('TempControlPluginUI')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        # 내부 변수 초기화
        self.initialize_variables()

        # UI 연결 설정
        self.setup_ui_connections()

        # 그래프 초기화
        self.setup_graphs()

        # 타이머 설정 (그래프 업데이트 및 제어 로직)
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_plots)
        self.update_timer.start(40)  # 40ms = 25Hz

        # ROS 2 스핀을 위한 스레드 생성
        self.spin_thread = threading.Thread(target=self.spin_ros)
        self.spin_thread.daemon = True
        self.spin_thread.start()

        # 초기화 완료
        self._widget.status_label.setText('플러그인이 초기화되었습니다')

    def initialize_variables(self):
        """
        내부 변수 초기화
        """
        # 온도 및 팬 상태 데이터
        self.temperatures = [0.0] * 6
        self.pwm_values = [0] * 6

        # 그래프 데이터
        self.time_data = []
        self.temp_data = [[] for _ in range(6)]
        self.pwm_data = [[] for _ in range(6)]
        self.auto_mode = False
        self.graph_running = False

        # PID 제어 변수
        self.integral = 0.0
        self.prev_error = 0.0

        # 로깅 변수
        self.log_enabled = False
        self.log_file = None

        # 데이터 저장 변수
        self.start_time = datetime.now()

    def setup_ui_connections(self):
        """
        UI 요소와 콜백 함수 연결
        """
        # 모드 선택 라디오 버튼
        self._widget.manual_mode_radio.toggled.connect(self.mode_changed)
        self._widget.auto_mode_radio.toggled.connect(self.mode_changed)

        # 그래프 제어 버튼
        self._widget.GraphStart.clicked.connect(self.start_graph)
        self._widget.StopBtn.clicked.connect(self.stop_graph)

        # 데이터 관리 버튼
        self._widget.DataSave.clicked.connect(self.save_data)
        self._widget.DataZero.clicked.connect(self.clear_data)
        self._widget.enable_logging_checkbox.toggled.connect(self.toggle_logging)

        # PWM 제어 버튼
        self._widget.apply_pwm_button.clicked.connect(self.apply_pwm)

        # PI 게인 버튼
        self._widget.PI_apply.clicked.connect(self.apply_pi_gains)

        # 온도값 설정 적용
        self._widget.target_temp_spin.valueChanged.connect(self.update_target_temperature)
        self._widget.limit_temp_spin.valueChanged.connect(self.update_limit_temperature)

    def setup_graphs(self):
        """
        그래프 초기화 설정
        """
        # 온도 그래프 설정
        self._widget.plot1.setBackground('w')
        self._widget.plot1.setTitle("온도 모니터링", color="k")
        self._widget.plot1.setLabel('left', '온도 (°C)', color="k")
        self._widget.plot1.setLabel('bottom', '시간 (초)', color="k")
        self._widget.plot1.showGrid(x=True, y=True)
        self._widget.plot1.setYRange(20, 90)

        # PWM 그래프 설정
        self._widget.plot2.setBackground('w')
        self._widget.plot2.setTitle("PWM 출력", color="k")
        self._widget.plot2.setLabel('left', 'PWM 듀티 사이클 (%)', color="k")
        self._widget.plot2.setLabel('bottom', '시간 (초)', color="k")
        self._widget.plot2.showGrid(x=True, y=True)
        self._widget.plot2.setYRange(0, 100)

        # 그래프 곡선 초기화 (각 채널별로 다른 색상)
        self.temp_curves = []
        self.pwm_curves = []

        colors = ['r', 'g', 'b', 'c', 'm', 'y']
        for i in range(6):
            temp_curve = self._widget.plot1.plot(pen=pg.mkPen(color=colors[i], width=2), name=f"구동기 {i+1}")
            pwm_curve = self._widget.plot2.plot(pen=pg.mkPen(color=colors[i], width=2), name=f"구동기 {i+1}")
            self.temp_curves.append(temp_curve)
            self.pwm_curves.append(pwm_curve)

        # 그래프 범례 추가
        self.legend1 = self._widget.plot1.addLegend()
        self.legend2 = self._widget.plot2.addLegend()

    def spin_ros(self):
        """
        별도의 스레드에서 ROS 2 스핀을 실행합니다.
        """
        try:
            # 노드가 활성화된 동안만 스핀합니다
            while rclpy.ok() and self.node.get_node_names():
                rclpy.spin_once(self.node, timeout_sec=0.1)
        except Exception as e:
            if rclpy.ok():  # ROS가 여전히 활성화된 상태에서 오류가 발생한 경우만 로그
                self.node.get_logger().error(f'Error in ROS spin thread: {str(e)}')

    def temp_callback(self, msg):
        """
        온도 데이터 수신 콜백
        """
        # 온도 데이터 저장
        for i in range(min(len(msg.temperature), 6)):
            self.temperatures[i] = msg.temperature[i]

        # 현재 시간 계산 (시작 시점부터의 경과 시간)
        current_time = (datetime.now() - self.start_time).total_seconds()

        # 그래프가 실행 중인 경우에만 데이터 추가
        if self.graph_running:
            # 시간 데이터 추가
            self.time_data.append(current_time)

            # 온도 및 PWM 데이터 추가
            for i in range(6):
                self.temp_data[i].append(self.temperatures[i])
                self.pwm_data[i].append(self.pwm_values[i])

            # 자동 모드인 경우 온도 기반 제어 로직 실행
            if self.auto_mode:
                self.process_auto_mode()

            # 데이터 로깅
            if self.log_enabled and self.log_file:
                self.log_data()

        # LCD 디스플레이 업데이트 (5번 구동기 온도 표시)
        if len(self.temperatures) >= 5:  # 인덱스는 0부터 시작하므로 5번째는 인덱스 4
            self._widget.CurrentTemp.display(f"{self.temperatures[4]:.1f}")
        else:
            self._widget.CurrentTemp.display("0.0")

    def update_plots(self):
        """
        그래프 업데이트 함수 (타이머에 의해 주기적으로 호출)
        """
        if not self.graph_running or len(self.time_data) == 0:
            return

        # 최대 표시할 데이터 포인트 수 (약 5분치 데이터)
        max_points = 7500  # 25Hz에서 5분 = 7500개 포인트

        # 데이터가 너무 많으면 오래된 데이터 제거
        if len(self.time_data) > max_points:
            self.time_data = self.time_data[-max_points:]
            for i in range(6):
                self.temp_data[i] = self.temp_data[i][-max_points:]
                self.pwm_data[i] = self.pwm_data[i][-max_points:]

        # 그래프 업데이트
        for i in range(6):
            self.temp_curves[i].setData(self.time_data, self.temp_data[i])
            self.pwm_curves[i].setData(self.time_data, self.pwm_data[i])

    def process_auto_mode(self):
        """
        자동 모드 동작 처리 (온도 기반 PID 제어)
        """
        # 목표 온도 및 제한 온도 가져오기
        target_temp = self._widget.target_temp_spin.value()
        limit_temp = self._widget.limit_temp_spin.value()

        # 주 제어 대상 (구동기 5번, 인덱스 4)
        main_actuator_idx = 4
        current_temp = self.temperatures[main_actuator_idx]

        # 목표 온도 메시지 발행
        target_temp_msg = Float64()
        target_temp_msg.data = float(target_temp)
        self.target_temp_pub.publish(target_temp_msg)

        # 온도가 제한 온도를 초과하면 경고
        if current_temp >= limit_temp:
            self._widget.status_label.setText(f"경고: 온도 {current_temp:.1f}°C가 제한 온도를 초과했습니다!")
            self._widget.status_label.setStyleSheet("color: red; font-weight: bold")

            # 팬 최대로 가동
            for i in range(6):
                self.pwm_values[i] = 100

            # PWM 값 발행
            self.publish_pwm_command()
            return

        # PID 제어 파라미터
        kp = self._widget.Kp_spin.value()
        ki = self._widget.Ki_spin.value()

        # PID 파라미터 메시지 발행
        kp_msg = Float64()
        ki_msg = Float64()
        kp_msg.data = float(kp)
        ki_msg.data = float(ki)
        self.kp_pub.publish(kp_msg)
        self.ki_pub.publish(ki_msg)

        # 오차 계산
        error = target_temp - current_temp

        # 적분항 갱신 (dt = 0.1초 가정)
        self.integral += error * 0.1

        # Anti-windup (적분 포화 방지)
        if self.integral > 100.0:
            self.integral = 100.0
        elif self.integral < 0.0:
            self.integral = 0.0

        # PID 제어 출력 계산
        output = kp * error + ki * self.integral

        # 출력 제한
        if output > 100.0:
            output = 100.0
        elif output < 0.0:
            output = 0.0

        # 출력값 설정 (모든 구동기에 동일한 값 적용)
        for i in range(6):
            self.pwm_values[i] = int(output)

        # PWM 값 발행
        self.publish_pwm_command()

        # 상태 업데이트
        self._widget.status_label.setText(f"자동 제어 중: 목표={target_temp:.1f}°C, 현재={current_temp:.1f}°C, PWM={int(output)}%")

    def mode_changed(self, checked):
        """
        제어 모드 변경 이벤트 처리
        """
        if self._widget.auto_mode_radio.isChecked():
            self.auto_mode = True
            self._widget.auto_settings_group.setEnabled(True)
            self._widget.pid_settings_group.setEnabled(False)
            self._widget.status_label.setText('자동 모드 활성화')

            # PID 제어 초기화
            self.integral = 0.0
            self.prev_error = 0.0
        else:
            self.auto_mode = False
            self._widget.auto_settings_group.setEnabled(False)
            self._widget.pid_settings_group.setEnabled(True)
            self._widget.status_label.setText('수동 모드 활성화')

    def start_graph(self):
        """
        그래프 모니터링 시작
        """
        self.graph_running = True
        self.start_time = datetime.now()
        self.time_data = []

        # 각 채널별 데이터 초기화
        for i in range(6):
            self.temp_data[i] = []
            self.pwm_data[i] = []

        self._widget.status_label.setText('그래프 모니터링이 시작되었습니다')

    def stop_graph(self):
        """
        그래프 모니터링 중지
        """
        self.graph_running = False
        self._widget.status_label.setText('그래프 모니터링이 중지되었습니다')

    def toggle_logging(self, enabled):
        """
        로깅 활성화/비활성화 토글
        """
        if enabled and not self.log_enabled:
            # 로깅 시작
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            log_dir = os.path.expanduser("~/wearable_robot_logs")
            os.makedirs(log_dir, exist_ok=True)

            log_filename = f"{log_dir}/temp_fan_log_{timestamp}.csv"
            self.log_file = open(log_filename, "w", newline='')

            # CSV 헤더 작성
            csv_writer = csv.writer(self.log_file)
            header = ["timestamp"]
            for i in range(6):
                header.append(f"temp{i+1}")
            for i in range(6):
                header.append(f"pwm{i+1}")
            csv_writer.writerow(header)

            self.log_enabled = True
            self._widget.status_label.setText(f"로깅이 시작되었습니다: {log_filename}")

        elif not enabled and self.log_enabled:
            # 로깅 종료
            if self.log_file:
                self.log_file.close()
                self.log_file = None

            self.log_enabled = False
            self._widget.status_label.setText("로깅이 중지되었습니다")

    def log_data(self):
        """
        현재 온도 및 팬 상태 데이터를 로그 파일에 기록
        """
        if not self.log_enabled or not self.log_file:
            return

        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

        # CSV 데이터 행 작성
        csv_writer = csv.writer(self.log_file)
        row = [timestamp]
        row.extend([f"{t:.2f}" for t in self.temperatures])
        row.extend([str(p) for p in self.pwm_values])

        csv_writer.writerow(row)
        self.log_file.flush()  # 즉시 디스크에 쓰기

    def save_data(self):
        """
        현재 수집된 그래프 데이터를 CSV 파일로 저장
        """
        if len(self.time_data) == 0:
            self._widget.status_label.setText("저장할 데이터가 없습니다")
            return

        # 파일 이름 설정
        filename = self._widget.FileName.text()
        if not filename:
            filename = "fan_temperature_data"

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        save_dir = os.path.expanduser("~/wearable_robot_data")
        os.makedirs(save_dir, exist_ok=True)

        filepath = f"{save_dir}/{filename}_{timestamp}.csv"

        try:
            with open(filepath, 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)

                # 헤더 행 작성
                header = ["time"]
                for i in range(6):
                    header.append(f"temp{i+1}")
                for i in range(6):
                    header.append(f"pwm{i+1}")
                csv_writer.writerow(header)

                # 데이터 행 작성
                for i in range(len(self.time_data)):
                    row = [self.time_data[i]]
                    for j in range(6):
                        if i < len(self.temp_data[j]):
                            row.append(self.temp_data[j][i])
                        else:
                            row.append(0.0)
                    for j in range(6):
                        if i < len(self.pwm_data[j]):
                            row.append(self.pwm_data[j][i])
                        else:
                            row.append(0)
                    csv_writer.writerow(row)

            self._widget.status_label.setText(f"데이터가 저장되었습니다: {filepath}")

        except Exception as e:
            self._widget.status_label.setText(f"데이터 저장 오류: {str(e)}")

    def clear_data(self):
        """
        수집된 데이터 초기화
        """
        self.time_data = []
        for i in range(6):
            self.temp_data[i] = []
            self.pwm_data[i] = []

        self._widget.status_label.setText("데이터가 초기화되었습니다")

    def apply_pwm(self):
        """
        수동 PWM 값 적용
        """
        if self.auto_mode:
            self._widget.status_label.setText("수동 PWM 적용을 위해 수동 모드로 전환하세요")
            return

        # PWM 값 설정
        pwm_value = int(self._widget.duty_spin.value())

        # 모든 구동기에 동일한 PWM 값 적용
        for i in range(6):
            self.pwm_values[i] = pwm_value

        # PWM 값 발행
        self.publish_pwm_command()

        self._widget.status_label.setText(f"수동 PWM 값 {pwm_value}%가 적용되었습니다")

    def apply_pi_gains(self):
        """
        PI 제어 게인 적용
        """
        # PI 게인 값 가져오기
        kp = self._widget.Kp_spin.value()
        ki = self._widget.Ki_spin.value()

        # PI 게인 메시지 발행
        kp_msg = Float64()
        ki_msg = Float64()
        kp_msg.data = float(kp)
        ki_msg.data = float(ki)

        self.kp_pub.publish(kp_msg)
        self.ki_pub.publish(ki_msg)

        self._widget.status_label.setText(f"PI 게인이 적용되었습니다: Kp={kp}, Ki={ki}")

        # 적분항 초기화
        self.integral = 0.0
        self.prev_error = 0.0

    def update_target_temperature(self, value):
        """
        목표 온도 업데이트
        """
        if not self.auto_mode:
            return

        target_temp_msg = Float64()
        target_temp_msg.data = float(value)
        self.target_temp_pub.publish(target_temp_msg)

        self._widget.status_label.setText(f"목표 온도가 {value}°C로 설정되었습니다")

    def update_limit_temperature(self, value):
        """
        제한 온도 업데이트
        """
        self._widget.status_label.setText(f"제한 온도가 {value}°C로 설정되었습니다")

    def publish_pwm_command(self):
        """
        ActuatorCommand 메시지 발행 (PWM 제어)
        """
        msg = ActuatorCommand()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'fan_control'
        msg.pwm = self.pwm_values

        self.pwm_pub.publish(msg)
        self.node.get_logger().debug('PWM command published')

    def publish_fan_command(self):
        """
        FanCommand 메시지 발행 (팬 상태 제어)
        """
        msg = FanCommand()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'fan_control'
        msg.fan = self.fan_states

        self.fan_pub.publish(msg)
        self.node.get_logger().debug('Fan command published')

    def shutdown_plugin(self):
        """
        플러그인이 종료될 때 호출됩니다.
        """
        # 타이머 중지
        self.update_timer.stop()

        # 로깅 중이면 파일 닫기
        if self.log_enabled and self.log_file:
            self.log_file.close()
            self.log_file = None

        # ROS 2 타이머 및 노드 종료
        self.node.destroy_timer(self.timer)
        self.node.destroy_node()
        # rqt가 관리하는 전체 ROS 2 컨텍스트이므로 여기서 rclpy.shutdown()을 호출하지 않습니다

def main(args=None):
    """
    rqt 플러그인 메인 함수 (독립 실행 모드에서 사용)
    """
    # 독립 실행 모드일 때는 rclpy를 직접 초기화합니다
    # rqt 내에서 실행될 때는 이미 초기화되어 있으므로 호출하지 않습니다

    from rqt_gui.main import Main
    main = Main()
    sys.exit(main.main(sys.argv, standalone='wearable_robot_rqt_plugins.temp_control_plugin.TempControlPlugin'))

if __name__ == '__main__':
    import sys

    # 독립 실행 모드일 때는 rclpy를 직접 초기화합니다
    rclpy.init(args=sys.argv)
    main()
