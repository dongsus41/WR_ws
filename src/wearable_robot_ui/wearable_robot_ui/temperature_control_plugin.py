import os
import time
import math
import numpy as np
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool, String
from std_srvs.srv import Trigger, SetBool
from wearable_robot_interfaces.msg import TemperatureData, ActuatorCommand, FanCommand

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot
from python_qt_binding.QtWidgets import QWidget, QFileDialog, QMessageBox
from python_qt_binding.QtGui import QFont
import pyqtgraph as pg

from rqt_gui_py.plugin import Plugin
from ament_index_python.packages import get_package_share_directory


class TemperatureControlPlugin(Plugin):
    def __init__(self, context):
        super(TemperatureControlPlugin, self).__init__(context)
        # 플러그인 설정
        self.setObjectName('TemperatureControlPlugin')

        # ROS 노드 초기화
        self._node = context.node
        self.logger = self._node.get_logger()
        self.logger.info('온도 제어 플러그인이 시작되었습니다.')

        # 메인 위젯 생성
        self._widget = QWidget()
        self._widget.setObjectName('TemperatureControlUi')

        # UI 파일 로드
        ui_file = os.path.join(get_package_share_directory('wearable_robot_ui'), 'resource', 'temperature_control.ui')
        loadUi(ui_file, self._widget)

        # 데이터 초기화
        self._initialize_data()

        # UI 요소 연결
        self._connect_ui_elements()

        # ROS 토픽 설정
        self._setup_ros_communication()

        # 타이머 설정
        self._update_timer = QTimer()
        self._update_timer.timeout.connect(self._update_gui)
        self._update_timer.start(50)  # 50ms 간격으로 업데이트

        # 서비스 클라이언트 초기화
        self._setup_service_clients()

        # 플러그인 위젯 추가
        context.add_widget(self._widget)

    def _initialize_data(self):
        # 데이터 초기화 (변수와 컨트롤 상태)
        self._current_temps = [0.0] * 6
        self._target_temp = 50.0
        self._pending_target_temp = self._target_temp
        self._pwm_value = 0
        self._actuator_idx = 5
        self._is_power_on = False
        self._is_emergency_stop = False
        self._is_temperature_safe = True
        self._safety_threshold = 80.0
        self._kp = 2.0
        self._ki = 0.1
        self._fan_states = [False] * 6  # 팬 상태 추가

        # 제어 모드 상태
        self._use_pid_control = True
        self._use_pwm_direct = False

        # 그래프 데이터 초기화
        self._history_size = 600  # 10분 데이터 (초당 1샘플)
        self._temp_history = np.zeros(self._history_size)
        self._pwm_history = np.zeros(self._history_size)
        self._target_temp_history = np.ones(self._history_size) * self._target_temp
        self._time_history = np.linspace(-self._history_size, 0, self._history_size)
        self._start_time = time.time()

        # 로깅 관련 초기화
        self._is_logging = False
        self._current_log_filename = ""

    def _connect_ui_elements(self):
        # 버튼 연결
        self._widget.GraphStart.clicked.connect(self._on_power_on)
        self._widget.StopBtn.clicked.connect(self._on_power_off)
        self._widget.Apply.clicked.connect(self._apply_settings)
        self._widget.DataSave.clicked.connect(self._save_csv_data)
        self._widget.DataZero.clicked.connect(self._reset_data)

        # 초기화 버튼 연결
        if hasattr(self._widget, 'Zeroset_2'):
            self._widget.Zeroset_2.clicked.connect(self._initialize_system)

        # 제어 버튼 연결
        if hasattr(self._widget, 'PWM_duty'):
            self._widget.PWM_duty.clicked.connect(self._on_pwm_direct_mode)

        if hasattr(self._widget, 'PID'):
            self._widget.PID.clicked.connect(self._on_pid_mode)

        if hasattr(self._widget, 'control_start'):
            self._widget.control_start.clicked.connect(self._on_control_toggle)

        # 그래프 설정
        self._setup_plots()

    def _setup_service_clients(self):
        """서비스 클라이언트 설정"""
        # 로그 관련 서비스 클라이언트
        self._reset_log_client = self._node.create_client(
            Trigger,
            'reset_temperature_log'
        )

        self._save_log_client = self._node.create_client(
            SetBool,
            'save_temperature_log'
        )

        # 클라이언트 연결 대기
        while not self._reset_log_client.wait_for_service(timeout_sec=1.0):
            self.logger.info('reset_temperature_log 서비스 대기 중...')

        while not self._save_log_client.wait_for_service(timeout_sec=1.0):
            self.logger.info('save_temperature_log 서비스 대기 중...')

        self.logger.info('로그 서비스에 연결되었습니다.')

    def _setup_plots(self):
        # 온도 그래프 설정
        self._widget.pw1.setBackground('w')
        self._widget.pw1.setTitle("온도 모니터링", color="k", size="12pt")
        self._widget.pw1.setLabel('left', '온도', units='°C', color="k")
        self._widget.pw1.setLabel('bottom', '시간', units='s', color="k")
        self._widget.pw1.showGrid(x=True, y=True)

        # 온도 데이터 라인 생성
        self._temp_curve = self._widget.pw1.plot(
            self._time_history,
            self._temp_history,
            pen=pg.mkPen(color='r', width=2),
            name="현재 온도"
        )

        # 목표 온도 라인 생성
        self._target_curve = self._widget.pw1.plot(
            self._time_history,
            self._target_temp_history,
            pen=pg.mkPen(color='g', width=2, style=Qt.DashLine),
            name="목표 온도"
        )

        # 안전 온도 임계값 라인
        self._safety_line = pg.InfiniteLine(
            pos=self._safety_threshold,
            angle=0,
            pen=pg.mkPen(color='r', width=2, style=Qt.DashLine),
            label='안전 온도={value:0.1f}°C',
            labelOpts={'position':0.9, 'color': (255,0,0), 'fill': (200,50,50,50)}
        )
        self._widget.pw1.addItem(self._safety_line)

        # PWM 그래프 설정
        self._widget.pw2.setBackground('w')
        self._widget.pw2.setTitle("PWM 출력", color="k", size="12pt")
        self._widget.pw2.setLabel('left', 'PWM', units='', color="k")
        self._widget.pw2.setLabel('bottom', '시간', units='s', color="k")
        self._widget.pw2.showGrid(x=True, y=True)
        self._widget.pw2.setYRange(0, 100)

        # PWM 데이터 라인 생성
        self._pwm_curve = self._widget.pw2.plot(
            self._time_history,
            self._pwm_history,
            pen=pg.mkPen(color='b', width=2),
            name="PWM 5"
        )

        # X축 연결 (같이 이동)
        self._widget.pw1.setXLink(self._widget.pw2)

    def _setup_ros_communication(self):
        # 온도 데이터 구독
        self._temp_sub = self._node.create_subscription(
            TemperatureData,
            'temperature_data',
            self._temp_callback,
            10
        )

        # PWM 상태 구독
        self._actuator_sub = self._node.create_subscription(
            ActuatorCommand,
            'actuator_command',
            self._actuator_callback,
            10
        )

        # 팬 상태 구독
        self._fan_sub = self._node.create_subscription(
            FanCommand,
            'fan_state',
            self._fan_callback,
            10
        )

        # 발행자 설정
        self._target_temp_pub = self._node.create_publisher(
            Float64,
            'target_temperature',
            10
        )

        self._kp_pub = self._node.create_publisher(
            Float64,
            'kp_param',
            10
        )

        self._ki_pub = self._node.create_publisher(
            Float64,
            'ki_param',
            10
        )

        self._emergency_pub = self._node.create_publisher(
            Bool,
            'emergency_stop',
            10
        )

        # 직접 PWM 제어를 위한 발행자
        self._direct_pwm_pub = self._node.create_publisher(
            ActuatorCommand,
            'direct_pwm_command',
            10
        )

        # 현재 로그 파일명 구독
        self._log_filename_sub = self._node.create_subscription(
            String,
            'current_log_filename',
            self._log_filename_callback,
            10
        )

    def _update_gui(self):
        # 현재 시간 계산
        current_time = time.time() - self._start_time

        # 데이터 업데이트
        self._temp_history = np.roll(self._temp_history, -1)
        self._temp_history[-1] = self._current_temps[self._actuator_idx]

        self._pwm_history = np.roll(self._pwm_history, -1)
        self._pwm_history[-1] = self._pwm_value

        self._target_temp_history = np.roll(self._target_temp_history, -1)
        self._target_temp_history[-1] = self._target_temp

        self._time_history = np.roll(self._time_history, -1)
        self._time_history[-1] = current_time

        # 그래프 업데이트
        self._temp_curve.setData(self._time_history, self._temp_history)
        self._target_curve.setData(self._time_history, self._target_temp_history)
        self._pwm_curve.setData(self._time_history, self._pwm_history)

        # 자동 스크롤
        self._widget.pw1.setXRange(max(0, current_time-120), current_time+5)

        # 현재 온도 표시
        if hasattr(self._widget, 'CurrentTemp'):
            self._widget.CurrentTemp.display(float(self._current_temps[self._actuator_idx]))

        # 파일 이름 표시
        if hasattr(self._widget, 'FileName') and self._current_log_filename:
            self._widget.FileName.setPlainText(self._current_log_filename)

    # 이하 콜백 함수 구현
    def _temp_callback(self, msg):
        for i in range(min(len(msg.temperature), len(self._current_temps))):
            self._current_temps[i] = msg.temperature[i]
        self._check_temperature_safety()

    def _actuator_callback(self, msg):
        if len(msg.pwm) > self._actuator_idx:
            self._pwm_value = msg.pwm[self._actuator_idx]

    def _fan_callback(self, msg):
        for i in range(min(len(msg.fan), len(self._fan_states))):
            self._fan_states[i] = msg.fan[i]

    def _log_filename_callback(self, msg):
        self._current_log_filename = msg.data

    def _apply_settings(self):
        """설정 적용 버튼 클릭 처리"""
        if not self._is_power_on:
            QMessageBox.warning(self._widget, "전원 꺼짐", "구동기 전원이 꺼져 있습니다. 먼저 Start 버튼을 눌러주세요.")
            return

        # 진폭 및 오프셋 값 불러오기
        if hasattr(self._widget, 'Amplitude') and hasattr(self._widget, 'Offset'):
            amplitude = self._widget.Amplitude.value()
            offset = self._widget.Offset.value()

            # 목표 온도 계산 및 설정
            self._target_temp = offset
            self._send_target_temp(self._target_temp)

            self.logger.info(f"설정 적용: 오프셋={offset}°C, 진폭={amplitude}")

        # PI 제어 게인 업데이트
        if hasattr(self._widget, 'Kp') and hasattr(self._widget, 'Ki'):
            self._kp = self._widget.Kp.value()
            self._ki = self._widget.Ki.value()
            self._send_pi_gains()

    def _send_target_temp(self, temp_value):
        """목표 온도 메시지 발행"""
        msg = Float64()
        msg.data = temp_value
        self._target_temp_pub.publish(msg)
        self.logger.info(f"목표 온도 설정: {temp_value:.1f}°C")

    def _send_pi_gains(self):
        """PI 게인 메시지 발행"""
        kp_msg = Float64()
        kp_msg.data = self._kp
        self._kp_pub.publish(kp_msg)

        ki_msg = Float64()
        ki_msg.data = self._ki
        self._ki_pub.publish(ki_msg)

        self.logger.info(f"PI 게인 설정: Kp={self._kp:.2f}, Ki={self._ki:.3f}")

    def _on_power_on(self):
        if self._is_power_on:
            return

        self._is_power_on = True
        self._update_power_ui(True)
        self.logger.info("구동기 전원이 켜졌습니다.")

        # 로깅 시작
        self._start_logging()

        # 기본 설정 적용
        self._apply_settings()

    def _on_power_off(self):
        if not self._is_power_on:
            return

        self._is_power_on = False
        self._update_power_ui(False)
        self.logger.info("구동기 전원이 꺼졌습니다.")

        # 로깅 중지
        self._stop_logging()

        # 직접 PWM 값 0으로 설정
        if self._use_pwm_direct:
            self._send_direct_pwm(0)

    def _update_power_ui(self, is_on):
        if is_on:
            self._widget.GraphStart.setEnabled(False)
            self._widget.StopBtn.setEnabled(True)
            self._widget.Apply.setEnabled(True)
            if hasattr(self._widget, 'PWM_duty'):
                self._widget.PWM_duty.setEnabled(True)
            if hasattr(self._widget, 'PID'):
                self._widget.PID.setEnabled(True)
        else:
            self._widget.GraphStart.setEnabled(True)
            self._widget.StopBtn.setEnabled(False)
            self._widget.Apply.setEnabled(False)
            if hasattr(self._widget, 'PWM_duty'):
                self._widget.PWM_duty.setEnabled(False)
            if hasattr(self._widget, 'PID'):
                self._widget.PID.setEnabled(False)
            if hasattr(self._widget, 'control_start'):
                self._widget.control_start.setChecked(False)

    def _on_emergency_stop(self):
        self._is_emergency_stop = True
        msg = Bool()
        msg.data = True
        self._emergency_pub.publish(msg)
        self.logger.warning("비상 정지가 활성화되었습니다.")

        # UI 업데이트
        self._update_power_ui(False)

        # 메시지 표시
        QMessageBox.critical(self._widget, "비상 정지",
            "비상 정지가 활성화되었습니다!\n모든 출력이 중단됩니다.")

    def _check_temperature_safety(self):
        if self._current_temps[self._actuator_idx] >= self._safety_threshold:
            if self._is_temperature_safe:
                self._is_temperature_safe = False
                QMessageBox.critical(self._widget, "안전 경고",
                    f"온도가 안전 임계값({self._safety_threshold}°C)을 초과했습니다!\n비상 정지를 실행합니다.")
                self._on_emergency_stop()

    def _on_pwm_direct_mode(self):
        """직접 PWM 제어 모드 활성화"""
        if not self._is_power_on:
            QMessageBox.warning(self._widget, "전원 꺼짐", "구동기 전원이 꺼져 있습니다.")
            return

        self._use_pid_control = False
        self._use_pwm_direct = True

        # UI 업데이트
        if hasattr(self._widget, 'PWM_duty'):
            self._widget.PWM_duty.setStyleSheet("background-color: green; color: white")
        if hasattr(self._widget, 'PID'):
            self._widget.PID.setStyleSheet("")

        self.logger.info("직접 PWM 제어 모드로 전환되었습니다.")

        # 현재 설정값 가져오기
        if hasattr(self._widget, 'duty'):
            pwm_value = int(self._widget.duty.value())
            self._send_direct_pwm(pwm_value)

    def _on_pid_mode(self):
        """PID 제어 모드 활성화"""
        if not self._is_power_on:
            QMessageBox.warning(self._widget, "전원 꺼짐", "구동기 전원이 꺼져 있습니다.")
            return

        self._use_pid_control = True
        self._use_pwm_direct = False

        # UI 업데이트
        if hasattr(self._widget, 'PID'):
            self._widget.PID.setStyleSheet("background-color: green; color: white")
        if hasattr(self._widget, 'PWM_duty'):
            self._widget.PWM_duty.setStyleSheet("")

        self.logger.info("PID 제어 모드로 전환되었습니다.")

        # PID 설정 적용
        self._apply_settings()

    def _on_control_toggle(self, checked):
        """제어 시작/중지 토글 버튼"""
        if not self._is_power_on:
            self._widget.control_start.setChecked(False)
            QMessageBox.warning(self._widget, "전원 꺼짐", "구동기 전원이 꺼져 있습니다.")
            return

        if checked:
            # 제어 시작
            if self._use_pwm_direct:
                # 직접 PWM 제어
                if hasattr(self._widget, 'duty'):
                    pwm_value = int(self._widget.duty.value())
                    self._send_direct_pwm(pwm_value)
            else:
                # PID 제어
                self._apply_settings()

            self.logger.info("제어가 시작되었습니다.")
        else:
            # 제어 중지
            if self._use_pwm_direct:
                # PWM 값 0으로 설정
                self._send_direct_pwm(0)

            self.logger.info("제어가 중지되었습니다.")

    def _send_direct_pwm(self, pwm_value):
        """직접 PWM 값 전송"""
        msg = ActuatorCommand()
        msg.header.stamp = self._node.get_clock().now().to_msg()

        # 모든 PWM 값을 0으로 초기화
        msg.pwm = [0] * 6

        # 해당 액추에이터만 설정
        msg.pwm[self._actuator_idx] = min(100, max(0, pwm_value))

        self._direct_pwm_pub.publish(msg)
        self.logger.info(f"직접 PWM 값 설정: {pwm_value}")

    def _save_csv_data(self):
        """현재 데이터를 CSV 파일로 저장"""
        if not self._is_logging:
            # 로깅 시작
            self._start_logging()
            QMessageBox.information(self._widget, "로깅 시작",
                f"데이터 로깅이 시작되었습니다.\n파일: {self._current_log_filename}")
        else:
            # 현재 로깅을 중지하고 파일 저장
            filename = self._current_log_filename
            self._stop_logging()

            QMessageBox.information(self._widget, "데이터 저장 완료",
                f"데이터가 성공적으로 저장되었습니다.\n파일: {filename}")

    def _reset_data(self):
        """데이터 초기화"""
        result = QMessageBox.question(self._widget, "데이터 초기화",
            "그래프 데이터와 로깅을 초기화하시겠습니까?\n현재 데이터는 모두 삭제됩니다.",
            QMessageBox.Yes | QMessageBox.No)

        if result == QMessageBox.Yes:
            # 로깅 중지
            self._stop_logging()

            # 로거 데이터 초기화 요청
            self._reset_logging()

            # 그래프 데이터 초기화
            self._start_time = time.time()
            self._temp_history = np.zeros(self._history_size)
            self._pwm_history = np.zeros(self._history_size)
            self._target_temp_history = np.ones(self._history_size) * self._target_temp
            self._time_history = np.linspace(-self._history_size, 0, self._history_size)

            self.logger.info("데이터가 초기화되었습니다.")

    def _start_logging(self):
        """로깅 시작"""
        if self._is_logging:
            return

        # 로깅 시작 서비스 호출
        req = SetBool.Request()
        req.data = True

        future = self._save_log_client.call_async(req)

        # 비동기 호출이므로 결과는 나중에 확인
        self._is_logging = True

    def _stop_logging(self):
        """로깅 중지"""
        if not self._is_logging:
            return

        # 로깅 중지 서비스 호출
        req = SetBool.Request()
        req.data = False

        future = self._save_log_client.call_async(req)

        # 비동기 호출이므로 결과는 나중에 확인
        self._is_logging = False

    def _reset_logging(self):
        """로그 데이터 초기화"""
        req = Trigger.Request()

        future = self._reset_log_client.call_async(req)

        # 파일명 초기화
        self._current_log_filename = ""
        if hasattr(self._widget, 'FileName'):
            self._widget.FileName.setPlainText("")

    def _initialize_system(self):
        """시스템 초기화"""
        # 시스템 초기화 로직
        self._on_power_off()  # 전원 끄기
        self._reset_data()    # 데이터 초기화

        # 기본값으로 복원
        if hasattr(self._widget, 'Amplitude'):
            self._widget.Amplitude.setValue(0.0)
        if hasattr(self._widget, 'Offset'):
            self._widget.Offset.setValue(0.0)
        if hasattr(self._widget, 'Kp'):
            self._widget.Kp.setValue(2.0)
        if hasattr(self._widget, 'Ki'):
            self._widget.Ki.setValue(0.1)
        if hasattr(self._widget, 'duty'):
            self._widget.duty.setValue(0)

        self._target_temp = 50.0
        self._pending_target_temp = self._target_temp

        QMessageBox.information(self._widget, "초기화 완료",
            "시스템이 초기화되었습니다.")

    def shutdown_plugin(self):
        """플러그인 종료"""
        self._update_timer.stop()

        # 로깅 중지
        if self._is_logging:
            self._stop_logging()

        # 전원 끄기
        if self._is_power_on:
            self._on_power_off()

        self.logger.info('온도 제어 플러그인이 종료되었습니다.')
