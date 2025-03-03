import os
import time
import math
import numpy as np
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool, String
from std_srvs.srv import Trigger, SetBool
from wearable_robot_interfaces.msg import TemperatureData, ActuatorCommand

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

        # 그래프 데이터 초기화
        self._history_size = 600  # 10분 데이터 (초당 1샘플)
        self._temp_history = np.zeros(self._history_size)
        self._pwm_history = np.zeros(self._history_size)
        self._target_temp_history = np.ones(self._history_size) * self._target_temp
        self._time_history = np.linspace(-self._history_size, 0, self._history_size)
        self._start_time = time.time()

        # 온도 프로필 관련 초기화
        self._is_profile_running = False
        self._profile_timer = None
        self._current_profile = None

    def _connect_ui_elements(self):
        # 버튼 연결
        self._widget.GraphStart.clicked.connect(self._on_power_on)
        self._widget.StopBtn.clicked.connect(self._on_power_off)
        self._widget.Apply.clicked.connect(self._apply_target_temp)
        self._widget.emergency_button.clicked.connect(self._on_emergency_stop) if hasattr(self._widget, 'emergency_button') else None
        self._widget.DataSave.clicked.connect(self._save_csv_data)
        self._widget.DataZero.clicked.connect(self._reset_data)
        self._widget.PID.clicked.connect(self._apply_gain_settings) if hasattr(self._widget, 'PID') else None

        # 슬라이더 및 스핀박스 연결
        if hasattr(self._widget, 'Amplitude'):
            self._widget.Amplitude.valueChanged.connect(self._on_temp_input_changed)

        # 그래프 설정
        self._setup_plots()

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

        # 발행자 설정
        self._param_pub = self._node.create_publisher(
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

        self._power_pub = self._node.create_publisher(
            Bool,
            'actuator_power',
            10
        )

        # 로깅 서비스 클라이언트
        self._reset_log_client = self._node.create_client(Trigger, 'reset_temperature_log')
        self._save_log_client = self._node.create_client(SetBool, 'save_temperature_log')

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

        # UI 업데이트
        if hasattr(self._widget, 'CurrentTemp'):
            self._widget.CurrentTemp.display(float(self._current_temps[self._actuator_idx]))

    # 이하 콜백 함수 구현
    def _temp_callback(self, msg):
        for i in range(min(len(msg.temperature), len(self._current_temps))):
            self._current_temps[i] = msg.temperature[i]
        self._check_temperature_safety()

    def _actuator_callback(self, msg):
        if len(msg.pwm) > self._actuator_idx:
            self._pwm_value = msg.pwm[self._actuator_idx]

    def _log_filename_callback(self, msg):
        self._current_log_filename = msg.data

    def _on_temp_input_changed(self, value):
        self._pending_target_temp = float(value)

    def _apply_target_temp(self):
        if not self._is_power_on:
            QMessageBox.warning(self._widget, "전원 꺼짐", "구동기 전원이 꺼져 있습니다.")
            return

        self._target_temp = self._pending_target_temp
        msg = Float64()
        msg.data = self._target_temp
        self._param_pub.publish(msg)
        self.logger.info(f"목표 온도 설정: {self._target_temp:.1f}°C")

    def _on_power_on(self):
        if self._is_power_on:
            return

        msg = Bool()
        msg.data = True
        self._power_pub.publish(msg)
        self._is_power_on = True
        self._update_power_ui(True)
        self.logger.info("구동기 전원이 켜졌습니다.")

    def _on_power_off(self):
        if not self._is_power_on:
            return

        msg = Bool()
        msg.data = False
        self._power_pub.publish(msg)
        self._is_power_on = False
        self._update_power_ui(False)
        self.logger.info("구동기 전원이 꺼졌습니다.")

    def _update_power_ui(self, is_on):
        if is_on:
            self._widget.GraphStart.setEnabled(False)
            self._widget.StopBtn.setEnabled(True)
            self._widget.Apply.setEnabled(True)
        else:
            self._widget.GraphStart.setEnabled(True)
            self._widget.StopBtn.setEnabled(False)
            self._widget.Apply.setEnabled(False)

    def _on_emergency_stop(self):
        self._is_emergency_stop = True
        msg = Bool()
        msg.data = True
        self._emergency_pub.publish(msg)
        self.logger.warning("비상 정지가 활성화되었습니다.")

    def _check_temperature_safety(self):
        if self._current_temps[self._actuator_idx] >= self._safety_threshold:
            if self._is_temperature_safe:
                self._is_temperature_safe = False
                QMessageBox.critical(self._widget, "안전 경고",
                    f"온도가 안전 임계값({self._safety_threshold}°C)을 초과했습니다!\n비상 정지를 실행합니다.")
                self._on_emergency_stop()

    def _apply_gain_settings(self):
        if not self._is_power_on:
            QMessageBox.warning(self._widget, "전원 꺼짐", "구동기 전원이 꺼져 있습니다.")
            return

        if hasattr(self._widget, 'Kp') and hasattr(self._widget, 'Ki'):
            self._kp = self._widget.Kp.value()
            self._ki = self._widget.Ki.value()

            kp_msg = Float64()
            kp_msg.data = self._kp
            ki_msg = Float64()
            ki_msg.data = self._ki

            self._kp_pub.publish(kp_msg)
            self._ki_pub.publish(ki_msg)

            self.logger.info(f"PI 게인 설정 변경: Kp={self._kp:.2f}, Ki={self._ki:.3f}")

    def _save_csv_data(self):
        self._save_log_file()
        # 파일 선택 대화상자 표시 및 저장 로직 구현

    def _reset_data(self):
        result = QMessageBox.question(self._widget, "데이터 초기화",
            "그래프 데이터와 로깅을 초기화하시겠습니까?\n현재 데이터는 모두 삭제됩니다.",
            QMessageBox.Yes | QMessageBox.No)

        if result == QMessageBox.Yes:
            self._start_time = time.time()
            self._temp_history = np.zeros(self._history_size)
            self._pwm_history = np.zeros(self._history_size)
            self._target_temp_history = np.ones(self._history_size) * self._target_temp
            self._reset_logging()

    def _reset_logging(self):
        # 로깅 리셋 서비스 호출 구현
        pass

    def _save_log_file(self):
        # 로그 파일 저장 서비스 호출 구현
        pass

    def shutdown_plugin(self):
        self._update_timer.stop()

        # 전원 끄기 메시지 발행
        if self._is_power_on:
            try:
                power_msg = Bool()
                power_msg.data = False
                self._power_pub.publish(power_msg)
            except Exception:
                pass

        self.logger.info('온도 제어 플러그인이 종료되었습니다.')
