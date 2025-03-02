import os
import rclpy
import numpy as np
import time
import csv
from datetime import datetime
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot
from python_qt_binding.QtWidgets import (QWidget, QVBoxLayout, QPushButton, QSlider, QLabel,
                                         QHBoxLayout, QLineEdit, QGroupBox, QSplitter,
                                         QDoubleSpinBox, QSpinBox, QGridLayout, QFileDialog,
                                         QMessageBox, QComboBox)
from python_qt_binding.QtGui import QFont, QColor, QPalette, QIntValidator, QDoubleValidator
import pyqtgraph as pg
import math

from rqt_gui_py.plugin import Plugin
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from std_msgs.msg import Float64, Bool
from wearable_robot_interfaces.msg import TemperatureData, ActuatorCommand

from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String

class TemperatureControlGUI(Plugin):
    """
    웨어러블 로봇의 온도 모니터링 및 제어를 위한 rqt 플러그인 (최종 개선 버전)
    """

    def __init__(self, context):
        """
        플러그인 초기화
        """
        super(TemperatureControlGUI, self).__init__(context)
        # 플러그인 제목 설정
        self.setObjectName('TemperatureControlGUI')

        # 메인 위젯 생성
        self._widget = QWidget()
        self._widget.setObjectName('TemperatureControlGUIWidget')
        self._widget.setWindowTitle('웨어러블 로봇 온도 제어')

        # 메인 레이아웃 설정
        self._layout = QVBoxLayout(self._widget)

        # ROS2 초기화
        self._node = context.node
        self.logger = self._node.get_logger()
        self.logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.logger.info('온도 제어 GUI 플러그인이 시작되었습니다.')

        # 데이터 초기화 - GUI 초기화 전에 수행
        self._current_temps = [0.0] * 6  # 6개 온도 센서 (5번만 표시)
        self._target_temp = 50.0  # 기본 목표 온도
        self._pending_target_temp = self._target_temp  # 적용 대기 중인 목표 온도
        self._pwm_value = 0  # PWM 5번 채널
        self._actuator_idx = 5  # 제어할 구동기 번호 (5번 고정)

        # PI 제어 파라미터
        self._kp = 2.0  # 비례 게인 기본값
        self._ki = 0.1  # 적분 게인 기본값

        # 시간 및 데이터 기록 관련
        self._start_time = time.time()
        self._history_size = 6000  # 5분 데이터 (1초에 1샘플)

        # 온도 기록을 위한 데이터
        self._temp_history = np.zeros(self._history_size)
        self._pwm_history = np.zeros(self._history_size)
        self._time_history = np.linspace(-self._history_size, 0, self._history_size)

        # 초기 시간 설정 (현재 시간에서 과거로)
        current_time = time.time()
        for i in range(self._history_size):
            self._time_history[i] = current_time - (self._history_size - i)

        # 안전 상태 추적
        self._is_emergency_stop = False
        self._is_temperature_safe = True
        self._safety_threshold = 80.0  # 안전 온도 임계값 (섭씨)

        # 프로그램된 온도 프로필 관련 변수
        self._is_profile_running = False
        self._profile_timer = None
        self._profile_start_time = 0
        self._current_profile = None
        self._profile_step_size = 1.0  # 초당 업데이트 빈도

        self._reset_log_client = self._node.create_client(Trigger, 'reset_temperature_log')
        self._save_log_client = self._node.create_client(SetBool, 'save_temperature_log')
        self._set_filename_client = self._node.create_client(SetBool, 'set_log_filename')

        # GUI 초기화 - 데이터 초기화 후 호출
        self._init_gui()

        # 구독자 생성
        self._temp_sub = self._node.create_subscription(
            TemperatureData,
            'temperature_data',
            self._temp_callback,
            10
        )
        self._actuator_sub = self._node.create_subscription(
            ActuatorCommand,
            'actuator_command',
            self._actuator_callback,
            10
        )

        # 발행자 생성
        self._param_pub = self._node.create_publisher(
            Float64,
            'target_temperature',
            10
        )

        # PI 게인 파라미터 발행자
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

        # 비상 정지 발행자
        self._emergency_pub = self._node.create_publisher(
            Bool,
            'emergency_stop',
            10
        )


        self._current_log_filename = ""
        self._log_filename_sub = self._node.create_subscription(
          String,
          'current_log_filename',
          self._log_filename_callback,
          10
        )

        self._log_directory = os.path.expanduser("~/temp_logs")

        # 타이머 설정
        self._update_timer = QTimer()
        self._update_timer.timeout.connect(self._update_gui)
        self._update_timer.start(50)  # 100ms 간격으로 업데이트

        # rqt에 위젯 추가
        context.add_widget(self._widget)

        # 로그 출력
        self.logger.info('개선된 온도 제어 GUI 플러그인이 시작되었습니다.')

    def _init_gui(self):
        """
        GUI 요소 초기화
        """
        # 상단 그래프 영역 생성
        graph_widget = QWidget()
        graph_layout = QVBoxLayout(graph_widget)
        self._layout.addWidget(graph_widget, 7)  # 그래프 영역이 더 큰 비율 차지

        # 온도 그래프 생성
        self._temp_plot = pg.PlotWidget()
        self._temp_plot.setBackground('w')
        self._temp_plot.setTitle("온도 모니터링", color="k", size="12pt")
        self._temp_plot.setLabel('left', '온도', units='°C', color="k")
        self._temp_plot.setLabel('bottom', '시간', units='s', color="k")
        self._temp_plot.showGrid(x=True, y=True)
        # self._temp_plot.enableAutoRange(axis='y')  # Y축 자동 스케일링

        # 온도 5번 데이터 라인 생성
        self._temp_curve = self._temp_plot.plot(
            self._time_history,
            self._temp_history,
            pen=pg.mkPen(color='r', width=2),
            name="현재 온도"
        )

        # 목표 온도 히스토리 초기화
        self._target_temp_history = np.zeros(self._history_size)
        for i in range(self._history_size):
            self._target_temp_history[i] = self._target_temp

        # 목표 온도 라인 생성 (동적)
        self._target_curve = self._temp_plot.plot(
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
        self._temp_plot.addItem(self._safety_line)

        # 범례 추가
        self._temp_plot.addLegend()

        # PWM 그래프 생성
        self._pwm_plot = pg.PlotWidget()
        self._pwm_plot.setBackground('w')
        self._pwm_plot.setTitle("PWM 출력", color="k", size="12pt")
        self._pwm_plot.setLabel('left', 'PWM', units='', color="k")
        self._pwm_plot.setLabel('bottom', '시간', units='s', color="k")
        self._pwm_plot.showGrid(x=True, y=True)
        self._pwm_plot.setYRange(0, 100)  # PWM 범위 0-100

        # PWM 데이터 라인 생성
        self._pwm_curve = self._pwm_plot.plot(
            self._time_history,
            self._pwm_history,
            pen=pg.mkPen(color='b', width=2),
            name="PWM 5"
        )

        # X축 연결 (같이 이동)
        self._temp_plot.setXLink(self._pwm_plot)

        # 그래프 추가
        graph_layout.addWidget(self._temp_plot, 3)
        graph_layout.addWidget(self._pwm_plot, 1)

        # 하단 제어 영역 생성
        control_widget = QWidget()
        control_layout = QHBoxLayout(control_widget)  # 좌우 배치로 변경
        self._layout.addWidget(control_widget, 4)  # 제어 영역 비율 증가

        # 왼쪽 위젯 - 온도 제어, 비상 정지 버튼
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)

        # 온도 제어 그룹
        temp_group = QGroupBox("온도 제어")
        temp_layout = QGridLayout(temp_group)

        # 목표 온도 슬라이더 및 입력 필드
        temp_layout.addWidget(QLabel("목표 온도:"), 0, 0)

        self._temp_slider = QSlider(Qt.Horizontal)
        self._temp_slider.setMinimum(30)
        self._temp_slider.setMaximum(80)
        self._temp_slider.setValue(int(self._target_temp))
        self._temp_slider.setTickPosition(QSlider.TicksBelow)
        self._temp_slider.setTickInterval(5)
        self._temp_slider.valueChanged.connect(self._on_temp_slider_changed)
        temp_layout.addWidget(self._temp_slider, 0, 1)

        # 온도 직접 입력 필드 추가
        self._temp_input = QDoubleSpinBox()
        self._temp_input.setMinimum(30.0)
        self._temp_input.setMaximum(80.0)
        self._temp_input.setValue(self._target_temp)
        self._temp_input.setSingleStep(0.5)
        self._temp_input.valueChanged.connect(self._on_temp_input_changed)
        self._temp_input.setFixedWidth(80)
        temp_layout.addWidget(self._temp_input, 0, 2)
        temp_layout.addWidget(QLabel("°C"), 0, 3)

        # 목표 온도 적용 버튼 추가
        self._apply_temp_button = QPushButton("목표 온도 적용")
        self._apply_temp_button.clicked.connect(self._apply_target_temp)
        temp_layout.addWidget(self._apply_temp_button, 0, 4)

        # 현재 온도 표시 (5번만)
        temp_layout.addWidget(QLabel("현재 온도 5:"), 1, 0)
        self._current_temp_label = QLabel(f"{self._current_temps[5]:.1f} °C")
        self._current_temp_label.setFont(QFont("Arial", 12))
        self._current_temp_label.setStyleSheet("color: red; font-weight: bold;")
        temp_layout.addWidget(self._current_temp_label, 1, 1, 1, 4)

        # PWM 상태 표시 (5번만)
        temp_layout.addWidget(QLabel("PWM 5:"), 2, 0)
        self._pwm_label = QLabel(f"{self._pwm_value}")
        self._pwm_label.setFont(QFont("Arial", 12))
        temp_layout.addWidget(self._pwm_label, 2, 1, 1, 4)

        left_layout.addWidget(temp_group)

        # 비상 정지 버튼 그룹
        emergency_group = QGroupBox("안전 제어")
        emergency_layout = QGridLayout(emergency_group)

        # 비상 정지 버튼
        self._emergency_button = QPushButton("비상 정지")
        self._emergency_button.setStyleSheet(
            "background-color: red; color: white; font-weight: bold; font-size: 14px; min-height: 40px;"
        )
        self._emergency_button.clicked.connect(self._on_emergency_stop)
        emergency_layout.addWidget(self._emergency_button, 0, 0, 1, 2)

        # 비상 정지 해제 버튼
        self._reset_button = QPushButton("시스템 초기화")
        self._reset_button.clicked.connect(self._on_reset_emergency)
        emergency_layout.addWidget(self._reset_button, 1, 0, 1, 2)

        # 안전 상태 표시
        self._safety_label = QLabel("안전 상태: 정상")
        self._safety_label.setStyleSheet("color: green; font-weight: bold;")
        emergency_layout.addWidget(self._safety_label, 2, 0, 1, 2)

        left_layout.addWidget(emergency_group)

        # 중앙 위젯 - PI 제어 게인 설정
        middle_widget = QWidget()
        middle_layout = QVBoxLayout(middle_widget)

        # PI 제어 게인 설정 그룹
        gain_group = QGroupBox("PI 제어 게인 설정")
        gain_layout = QGridLayout(gain_group)

        # Kp 게인 설정
        gain_layout.addWidget(QLabel("비례 게인 (Kp):"), 0, 0)
        self._kp_spinbox = QDoubleSpinBox()
        self._kp_spinbox.setMinimum(0.0)
        self._kp_spinbox.setMaximum(10.0)
        self._kp_spinbox.setValue(self._kp)
        self._kp_spinbox.setSingleStep(0.1)
        self._kp_spinbox.valueChanged.connect(self._on_kp_changed)
        gain_layout.addWidget(self._kp_spinbox, 0, 1)

        # Ki 게인 설정
        gain_layout.addWidget(QLabel("적분 게인 (Ki):"), 1, 0)
        self._ki_spinbox = QDoubleSpinBox()
        self._ki_spinbox.setMinimum(0.0)
        self._ki_spinbox.setMaximum(2.0)
        self._ki_spinbox.setValue(self._ki)
        self._ki_spinbox.setSingleStep(0.01)
        self._ki_spinbox.valueChanged.connect(self._on_ki_changed)
        gain_layout.addWidget(self._ki_spinbox, 1, 1)

        # 설정 적용 버튼
        apply_button = QPushButton("게인 설정 적용")
        apply_button.clicked.connect(self._apply_gain_settings)
        gain_layout.addWidget(apply_button, 2, 0, 1, 2)

        middle_layout.addWidget(gain_group)

        # 데이터 관리 그룹
        data_group = QGroupBox("데이터 관리")
        data_layout = QGridLayout(data_group)

        # CSV 파일 저장 버튼
        self._save_csv_button = QPushButton("CSV 저장")
        self._save_csv_button.clicked.connect(self._save_csv_data)
        data_layout.addWidget(self._save_csv_button, 0, 0)

        # 데이터 초기화 버튼
        self._reset_data_button = QPushButton("데이터 초기화")
        self._reset_data_button.clicked.connect(self._reset_data)
        data_layout.addWidget(self._reset_data_button, 0, 1)

        middle_layout.addWidget(data_group)

        # 오른쪽 위젯 - 프로그램된 온도 프로필
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)

        # 온도 프로필 그룹
        profile_group = QGroupBox("온도 프로필")
        profile_layout = QGridLayout(profile_group)

        # 프로필 선택 콤보박스
        profile_layout.addWidget(QLabel("온도 프로필 선택:"), 0, 0)
        self._profile_combo = QComboBox()
        self._profile_combo.addItems([
            "스텝 함수",
            "사인 함수",
            "삼각파"
        ])
        profile_layout.addWidget(self._profile_combo, 0, 1, 1, 2)

        # 프로필 실행 버튼
        self._run_profile_button = QPushButton("프로필 실행")
        self._run_profile_button.clicked.connect(self._on_run_profile)
        profile_layout.addWidget(self._run_profile_button, 1, 0)

        # 프로필 중지 버튼
        self._stop_profile_button = QPushButton("프로필 중지")
        self._stop_profile_button.clicked.connect(self._on_stop_profile)
        self._stop_profile_button.setEnabled(False)
        profile_layout.addWidget(self._stop_profile_button, 1, 1)

        # 프로필 상태 표시
        self._profile_status = QLabel("프로필 상태: 정지")
        profile_layout.addWidget(self._profile_status, 2, 0, 1, 2)

        right_layout.addWidget(profile_group)

        # 프로필 파라미터 그룹
        param_group = QGroupBox("프로필 파라미터")
        param_layout = QGridLayout(param_group)

        # 최소 온도 설정
        param_layout.addWidget(QLabel("최소 온도 (°C):"), 0, 0)
        self._min_temp_spinbox = QDoubleSpinBox()
        self._min_temp_spinbox.setMinimum(30.0)
        self._min_temp_spinbox.setMaximum(75.0)
        self._min_temp_spinbox.setValue(40.0)
        self._min_temp_spinbox.setSingleStep(1.0)
        param_layout.addWidget(self._min_temp_spinbox, 0, 1)

        # 최대 온도 설정
        param_layout.addWidget(QLabel("최대 온도 (°C):"), 1, 0)
        self._max_temp_spinbox = QDoubleSpinBox()
        self._max_temp_spinbox.setMinimum(35.0)
        self._max_temp_spinbox.setMaximum(78.0)
        self._max_temp_spinbox.setValue(60.0)
        self._max_temp_spinbox.setSingleStep(1.0)
        param_layout.addWidget(self._max_temp_spinbox, 1, 1)

        # 프로필 주기 설정
        param_layout.addWidget(QLabel("주기 (초):"), 2, 0)
        self._period_spinbox = QDoubleSpinBox()
        self._period_spinbox.setMinimum(10.0)
        self._period_spinbox.setMaximum(300.0)
        self._period_spinbox.setValue(60.0)
        self._period_spinbox.setSingleStep(5.0)
        param_layout.addWidget(self._period_spinbox, 2, 1)

        # 실행 시간 설정
        param_layout.addWidget(QLabel("실행 시간 (초):"), 3, 0)
        self._duration_spinbox = QDoubleSpinBox()
        self._duration_spinbox.setMinimum(10.0)
        self._duration_spinbox.setMaximum(600.0)
        self._duration_spinbox.setValue(300.0)
        self._duration_spinbox.setSingleStep(30.0)
        param_layout.addWidget(self._duration_spinbox, 3, 1)

        right_layout.addWidget(param_group)

        # 레이아웃 배치
        control_layout.addWidget(left_widget, 2)  # 왼쪽 영역
        control_layout.addWidget(middle_widget, 1)  # 중앙 영역
        control_layout.addWidget(right_widget, 2)  # 오른쪽 영역

    def _temp_callback(self, msg):
        """
        온도 데이터 수신 콜백
        """
        for i in range(min(len(msg.temperature), len(self._current_temps))):
            self._current_temps[i] = msg.temperature[i]

        # 온도 80도 이상이면 안전 점검
        self._check_temperature_safety()

    def _check_temperature_safety(self):
        """
        온도 안전성 검사 및 처리
        """
        if self._current_temps[self._actuator_idx] >= self._safety_threshold:
            if self._is_temperature_safe:
                self._is_temperature_safe = False
                self._safety_label.setText("안전 상태: 온도 초과!")
                self._safety_label.setStyleSheet("color: red; font-weight: bold;")

                # 비상 정지 메시지 표시
                QMessageBox.critical(self._widget,
                    "안전 경고",
                    f"온도가 안전 임계값({self._safety_threshold}°C)을 초과했습니다!\n비상 정지를 실행합니다.",
                    QMessageBox.Ok)

                # 비상 정지 실행
                self._emergency_stop()
        else:
            if not self._is_temperature_safe and not self._is_emergency_stop:
                self._is_temperature_safe = True
                self._safety_label.setText("안전 상태: 정상")
                self._safety_label.setStyleSheet("color: green; font-weight: bold;")

    def _actuator_callback(self, msg):
        """
        PWM 데이터 수신 콜백
        """
        if len(msg.pwm) > self._actuator_idx:
            self._pwm_value = msg.pwm[self._actuator_idx]

    def _update_gui(self):
        """
        GUI 업데이트
        """
        try:
            # 현재 시간 가져오기
            current_time = time.time() - self._start_time

            # 온도 및 PWM 히스토리 업데이트
            self._temp_history = np.roll(self._temp_history, -1)
            self._temp_history[-1] = self._current_temps[self._actuator_idx]

            self._pwm_history = np.roll(self._pwm_history, -1)
            self._pwm_history[-1] = self._pwm_value

            # 목표 온도 히스토리 업데이트
            self._target_temp_history = np.roll(self._target_temp_history, -1)
            self._target_temp_history[-1] = self._target_temp

            # 시간 히스토리 업데이트
            self._time_history = np.roll(self._time_history, -1)
            self._time_history[-1] = current_time

            # 현재 온도 5번 라벨 업데이트 (빨간색)
            self._current_temp_label.setText(f"{self._current_temps[self._actuator_idx]:.1f} °C")

            # PWM 라벨 업데이트
            self._pwm_label.setText(f"{self._pwm_value}")

            # 그래프 데이터 업데이트
            self._temp_curve.setData(self._time_history, self._temp_history)
            self._target_curve.setData(self._time_history, self._target_temp_history)
            self._pwm_curve.setData(self._time_history, self._pwm_history)

            # X축 표시 형식 업데이트 - 시간 간격 적절히 조정
            self._temp_plot.setLimits(xMin=self._time_history[0], xMax=self._time_history[-1]+10)
            self._pwm_plot.setLimits(xMin=self._time_history[0], xMax=self._time_history[-1]+10)

            # 자동 스크롤 활성화 - 최근 데이터가 보이도록
            self._temp_plot.setXRange(max(0, current_time-120), current_time+5)

            # 프로필이 실행 중이면 온도 업데이트
            if self._is_profile_running:
                self._update_temperature_profile()

        except Exception as e:
            # 예외 처리 - 업데이트 중 오류가 발생하더라도 계속 실행되도록
            self.logger.error(f"GUI 업데이트 중 오류 발생: {str(e)}")

    def _on_temp_slider_changed(self, value):
        """
        온도 슬라이더 변경 핸들러
        """
        # 슬라이더가 변경되면 입력 필드도 업데이트 (신호 차단)
        self._pending_target_temp = float(value)
        self._temp_input.blockSignals(True)
        self._temp_input.setValue(self._pending_target_temp)
        self._temp_input.blockSignals(False)

    def _on_temp_input_changed(self, value):
        """
        온도 입력 필드 변경 핸들러
        """
        # 입력 필드가 변경되면 슬라이더도 업데이트 (신호 차단)
        self._pending_target_temp = float(value)
        self._temp_slider.blockSignals(True)
        self._temp_slider.setValue(int(self._pending_target_temp))
        self._temp_slider.blockSignals(False)

    def _apply_target_temp(self):
        """
        목표 온도 적용 버튼 핸들러
        """
        self._target_temp = self._pending_target_temp
        # 새 목표 온도 발행
        self._publish_target_temp()

    def _publish_target_temp(self):
        """
        목표 온도 발행
        """
        msg = Float64()
        msg.data = self._target_temp
        self._param_pub.publish(msg)
        self.logger.info(f"목표 온도 설정: {self._target_temp:.1f}°C")

    def _on_kp_changed(self, value):
        """
        Kp 게인 변경 핸들러
        """
        self._kp = value

    def _on_ki_changed(self, value):
        """
        Ki 게인 변경 핸들러
        """
        self._ki = value

    def _apply_gain_settings(self):
        """
        PI 게인 설정 적용
        """
        # PI 게인 메시지 발행
        kp_msg = Float64()
        kp_msg.data = self._kp
        ki_msg = Float64()
        ki_msg.data = self._ki

        # 메시지 발행
        self._kp_pub.publish(kp_msg)
        self._ki_pub.publish(ki_msg)

        self.logger.info(f"PI 게인 설정 변경: Kp={self._kp:.2f}, Ki={self._ki:.3f}")

    def _log_filename_callback(self, msg):
        """
        현재 로그 파일명 업데이트
        """
        self._current_log_filename = msg.data
        self.logger.debug(f"Current log filename: {self._current_log_filename}")

    def _reset_logging(self):
        """
        로깅 초기화 서비스 호출
        """
        if not self._reset_log_client.wait_for_service(timeout_sec=1.0):
            self.logger.error("Reset logging service not available")
            QMessageBox.warning(self._widget,
                "서비스 사용 불가",
                "로깅 초기화 서비스를 사용할 수 없습니다.",
                QMessageBox.Ok)
            return False

        request = Trigger.Request()
        future = self._reset_log_client.call_async(request)

        # GUI가 멈추지 않도록 비동기 방식으로 처리
        self._node.executor.spin_until_future_complete(future, timeout_sec=2.0)

        if future.done():
            response = future.result()
            if response.success:
                self.logger.info(f"Logging reset: {response.message}")
                return True
            else:
                self.logger.error(f"Failed to reset logging: {response.message}")
                return False
        else:
            self.logger.error("Failed to call reset logging service (timeout)")
            return False

    def _save_log_file(self, custom_filename=""):
        """
        현재 로그 파일 저장 서비스 호출
        """
        if not self._save_log_client.wait_for_service(timeout_sec=1.0):
            self.logger.error("Save log file service not available")
            QMessageBox.warning(self._widget,
                "서비스 사용 불가",
                "로그 파일 저장 서비스를 사용할 수 없습니다.",
                QMessageBox.Ok)
            return False

        request = SetBool.Request()
        request.data = True
        future = self._save_log_client.call_async(request)

        # GUI가 멈추지 않도록 비동기 방식으로 처리
        self._node.executor.spin_until_future_complete(future, timeout_sec=2.0)

        if future.done():
            response = future.result()
            if response.success:
                self.logger.info(f"Log file saved: {response.message}")
                return True
            else:
                self.logger.error(f"Failed to save log file: {response.message}")
                return False
        else:
            self.logger.error("Failed to call save log file service (timeout)")
            return False

    def _set_log_filename(self, filename):
        """
        로그 파일명 설정 서비스 호출
        """
        if not self._set_filename_client.wait_for_service(timeout_sec=1.0):
            self.logger.error("Set log filename service not available")
            return False

        request = SetBool.Request()
        request.data = True  # 실제 구현에서는 파일명 전달을 위한 적절한 메시지 타입 사용
        future = self._set_filename_client.call_async(request)

        # GUI가 멈추지 않도록 비동기 방식으로 처리
        self._node.executor.spin_until_future_complete(future, timeout_sec=2.0)

        if future.done():
            response = future.result()
            if response.success:
                self.logger.info(f"Log filename set: {response.message}")
                return True
            else:
                self.logger.error(f"Failed to set log filename: {response.message}")
                return False
        else:
            self.logger.error("Failed to call set log filename service (timeout)")
            return False


    def _on_emergency_stop(self):
        """
        비상 정지 버튼 핸들러
        """
        self._emergency_stop()
        QMessageBox.warning(self._widget,
            "비상 정지",
            "비상 정지가 활성화되었습니다. 모든 출력이 중단되었습니다.",
            QMessageBox.Ok)

    def _emergency_stop(self):
        """
        비상 정지 실행
        """
        self._is_emergency_stop = True

        # 비상 정지 메시지 발행
        msg = Bool()
        msg.data = True
        self._emergency_pub.publish(msg)

        # 버튼 상태 업데이트
        self._emergency_button.setEnabled(False)
        self._apply_temp_button.setEnabled(False)
        self._run_profile_button.setEnabled(False)

        # 프로필 중지
        if self._is_profile_running:
            self._on_stop_profile()

        # 안전 상태 표시 업데이트
        self._safety_label.setText("안전 상태: 비상 정지 중")
        self._safety_label.setStyleSheet("color: red; font-weight: bold;")

        self.logger.warning("비상 정지가 활성화되었습니다")

    def _on_reset_emergency(self):
        """
        비상 정지 해제 핸들러
        """
        if self._is_emergency_stop:
            # 비상 정지 해제 메시지 발행
            msg = Bool()
            msg.data = False
            self._emergency_pub.publish(msg)

            # 버튼 상태 초기화
            self._emergency_button.setEnabled(True)
            self._apply_temp_button.setEnabled(True)
            self._run_profile_button.setEnabled(True)

            # 상태 초기화
            self._is_emergency_stop = False
            if self._current_temps[self._actuator_idx] < self._safety_threshold:
                self._is_temperature_safe = True
                self._safety_label.setText("안전 상태: 정상")
                self._safety_label.setStyleSheet("color: green; font-weight: bold;")

            self.logger.info("시스템이 초기화되었습니다")

    def _save_csv_data(self):
        """
        로거 노드의 CSV 파일을 사용자 지정 위치로 복사
        """
        try:
            # 현재 로그 파일 확인
            if not self._current_log_filename:
                # 로그 파일명을 얻지 못했다면, 기본 위치에서 최신 파일 찾기 시도
                log_dir = os.path.expanduser("~/temp_logs")
                if os.path.exists(log_dir):
                    csv_files = [f for f in os.listdir(log_dir) if f.endswith('.csv')]
                    if csv_files:
                        # 가장 최근 파일 선택
                        csv_files.sort(reverse=True)  # 파일명에 날짜가 포함되어 있으므로 정렬하면 최신 순
                        self._current_log_filename = csv_files[0]
                        self.logger.info(f"최신 로그 파일을 찾았습니다: {self._current_log_filename}")
                    else:
                        QMessageBox.warning(self._widget,
                            "파일 없음",
                            "로그 디렉토리에 CSV 파일이 없습니다.",
                            QMessageBox.Ok)
                        return
                else:
                    QMessageBox.warning(self._widget,
                        "디렉토리 없음",
                        f"로그 디렉토리가 존재하지 않습니다: {log_dir}",
                        QMessageBox.Ok)
                    return

            # 로거 노드에 저장 요청
            save_success = self._save_log_file()
            if not save_success:
                self.logger.warning("로거 노드에 저장 요청 실패, 기존 파일 사용 시도")

            # 소스 파일 경로 구성
            log_dir = os.path.expanduser("~/temp_logs")
            source_path = os.path.join(log_dir, self._current_log_filename)

            # 파일 존재 확인
            if not os.path.exists(source_path):
                self.logger.error(f"원본 로그 파일을 찾을 수 없습니다: {source_path}")
                QMessageBox.critical(self._widget,
                    "파일 오류",
                    f"원본 로그 파일을 찾을 수 없습니다:\n{source_path}",
                    QMessageBox.Ok)
                return

            # 사용자 지정 파일 위치 선택 - 절대 경로 사용
            current_datetime = datetime.now().strftime("%Y%m%d_%H%M%S")
            default_filename = f"temp_control_data_{current_datetime}.csv"
            home_dir = os.path.expanduser("~")
            default_save_path = os.path.join(home_dir, default_filename)

            self.logger.info(f"파일 선택 대화상자 열기. 기본 경로: {default_save_path}")

            # QFileDialog를 최상위 위젯으로 직접 생성
            filename = QFileDialog.getSaveFileName(
                self._widget,
                "CSV 데이터 저장",
                default_save_path,
                "CSV 파일 (*.csv)"
            )[0]  # 첫 번째 요소만 가져옴(파일 경로)

            self.logger.info(f"선택된 파일 경로: {filename}")

            if not filename:
                self.logger.info("파일 저장 취소됨")
                return

            # 파일 복사 전 상세 정보 로깅
            self.logger.info(f"복사 시작 - 소스: {source_path} -> 대상: {filename}")
            source_size = os.path.getsize(source_path)
            self.logger.info(f"소스 파일 크기: {source_size} 바이트")

            # 파일 복사
            import shutil
            shutil.copy2(source_path, filename)

            # 복사 확인
            if os.path.exists(filename):
                dest_size = os.path.getsize(filename)
                self.logger.info(f"복사 완료. 대상 파일 크기: {dest_size} 바이트")

                # 성공 메시지
                QMessageBox.information(self._widget,
                    "저장 완료",
                    f"로그 파일이 다음 위치에 저장되었습니다:\n{filename}",
                    QMessageBox.Ok)
            else:
                self.logger.error(f"복사 후 파일이 존재하지 않음: {filename}")
                QMessageBox.critical(self._widget,
                    "저장 실패",
                    f"파일 복사 후 확인에 실패했습니다.",
                    QMessageBox.Ok)

        except Exception as e:
            self.logger.error(f"CSV 저장 처리 중 오류 발생: {str(e)}", exc_info=True)
            QMessageBox.critical(self._widget,
                "저장 오류",
                f"CSV 파일 저장 과정에서 오류가 발생했습니다:\n{str(e)}",
                QMessageBox.Ok)

    def _reset_data(self):
        """
        데이터 초기화 및 로깅 초기화
        """
        # 사용자 확인 요청
        result = QMessageBox.question(self._widget,
            "데이터 초기화",
            "그래프 데이터와 로깅을 초기화하시겠습니까?\n현재 데이터는 모두 삭제됩니다.",
            QMessageBox.Yes | QMessageBox.No)

        if result == QMessageBox.Yes:
            # 시작 시간 재설정
            self._start_time = time.time()

            # 데이터 초기화
            self._temp_history = np.zeros(self._history_size)
            self._pwm_history = np.zeros(self._history_size)
            self._target_temp_history = np.ones(self._history_size) * self._target_temp
            self._time_history = np.linspace(-self._history_size, 0, self._history_size)

            # 초기 시간 설정 (현재 시간에서 과거로)
            current_time = time.time() - self._start_time
            for i in range(self._history_size):
                self._time_history[i] = current_time - (self._history_size - i)

            # 그래프 업데이트
            self._temp_curve.setData(self._time_history, self._temp_history)
            self._target_curve.setData(self._time_history, self._target_temp_history)
            self._pwm_curve.setData(self._time_history, self._pwm_history)

            # 로거 노드의 로깅 초기화
            try:
                if self._reset_logging():
                    self.logger.info("그래프 데이터와 로깅이 초기화되었습니다")
                else:
                    self.logger.warning("그래프 데이터만 초기화되었습니다. 로깅 초기화 실패")
            except Exception as e:
                self.logger.error(f"로깅 초기화 중 오류 발생: {str(e)}")
                QMessageBox.warning(self._widget,
                    "로깅 초기화 오류",
                    f"그래프는 초기화되었으나 로깅 초기화 중 오류가 발생했습니다: {str(e)}",
                    QMessageBox.Ok)

    def _on_run_profile(self):
        """
        온도 프로필 실행 버튼 핸들러
        """
        try:
            if self._is_profile_running:
                return

            profile_idx = self._profile_combo.currentIndex()
            if profile_idx < 0:
                return

            # 프로필 설정
            self._current_profile = profile_idx
            self._profile_start_time = time.time()
            self._is_profile_running = True

            # 버튼 상태 업데이트
            self._run_profile_button.setEnabled(False)
            self._stop_profile_button.setEnabled(True)
            self._profile_status.setText(f"프로필 상태: {self._profile_combo.currentText()} 실행 중")

            # 타이머 생성
            if self._profile_timer is None:
                self._profile_timer = QTimer()
                self._profile_timer.timeout.connect(self._update_temperature_profile)
                self._profile_timer.start(int(self._profile_step_size * 1000))  # ms 단위 설정
            else:
                self._profile_timer.start()

            self.logger.info(f"온도 프로필이 시작되었습니다: {self._profile_combo.currentText()}")

        except Exception as e:
            self.logger.error(f"프로필 실행 중 오류 발생: {str(e)}")

    def _on_stop_profile(self):
        """
        온도 프로필 중지 버튼 핸들러
        """
        if not self._is_profile_running:
            return

        # 타이머 중지
        if self._profile_timer is not None:
            self._profile_timer.stop()

        # 상태 초기화
        self._is_profile_running = False
        self._current_profile = None

        # 버튼 상태 업데이트
        self._run_profile_button.setEnabled(True)
        self._stop_profile_button.setEnabled(False)
        self._profile_status.setText("프로필 상태: 정지")

        self.logger.info("온도 프로필이 중지되었습니다")

    def _update_temperature_profile(self):
        """
        온도 프로필 업데이트 및 목표 온도 설정
        """
        if not self._is_profile_running or self._current_profile is None:
            return

        try:
            # 현재 시간 계산
            current_time = time.time() - self._profile_start_time
            period = self._period_spinbox.value()
            max_duration = self._duration_spinbox.value()

            # 최대 시간 도달 시 중지
            if current_time > max_duration:
                self._on_stop_profile()
                return

            # 현재 시간의 주기 내 위치 계산
            phase = (current_time % period) / period

            # 사용자 설정 온도 범위 가져오기
            min_temp = self._min_temp_spinbox.value()
            max_temp = self._max_temp_spinbox.value()
            temp_range = max_temp - min_temp
            mid_temp = min_temp + (temp_range / 2)

            # 프로필에 따른 목표 온도 계산
            new_target = mid_temp  # 기본값

            if self._current_profile == 0:  # 스텝 함수
                if phase < 0.5:
                    new_target = min_temp
                else:
                    new_target = max_temp

            elif self._current_profile == 1:  # 사인 함수
                amplitude = temp_range / 2
                new_target = mid_temp + amplitude * math.sin(2 * math.pi * phase)

            elif self._current_profile == 2:  # 삼각파
                if phase < 0.5:
                    new_target = min_temp + (phase * 2 * temp_range)  # 0->0.5에서 min->max
                else:
                    new_target = max_temp - ((phase - 0.5) * 2 * temp_range)  # 0.5->1에서 max->min

            # 안전 범위 내로 제한
            if new_target > 78.0:
                new_target = 78.0
            elif new_target < 30.0:
                new_target = 30.0

            # 목표 온도 업데이트
            self._target_temp = new_target

            # GUI 업데이트
            self._temp_input.blockSignals(True)
            self._temp_slider.blockSignals(True)
            self._temp_input.setValue(self._target_temp)
            self._temp_slider.setValue(int(self._target_temp))
            self._temp_input.blockSignals(False)
            self._temp_slider.blockSignals(False)

            # 새 목표 온도 발행
            self._publish_target_temp()

        except Exception as e:
            self.logger.error(f"온도 프로필 업데이트 중 오류 발생: {str(e)}")
            self._on_stop_profile()  # 오류 발생 시 프로필 중지

    def shutdown_plugin(self):
        """
        플러그인 종료시 호출
        """
        # 타이머 중지
        self._update_timer.stop()
        if self._profile_timer is not None:
            self._profile_timer.stop()

        # 비상 정지 메시지 발행 해제
        if self._is_emergency_stop:
            try:
                msg = Bool()
                msg.data = False
                self._emergency_pub.publish(msg)
            except Exception:
                pass

        self.logger.info('온도 제어 GUI 플러그인이 종료되었습니다.')

    def save_settings(self, plugin_settings, instance_settings):
        """
        설정 저장
        """
        instance_settings.set_value('target_temperature', self._target_temp)
        instance_settings.set_value('kp', self._kp)
        instance_settings.set_value('ki', self._ki)
        instance_settings.set_value('period', self._period_spinbox.value())
        instance_settings.set_value('duration', self._duration_spinbox.value())

    def restore_settings(self, plugin_settings, instance_settings):
        """
        설정 복원
        """
        # 목표 온도 복원
        self._target_temp = float(instance_settings.value('target_temperature', 50.0))
        self._pending_target_temp = self._target_temp
        self._temp_slider.setValue(int(self._target_temp))
        self._temp_input.setValue(self._target_temp)
        self._target_line.setValue(self._target_temp)

        # PI 게인 복원
        self._kp = float(instance_settings.value('kp', 2.0))
        self._ki = float(instance_settings.value('ki', 0.1))
        self._kp_spinbox.setValue(self._kp)
        self._ki_spinbox.setValue(self._ki)

        # 프로필 설정 복원
        period = float(instance_settings.value('period', 60.0))
        duration = float(instance_settings.value('duration', 300.0))
        self._period_spinbox.setValue(period)
        self._duration_spinbox.setValue(duration)


    def _save_log_file(self):
      """
      현재 로그 파일 저장 서비스 호출
      """
      try:
          if not self._save_log_client.wait_for_service(timeout_sec=1.0):
              self.logger.error("Save log file service not available")
              return False

          request = SetBool.Request()
          request.data = True
          future = self._save_log_client.call_async(request)

          # GUI가 멈추지 않도록 비동기 방식으로 처리
          rclpy.spin_until_future_complete(self._node, future, timeout_sec=2.0)

          if future.done():
              response = future.result()
              if response.success:
                  self.logger.info(f"Log file saved: {response.message}")
                  return True
              else:
                  self.logger.error(f"Failed to save log file: {response.message}")
                  return False
          else:
              self.logger.error("Failed to call save log file service (timeout)")
              return False
      except Exception as e:
          self.logger.error(f"Error calling save log service: {str(e)}", exc_info=True)
          return False
