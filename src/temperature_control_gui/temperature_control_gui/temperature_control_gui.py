import os
import rclpy
import numpy as np
import time
from datetime import datetime
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot
from python_qt_binding.QtWidgets import (QWidget, QVBoxLayout, QPushButton, QSlider, QLabel,
                                         QHBoxLayout, QLineEdit, QGroupBox, QSplitter,
                                         QDoubleSpinBox, QSpinBox, QGridLayout)
from python_qt_binding.QtGui import QFont, QColor, QPalette, QIntValidator, QDoubleValidator
import pyqtgraph as pg

from rqt_gui_py.plugin import Plugin
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from std_msgs.msg import Float64
from wearable_robot_interfaces.msg import TemperatureData, ActuatorCommand

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
        self._history_size = 300  # 5분 데이터 (1초에 1샘플)

        # 온도 기록을 위한 데이터
        self._temp_history = np.zeros(self._history_size)
        self._pwm_history = np.zeros(self._history_size)
        self._time_history = np.linspace(-self._history_size, 0, self._history_size)

        # 초기 시간 설정 (현재 시간에서 과거로)
        current_time = time.time()
        for i in range(self._history_size):
            self._time_history[i] = current_time - (self._history_size - i)

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

        # 타이머 설정
        self._update_timer = QTimer()
        self._update_timer.timeout.connect(self._update_gui)
        self._update_timer.start(10)  # 1초 간격으로 업데이트

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
        self._temp_plot.enableAutoRange(axis='y')  # Y축 자동 스케일링

        # 온도 5번 데이터 라인 생성
        self._temp_curve = self._temp_plot.plot(
            self._time_history,
            self._temp_history,
            pen=pg.mkPen(color='r', width=2),
            name="온도 5"
        )

        # 목표 온도 라인
        self._target_line = pg.InfiniteLine(
            pos=self._target_temp,
            angle=0,
            pen=pg.mkPen(color='k', width=2, style=Qt.DashLine),
            label='목표 온도={value:0.1f}°C',
            labelOpts={'position':0.1, 'color': (0,0,0), 'fill': (200,200,200,50)}
        )
        self._temp_plot.addItem(self._target_line)

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
        self._layout.addWidget(control_widget, 3)

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

        control_layout.addWidget(temp_group, 1)

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

        control_layout.addWidget(gain_group, 1)

    def _temp_callback(self, msg):
        """
        온도 데이터 수신 콜백
        """
        for i in range(min(len(msg.temperature), len(self._current_temps))):
            self._current_temps[i] = msg.temperature[i]

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
            self._temp_history[-1] = self._current_temps[5]

            self._pwm_history = np.roll(self._pwm_history, -1)
            self._pwm_history[-1] = self._pwm_value

            # 시간 히스토리 업데이트
            self._time_history = np.roll(self._time_history, -1)
            self._time_history[-1] = current_time

            # 현재 온도 5번 라벨 업데이트 (빨간색)
            self._current_temp_label.setText(f"{self._current_temps[5]:.1f} °C")

            # PWM 라벨 업데이트
            self._pwm_label.setText(f"{self._pwm_value}")

            # 그래프 데이터 업데이트
            self._temp_curve.setData(self._time_history, self._temp_history)
            self._pwm_curve.setData(self._time_history, self._pwm_history)

            # X축 표시 형식 업데이트 - 시간 간격 적절히 조정
            self._temp_plot.setLimits(xMin=self._time_history[0], xMax=self._time_history[-1]+10)
            self._pwm_plot.setLimits(xMin=self._time_history[0], xMax=self._time_history[-1]+10)

            # 자동 스크롤 활성화 - 최근 데이터가 보이도록
            self._temp_plot.setXRange(max(0, current_time-120), current_time+5)

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
        self._target_line.setValue(self._target_temp)

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

    def shutdown_plugin(self):
        """
        플러그인 종료시 호출
        """
        self._update_timer.stop()
        self.logger.info('온도 제어 GUI 플러그인이 종료되었습니다.')

    def save_settings(self, plugin_settings, instance_settings):
        """
        설정 저장
        """
        instance_settings.set_value('target_temperature', self._target_temp)
        instance_settings.set_value('kp', self._kp)
        instance_settings.set_value('ki', self._ki)

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
