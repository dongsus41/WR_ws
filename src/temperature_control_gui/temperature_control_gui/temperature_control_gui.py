import os
import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.animation import FuncAnimation

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSlider, QPushButton, QSpinBox
from python_qt_binding.QtCore import Qt, QTimer

from rqt_gui_py.plugin import Plugin
from wearable_robot_interfaces.msg import TemperatureData
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

class TemperatureControlGUI(Plugin):
    def __init__(self, context):
        super(TemperatureControlGUI, self).__init__(context)
        # 플러그인 제목 설정
        self.setObjectName('TemperatureControlGUI')

        # QWidget 생성
        self._widget = QWidget()
        self._widget.setObjectName('TemperatureControlGUIWidget')
        self._widget.setWindowTitle('Temperature Control GUI')

        # 레이아웃 설정
        main_layout = QVBoxLayout(self._widget)

        # Matplotlib 그래프 설정
        self.fig = Figure(figsize=(5, 4), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvas(self.fig)
        main_layout.addWidget(self.canvas)

        # 컨트롤 위젯 레이아웃
        control_layout = QHBoxLayout()
        main_layout.addLayout(control_layout)

        # 목표 온도 슬라이더 및 표시
        temp_layout = QVBoxLayout()
        control_layout.addLayout(temp_layout)

        temp_label = QLabel("목표 온도 (°C)")
        temp_layout.addWidget(temp_label)

        slider_layout = QHBoxLayout()
        temp_layout.addLayout(slider_layout)

        self.temp_slider = QSlider(Qt.Horizontal)
        self.temp_slider.setMinimum(30)
        self.temp_slider.setMaximum(80)
        self.temp_slider.setValue(50)
        self.temp_slider.setTickPosition(QSlider.TicksBelow)
        self.temp_slider.setTickInterval(5)
        slider_layout.addWidget(self.temp_slider)

        self.temp_spinbox = QSpinBox()
        self.temp_spinbox.setMinimum(30)
        self.temp_spinbox.setMaximum(80)
        self.temp_spinbox.setValue(50)
        slider_layout.addWidget(self.temp_spinbox)

        # 적용 버튼
        self.apply_button = QPushButton("적용")
        temp_layout.addWidget(self.apply_button)

        # 현재 온도 표시
        self.current_temp_label = QLabel("현재 온도: -- °C")
        main_layout.addWidget(self.current_temp_label)

        # 상태 표시
        self.status_label = QLabel("상태: 준비")
        main_layout.addWidget(self.status_label)

        # 위젯 표시
        context.add_widget(self._widget)

        # 슬라이더와 스핀박스 연결
        self.temp_slider.valueChanged.connect(self.temp_spinbox.setValue)
        self.temp_spinbox.valueChanged.connect(self.temp_slider.setValue)

        # 적용 버튼 연결
        self.apply_button.clicked.connect(self.apply_target_temperature)

        # ROS 노드 초기화
        rclpy.init(args=None)
        self.node = rclpy.create_node('temperature_control_gui')

        # 데이터 저장용 변수
        self.temperature_data = []
        self.target_temp_data = []
        self.time_data = []
        self.start_time = self.node.get_clock().now().seconds_nanoseconds()[0]

        # 온도 데이터 구독
        self.temp_subscription = self.node.create_subscription(
            TemperatureData,
            'temperature_data',
            self.temperature_callback,
            10)

        # 파라미터 서비스 클라이언트
        self.parameter_client = self.node.create_client(
            SetParameters,
            '/actuator_temp_control_node/set_parameters')

        # 데이터 폴링 및 그래프 업데이트 타이머
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(100)  # 100ms = 10Hz

        # 현재 목표 온도 가져오기
        self.current_target_temp = 50.0  # 기본값
        self.active_actuator = 5  # 기본값

    def shutdown_plugin(self):
        # 플러그인 종료 시 정리
        self.timer.stop()
        self.node.destroy_node()
        rclpy.shutdown()

    def temperature_callback(self, msg):
        # 온도 데이터 수신 시 처리
        if len(msg.temperature) > self.active_actuator:
            current_temp = msg.temperature[self.active_actuator]
            current_time = self.node.get_clock().now().seconds_nanoseconds()[0] - self.start_time

            # 데이터 저장
            self.temperature_data.append(current_temp)
            self.target_temp_data.append(self.current_target_temp)
            self.time_data.append(current_time)

            # 최대 60초 데이터만 보관
            if len(self.time_data) > 600:  # 10Hz * 60초 = 600 데이터 포인트
                self.temperature_data.pop(0)
                self.target_temp_data.pop(0)
                self.time_data.pop(0)

    def update_gui(self):
        # ROS 메시지 처리
        rclpy.spin_once(self.node, timeout_sec=0.001)

        # 현재 온도 표시 업데이트
        if self.temperature_data:
            self.current_temp_label.setText(f"현재 온도: {self.temperature_data[-1]:.1f} °C")

        # 그래프 업데이트
        self.update_plot()

    def update_plot(self):
        # 그래프 데이터가 있을 때만 업데이트
        if not self.time_data:
            return

        # 그래프 초기화
        self.ax.clear()

        # 온도 데이터 그래프 그리기
        self.ax.plot(self.time_data, self.temperature_data, 'r-', label='현재 온도')
        self.ax.plot(self.time_data, self.target_temp_data, 'b--', label='목표 온도')

        # 그래프 설정
        self.ax.set_xlabel('시간 (초)')
        self.ax.set_ylabel('온도 (°C)')
        self.ax.set_title('구동기 온도 모니터링')
        self.ax.legend(loc='upper right')
        self.ax.grid(True)

        # 최근 30초 데이터만 표시
        if self.time_data[-1] - self.time_data[0] > 30:
            self.ax.set_xlim(self.time_data[-1] - 30, self.time_data[-1])

        # Y축 범위 설정
        min_temp = min(min(self.temperature_data), min(self.target_temp_data)) - 5
        max_temp = max(max(self.temperature_data), max(self.target_temp_data)) + 5
        self.ax.set_ylim(min_temp, max_temp)

        # 그래프 업데이트
        self.fig.tight_layout()
        self.canvas.draw()

    def apply_target_temperature(self):
        # 목표 온도 값 가져오기
        target_temp = float(self.temp_spinbox.value())
        self.current_target_temp = target_temp

        # 파라미터 변경 요청 생성
        req = SetParameters.Request()

        # 파라미터 값 생성
        param_value = ParameterValue()
        param_value.type = ParameterType.PARAMETER_DOUBLE
        param_value.double_value = target_temp

        # 파라미터 객체 생성
        parameter = Parameter()
        parameter.name = 'target_temperature'
        parameter.value = param_value

        req.parameters = [parameter]

        # 파라미터 서비스 호출
        self.status_label.setText("상태: 목표 온도 변경 중...")

        future = self.parameter_client.call_async(req)
        future.add_done_callback(self.parameter_callback)

    def parameter_callback(self, future):
        try:
            response = future.result()
            if all(result.successful for result in response.results):
                self.status_label.setText(f"상태: 목표 온도 변경 성공 ({self.current_target_temp:.1f}°C)")
            else:
                error_msgs = [result.reason for result in response.results if not result.successful]
                self.status_label.setText(f"상태: 목표 온도 변경 실패 ({', '.join(error_msgs)})")
        except Exception as e:
            self.status_label.setText(f"상태: 목표 온도 변경 오류 ({str(e)})")
