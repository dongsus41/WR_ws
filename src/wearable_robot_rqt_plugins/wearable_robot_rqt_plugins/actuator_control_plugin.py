#!/usr/bin/env python3

import os
import threading
import csv
import datetime
import numpy as np
import rclpy
import sys
from std_srvs.srv import Trigger
from std_srvs.srv import SetBool
from std_msgs.msg import String, Bool, Float64
import rclpy.task
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from wearable_robot_interfaces.msg import ActuatorCommand, TemperatureData
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QMessageBox, QVBoxLayout, QLabel
from python_qt_binding.QtCore import Qt, QTimer



# PyQtGraph 라이브러리 가져오기 및 확실한 오류 처리
try:
    import pyqtgraph as pg
    PYQTGRAPH_AVAILABLE = True
except ImportError:
    PYQTGRAPH_AVAILABLE = False
    print("[ERROR]: PyQtGraph 라이브러리가 설치되지 않았습니다.")
    print("pip install pyqtgraph 명령어로 설치하세요.")


class ActuatorControlPlugin(Plugin):
    """
    웨어러블 로봇의 구동기 제어 및 온도 모니터링을 위한 rqt 플러그인
    """
    def __init__(self, context):
        super(ActuatorControlPlugin, self).__init__(context)
        # 플러그인 제목 설정
        self.setObjectName('ActuatorControlPlugin')

        # 중요: 클래스 속성 초기화를 먼저 진행
        # 그래프 플로팅 관련 변수 초기화
        self.is_plotting = False            # 그래프 플로팅 상태
        self.buffer_size = 300              # 5분 (300초) 동안의 데이터 저장
        self.time_data = []                 # 시간 데이터 (x축)
        self.temp_data = []                 # 온도 데이터
        self.target_temp_data = []          # 목표 온도 데이터
        self.pwm_data = []                  # PWM 데이터
        self.start_time = 0.0              # 그래프 시작 시간

        # 내부 상태 변수 초기화
        self.temperature = 0.0              # 현재 온도
        self.current_pwm = 0                # 현재 PWM 값
        self.auto_mode = False              # 자동 제어 모드 여부
        self.is_emergency_stop = False      # 비상 정지 상태
        self.safety_temp_threshold = 80.0   # 안전 온도 임계값 (°C)
        self.is_data_logger_available = False  # 데이터 로거 사용 가능 여부
        self.is_logger_active = False       # 로거 활성화 상태

        # 그래프 관련 객체 초기화
        self.temp_plot_widget = None
        self.pwm_plot_widget = None
        self.temp_line = None
        self.target_temp_line = None
        self.pwm_line = None
        self.temp_legend = None

        # ROS 노드 초기화
        try:
            import rclpy
            if not rclpy.ok():
                rclpy.init()


            node_name = "actuator_control_plugin_internal"
            self.node = Node(node_name)

            self.node.get_logger().info(f"노드 생성 성공: {node_name}")

            # QoS 설정
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )

        except Exception as e:
            print(f"[ERROR]: 노드 생성 실패: {str(e)}")
            return

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
        self.filename_publisher = self.node.create_publisher(
            String, 'set_log_filename', qos_profile)  # 로깅 파일명 설정

        # 구독자 생성
        self.temp_sub = self.node.create_subscription(
            TemperatureData, 'temperature_data', self.temp_callback, qos_profile)  # 온도 데이터
        self.pwm_sub = self.node.create_subscription(
            ActuatorCommand, 'pwm_state', self.pwm_callback, qos_profile)  # 현재 PWM 상태
        self.logging_status_sub = self.node.create_subscription(
            Bool, 'logging_status', self.logging_status_callback, qos_profile)  # 로깅 상태

        # 주기적인 작업을 위한 타이머 (1초마다 안전 검사)
        self.timer = self.node.create_timer(1.0, self.timer_callback)

        self._shutdown_flag = False

        # ROS 2 스핀을 위한 스레드 생성
        self.spin_thread = threading.Thread(target=self.spin_ros)
        self.spin_thread.daemon = True
        self.spin_thread.start()

        # 클래스 내 __init__에 추가
        self.data_lock = threading.RLock()  # 재진입 가능한 락 사용

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

        # 데이터 로깅 경로 설정
        self.data_logging_path = os.path.expanduser("~/temperature_logs")

        # 로그 저장 경로가 없으면 생성
        if not os.path.exists(self.data_logging_path):
            os.makedirs(self.data_logging_path, exist_ok=True)

        # 그래프 UI 설정 및 초기화
        self.setup_graphs()

        # 그래프 제어 버튼 연결
        self._widget.graph_start_button.clicked.connect(self.start_plotting)
        self._widget.graph_stop_button.clicked.connect(self.stop_plotting)
        self._widget.reset_graph_button.clicked.connect(self.reset_graph_data)

        # 로깅 서비스 클라이언트 초기화
        self.reset_logger_client = self.node.create_client(Trigger, 'reset_temperature_log')
        self.save_logger_client = self.node.create_client(SetBool, 'save_temperature_log')

        # 서비스 가용성 검사 타이머 (5초마다)
        self.service_check_timer = QTimer()
        self.service_check_timer.timeout.connect(self.check_logger_services)
        self.service_check_timer.start(5000)  # 5초마다 검사

        # 데이터 저장 버튼 연결
        self._widget.save_data_button.clicked.connect(self.request_data_save)
        self._widget.reset_graph_button.clicked.connect(self.reset_data_logging)

        # 로그 파일명 수신 구독
        self.log_filename_sub = self.node.create_subscription(
            String, 'current_log_filename', self.log_filename_callback, qos_profile)

        # Qt 타이머 추가 (UI 업데이트용)
        self.qt_timer = QTimer()
        self.qt_timer.timeout.connect(self.update_ui)
        self.qt_timer.start(100)  # 100ms 마다 UI 업데이트

        # 그래프 업데이트 타이머
        self.graph_timer = QTimer()
        self.graph_timer.timeout.connect(self.update_graphs)
        self.graph_timer.setInterval(100)  # 100ms마다 그래프 업데이트 (10Hz)

        # 초기화를 완료했음을 표시
        self._widget.status_label.setText('플러그인이 초기화되었습니다')
        self.node.get_logger().info('구동기 제어 플러그인이 초기화되었습니다')

        # 초기 모드 발행 (기본: 수동 모드)
        self.publish_control_mode(False)

        # 초기 서비스 가용성 검사
        self.check_logger_services()

    def log_filename_callback(self, msg):
        """로그 파일 이름 수신 콜백"""
        if msg.data:
            self._widget.log_status_label.setText(f"로깅 상태: 활성화 - {msg.data}")
            self.node.get_logger().info(f"로그 파일명 업데이트: {msg.data}")

    def check_logger_services(self):
        """데이터 로거 서비스 가용성 검사"""
        reset_available = self.reset_logger_client.service_is_ready()
        save_available = self.save_logger_client.service_is_ready()

        # 이전 상태와 비교하여 변경이 있을 때만 로그 출력
        if reset_available and save_available != self.is_data_logger_available:
            self.is_data_logger_available = True
            self.node.get_logger().info("데이터 로거 서비스 연결 성공")
            self._widget.status_label.setText("데이터 로거 서비스 연결됨")
        elif not (reset_available and save_available) and self.is_data_logger_available:
            self.is_data_logger_available = False
            self.node.get_logger().warn("데이터 로거 서비스를 사용할 수 없습니다. data_logger 노드가 실행 중인지 확인하세요.")
            self._widget.status_label.setText("데이터 로거 서비스 연결 실패")

        # UI 버튼 상태 업데이트
        self._widget.save_data_button.setEnabled(self.is_data_logger_available)

    def logging_status_callback(self, msg):
        """로깅 상태 콜백"""
        self.is_logger_active = msg.data
        if msg.data:
            if not self.is_plotting:
                # 그래프 플로팅도 자동으로 시작
                self.start_plotting()
            self._widget.log_status_label.setText("로깅 상태: 활성화")
            self._widget.save_data_button.setText("로깅 중지")
        else:
            self._widget.log_status_label.setText("로깅 상태: 대기 중")
            self._widget.save_data_button.setText("데이터 저장")

        self.node.get_logger().debug(f"로깅 상태 업데이트: {msg.data}")

    def setup_graphs(self):
        """
        그래프 위젯 초기화 및 설정
        """
        try:
            if not PYQTGRAPH_AVAILABLE:
                self.node.get_logger().error("PyQtGraph 라이브러리가 설치되지 않았습니다. 그래프 기능을 사용할 수 없습니다.")
                # 그래프 영역에 오류 메시지 표시
                error_layout = QVBoxLayout()
                error_label = QLabel("PyQtGraph 라이브러리가 설치되지 않았습니다.\n'pip install pyqtgraph' 명령어로 설치하세요.")
                error_label.setStyleSheet("color: red; font-weight: bold;")
                error_layout.addWidget(error_label)
                self._widget.temp_graph_frame.setLayout(error_layout)
                self._widget.pwm_graph_frame.setLayout(QVBoxLayout())
                return

            # PyQtGraph 기본 설정
            pg.setConfigOptions(antialias=True, useOpenGL=False)  # OpenGL 비활성화로 호환성 향상

            # 온도 그래프 설정
            self.temp_plot_widget = pg.PlotWidget()
            self.temp_plot_widget.setBackground('w')
            self.temp_plot_widget.setTitle("온도 그래프", color="k", size="12pt")
            self.temp_plot_widget.setLabel("left", "온도 (°C)", color="k")
            self.temp_plot_widget.setLabel("bottom", "시간 (초)", color="k")
            self.temp_plot_widget.showGrid(x=True, y=True, alpha=0.3)

            # 온도 Y축 범위 설정 (20°C ~ 80°C)
            self.temp_plot_widget.setYRange(20, 100, padding=0.05)

            # 사용자 확대/축소 활성화 (자동 범위 조정은 유지)
            self.temp_plot_widget.setMouseEnabled(x=True, y=True)

            # 온도 한계선 추가 (80°C)
            limit_line = pg.InfiniteLine(
                pos=self.safety_temp_threshold,  # 80°C
                angle=0,  # 수평선
                pen=pg.mkPen(color=(255, 0, 0), width=1.5, style=Qt.DashLine),
                label=f"안전 한계: {self.safety_temp_threshold}°C",
                labelOpts={'color': (255, 0, 0), 'position': 0.95}
            )
            self.temp_plot_widget.addItem(limit_line)

            # 온도 그래프 데이터 라인 생성
            self.temp_line = self.temp_plot_widget.plot(
                pen=pg.mkPen(color=(255, 0, 0), width=2),
                name="현재 온도",
                symbol='o',  # 원형 마커
                symbolSize=4,  # 마커 크기 증가
                symbolBrush=(255, 0, 0)  # 마커 색상
            )
            self.target_temp_line = self.temp_plot_widget.plot(
                pen=pg.mkPen(color=(0, 0, 255), width=2, style=Qt.DashLine),
                name="목표 온도"
            )

            # 범례 추가
            self.temp_legend = pg.LegendItem(offset=(30, 20))
            self.temp_legend.setParentItem(self.temp_plot_widget.graphicsItem())
            self.temp_legend.addItem(self.temp_line, "현재 온도")
            self.temp_legend.addItem(self.target_temp_line, "목표 온도")

            # PWM 그래프 설정
            self.pwm_plot_widget = pg.PlotWidget()
            self.pwm_plot_widget.setBackground('w')
            self.pwm_plot_widget.setTitle("PWM 그래프", color="k", size="12pt")
            self.pwm_plot_widget.setLabel("left", "PWM 값", color="k")
            self.pwm_plot_widget.setLabel("bottom", "시간 (초)", color="k")
            self.pwm_plot_widget.showGrid(x=True, y=True, alpha=0.3)

            # PWM Y축 범위 설정 (0-100)
            self.pwm_plot_widget.setYRange(0, 100, padding=0.05)

            # 사용자 확대/축소 활성화
            self.pwm_plot_widget.setMouseEnabled(x=True, y=True)

            # PWM 그래프 데이터 라인 생성
            self.pwm_line = self.pwm_plot_widget.plot(
                pen=pg.mkPen(color=(0, 128, 0), width=2),
                name="PWM 값",
                symbol='o',  # 원형 마커
                symbolSize=4,  # 마커 크기 증가
                symbolBrush=(0, 128, 0)  # 마커 색상
            )

            # X축 연결 (두 그래프의 시간축을 동기화)
            self.temp_plot_widget.setXLink(self.pwm_plot_widget)

            # 기존 레이아웃 제거 (이미 레이아웃이 있을 경우)
            if self._widget.temp_graph_frame.layout() is not None:
                old_layout = self._widget.temp_graph_frame.layout()
                while old_layout.count():
                    item = old_layout.takeAt(0)
                    widget = item.widget()
                    if widget is not None:
                        widget.deleteLater()
                # Qt는 레이아웃을 직접 삭제하지 않으므로 빈 위젯으로 대체
                QWidget().setLayout(old_layout)

            if self._widget.pwm_graph_frame.layout() is not None:
                old_layout = self._widget.pwm_graph_frame.layout()
                while old_layout.count():
                    item = old_layout.takeAt(0)
                    widget = item.widget()
                    if widget is not None:
                        widget.deleteLater()
                QWidget().setLayout(old_layout)

            # 그래프 프레임에 위젯 추가
            temp_layout = QVBoxLayout()
            temp_layout.addWidget(self.temp_plot_widget)
            self._widget.temp_graph_frame.setLayout(temp_layout)

            pwm_layout = QVBoxLayout()
            pwm_layout.addWidget(self.pwm_plot_widget)
            self._widget.pwm_graph_frame.setLayout(pwm_layout)



            # 그래프 자동 범위 설정
            #self.temp_plot_widget.autoRange()
            #self.pwm_plot_widget.autoRange()

            self.node.get_logger().info("그래프 초기화 완료")
        except Exception as e:
            import traceback
            self.node.get_logger().error(f"그래프 초기화 오류: {str(e)}")
            self.node.get_logger().error(traceback.format_exc())
            # UI에 오류 메시지 표시
            error_layout = QVBoxLayout()
            error_label = QLabel(f"그래프 초기화 오류: {str(e)}")
            error_label.setStyleSheet("color: red; font-weight: bold;")
            error_layout.addWidget(error_label)
            self._widget.temp_graph_frame.setLayout(error_layout)
            self._widget.pwm_graph_frame.setLayout(QVBoxLayout())

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
            self._widget.auto_settings_group.setEnabled(auto_mode)
            self._widget.manual_settings_group.setEnabled(not auto_mode)

    def shutdown_plugin(self):
        """플러그인이 종료될 때 호출되는 메서드"""

        self._shutdown_flag = True

        # Qt 타이머 중지
        if hasattr(self, 'qt_timer') and self.qt_timer.isActive():
            self.qt_timer.stop()

        # 그래프 타이머 중지
        if hasattr(self, 'graph_timer') and self.graph_timer.isActive():
            self.graph_timer.stop()

        # 서비스 검사 타이머 중지
        if hasattr(self, 'service_check_timer') and self.service_check_timer.isActive():
            self.service_check_timer.stop()

        # 비상 정지 해제 (안전을 위해)
        if hasattr(self, 'emergency_pub'):
            msg = Bool()
            msg.data = False
            self.emergency_pub.publish(msg)

        # 스핀 스레드 종료 대기
        if hasattr(self, 'spin_thread') and self.spin_thread.is_alive():
            self.spin_thread.join(timeout=1.0)  # 최대 1초 대기

        # ROS 노드 정리
        if hasattr(self, 'node') and self.node:
            self.node.get_logger().info("플러그인 종료 중...")
            self.node.destroy_node()

        print("[INFO]: 플러그인 자원이 정상적으로 해제되었습니다")

    def spin_ros(self):
        """ROS 2 노드 스핀을 위한 메서드"""
        try:
            # SingleThreadedExecutor 사용
            from rclpy.executors import SingleThreadedExecutor
            executor = SingleThreadedExecutor()
            executor.add_node(self.node)

            # executor 스핀
            while rclpy.ok() and not self._shutdown_flag:
                executor.spin_once(timeout_sec=0.1)  # 100ms 타임아웃으로 반응성 유지
        except Exception as e:
            if hasattr(self, 'node') and self.node:
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

            # 목표 온도 설정 - 자동 모드일 때만
            if self.auto_mode:
                target_temp = self._widget.target_temp_spin.value()
            else:
                target_temp = 0.0 # 수동 모드일 때는 목표 온도 없음

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

            # 그래프 버튼 상태 업데이트
            self._widget.graph_start_button.setEnabled(not self.is_plotting)
            self._widget.graph_stop_button.setEnabled(self.is_plotting)

            # 로깅 상태 업데이트
            if self.is_plotting:
                elapsed_time = 0
                if self.start_time:
                    elapsed_time = (datetime.datetime.now() - self.start_time).total_seconds()

                # 이미 로깅 상태 문자열이 있는지 확인하고, 없으면 기본 메시지 표시
                status_text = self._widget.log_status_label.text()
                if "로깅 상태: 활성화" not in status_text:
                    self._widget.log_status_label.setText(f"로깅 상태: 활성화 (경과 시간: {int(elapsed_time)}초)")
                else:
                    # 기존 텍스트에서 시간 부분만 업데이트
                    time_text = f"(경과 시간: {int(elapsed_time)}초)"
                    if " (" in status_text:
                        parts = status_text.split(" (")
                        self._widget.log_status_label.setText(f"{parts[0]} {time_text}")
                    else:
                        self._widget.log_status_label.setText(f"{status_text} {time_text}")

            # 저장 버튼 텍스트 업데이트
            if self.is_logger_active:
                self._widget.save_data_button.setText("로깅 중지")
            else:
                self._widget.save_data_button.setText("데이터 저장")

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

                # 그래프 데이터 수집 (그래프 활성화 상태일 때만)
                if self.is_plotting and hasattr(self, 'start_time') and self.start_time is not None:
                    current_time = datetime.datetime.now()
                    elapsed_time = (current_time - self.start_time).total_seconds()

                    # 데이터 락 획득 후 배열 수정
                    with self.data_lock:
                        self.time_data.append(elapsed_time)
                        self.temp_data.append(self.temperature)

                        # 목표 온도 (자동 모드일 때는 설정값, 수동 모드일 때는 0)
                        if self.auto_mode:
                            target_temp = self._widget.target_temp_spin.value()
                        else:
                            target_temp = 0.0
                        self.target_temp_data.append(target_temp)

                        # PWM 값
                        self.pwm_data.append(self.current_pwm)

                        # 버퍼 크기 제한 (5분 = 300초 기준, 하지만 데이터 포인트가 많아질 수 있음)
                        # 시간 기준으로 오래된 데이터 제거
                        max_age = 300.0  # 5분
                        while self.time_data and (elapsed_time - self.time_data[0]) > max_age:
                            self.time_data.pop(0)
                            self.temp_data.pop(0)
                            self.target_temp_data.pop(0)
                            self.pwm_data.pop(0)

                    # 로그 출력
                    if len(self.time_data) % 1000 == 0:  # 500개마다 로그 출력
                        self.node.get_logger().info(f"데이터 개수: {len(self.time_data)}, 시간 범위: {elapsed_time - self.time_data[0]:.1f}초")
        except Exception as e:
            import traceback
            self.node.get_logger().error(f'온도 데이터 처리 오류: {str(e)}')
            self.node.get_logger().error(traceback.format_exc())

    def update_graphs(self):
        """
        그래프 데이터 업데이트 (그래프 타이머에서 호출)
        """
        if not self.is_plotting or not PYQTGRAPH_AVAILABLE:
            return

        try:
            # 데이터가 있을 경우에만 그래프 업데이트
            if hasattr(self, 'temp_line') and hasattr(self, 'pwm_line'):
                # 락 획득 후 데이터 복사 (스레드 안전)
                with self.data_lock:
                    if not self.time_data or len(self.time_data) < 2:
                        return  # 데이터가 없거나 불충분하면 업데이트 중단

                    # 데이터 안전하게 복사
                    time_data = list(self.time_data)
                    temp_data = list(self.temp_data)
                    target_temp_data = list(self.target_temp_data)
                    pwm_data = list(self.pwm_data)

                # 일관성 검사 (추가 보호)
                min_length = min(len(time_data), len(temp_data), len(target_temp_data), len(pwm_data))
                if min_length < 2:
                    return  # 충분한 데이터가 없으면 업데이트 중단

                # 모든 배열을 같은 길이로 자름
                time_data = time_data[:min_length]
                temp_data = temp_data[:min_length]
                target_temp_data = target_temp_data[:min_length]
                pwm_data = pwm_data[:min_length]

                # 시간 창 계산 (지정된 시간 창만 표시)
                window_size = 60.0  # 60초 창
                latest_time = time_data[-1]
                start_time = max(0, latest_time - window_size)

                # 시간 범위로 필터링
                visible_data = [(i, t, v1, v2, v3) for i, (t, v1, v2, v3) in
                                enumerate(zip(time_data, temp_data, target_temp_data, pwm_data))
                                if t >= start_time]

                if not visible_data:
                    return

                # 인덱스, 시간, 온도, 타겟 온도, PWM으로 분리
                indices, display_time, display_temp, display_target, display_pwm = zip(*visible_data)

                # 다운샘플링 로직 - 최대 400개 포인트로 제한
                target_points = 400

                if len(display_time) > target_points:
                    # 다운샘플링 인덱스 계산
                    indices = list(indices)  # 튜플을 리스트로 변환

                    # 다운샘플링 방법: 시간 기준 균등 분할
                    # 시작과 끝 포인트는 항상 포함
                    # 중간 부분은 균등하게 분포
                    if len(indices) >= 2:
                        # 첫 포인트와 마지막 포인트
                        sampled_indices = [indices[0]]

                        # 중간 포인트들 (균등 간격)
                        step = (len(indices) - 2) / (target_points - 2)
                        for i in range(1, target_points - 1):
                            idx = indices[min(int(i * step) + 1, len(indices) - 1)]
                            sampled_indices.append(idx)

                        # 마지막 포인트 추가
                        sampled_indices.append(indices[-1])

                        # 중복 제거 및 정렬
                        sampled_indices = sorted(set(sampled_indices))

                        # 샘플링된 데이터로 새로운 데이터셋 생성
                        display_time = [time_data[i] for i in sampled_indices]
                        display_temp = [temp_data[i] for i in sampled_indices]
                        display_target = [target_temp_data[i] for i in sampled_indices]
                        display_pwm = [pwm_data[i] for i in sampled_indices]

                # 그래프 업데이트
                self.temp_line.setData(display_time, display_temp)
                self.target_temp_line.setData(display_time, display_target)
                self.pwm_line.setData(display_time, display_pwm)

                # X축 범위 고정 (60초 창)
                self.temp_plot_widget.setXRange(start_time, latest_time, padding=0.02)
                self.pwm_plot_widget.setXRange(start_time, latest_time, padding=0.02)

                # 자동 범위 조정 비활성화
                self.temp_plot_widget.getViewBox().setAutoVisible(y=True, x=False)
                self.pwm_plot_widget.getViewBox().setAutoVisible(y=True, x=False)

                # 상태 업데이트
                if hasattr(self, '_widget'):
                    self._widget.status_label.setText(f"데이터 플로팅 중: {len(display_time)}포인트 ({len(time_data)}개 중)")

        except Exception as e:
            import traceback
            self.node.get_logger().error(f"그래프 업데이트 오류: {str(e)}")
            self.node.get_logger().error(traceback.format_exc())

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

    def start_plotting(self):
        """
        그래프 플로팅 시작
        """
        if self.is_plotting or not PYQTGRAPH_AVAILABLE:
            return

        # 그래프 데이터 초기화
        self.reset_graph_data()

        # 플로팅 상태 활성화
        self.is_plotting = True
        self.start_time = datetime.datetime.now()

        # 그래프 타이머 시작
        self.graph_timer.start()

        # UI 상태 업데이트
        self._widget.graph_start_button.setEnabled(False)
        self._widget.graph_stop_button.setEnabled(True)
        self._widget.status_label.setText("그래프 플로팅이 시작되었습니다")
        self.node.get_logger().info("그래프 플로팅이 시작되었습니다")

        # 초기 데이터 추가
        with self.data_lock:
            self.time_data = [0.0]
            self.temp_data = [self.temperature]

            if self.auto_mode:
                target_temp = self._widget.target_temp_spin.value()
            else:
                target_temp = 0.0
            self.target_temp_data = [target_temp]
            self.pwm_data = [self.current_pwm]

    def stop_plotting(self):
        """
        그래프 플로팅 중지
        """
        if not self.is_plotting:
            return

        # 플로팅 상태 비활성화
        self.is_plotting = False

        # 그래프 타이머 중지
        self.graph_timer.stop()

        # UI 상태 업데이트
        self._widget.graph_start_button.setEnabled(True)
        self._widget.graph_stop_button.setEnabled(False)
        self._widget.status_label.setText("그래프 플로팅이 중지되었습니다")
        self.node.get_logger().info("그래프 플로팅이 중지되었습니다")

    def reset_graph_data(self):
        """
        그래프 데이터 초기화
        """
        # 데이터 배열 초기화
        self.time_data = []
        self.temp_data = []
        self.target_temp_data = []
        self.pwm_data = []

        # 시작 시간 재설정
        self.start_time = datetime.datetime.now()

        # 그래프 초기화
        if hasattr(self, 'temp_line') and hasattr(self, 'target_temp_line') and hasattr(self, 'pwm_line'):
            self.temp_line.setData([], [])
            self.target_temp_line.setData([], [])
            self.pwm_line.setData([], [])

            # 그래프 범위 초기화
            if hasattr(self, 'temp_plot_widget') and hasattr(self, 'pwm_plot_widget'):
                self.temp_plot_widget.setXRange(0, 300)
                self.pwm_plot_widget.setXRange(0, 300)

                # 빠른 초기화를 위한 더미 데이터 추가
                init_time = list(range(5))
                init_temp = [30, 35, 40, 38, 36]
                init_target = [50] * 5
                init_pwm = [10, 30, 50, 40, 20]

                self.temp_line.setData(init_time, init_temp)
                self.target_temp_line.setData(init_time, init_target)
                self.pwm_line.setData(init_time, init_pwm)

        self._widget.status_label.setText("그래프 데이터가 초기화되었습니다")
        self.node.get_logger().info("그래프 데이터가 초기화되었습니다")

    def save_data_to_file(self, custom_filename=None):
        """
        수집된 데이터를 CSV 파일로 저장
        """
        try:
            # 데이터가 없으면 저장하지 않음
            if not self.time_data:
                self._widget.status_label.setText("저장할 데이터가 없습니다")
                self.node.get_logger().warn("저장할 데이터가 없습니다")
                return False, "저장할 데이터가 없습니다"

            # 파일 이름 생성 (사용자 입력 + 날짜시간)
            base_filename = self._widget.filename_edit.text() if not custom_filename else custom_filename
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"{base_filename}_{timestamp}.csv"
            filepath = os.path.join(self.data_logging_path, filename)

            # CSV 파일 작성
            with open(filepath, 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)

                # 헤더 작성
                csv_writer.writerow(["시간(초)", "온도(°C)", "목표온도(°C)", "PWM"])

                # 데이터 작성
                for i in range(len(self.time_data)):
                    csv_writer.writerow([
                        f"{self.time_data[i]:.2f}",
                        f"{self.temp_data[i]:.2f}",
                        f"{self.target_temp_data[i]:.2f}",
                        self.pwm_data[i]
                    ])

            self._widget.status_label.setText(f"데이터가 저장되었습니다: {filename}")
            self.node.get_logger().info(f"데이터가 저장되었습니다: {filepath}")

            # 성공 메시지 표시
            QMessageBox.information(self._widget, "저장 완료",
                                   f"데이터가 성공적으로 저장되었습니다.\n파일 위치: {filepath}")
            return True, filepath

        except Exception as e:
            error_msg = f"데이터 저장 중 오류 발생: {str(e)}"
            self._widget.status_label.setText(error_msg)
            self.node.get_logger().error(f"데이터 저장 오류: {str(e)}")

            # 오류 메시지 표시
            QMessageBox.critical(self._widget, "저장 오류", f"데이터 저장 중 오류가 발생했습니다.\n{str(e)}")
            return False, error_msg

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
        if not checked:
            return

        if self._widget.auto_mode_radio.isChecked():
            self.auto_mode = True
            self._widget.auto_settings_group.setEnabled(True)
            self._widget.manual_settings_group.setEnabled(False)
            self._widget.status_label.setText('자동 모드 활성화')
            self._widget.status_label.setStyleSheet("")

            # 자동 모드로 전환 시 현재 설정된 파라미터 즉시 발행
            self.publish_control_parameters()
            self.node.get_logger().info('자동 제어 모드로 전환합니다')
            self.publish_control_mode(True)
        else:
            self.auto_mode = False
            self._widget.auto_settings_group.setEnabled(False)
            self._widget.manual_settings_group.setEnabled(True)
            self._widget.status_label.setText('수동 모드 활성화')
            self._widget.status_label.setStyleSheet("")

            self.node.get_logger().info('수동 제어 모드로 전환합니다')
            self.publish_control_mode(False)

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

    def request_data_save(self):
        """
        data_logger에 데이터 저장 요청하거나, 이미 로깅 중이면 중지 요청
        """
        # 로거 노드가 없는 경우 자체적으로 파일 저장
        if not self.is_data_logger_available:
            self.node.get_logger().warn("데이터 로거 서비스가 사용 불가능합니다. 내부 저장 기능 사용")
            self._widget.status_label.setText("데이터 로거 서비스 연결 실패. 내부 저장 기능 사용 중...")

            # 파일명 설정
            custom_filename = self._widget.filename_edit.text()
            success, result = self.save_data_to_file(custom_filename)

            if success:
                self._widget.status_label.setText(f"내부 저장 성공: {os.path.basename(result)}")
            else:
                self._widget.status_label.setText(f"내부 저장 실패: {result}")

            return

        try:
            # 로깅 상태에 따라 시작/중지 결정
            if self.is_logger_active:
                # 로깅 중지 요청
                request = SetBool.Request()
                request.data = False
                self._widget.status_label.setText("로깅 중지 요청 중...")
                self.node.get_logger().info("로깅 중지 요청")
            else:
                # 로깅 시작 요청
                # 사용자 파일명 발행
                filename_msg = String()
                filename_msg.data = self._widget.filename_edit.text()
                self.filename_publisher.publish(filename_msg)
                self.node.get_logger().info(f"파일명 설정: {filename_msg.data}")

                # 로깅 시작 요청
                request = SetBool.Request()
                request.data = True
                self._widget.status_label.setText("로깅 시작 요청 중...")
                self.node.get_logger().info("로깅 시작 요청")

            # 비동기 서비스 호출
            future = self.save_logger_client.call_async(request)
            future.add_done_callback(self.handle_save_response)

        except Exception as e:
            error_msg = f"데이터 저장 요청 오류: {str(e)}"
            self._widget.status_label.setText(error_msg)
            self.node.get_logger().error(error_msg)

            # 오류 메시지 표시
            QMessageBox.critical(self._widget, "서비스 오류", f"데이터 저장 요청 중 오류가 발생했습니다.\n{str(e)}")

    def reset_data_logging(self):
        """data_logger에 로깅 초기화 요청"""
        if not self.is_data_logger_available:
            self.node.get_logger().warn("데이터 로거 서비스가 사용 불가능합니다")
            self._widget.status_label.setText("데이터 로거 서비스 연결 실패. 내부 그래프만 초기화합니다.")
            self.reset_graph_data()
            return

        try:
            request = Trigger.Request()
            future = self.reset_logger_client.call_async(request)
            future.add_done_callback(self.handle_reset_response)

            self._widget.status_label.setText("로그 데이터 초기화 요청 중...")
            self.node.get_logger().info("로그 데이터 초기화 요청")

            # 그래프도 초기화
            self.reset_graph_data()
        except Exception as e:
            error_msg = f"로그 초기화 요청 오류: {str(e)}"
            self._widget.status_label.setText(error_msg)
            self.node.get_logger().error(error_msg)

            # 오류 메시지 표시
            QMessageBox.critical(self._widget, "서비스 오류", f"데이터 초기화 요청 중 오류가 발생했습니다.\n{str(e)}")

    def handle_save_response(self, future):
        """
        저장 서비스 응답 처리
        """
        try:
            response = future.result()
            if response.success:
                self._widget.status_label.setText(f"데이터 저장/로깅 요청 성공: {response.message}")
                self.node.get_logger().info(f"데이터 저장/로깅 요청 성공: {response.message}")

                # 로깅 시작시 그래프도 시작
                if "시작" in response.message and not self.is_plotting:
                    self.start_plotting()
            else:
                self._widget.status_label.setText(f"데이터 저장/로깅 요청 실패: {response.message}")
                self.node.get_logger().warn(f"데이터 저장/로깅 요청 실패: {response.message}")

                # 실패 메시지 표시
                QMessageBox.warning(self._widget, "요청 실패", f"데이터 저장/로깅 요청이 실패했습니다.\n{response.message}")
        except Exception as e:
            error_msg = f"저장 응답 처리 오류: {str(e)}"
            self._widget.status_label.setText(error_msg)
            self.node.get_logger().error(error_msg)

            # 오류 메시지 표시
            QMessageBox.critical(self._widget, "응답 처리 오류", f"서비스 응답 처리 중 오류가 발생했습니다.\n{str(e)}")

    def handle_reset_response(self, future):
        """
        초기화 서비스 응답 처리
        """
        try:
            response = future.result()
            if response.success:
                self._widget.status_label.setText(f"데이터 초기화 성공: {response.message}")
                self.node.get_logger().info(f"데이터 초기화 성공: {response.message}")
            else:
                self._widget.status_label.setText(f"데이터 초기화 실패: {response.message}")
                self.node.get_logger().warn(f"데이터 초기화 실패: {response.message}")

                # 실패 메시지 표시
                QMessageBox.warning(self._widget, "초기화 실패", f"데이터 초기화가 실패했습니다.\n{response.message}")
        except Exception as e:
            error_msg = f"초기화 응답 처리 오류: {str(e)}"
            self._widget.status_label.setText(error_msg)
            self.node.get_logger().error(error_msg)

            # 오류 메시지 표시
            QMessageBox.critical(self._widget, "응답 처리 오류", f"초기화 응답 처리 중 오류가 발생했습니다.\n{str(e)}")

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
