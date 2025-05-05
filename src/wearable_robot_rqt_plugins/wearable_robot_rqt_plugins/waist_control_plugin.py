#!/usr/bin/env python3

import os
import csv
import threading
import subprocess
import signal
import datetime
import rclpy
import sys
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from wearable_robot_interfaces.msg import ActuatorCommand, TemperatureData, DisplacementData, FanCommand, BackIntention
from std_msgs.msg import Float64
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QMessageBox, QVBoxLayout, QLabel
from python_qt_binding.QtCore import Qt, QTimer
import pyqtgraph as pg
import numpy as np
from collections import deque

class WaistControlPlugin(Plugin):
    """
    웨어러블 로봇의 구동기 제어 및 데이터 모니터링을 위한 rqt 플러그인
    고속(125Hz) 데이터 수집 및 로깅 시스템 구현
    """
    def __init__(self, context):
        super(WaistControlPlugin, self).__init__(context)
        # 플러그인 제목 설정
        self.setObjectName('WaistControlPlugin')

        # 그래프 플로팅 관련 변수 초기화
        self.is_plotting = False
        self.buffer_size = 300 * 125  # 300초 * 125Hz = 37500 샘플 저장 가능
        self.time_data = deque(maxlen=self.buffer_size)  # 더 효율적인 데이터 관리를 위해 deque 사용
        self.temp_data_4 = deque(maxlen=self.buffer_size)
        self.temp_data_5 = deque(maxlen=self.buffer_size)
        self.target_temp_data = deque(maxlen=self.buffer_size)
        self.pwm_data_4 = deque(maxlen=self.buffer_size)
        self.pwm_data_5 = deque(maxlen=self.buffer_size)
        self.displacement_data = deque(maxlen=self.buffer_size)
        self.start_time = None

        # 데이터 수집 설정
        self.sample_interval = 1.0/125.0  # 125Hz = 0.008초
        self.data_collection_active = False
        self.collection_thread = None

        # 내부 상태 변수 초기화
        self.temperature_4 = 0.0           # 구동기 4번 온도
        self.temperature_5 = 0.0           # 구동기 5번 온도
        self.current_pwm_4 = 0             # 구동기 4번 PWM 값
        self.current_pwm_5 = 0             # 구동기 5번 PWM 값
        self.fan_state_4 = False           # 구동기 4번 팬 상태
        self.fan_state_5 = False           # 구동기 5번 팬 상태
        self.displacement = 0.0            # 허리 변위 센서값
        self.is_emergency_stop = False     # 비상 정지 상태
        self.safety_temp_threshold = 80.0  # 안전 온도 임계값 (°C)
        self.intention_active = False      # 의도 활성화 상태

        # 성능 모니터링
        self.data_points_collected = 0     # 수집된 데이터 포인트 수
        self.collection_start_time = None  # 수집 시작 시간
        self.last_collection_time = 0      # 마지막 수집 시간
        self.collection_intervals = deque(maxlen=1000)  # 수집 간격 기록
        self.actual_collection_rate = 0.0  # 실제 수집 속도

        # 그래프 관련 객체 초기화
        self.temp_plot_widget = None
        self.pwm_plot_widget = None
        self.displacement_plot_widget = None
        self.temp_line_4 = None
        self.temp_line_5 = None
        self.target_temp_line = None
        self.pwm_line_4 = None
        self.pwm_line_5 = None
        self.displacement_line = None
        self.temp_legend = None

        # 데이터 로깅 관련 변수
        self.is_recording = False
        self.csv_file = None
        self.csv_writer = None
        self.csv_filename = ""
        self.csv_lock = threading.Lock()
        self.csv_start_time = None
        self.data_buffer = []
        self.data_points_count = 0
        self.buffer_size_limit = 1000      # 버퍼에 1000개 데이터 쌓이면 저장
        self.logging_thread = None
        self.logging_thread_active = False

        # ROS 노드 초기화
        try:
            if not rclpy.ok():
                rclpy.init()

            node_name = "waist_control_plugin_internal"
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
        self.target_temp_pub = self.node.create_publisher(
            Float64, 'target_temperature_setting', qos_profile)  # 타겟 온도

        self.threshold_pub = self.node.create_publisher(
            Float64, 'angle_threshold_setting', qos_profile)  # 각도 임계값

        self.intention_pub = self.node.create_publisher(
            BackIntention, 'intention_data', qos_profile)  # 의도 시뮬레이션

        # 구독자 생성
        self.temp_sub = self.node.create_subscription(
            TemperatureData, 'temperature_data', self.temp_callback, qos_profile)  # 온도 데이터

        self.pwm_sub = self.node.create_subscription(
            ActuatorCommand, 'pwm_state', self.pwm_callback, qos_profile)  # PWM 상태

        self.fan_sub = self.node.create_subscription(
            FanCommand, 'fan_state', self.fan_callback, qos_profile)  # 팬 상태

        self.displacement_sub = self.node.create_subscription(
            DisplacementData, 'displacement_data', self.displacement_callback, qos_profile)  # 변위 데이터

        self._shutdown_flag = False

        # ROS 2 스핀을 위한 스레드 생성
        self.spin_thread = threading.Thread(target=self.spin_ros)
        self.spin_thread.daemon = True
        self.spin_thread.start()

        # 스레드 안전 락
        self.data_lock = threading.RLock()

        # UI 설정
        self._widget = QWidget()
        ui_file = self.find_ui_file('waist_control.ui')

        if ui_file is None:
            raise FileNotFoundError("waist_control.ui 파일을 찾을 수 없습니다.")

        loadUi(ui_file, self._widget)
        self._widget.setObjectName('WaistControlPluginUI')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        # UI 이벤트 연결
        self._widget.apply_button.clicked.connect(self.apply_settings)
        self._widget.emergency_stop_button.clicked.connect(self.emergency_stop)
        self._widget.intention_button.clicked.connect(self.toggle_intention)

        # 그래프 UI 설정 및 초기화
        self.setup_graphs()

        # 그래프 제어 버튼 연결
        self._widget.graph_start_button.clicked.connect(self.start_plotting)
        self._widget.graph_stop_button.clicked.connect(self.stop_plotting)
        self._widget.reset_graph_button.clicked.connect(self.reset_graph_data)

        # 데이터 저장 버튼 연결
        self._widget.save_data_button.clicked.connect(self.request_data_save)

        # Qt 타이머 추가 (UI 업데이트용 - 10Hz)
        self.qt_timer = QTimer()
        self.qt_timer.timeout.connect(self.update_ui)
        self.qt_timer.start(100)  # 100ms 마다 UI 업데이트

        # 그래프 업데이트 타이머 (10Hz)
        self.graph_timer = QTimer()
        self.graph_timer.timeout.connect(self.update_graphs)
        self.graph_timer.setInterval(100)  # 100ms마다 그래프 업데이트 (10Hz)

        # 초기화 완료 표시
        self._widget.status_label.setText('플러그인이 초기화되었습니다')
        self.node.get_logger().info('웨어러블 로봇 허리 보조 제어 플러그인이 초기화되었습니다 (125Hz 데이터 수집)')

    def find_ui_file(self, filename='waist_control.ui'):
        # UI 파일 경로를 찾기 위한 여러 가능한 위치 시도
        candidate_paths = [
            os.path.join(os.path.dirname(os.path.realpath(__file__)), 'resource', filename),
            os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'resource', filename),
            os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), 'share',
                        'wearable_robot_rqt_plugins', 'resource', filename),
            os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), 'resource', filename),
            os.path.expanduser(f"~/wearable_robot_ws/src/wearable_robot_rqt_plugins/resource/{filename}")
        ]

        # 각 경로를 시도하여 존재하는 첫 번째 경로 사용
        for path in candidate_paths:
            if os.path.exists(path):
                self.node.get_logger().info(f"UI 파일을 찾았습니다: {path}")
                return path
        return None

    def spin_ros(self):
        """ROS 2 노드 스핀을 위한 메서드"""
        try:
            from rclpy.executors import SingleThreadedExecutor
            executor = SingleThreadedExecutor()
            executor.add_node(self.node)

            while rclpy.ok() and not self._shutdown_flag:
                executor.spin_once(timeout_sec=0.001)  # 1ms 타임아웃으로 더 자주 처리
        except Exception as e:
            if hasattr(self, 'node') and self.node:
                self.node.get_logger().error(f"ROS 스핀 스레드 오류: {str(e)}")
        finally:
            if hasattr(self, 'node') and self.node:
                self.node.get_logger().info("스핀 스레드가 종료됩니다")

    def setup_graphs(self):
        """그래프 위젯 초기화 및 설정"""
        try:
            # PyQtGraph 기본 설정
            pg.setConfigOptions(antialias=True, useOpenGL=False)  # OpenGL 비활성화로 호환성 향상

            # 온도 그래프 설정
            self.temp_plot_widget = pg.PlotWidget()
            self.temp_plot_widget.setBackground('w')
            self.temp_plot_widget.setTitle("온도 그래프", color="k", size="12pt")
            self.temp_plot_widget.setLabel("left", "온도 (°C)", color="k")
            self.temp_plot_widget.setLabel("bottom", "시간 (초)", color="k")
            self.temp_plot_widget.showGrid(x=True, y=True, alpha=0.3)
            self.temp_plot_widget.setYRange(20, 100, padding=0.05)
            self.temp_plot_widget.setMouseEnabled(x=True, y=True)

            # 온도 한계선 추가 (80°C)
            limit_line = pg.InfiniteLine(
                pos=self.safety_temp_threshold,
                angle=0,
                pen=pg.mkPen(color=(255, 0, 0), width=1.5, style=Qt.DashLine),
                label=f"안전 한계: {self.safety_temp_threshold}°C",
                labelOpts={'color': (255, 0, 0), 'position': 0.95}
            )
            self.temp_plot_widget.addItem(limit_line)

            # 온도 그래프 데이터 라인 생성
            self.temp_line_4 = self.temp_plot_widget.plot(
                pen=pg.mkPen(color=(255, 0, 0), width=2),
                name="구동기 4번 온도",
                symbol=None  # 125Hz에서는 심볼 없이 라인만 표시 (성능 향상)
            )
            self.temp_line_5 = self.temp_plot_widget.plot(
                pen=pg.mkPen(color=(0, 0, 255), width=2),
                name="구동기 5번 온도",
                symbol=None
            )
            self.target_temp_line = self.temp_plot_widget.plot(
                pen=pg.mkPen(color=(0, 128, 0), width=2, style=Qt.DashLine),
                name="목표 온도"
            )

            # 범례 추가
            self.temp_legend = pg.LegendItem(offset=(30, 20))
            self.temp_legend.setParentItem(self.temp_plot_widget.graphicsItem())
            self.temp_legend.addItem(self.temp_line_4, "구동기 4번 온도")
            self.temp_legend.addItem(self.temp_line_5, "구동기 5번 온도")
            self.temp_legend.addItem(self.target_temp_line, "목표 온도")

            # PWM 그래프 설정
            self.pwm_plot_widget = pg.PlotWidget()
            self.pwm_plot_widget.setBackground('w')
            self.pwm_plot_widget.setTitle("PWM 그래프", color="k", size="12pt")
            self.pwm_plot_widget.setLabel("left", "PWM 값", color="k")
            self.pwm_plot_widget.setLabel("bottom", "시간 (초)", color="k")
            self.pwm_plot_widget.showGrid(x=True, y=True, alpha=0.3)
            self.pwm_plot_widget.setYRange(0, 100, padding=0.05)
            self.pwm_plot_widget.setMouseEnabled(x=True, y=True)

            # PWM 그래프 데이터 라인 생성
            self.pwm_line_4 = self.pwm_plot_widget.plot(
                pen=pg.mkPen(color=(255, 0, 0), width=2),
                name="구동기 4번 PWM",
                symbol=None
            )
            self.pwm_line_5 = self.pwm_plot_widget.plot(
                pen=pg.mkPen(color=(0, 0, 255), width=2),
                name="구동기 5번 PWM",
                symbol=None
            )

            # PWM 그래프 범례 추가
            self.pwm_legend = pg.LegendItem(offset=(30, 20))
            self.pwm_legend.setParentItem(self.pwm_plot_widget.graphicsItem())
            self.pwm_legend.addItem(self.pwm_line_4, "구동기 4번 PWM")
            self.pwm_legend.addItem(self.pwm_line_5, "구동기 5번 PWM")

            # 변위 센서 그래프 설정
            self.displacement_plot_widget = pg.PlotWidget()
            self.displacement_plot_widget.setBackground('w')
            self.displacement_plot_widget.setTitle("허리 변위 그래프", color="k", size="12pt")
            self.displacement_plot_widget.setLabel("left", "변위 값", color="k")
            self.displacement_plot_widget.setLabel("bottom", "시간 (초)", color="k")
            self.displacement_plot_widget.showGrid(x=True, y=True, alpha=0.3)
            self.displacement_plot_widget.setYRange(0, 5, padding=0.05)
            self.displacement_plot_widget.setMouseEnabled(x=True, y=True)

            # 변위 센서 그래프 데이터 라인 생성
            self.displacement_line = self.displacement_plot_widget.plot(
                pen=pg.mkPen(color=(0, 128, 0), width=2),
                name="허리 변위",
                symbol=None
            )

            # 임계값 라인 추가
            threshold_value = self._widget.angle_threshold_spin.value()
            self.threshold_line = pg.InfiniteLine(
                pos=threshold_value,
                angle=0,
                pen=pg.mkPen(color=(255, 0, 0), width=1.5, style=Qt.DashLine),
                label=f"임계값: {threshold_value}V",
                labelOpts={'color': (255, 0, 0), 'position': 0.95}
            )
            self.displacement_plot_widget.addItem(self.threshold_line)

            # X축 연결 (세 그래프의 시간축을 동기화)
            self.temp_plot_widget.setXLink(self.pwm_plot_widget)
            self.displacement_plot_widget.setXLink(self.temp_plot_widget)

            # 기존 레이아웃 제거 및 그래프 프레임에 위젯 추가
            self.clear_and_set_layout(self._widget.temp_graph_frame, self.temp_plot_widget)
            self.clear_and_set_layout(self._widget.pwm_graph_frame, self.pwm_plot_widget)
            self.clear_and_set_layout(self._widget.displacement_graph_frame, self.displacement_plot_widget)

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
            self._widget.displacement_graph_frame.setLayout(QVBoxLayout())

    def clear_and_set_layout(self, frame, widget):
        """프레임의 기존 레이아웃을 제거하고 새 위젯 추가"""
        if frame.layout() is not None:
            old_layout = frame.layout()
            while old_layout.count():
                item = old_layout.takeAt(0)
                widget = item.widget()
                if widget is not None:
                    widget.deleteLater()
            # Qt는 레이아웃을 직접 삭제하지 않으므로 빈 위젯으로 대체
            QWidget().setLayout(old_layout)

        new_layout = QVBoxLayout()
        new_layout.addWidget(widget)
        frame.setLayout(new_layout)

    def update_ui(self):
        """UI 업데이트 함수 (Qt 타이머에서 호출)"""
        if not hasattr(self, '_widget') or self._widget is None:
            return

        try:
            # 온도와 PWM 값 업데이트
            self._widget.temp_label_4.setText(f"{self.temperature_4:.1f}°C")
            self._widget.temp_label_5.setText(f"{self.temperature_5:.1f}°C")
            self._widget.pwm_label_4.setText(f"PWM: {self.current_pwm_4}")
            self._widget.pwm_label_5.setText(f"PWM: {self.current_pwm_5}")
            self._widget.fan_label_4.setText(f"팬: {'ON' if self.fan_state_4 else 'OFF'}")
            self._widget.fan_label_5.setText(f"팬: {'ON' if self.fan_state_5 else 'OFF'}")

            # 변위 센서 값 업데이트
            self._widget.displacement_label.setText(f"{self.displacement:.2f}")

            # 의도 상태 업데이트
            self._widget.intention_status_label.setText(f"현재 의도: {'활성화됨' if self.intention_active else '없음'}")

            # 동작 상태 업데이트
            # 변위값이 임계값을 초과하고 의도가 활성화된 경우 "활성화됨" 표시
            threshold = self._widget.angle_threshold_spin.value()
            if self.intention_active and self.displacement > threshold:
                self._widget.activation_status_label.setText("활성화됨")
                self._widget.activation_status_label.setStyleSheet("color: green; font-weight: bold;")
            else:
                self._widget.activation_status_label.setText("비활성화됨")
                self._widget.activation_status_label.setStyleSheet("color: red;")

            # 비상 정지 상태 표시
            if self.is_emergency_stop:
                self._widget.status_label.setText("비상 정지 상태")
                self._widget.status_label.setStyleSheet("color: red; font-weight: bold;")
                self._widget.apply_button.setEnabled(False)
                self._widget.emergency_stop_button.setText("비상 정지 해제")
            else:
                self._widget.apply_button.setEnabled(True)
                self._widget.emergency_stop_button.setText("비상 정지")
                self._widget.status_label.setStyleSheet("")

            # 그래프 버튼 상태 업데이트
            self._widget.graph_start_button.setEnabled(not self.is_plotting)
            self._widget.graph_stop_button.setEnabled(self.is_plotting)

            # 로깅 상태 업데이트
            if self.is_recording:
                self._widget.save_data_button.setText("기록 중지")

                # 데이터 수집 속도 표시
                now = datetime.datetime.now()
                if hasattr(self, 'csv_start_time') and self.csv_start_time is not None:
                    duration = (now - self.csv_start_time).total_seconds()
                    if duration > 0:
                        rate = self.data_points_count / duration
                        self._widget.log_status_label.setText(
                            f"로깅 상태: 활성화 - {os.path.basename(self.csv_filename)} - "
                            f"{self.data_points_count}개 기록 (평균 {rate:.1f}Hz)"
                        )
            else:
                self._widget.save_data_button.setText("데이터 저장")

            # 플로팅 상태면 데이터 포인트 수와 실제 수집 속도 표시
            if self.is_plotting:
                with self.data_lock:
                    num_points = len(self.time_data)

                elapsed_time = 0
                if self.collection_start_time:
                    elapsed_time = (datetime.datetime.now() - self.collection_start_time).total_seconds()

                # 실제 수집 속도 계산
                if len(self.collection_intervals) > 0 and elapsed_time > 0:
                    avg_interval = sum(self.collection_intervals) / len(self.collection_intervals)
                    self.actual_collection_rate = 1.0 / avg_interval if avg_interval > 0 else 0

                    self._widget.status_label.setText(
                        f"데이터 수집 중: {num_points}포인트 (시간: {elapsed_time:.1f}초, 목표: 125Hz, 실제: {self.actual_collection_rate:.1f}Hz)"
                    )

        except RuntimeError as e:
            if "deleted" in str(e):
                # 위젯이 삭제된 경우 타이머 중지
                if hasattr(self, 'qt_timer'):
                    self.qt_timer.stop()

    # 콜백 함수들은 데이터를 저장만 함
    def temp_callback(self, msg):
        """온도 데이터 수신 콜백"""
        try:
            # 구동기 4번과 5번 온도 데이터 저장
            if len(msg.temperature) > 5:
                self.temperature_4 = msg.temperature[4]
                self.temperature_5 = msg.temperature[5]
        except Exception as e:
            self.node.get_logger().error(f'온도 데이터 처리 오류: {str(e)}')

    def pwm_callback(self, msg):
        """현재 PWM 상태 수신 콜백"""
        try:
            # 구동기 4번과 5번 PWM 값 저장
            if len(msg.pwm) > 5:
                self.current_pwm_4 = msg.pwm[4]
                self.current_pwm_5 = msg.pwm[5]
        except Exception as e:
            self.node.get_logger().error(f'PWM 데이터 처리 오류: {str(e)}')

    def fan_callback(self, msg):
        """팬 상태 수신 콜백"""
        try:
            # 구동기 4번과 5번 팬 상태 저장
            if len(msg.fan) > 5:
                self.fan_state_4 = msg.fan[4]
                self.fan_state_5 = msg.fan[5]
        except Exception as e:
            self.node.get_logger().error(f'팬 데이터 처리 오류: {str(e)}')

    def displacement_callback(self, msg):
        """변위 센서 데이터 수신 콜백"""
        try:
            # 변위 센서 데이터 저장 (첫 번째 센서 데이터만 사용)
            if len(msg.displacement) > 0:
                self.displacement = msg.displacement[0]
        except Exception as e:
            self.node.get_logger().error(f'변위 데이터 처리 오류: {str(e)}')

    def high_speed_collection_loop(self):
        """125Hz 속도로 데이터 수집하는 루프"""
        interval = self.sample_interval
        next_time = time.time()
        self.collection_start_time = datetime.datetime.now()
        self.last_collection_time = time.time()
        self.data_points_collected = 0
        self.collection_intervals.clear()

        self.node.get_logger().info(f"고속 데이터 수집 시작 (목표: 125Hz, 간격: {interval*1000:.1f}ms)")

        try:
            while self.data_collection_active and not self._shutdown_flag:
                # 현재 시간 확인
                current_time = time.time()

                # 다음 수집 시간이 됐는지 확인
                if current_time >= next_time:
                    # 실제 수집 간격 측정 및 기록
                    if self.last_collection_time > 0:
                        actual_interval = current_time - self.last_collection_time
                        self.collection_intervals.append(actual_interval)

                    self.last_collection_time = current_time

                    # 데이터 수집
                    self.collect_data_point()

                    # 다음 수집 시간 계산
                    next_time += interval

                    # 만약 너무 늦어진 경우 일부 프레임 건너뛰기
                    if current_time > next_time + interval * 5:  # 5프레임 이상 늦어짐
                        skipped_frames = int((current_time - next_time) / interval)
                        next_time = current_time + interval  # 다음 프레임으로 재설정
                        self.node.get_logger().warn(f"데이터 수집 지연: {skipped_frames}프레임 건너뜀")

                # CPU 점유율 최소화를 위한 짧은 슬립
                # 8ms 간격으로 수집 대기 시간은 0.5ms 이하로 유지
                sleep_time = min(0.0005, max(0, next_time - time.time()))
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except Exception as e:
            import traceback
            self.node.get_logger().error(f"고속 데이터 수집 루프 오류: {str(e)}")
            self.node.get_logger().error(traceback.format_exc())

        finally:
            self.node.get_logger().info("고속 데이터 수집 루프 종료")

            # 실제 수집 속도 계산 및 로깅
            if len(self.collection_intervals) > 0:
                avg_interval = sum(self.collection_intervals) / len(self.collection_intervals)
                actual_rate = 1.0 / avg_interval if avg_interval > 0 else 0
                self.node.get_logger().info(
                    f"수집 통계: {self.data_points_collected}개 데이터, "
                    f"평균 간격: {avg_interval*1000:.2f}ms, 실제 속도: {actual_rate:.2f}Hz"
                )

    def logging_worker(self):
        """별도 스레드에서 데이터 로깅 처리"""
        self.node.get_logger().info("로깅 스레드 시작")

        try:
            while self.logging_thread_active and self.is_recording and not self._shutdown_flag:
                # 버퍼에 데이터가 충분히 쌓이면 파일에 쓰기
                if len(self.data_buffer) >= self.buffer_size_limit:
                    self.flush_buffer()

                # 100ms 대기
                time.sleep(0.1)

            # 종료 시 남은 버퍼 비우기
            if self.data_buffer and self.is_recording:
                self.flush_buffer()

        except Exception as e:
            self.node.get_logger().error(f"로깅 스레드 오류: {str(e)}")

        finally:
            self.node.get_logger().info("로깅 스레드 종료")

    def collect_data_point(self):
        """데이터 포인트 수집"""
        if not self.is_plotting:
            return

        try:
            current_time = datetime.datetime.now()
            if self.start_time is None:
                self.start_time = current_time

            elapsed_time = (current_time - self.start_time).total_seconds()

            # 목표 온도 값 가져오기
            target_temp = self._widget.target_temp_spin.value()

            # 데이터 락 획득 후 배열에 추가
            with self.data_lock:
                self.time_data.append(elapsed_time)
                self.temp_data_4.append(self.temperature_4)
                self.temp_data_5.append(self.temperature_5)
                self.target_temp_data.append(target_temp)
                self.pwm_data_4.append(self.current_pwm_4)
                self.pwm_data_5.append(self.current_pwm_5)
                self.displacement_data.append(self.displacement)

                self.data_points_collected += 1

            # 데이터 로깅 (기록이 활성화된 경우)
            if self.is_recording:
                # 로깅 버퍼에 데이터 추가
                self.data_buffer.append([
                    current_time.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],  # 밀리초까지 표시
                    f"{elapsed_time:.6f}",          # 경과 시간
                    f"{self.temperature_4:.2f}",    # 구동기 4번 온도
                    f"{self.temperature_5:.2f}",    # 구동기 5번 온도
                    f"{target_temp:.2f}",           # 목표 온도
                    f"{self.current_pwm_4}",        # 구동기 4번 PWM
                    f"{self.current_pwm_5}",        # 구동기 5번 PWM
                    f"{self.displacement:.4f}",     # 변위 센서값
                    f"{1 if self.intention_active else 0}",  # 의도 상태
                    f"{1 if self.fan_state_4 else 0}",      # 구동기 4번 팬 상태
                    f"{1 if self.fan_state_5 else 0}"       # 구동기 5번 팬 상태
                ])
                self.data_points_count += 1

        except Exception as e:
            import traceback
            self.node.get_logger().error(f"데이터 수집 오류: {str(e)}")
            self.node.get_logger().error(traceback.format_exc())

    def flush_buffer(self):
        """버퍼 데이터를 파일에 기록"""
        if not self.data_buffer:
            return

        try:
            with self.csv_lock:
                buffer_copy = list(self.data_buffer)
                self.data_buffer = []

                if self.csv_writer is not None:
                    for row in buffer_copy:
                        self.csv_writer.writerow(row)
                    self.csv_file.flush()
        except Exception as e:
            self.node.get_logger().error(f"버퍼 플러시 오류: {str(e)}")

    def start_plotting(self):
        """그래프 플로팅 시작"""
        if self.is_plotting:
            return

        # 그래프 데이터 초기화
        self.reset_graph_data()

        # 시작 시간 설정
        self.start_time = datetime.datetime.now()

        # 플로팅 상태 활성화
        self.is_plotting = True

        # 데이터 수집 스레드 시작
        self.data_collection_active = True
        self.collection_thread = threading.Thread(target=self.high_speed_collection_loop)
        self.collection_thread.daemon = True
        self.collection_thread.start()

        # 그래프 타이머 시작
        self.graph_timer.start()

        # UI 상태 업데이트
        self._widget.graph_start_button.setEnabled(False)
        self._widget.graph_stop_button.setEnabled(True)
        self._widget.status_label.setText("125Hz 고속 데이터 수집 및 그래프 플로팅이 시작되었습니다")
        self.node.get_logger().info("그래프 플로팅이 시작되었습니다 (125Hz)")

    def stop_plotting(self):
        """그래프 플로팅 중지"""
        if not self.is_plotting:
            return

        # 플로팅 상태 비활성화
        self.is_plotting = False

        # 데이터 수집 스레드 중지
        self.data_collection_active = False
        if self.collection_thread and self.collection_thread.is_alive():
            self.collection_thread.join(timeout=1.0)  # 1초 대기

        # 그래프 타이머 중지
        self.graph_timer.stop()

        # UI 상태 업데이트
        self._widget.graph_start_button.setEnabled(True)
        self._widget.graph_stop_button.setEnabled(False)
        self._widget.status_label.setText("그래프 플로팅이 중지되었습니다")
        self.node.get_logger().info("그래프 플로팅이 중지되었습니다")

    def reset_graph_data(self):
        """그래프 데이터 초기화"""
        # 데이터 배열 초기화
        with self.data_lock:
            self.time_data.clear()
            self.temp_data_4.clear()
            self.temp_data_5.clear()
            self.target_temp_data.clear()
            self.pwm_data_4.clear()
            self.pwm_data_5.clear()
            self.displacement_data.clear()
            self.collection_intervals.clear()

        # 시작 시간 초기화
        self.start_time = None
        self.data_points_collected = 0
        self.actual_collection_rate = 0.0

        # 그래프 초기화
        if (hasattr(self, 'temp_line_4') and hasattr(self, 'temp_line_5') and
            hasattr(self, 'target_temp_line') and hasattr(self, 'pwm_line_4') and
            hasattr(self, 'pwm_line_5') and hasattr(self, 'displacement_line')):
            self.temp_line_4.setData([], [])
            self.temp_line_5.setData([], [])
            self.target_temp_line.setData([], [])
            self.pwm_line_4.setData([], [])
            self.pwm_line_5.setData([], [])
            self.displacement_line.setData([], [])

            # 그래프 범위 초기화
            if hasattr(self, 'temp_plot_widget') and hasattr(self, 'pwm_plot_widget') and hasattr(self, 'displacement_plot_widget'):
                self.temp_plot_widget.setXRange(0, 60)  # 초기 화면은 1분 범위
                self.pwm_plot_widget.setXRange(0, 60)
                self.displacement_plot_widget.setXRange(0, 60)

        self._widget.status_label.setText("그래프 데이터가 초기화되었습니다")
        self.node.get_logger().info("그래프 데이터가 초기화되었습니다")

    def update_graphs(self):
        """10Hz로 그래프 데이터 업데이트 (UI 스레드)"""
        if not self.is_plotting:
            return

        try:
            # 락 획득 후 데이터 복사 (스레드 안전)
            with self.data_lock:
                if not self.time_data or len(self.time_data) < 2:
                    return  # 데이터가 없거나 불충분하면 업데이트 중단

                # 데이터 안전하게 복사
                time_data = list(self.time_data)
                temp_data_4 = list(self.temp_data_4)
                temp_data_5 = list(self.temp_data_5)
                target_temp_data = list(self.target_temp_data)
                pwm_data_4 = list(self.pwm_data_4)
                pwm_data_5 = list(self.pwm_data_5)
                displacement_data = list(self.displacement_data)

            # 모든 데이터 길이 통일 (가장 짧은 데이터 기준)
            min_length = min(
                len(time_data), len(temp_data_4), len(temp_data_5), len(target_temp_data),
                len(pwm_data_4), len(pwm_data_5), len(displacement_data)
            )

            if min_length < 2:
                return  # 데이터가 부족하면 업데이트 중단

            time_data = time_data[:min_length]
            temp_data_4 = temp_data_4[:min_length]
            temp_data_5 = temp_data_5[:min_length]
            target_temp_data = target_temp_data[:min_length]
            pwm_data_4 = pwm_data_4[:min_length]
            pwm_data_5 = pwm_data_5[:min_length]
            displacement_data = displacement_data[:min_length]

            # 다운샘플링 - 125Hz에서는 매우 중요, 고속 데이터의 경우 1/10만 표시
            step = max(1, min_length // 500)  # 화면에 최대 500개 포인트 표시
            time_data = time_data[::step]
            temp_data_4 = temp_data_4[::step]
            temp_data_5 = temp_data_5[::step]
            target_temp_data = target_temp_data[::step]
            pwm_data_4 = pwm_data_4[::step]
            pwm_data_5 = pwm_data_5[::step]
            displacement_data = displacement_data[::step]

            # 그래프 업데이트
            self.temp_line_4.setData(time_data, temp_data_4)
            self.temp_line_5.setData(time_data, temp_data_5)
            self.target_temp_line.setData(time_data, target_temp_data)
            self.pwm_line_4.setData(time_data, pwm_data_4)
            self.pwm_line_5.setData(time_data, pwm_data_5)
            self.displacement_line.setData(time_data, displacement_data)

            # 임계값 라인 업데이트
            if hasattr(self, 'threshold_line'):
                threshold_value = self._widget.angle_threshold_spin.value()
                self.threshold_line.setValue(threshold_value)
                self.threshold_line.label.setText(f"임계값: {threshold_value}V")

            # 시간 창 계산 (60초 창)
            window_size = 60.0
            if time_data:
                latest_time = time_data[-1]
                start_time = max(0, latest_time - window_size)

                # X축 범위 설정
                self.temp_plot_widget.setXRange(start_time, latest_time, padding=0.02)
                self.pwm_plot_widget.setXRange(start_time, latest_time, padding=0.02)
                self.displacement_plot_widget.setXRange(start_time, latest_time, padding=0.02)

                # 자동 범위 조정 비활성화
                self.temp_plot_widget.getViewBox().setAutoVisible(y=True, x=False)
                self.pwm_plot_widget.getViewBox().setAutoVisible(y=True, x=False)
                self.displacement_plot_widget.getViewBox().setAutoVisible(y=True, x=False)

        except Exception as e:
            import traceback
            self.node.get_logger().error(f"그래프 업데이트 오류: {str(e)}")
            self.node.get_logger().error(traceback.format_exc())

    def toggle_intention(self):
        """의도 활성화/비활성화 토글"""
        self.intention_active = not self.intention_active

        # 의도 메시지 발행
        intention_msg = BackIntention()
        intention_msg.header.stamp = self.node.get_clock().now().to_msg()
        intention_msg.intention_id = 1 if self.intention_active else 0

        self.intention_pub.publish(intention_msg)

        # UI 업데이트
        self._widget.intention_status_label.setText(f"현재 의도: {'활성화됨' if self.intention_active else '없음'}")
        self._widget.status_label.setText(f"의도가 {'활성화' if self.intention_active else '비활성화'}되었습니다")
        self.node.get_logger().info(f"의도 {'활성화' if self.intention_active else '비활성화'} 명령 전송")

    def apply_settings(self):
        """현재 UI 설정에 따라 명령 발행"""
        if self.is_emergency_stop:
            self.node.get_logger().warn('비상 정지 중입니다. 제어 명령이 무시됩니다.')
            return

        # 목표 온도 설정
        target_temp = self._widget.target_temp_spin.value()
        target_temp_msg = Float64()
        target_temp_msg.data = target_temp
        self.target_temp_pub.publish(target_temp_msg)

        # 임계값 설정
        threshold = self._widget.angle_threshold_spin.value()
        threshold_msg = Float64()
        threshold_msg.data = threshold
        self.threshold_pub.publish(threshold_msg)

        # 임계값 라인 업데이트
        if hasattr(self, 'threshold_line'):
            self.threshold_line.setValue(threshold)
            self.threshold_line.label.setText(f"임계값: {threshold}V")

        self._widget.status_label.setText(f"목표 온도가 {target_temp}°C로, 임계값이 {threshold}V로 설정되었습니다")
        self.node.get_logger().info(f"설정 적용: 목표 온도={target_temp}°C, 임계값={threshold}V")

    def emergency_stop(self):
        """비상 정지 버튼 이벤트 처리"""
        # 현재 상태의 반대로 설정
        self.is_emergency_stop = not self.is_emergency_stop

        # 비상 정지 서비스 호출 (향후 구현)
        if self.is_emergency_stop:
            self._widget.status_label.setText('비상 정지가 활성화되었습니다')
            self._widget.status_label.setStyleSheet("color: red; font-weight: bold;")
            self._widget.emergency_stop_button.setText("비상 정지 해제")
        else:
            self._widget.status_label.setText('비상 정지가 해제되었습니다')
            self._widget.status_label.setStyleSheet("")
            self._widget.emergency_stop_button.setText("비상 정지")

        self.node.get_logger().info(f'비상 정지 {"활성화" if self.is_emergency_stop else "해제"} 명령 전송')

    def start_recording(self):
        """CSV 파일로 데이터 기록 시작"""
        if self.is_recording:
            return

        try:
            # 파일명에 날짜/시간 추가
            base_name = self._widget.filename_edit.text() if hasattr(self._widget, 'filename_edit') else "waist_robot_log"
            if not base_name:
                base_name = "waist_robot_log"

            timestamp = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
            output_dir = os.path.expanduser("~/ros2_csv_logs")
            os.makedirs(output_dir, exist_ok=True)

            self.csv_filename = os.path.join(output_dir, f"{base_name}_{timestamp}.csv")

            # CSV 파일 생성 및 헤더 작성
            with self.csv_lock:
                self.csv_file = open(self.csv_filename, 'w', newline='')
                self.csv_writer = csv.writer(self.csv_file)
                self.csv_writer.writerow([
                    "timestamp",
                    "elapsed_time_sec",
                    "temperature_4",
                    "temperature_5",
                    "target_temperature",
                    "pwm_4",
                    "pwm_5",
                    "displacement",
                    "intention",
                    "fan_4",
                    "fan_5"
                ])

            self.is_recording = True
            self.csv_start_time = datetime.datetime.now()
            self.data_buffer = []  # 버퍼 초기화
            self.data_points_count = 0  # 데이터 포인트 카운터 초기화

            # 로깅 스레드 시작
            self.logging_thread_active = True
            self.logging_thread = threading.Thread(target=self.logging_worker)
            self.logging_thread.daemon = True
            self.logging_thread.start()

            self._widget.status_label.setText(f"125Hz 고속 데이터 기록이 시작되었습니다: {os.path.basename(self.csv_filename)}")
            self._widget.save_data_button.setText("기록 중지")
            self.node.get_logger().info(f"CSV 데이터 기록이 시작되었습니다 (125Hz): {self.csv_filename}")

            # 로깅 상태 메시지 업데이트
            self._widget.log_status_label.setText(
                f"로깅 상태: 활성화 - {os.path.basename(self.csv_filename)}"
            )

            # 그래프 플로팅도 자동으로 시작
            if not self.is_plotting:
                self.start_plotting()

        except Exception as e:
            error_msg = f"CSV 파일 기록 시작 오류: {str(e)}"
            self._widget.status_label.setText(error_msg)
            self.node.get_logger().error(error_msg)

            # 오류 메시지 표시
            QMessageBox.critical(self._widget, "저장 오류", f"CSV 파일 기록 시작 중 오류가 발생했습니다.\n{str(e)}")

    def stop_recording(self):
        """CSV 파일로 데이터 기록 중지"""
        if not self.is_recording:
            return

        try:
            # 로깅 스레드 중지
            self.logging_thread_active = False
            if self.logging_thread and self.logging_thread.is_alive():
                self.logging_thread.join(timeout=1.0)  # 1초 대기

            # 버퍼에 남아있는 데이터를 파일에 쓰기
            with self.csv_lock:
                if self.csv_writer is not None and self.data_buffer:
                    for row in self.data_buffer:
                        self.csv_writer.writerow(row)
                    self.data_buffer = []

                # 파일 닫기
                if self.csv_file is not None:
                    self.csv_file.close()
                    self.csv_file = None
                    self.csv_writer = None

            self.is_recording = False

            total_duration = 0
            if hasattr(self, 'csv_start_time') and self.csv_start_time is not None:
                total_duration = (datetime.datetime.now() - self.csv_start_time).total_seconds()

            rate = self.data_points_count / total_duration if total_duration > 0 else 0

            self._widget.status_label.setText(
                f"데이터 기록이 중지되었습니다: {os.path.basename(self.csv_filename)} - "
                f"{self.data_points_count}개 기록 (평균 {rate:.1f}Hz)"
            )
            self._widget.save_data_button.setText("데이터 저장")
            self.node.get_logger().info(
                f"CSV 데이터 기록이 중지되었습니다: {self.csv_filename} - "
                f"{self.data_points_count}개 기록, 기간: {total_duration:.1f}초, 평균 속도: {rate:.1f}Hz"
            )

            # 로깅 상태 메시지 업데이트
            self._widget.log_status_label.setText("로깅 상태: 대기 중")

        except Exception as e:
            error_msg = f"CSV 파일 기록 중지 오류: {str(e)}"
            self._widget.status_label.setText(error_msg)
            self.node.get_logger().error(error_msg)

            # 오류 메시지 표시
            QMessageBox.critical(self._widget, "저장 오류", f"CSV 파일 기록 중지 중 오류가 발생했습니다.\n{str(e)}")

    def request_data_save(self):
        """CSV 파일로 데이터 저장 시작/중지"""
        try:
            if self.is_recording:
                # 녹화 중지
                self.stop_recording()
            else:
                # 녹화 시작
                self.start_recording()
        except Exception as e:
            error_msg = f"CSV 파일 기록 오류: {str(e)}"
            self._widget.status_label.setText(error_msg)
            self.node.get_logger().error(error_msg)
            QMessageBox.critical(self._widget, "저장 오류", f"CSV 파일 기록 중 오류가 발생했습니다.\n{str(e)}")

    def shutdown_plugin(self):
        """플러그인이 종료될 때 호출되는 메서드"""
        self._shutdown_flag = True

        # Qt 타이머 중지
        if hasattr(self, 'qt_timer') and self.qt_timer.isActive():
            self.qt_timer.stop()

        # 데이터 수집 및 그래프 타이머 중지
        self.data_collection_active = False
        if hasattr(self, 'collection_thread') and self.collection_thread and self.collection_thread.is_alive():
            self.collection_thread.join(timeout=1.0)

        if hasattr(self, 'graph_timer') and self.graph_timer.isActive():
            self.graph_timer.stop()

        # 로깅 스레드 중지 및 CSV 파일 닫기
        self.logging_thread_active = False
        if hasattr(self, 'logging_thread') and self.logging_thread and self.logging_thread.is_alive():
            self.logging_thread.join(timeout=1.0)

        if hasattr(self, 'is_recording') and self.is_recording:
            self.stop_recording()

        # 스핀 스레드 종료 대기
        if hasattr(self, 'spin_thread') and self.spin_thread.is_alive():
            self.spin_thread.join(timeout=1.0)  # 최대 1초 대기

        # ROS 노드 정리
        if hasattr(self, 'node') and self.node:
            self.node.get_logger().info("플러그인 종료 중...")
            self.node.destroy_node()

        # 비상 정지 해제 명령 시도
        if self.is_emergency_stop:
            try:
                # 비상 정지 해제 메시지 발행 (향후 구현)
                print("종료 전 비상 정지 해제 명령 전송")
            except:
                pass

        print("[INFO]: 플러그인 자원이 정상적으로 해제되었습니다")

def main(args=None):
    """
    rqt 플러그인 메인 함수 (독립 실행 모드에서 사용)
    """
    from rqt_gui.main import Main
    main = Main()
    sys.exit(main.main(sys.argv, standalone='wearable_robot_rqt_plugins.waist_control_plugin.WaistControlPlugin'))

if __name__ == '__main__':
    import sys
    rclpy.init(args=sys.argv)
    main()
