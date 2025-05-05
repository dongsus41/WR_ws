#!/usr/bin/env python3

import os
import csv
import threading
import subprocess
import signal
import datetime
import rclpy
import sys
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from wearable_robot_interfaces.msg import ActuatorCommand, TemperatureData
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QMessageBox, QVBoxLayout, QLabel
from python_qt_binding.QtCore import Qt, QTimer
import pyqtgraph as pg

class WaistControlPlugin(Plugin):
    """
    웨어러블 로봇의 구동기 제어 및 온도 모니터링을 위한 rqt 플러그인
    펌웨어 기반 제어로 변경된 버전
    """
    def __init__(self, context):
        super(WaistControlPlugin, self).__init__(context)
        # 플러그인 제목 설정
        self.setObjectName('WaistControlPlugin')

        # 그래프 플로팅 관련 변수 초기화
        self.is_plotting = False
        self.buffer_size = 300
        self.time_data = []
        self.temp_data = []
        self.target_temp_data = []
        self.pwm_data = []
        self.start_time = 0.0

        # 내부 상태 변수 초기화
        self.temperature = 0.0              # 현재 온도
        self.current_pwm = 0                # 현재 PWM 값
        self.auto_mode = False              # 자동 제어 모드 여부
        self.is_emergency_stop = False      # 비상 정지 상태
        self.safety_temp_threshold = 80.0   # 안전 온도 임계값 (°C)
        self.recording_process = None

        # 그래프 관련 객체 초기화
        self.temp_plot_widget = None
        self.pwm_plot_widget = None
        self.temp_line = None
        self.target_temp_line = None
        self.pwm_line = None
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

        # ROS 노드 초기화
        try:
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
            ActuatorCommand, 'actuator_command', qos_profile)  # CAN으로 명령 전송

        # 구독자 생성
        self.temp_sub = self.node.create_subscription(
            TemperatureData, 'temperature_data', self.temp_callback, qos_profile)  # 온도 데이터
        self.pwm_sub = self.node.create_subscription(
            ActuatorCommand, 'pwm_state', self.pwm_callback, qos_profile)  # 현재 PWM 상태

        # CAN 명령 메시지 초기화
        self.command_msg = ActuatorCommand()
        self.command_msg.pwm = [0] * 6  # 6개 채널 모두 0으로 초기화

        self._shutdown_flag = False

        # ROS 2 스핀을 위한 스레드 생성
        self.spin_thread = threading.Thread(target=self.spin_ros)
        self.spin_thread.daemon = True
        self.spin_thread.start()

        # 스레드 안전 락
        self.data_lock = threading.RLock()

        # UI 설정
        self._widget = QWidget()
        ui_file = self.find_ui_file()

        if ui_file is None:
            raise FileNotFoundError("actuator_control.ui 파일을 찾을 수 없습니다.")

        loadUi(ui_file, self._widget)
        self._widget.setObjectName('WaistControlPluginUI')

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
        os.makedirs(self.data_logging_path, exist_ok=True)

        # 그래프 UI 설정 및 초기화
        self.setup_graphs()

        # 그래프 제어 버튼 연결
        self._widget.graph_start_button.clicked.connect(self.start_plotting)
        self._widget.graph_stop_button.clicked.connect(self.stop_plotting)
        self._widget.reset_graph_button.clicked.connect(self.reset_graph_data)

        # 데이터 저장 버튼 연결
        self._widget.save_data_button.clicked.connect(self.request_data_save)

        # Qt 타이머 추가 (UI 업데이트용)
        self.qt_timer = QTimer()
        self.qt_timer.timeout.connect(self.update_ui)
        self.qt_timer.start(100)  # 100ms 마다 UI 업데이트

        # 그래프 업데이트 타이머
        self.graph_timer = QTimer()
        self.graph_timer.timeout.connect(self.update_graphs)
        self.graph_timer.setInterval(100)  # 100ms마다 그래프 업데이트 (10Hz)

        # 초기화 완료 표시
        self._widget.status_label.setText('플러그인이 초기화되었습니다')
        self.node.get_logger().info('구동기 제어 플러그인이 초기화되었습니다')

    def find_ui_file(self):
        # UI 파일 경로를 찾기 위한 여러 가능한 위치 시도
        candidate_paths = [
            os.path.join(os.path.dirname(os.path.realpath(__file__)), 'resource', 'actuator_control.ui'),
            os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'resource', 'actuator_control.ui'),
            os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), 'share',
                        'wearable_robot_rqt_plugins', 'resource', 'actuator_control.ui'),
            os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), 'resource', 'actuator_control.ui'),
            os.path.expanduser("~/wearable_robot_ws/src/wearable_robot_rqt_plugins/resource/actuator_control.ui")
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
                executor.spin_once(timeout_sec=0.1)  # 100ms 타임아웃
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
            self.temp_line = self.temp_plot_widget.plot(
                pen=pg.mkPen(color=(255, 0, 0), width=2),
                name="현재 온도",
                symbol='o',
                symbolSize=4,
                symbolBrush=(255, 0, 0)
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
            self.pwm_plot_widget.setYRange(0, 100, padding=0.05)
            self.pwm_plot_widget.setMouseEnabled(x=True, y=True)

            # PWM 그래프 데이터 라인 생성
            self.pwm_line = self.pwm_plot_widget.plot(
                pen=pg.mkPen(color=(0, 128, 0), width=2),
                name="PWM 값",
                symbol='o',
                symbolSize=4,
                symbolBrush=(0, 128, 0)
            )

            # X축 연결 (두 그래프의 시간축을 동기화)
            self.temp_plot_widget.setXLink(self.pwm_plot_widget)

            # 기존 레이아웃 제거 및 그래프 프레임에 위젯 추가
            self.clear_and_set_layout(self._widget.temp_graph_frame, self.temp_plot_widget)
            self.clear_and_set_layout(self._widget.pwm_graph_frame, self.pwm_plot_widget)

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
            self._widget.temp_label.setText(f"{self.temperature:.1f}°C")
            self._widget.pwm_label.setText(f"{self.current_pwm}")

            # 비상 정지 상태 표시
            if self.is_emergency_stop:
                self._widget.status_label.setText("비상 정지 상태")
                self._widget.status_label.setStyleSheet("color: red; font-weight: bold;")
                self._widget.apply_button.setEnabled(False)
                self._widget.emergency_stop_button.setText("비상 정지 해제")
                self._widget.manual_mode_radio.setEnabled(False)
                self._widget.auto_mode_radio.setEnabled(False)
                self._widget.manual_settings_group.setEnabled(False)
                self._widget.auto_settings_group.setEnabled(False)
            else:
                self._widget.apply_button.setEnabled(True)
                self._widget.emergency_stop_button.setText("비상 정지")
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
            if self.is_recording:
                self._widget.save_data_button.setText("기록 중지")
                if "로깅 상태: 활성화" not in self._widget.log_status_label.text():
                    self._widget.log_status_label.setText("로깅 상태: 활성화")
            else:
                self._widget.save_data_button.setText("데이터 저장")

        except RuntimeError as e:
            if "deleted" in str(e):
                # 위젯이 삭제된 경우 타이머 중지
                if hasattr(self, 'qt_timer'):
                    self.qt_timer.stop()

    def temp_callback(self, msg):
        """온도 데이터 수신 콜백"""
        try:
            # 구동기 5번(인덱스 5) 온도 데이터 저장
            if len(msg.temperature) > 5:
                self.temperature = msg.temperature[5]

                # 그래프 데이터 수집 (그래프 활성화 상태일 때만)
                if self.is_plotting and hasattr(self, 'start_time') and self.start_time is not None:
                    current_time = datetime.datetime.now()
                    elapsed_time = (current_time - self.start_time).total_seconds()

                    # 데이터 락 획득 후 배열 수정
                    with self.data_lock:
                        self.time_data.append(elapsed_time)
                        self.temp_data.append(self.temperature)

                        # 목표 온도 (자동 모드일 때는 설정값, 수동 모드일 때는 0)
                        target_temp = self._widget.target_temp_spin.value() if self.auto_mode else 0.0
                        self.target_temp_data.append(target_temp)

                        # PWM 값
                        self.pwm_data.append(self.current_pwm)

                        # 버퍼 크기 제한 (5분 = 300초 기준)
                        max_age = 300.0  # 5분
                        while self.time_data and (elapsed_time - self.time_data[0]) > max_age:
                            self.time_data.pop(0)
                            self.temp_data.pop(0)
                            self.target_temp_data.pop(0)
                            self.pwm_data.pop(0)

                    # CSV 파일 데이터 기록 (기록 활성화 상태일 때만)
                    if self.is_recording:
                        self.record_data_point(current_time, elapsed_time, self.temperature,
                                             target_temp, self.current_pwm)

        except Exception as e:
            import traceback
            self.node.get_logger().error(f'온도 데이터 처리 오류: {str(e)}')
            self.node.get_logger().error(traceback.format_exc())

    def record_data_point(self, current_time, elapsed_time, temperature, target_temp, pwm):
        """데이터 포인트를 기록 버퍼에 추가"""
        try:
            # 버퍼에 데이터 추가
            self.data_buffer.append([
                current_time.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],  # 밀리초까지 표시
                f"{elapsed_time:.6f}",  # 경과 시간
                f"{temperature:.2f}",   # 온도
                f"{target_temp:.2f}",   # 목표 온도
                f"{pwm}"                # PWM 값
            ])
            self.data_points_count += 1

            # 버퍼가 일정 크기에 도달하면 파일에 쓰기
            if len(self.data_buffer) >= 50:  # 50개 데이터마다 기록
                self.flush_buffer()

                # 로깅 상태 업데이트 (1000개 데이터 포인트마다)
                if self.data_points_count % 1000 == 0:
                    now = datetime.datetime.now()
                    if hasattr(self, 'csv_start_time') and self.csv_start_time is not None:
                        duration = (now - self.csv_start_time).total_seconds()
                        rate = self.data_points_count / duration if duration > 0 else 0
                        self._widget.log_status_label.setText(
                            f"로깅 상태: 활성화 - {os.path.basename(self.csv_filename)} - "
                            f"{self.data_points_count}개 기록 (평균 {rate:.1f}Hz)"
                        )
        except Exception as e:
            self.node.get_logger().error(f"데이터 버퍼링 오류: {str(e)}")

    def flush_buffer(self):
        """버퍼 데이터를 파일에 기록"""
        if not self.data_buffer:
            return

        try:
            with self.csv_lock:
                if self.csv_writer is not None:
                    for row in self.data_buffer:
                        self.csv_writer.writerow(row)
                    self.csv_file.flush()
            self.data_buffer = []
        except Exception as e:
            self.node.get_logger().error(f"버퍼 플러시 오류: {str(e)}")

    def pwm_callback(self, msg):
        """현재 PWM 상태 수신 콜백"""
        try:
            # 구동기 5번(인덱스 5) PWM 값 저장
            if len(msg.pwm) > 5:
                self.current_pwm = msg.pwm[5]
                self.node.get_logger().debug(f'구동기 5번 현재 PWM: {self.current_pwm}')
        except Exception as e:
            self.node.get_logger().error(f'PWM 데이터 처리 오류: {str(e)}')

    def start_plotting(self):
        """그래프 플로팅 시작"""
        if self.is_plotting:
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
            target_temp = self._widget.target_temp_spin.value() if self.auto_mode else 0.0
            self.target_temp_data = [target_temp]
            self.pwm_data = [self.current_pwm]

    def stop_plotting(self):
        """그래프 플로팅 중지"""
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
        """그래프 데이터 초기화"""
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

        self._widget.status_label.setText("그래프 데이터가 초기화되었습니다")
        self.node.get_logger().info("그래프 데이터가 초기화되었습니다")

    def update_graphs(self):
        """그래프 데이터 업데이트"""
        if not self.is_plotting:
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

                # 데이터 일관성 확인
                min_length = min(len(time_data), len(temp_data), len(target_temp_data), len(pwm_data))
                if min_length < 2:
                    return

                # 모든 배열을 같은 길이로 자름
                time_data = time_data[:min_length]
                temp_data = temp_data[:min_length]
                target_temp_data = target_temp_data[:min_length]
                pwm_data = pwm_data[:min_length]

                # 시간 창 계산 (1분 창)
                window_size = 60.0
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

                # 다운샘플링 - 데이터 포인트가 많을 경우
                if len(display_time) > 400:
                    # 간단한 다운샘플링 로직
                    step = len(display_time) // 400
                    display_time = display_time[::step]
                    display_temp = display_temp[::step]
                    display_target = display_target[::step]
                    display_pwm = display_pwm[::step]

                # 그래프 업데이트
                self.temp_line.setData(display_time, display_temp)
                self.target_temp_line.setData(display_time, display_target)
                self.pwm_line.setData(display_time, display_pwm)

                # X축 범위 설정
                self.temp_plot_widget.setXRange(start_time, latest_time, padding=0.02)
                self.pwm_plot_widget.setXRange(start_time, latest_time, padding=0.02)

                # 자동 범위 조정 비활성화
                self.temp_plot_widget.getViewBox().setAutoVisible(y=True, x=False)
                self.pwm_plot_widget.getViewBox().setAutoVisible(y=True, x=False)

                # 상태 업데이트
                if hasattr(self, '_widget'):
                    self._widget.status_label.setText(f"데이터 플로팅 중: {len(display_time)}포인트")

        except Exception as e:
            import traceback
            self.node.get_logger().error(f"그래프 업데이트 오류: {str(e)}")
            self.node.get_logger().error(traceback.format_exc())

    def mode_changed(self, checked):
        """제어 모드 변경 이벤트 처리"""
        if not checked:
            return

        if self._widget.auto_mode_radio.isChecked():
            self.auto_mode = True
            self._widget.auto_settings_group.setEnabled(True)
            self._widget.manual_settings_group.setEnabled(False)
            self._widget.status_label.setText('자동 모드 활성화 요청 중...')

            # CAN 명령 메시지 준비
            self.command_msg.pwm = [0] * 6
            self.command_msg.pwm[5] = 0  # 구동기 5번 PWM 초기화

            # 활성화 정보 전송 - 펌웨어에 자동 모드 명령 전송
            self.send_command_with_pid_enable(True)

        else:
            self.auto_mode = False
            self._widget.auto_settings_group.setEnabled(False)
            self._widget.manual_settings_group.setEnabled(True)
            self._widget.status_label.setText('수동 모드 활성화 요청 중...')

            # 펌웨어에 수동 모드 명령 전송
            self.send_command_with_pid_enable(False)

    def send_command_with_pid_enable(self, enable_pid):
        """PID 활성화 상태와 함께 명령 전송"""
        # ActuatorCommand 메시지에는 PWM 값만 있으므로, 펌웨어가 CAN 메시지의
        # 특정 바이트를 PID 활성화 상태로 해석하도록 해야 함
        command_msg = ActuatorCommand()
        command_msg.header.stamp = self.node.get_clock().now().to_msg()
        command_msg.pwm = [0] * 6

        # 5번 인덱스에 특수한 값을 설정하여 모드 변경을 알림
        # 실제 전송 형식은 펌웨어와 협의 필요
        if enable_pid:
            # 자동 모드 활성화를 알리는 특별한 값
            command_msg.pwm[5] = 254
        else:
            # 수동 모드 활성화를 알리는 값
            command_msg.pwm[5] = 255

        self.actuator_pub.publish(command_msg)
        self.node.get_logger().info(f"모드 변경 명령 전송: {'자동' if enable_pid else '수동'} 모드")

    def apply_settings(self):
        """현재 UI 설정에 따라 명령 발행"""
        if self.is_emergency_stop:
            self.node.get_logger().warn('비상 정지 중입니다. 제어 명령이 무시됩니다.')
            return

        command_msg = ActuatorCommand()
        command_msg.header.stamp = self.node.get_clock().now().to_msg()
        command_msg.pwm = [0] * 6  # 모든 채널 0으로 초기화

        if self.auto_mode:
            # 자동 모드: 펌웨어에 목표 온도 전송
            # 간단한 방식: PWM 값 배열의 특정 위치를 특별한 값으로 설정하여
            # 모드와 목표 온도를 인코딩
            target_temp = self._widget.target_temp_spin.value()

            # 예: 250은 특별 값, 뒤이어 목표 온도 값을 2배하여 저장
            # (실제로는 펌웨어와 정확한 프로토콜 협의 필요)
            command_msg.pwm[0] = 250  # 특별 명령 코드
            command_msg.pwm[1] = int(target_temp * 4)  # 목표 온도를 4배 (0.25도 단위로 변환)
            command_msg.pwm[5] = 1    # PID 활성화 플래그

            self.actuator_pub.publish(command_msg)
            self._widget.status_label.setText(f'목표 온도가 {target_temp}°C로 설정되었습니다')

        else:
            # 수동 모드: PWM 값 직접 전송
            pwm_value = self._widget.pwm_spin.value()
            command_msg.pwm[5] = pwm_value  # 구동기 5번 PWM 설정

            self.actuator_pub.publish(command_msg)
            self._widget.status_label.setText(f'구동기 5번 PWM이 {pwm_value}으로 설정되었습니다')

        self.node.get_logger().info(f'명령 전송: {"목표 온도=" + str(target_temp) if self.auto_mode else "PWM=" + str(pwm_value)}')

    def emergency_stop(self):
        """비상 정지 버튼 이벤트 처리"""
        # 현재 상태의 반대로 설정
        self.is_emergency_stop = not self.is_emergency_stop

        # 비상 정지 명령 전송
        command_msg = ActuatorCommand()
        command_msg.header.stamp = self.node.get_clock().now().to_msg()
        command_msg.pwm = [0] * 6

        if self.is_emergency_stop:
            # 비상 정지 활성화
            command_msg.pwm[0] = 240  # 비상 정지 특별 코드
            self._widget.status_label.setText('비상 정지가 활성화되었습니다')
        else:
            # 비상 정지 해제
            command_msg.pwm[0] = 241  # 비상 정지 해제 특별 코드
            self._widget.status_label.setText('비상 정지가 해제되었습니다')

        self.actuator_pub.publish(command_msg)
        self.node.get_logger().info(f'비상 정지 {"활성화" if self.is_emergency_stop else "해제"} 명령 전송')

    def start_recording(self):
        """CSV 파일로 데이터 기록 시작"""
        if self.is_recording:
            return

        try:
            # 파일명에 날짜/시간 추가
            base_name = self._widget.filename_edit.text() if hasattr(self._widget, 'filename_edit') else "wearable_robot"
            if not base_name:
                base_name = "wearable_robot"

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
                    "temperature",
                    "target_temperature",
                    "pwm"
                ])

            self.is_recording = True
            self.csv_start_time = datetime.datetime.now()
            self.data_buffer = []  # 버퍼 초기화
            self.data_points_count = 0  # 데이터 포인트 카운터 초기화

            self._widget.status_label.setText(f"데이터 기록이 시작되었습니다: {os.path.basename(self.csv_filename)}")
            self._widget.save_data_button.setText("기록 중지")
            self.node.get_logger().info(f"CSV 데이터 기록이 시작되었습니다: {self.csv_filename}")

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
            # 버퍼에 남아있는 데이터를 파일에 쓰기
            with self.csv_lock:
                if self.csv_writer is not None and self.data_buffer:
                    for row in self.data_buffer:
                        self.csv_writer.writerow(row)

                # 파일 닫기
                if self.csv_file is not None:
                    self.csv_file.close()
                    self.csv_file = None
                    self.csv_writer = None

            self.is_recording = False
            self.data_buffer = []

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

        # 그래프 타이머 중지
        if hasattr(self, 'graph_timer') and self.graph_timer.isActive():
            self.graph_timer.stop()

        # CSV 파일 닫기
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
                command_msg = ActuatorCommand()
                command_msg.pwm = [0] * 6
                command_msg.pwm[0] = 241  # 비상 정지 해제 코드
                self.actuator_pub.publish(command_msg)
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
    sys.exit(main.main(sys.argv, standalone='wearable_robot_rqt_plugins.actuator_control_plugin.WaistControlPlugin'))

if __name__ == '__main__':
    import sys
    rclpy.init(args=sys.argv)
    main()
