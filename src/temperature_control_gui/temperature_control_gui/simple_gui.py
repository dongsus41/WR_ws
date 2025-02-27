#!/usr/bin/env python3

import sys
import numpy as np
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QLabel,
                           QSlider, QPushButton, QSpinBox, QWidget, QDoubleSpinBox,
                           QGroupBox, QGridLayout, QSplitter)
from PyQt5.QtCore import Qt, QTimer
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from wearable_robot_interfaces.msg import TemperatureData, ActuatorCommand
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

class TemperatureControlWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Wearable Robot Temperature Control System")
        self.setGeometry(100, 100, 1600, 900)

        # Initialize parameter values
        self.current_target_temp = 50.0
        self.active_actuator = 5  # Default actuator index
        self.current_temp = 0.0
        self.current_pwm = 0
        self.kp_value = 2.0
        self.ki_value = 0.1
        self.max_pwm_value = 100.0
        self.min_pwm_value = 0.0

        # ROS2 initialization
        rclpy.init(args=None)
        self.node = rclpy.create_node('temperature_control_gui')

        # Main widget setup
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # Create left and right panels
        left_panel = QWidget()
        right_panel = QWidget()

        # Set up layouts
        left_layout = QVBoxLayout(left_panel)
        right_layout = QVBoxLayout(right_panel)

        # Create splitter for resizable panels
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(left_panel)
        splitter.addWidget(right_panel)
        splitter.setSizes([1000, 600])

        main_layout.addWidget(splitter)

        # Set up left panel (graphs and status)
        self.setup_left_panel(left_layout)

        # Set up right panel (control parameters)
        self.setup_right_panel(right_layout)

        # Data storage for graphs
        self.temperature_data = []
        self.target_temp_data = []
        self.pwm_data = []
        self.time_data = []
        self.start_time = self.node.get_clock().now().seconds_nanoseconds()[0]
        self.last_graph_update = 0  # Track last update time

        # Temperature data subscription
        self.temp_subscription = self.node.create_subscription(
            TemperatureData,
            'temperature_data',
            self.temperature_callback,
            10)

        # PWM data subscription
        self.pwm_subscription = self.node.create_subscription(
            ActuatorCommand,
            'pwm_state',
            self.pwm_callback,
            10)

        # Parameter service clients
        self.parameter_client = self.node.create_client(
            SetParameters,
            '/actuator_temp_control_node/set_parameters')

        self.get_param_client = self.node.create_client(
            GetParameters,
            '/actuator_temp_control_node/get_parameters')

        # Set up timers for GUI and ROS2 updates
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.process_ros_events)
        self.ros_timer.start(10)  # Process ROS events every 10ms

        self.gui_timer = QTimer()
        self.gui_timer.timeout.connect(self.update_gui)
        self.gui_timer.start(10)  # Update GUI every 100ms

        self.graph_timer = QTimer()
        self.graph_timer.timeout.connect(self.update_plot)
        self.graph_timer.start(50)  # Update graphs every 500ms

        # Get current parameter values
        self.get_current_parameters()

    def setup_left_panel(self, layout):
        """Set up left panel with graphs and status display"""
        # Graph widget
        graph_box = QGroupBox("Temperature and PWM Monitoring")
        graph_layout = QVBoxLayout(graph_box)

        # Create graphs
        self.temp_fig = Figure(figsize=(9, 9), dpi=80)

        # Temperature graph
        self.temp_ax = self.temp_fig.add_subplot(211)

        # PWM graph
        self.pwm_ax = self.temp_fig.add_subplot(212, sharex=self.temp_ax)

        self.canvas = FigureCanvas(self.temp_fig)
        graph_layout.addWidget(self.canvas)

        layout.addWidget(graph_box)

        # Status display area
        status_box = QGroupBox("System Status")
        status_layout = QVBoxLayout(status_box)

        self.current_temp_label = QLabel("Current Temp: -- °C")
        self.current_temp_label.setStyleSheet("font-size: 14pt; font-weight: bold;")
        status_layout.addWidget(self.current_temp_label)

        self.current_pwm_label = QLabel("Current PWM: --%")
        self.current_pwm_label.setStyleSheet("font-size: 12pt;")
        status_layout.addWidget(self.current_pwm_label)

        self.status_label = QLabel("Status: Ready")
        status_layout.addWidget(self.status_label)

        layout.addWidget(status_box)

    def setup_right_panel(self, layout):
        """Set up right panel with control parameters"""
        # Target temperature settings
        target_box = QGroupBox("Target Temperature Setting")
        target_layout = QVBoxLayout(target_box)

        temp_layout = QHBoxLayout()
        temp_layout.addWidget(QLabel("Target Temp (°C):"))

        self.temp_spinbox = QSpinBox()
        self.temp_spinbox.setMinimum(20)
        self.temp_spinbox.setMaximum(80)
        self.temp_spinbox.setValue(int(self.current_target_temp))
        self.temp_spinbox.setSingleStep(1)
        temp_layout.addWidget(self.temp_spinbox)

        target_layout.addLayout(temp_layout)

        self.temp_slider = QSlider(Qt.Horizontal)
        self.temp_slider.setMinimum(20)
        self.temp_slider.setMaximum(80)
        self.temp_slider.setValue(int(self.current_target_temp))
        self.temp_slider.setTickPosition(QSlider.TicksBelow)
        self.temp_slider.setTickInterval(5)
        target_layout.addWidget(self.temp_slider)

        # Connect slider and spinbox
        self.temp_slider.valueChanged.connect(self.temp_spinbox.setValue)
        self.temp_spinbox.valueChanged.connect(self.temp_slider.setValue)

        # Apply temperature button
        self.apply_temp_button = QPushButton("Apply Target Temperature")
        self.apply_temp_button.clicked.connect(self.apply_target_temperature)
        target_layout.addWidget(self.apply_temp_button)

        layout.addWidget(target_box)

        # PI controller parameters
        pi_box = QGroupBox("PI Controller Parameters")
        pi_layout = QGridLayout(pi_box)

        # Kp (proportional gain)
        pi_layout.addWidget(QLabel("Kp (Proportional Gain):"), 0, 0)
        self.kp_spinbox = QDoubleSpinBox()
        self.kp_spinbox.setRange(0.1, 100.0)
        self.kp_spinbox.setValue(self.kp_value)
        self.kp_spinbox.setSingleStep(0.1)
        self.kp_spinbox.setDecimals(2)
        pi_layout.addWidget(self.kp_spinbox, 0, 1)

        # Ki (integral gain)
        pi_layout.addWidget(QLabel("Ki (Integral Gain):"), 1, 0)
        self.ki_spinbox = QDoubleSpinBox()
        self.ki_spinbox.setRange(0.01, 10.0)
        self.ki_spinbox.setValue(self.ki_value)
        self.ki_spinbox.setSingleStep(0.01)
        self.ki_spinbox.setDecimals(3)
        pi_layout.addWidget(self.ki_spinbox, 1, 1)

        # Apply PI parameters button
        self.apply_pi_button = QPushButton("Apply PI Parameters")
        self.apply_pi_button.clicked.connect(self.apply_pi_parameters)
        pi_layout.addWidget(self.apply_pi_button, 2, 0, 1, 2)

        layout.addWidget(pi_box)

        # Output limit settings
        limits_box = QGroupBox("Output Limits")
        limits_layout = QGridLayout(limits_box)

        # Max PWM
        limits_layout.addWidget(QLabel("Max PWM (%):"), 0, 0)
        self.max_pwm_spinbox = QDoubleSpinBox()
        self.max_pwm_spinbox.setRange(0.0, 100.0)
        self.max_pwm_spinbox.setValue(self.max_pwm_value)
        self.max_pwm_spinbox.setSingleStep(1.0)
        limits_layout.addWidget(self.max_pwm_spinbox, 0, 1)

        # Min PWM
        limits_layout.addWidget(QLabel("Min PWM (%):"), 1, 0)
        self.min_pwm_spinbox = QDoubleSpinBox()
        self.min_pwm_spinbox.setRange(0.0, 50.0)
        self.min_pwm_spinbox.setValue(self.min_pwm_value)
        self.min_pwm_spinbox.setSingleStep(1.0)
        limits_layout.addWidget(self.min_pwm_spinbox, 1, 1)

        # Apply output limits button
        self.apply_limits_button = QPushButton("Apply Output Limits")
        self.apply_limits_button.clicked.connect(self.apply_output_limits)
        limits_layout.addWidget(self.apply_limits_button, 2, 0, 1, 2)

        layout.addWidget(limits_box)

        # Refresh parameters button
        self.refresh_button = QPushButton("Refresh Parameters")
        self.refresh_button.clicked.connect(self.get_current_parameters)
        layout.addWidget(self.refresh_button)

        # Add stretch for layout spacing
        layout.addStretch()

    def process_ros_events(self):
        """Process ROS2 events"""
        # Quick spin to process callbacks
        rclpy.spin_once(self.node, timeout_sec=0.001)

    def temperature_callback(self, msg):
        """Handle temperature data reception"""
        if len(msg.temperature) > self.active_actuator:
            self.current_temp = msg.temperature[self.active_actuator]
            current_time = self.node.get_clock().now().seconds_nanoseconds()[0] - self.start_time

            # Store data for graphing
            self.temperature_data.append(self.current_temp)
            self.target_temp_data.append(self.current_target_temp)
            self.time_data.append(current_time)

            # Keep only the last 600 data points (60 seconds at 10Hz)
            if len(self.time_data) > 600:
                self.temperature_data.pop(0)
                self.target_temp_data.pop(0)
                self.time_data.pop(0)
                if len(self.pwm_data) > 0:
                    self.pwm_data.pop(0)

    def pwm_callback(self, msg):
        """Handle PWM data reception"""
        if len(msg.pwm) > self.active_actuator:
            self.current_pwm = msg.pwm[self.active_actuator]

            # Add to graph data
            if len(self.time_data) > len(self.pwm_data):
                # Fill in missing data
                while len(self.pwm_data) < len(self.time_data) - 1:
                    self.pwm_data.append(self.current_pwm)
                self.pwm_data.append(self.current_pwm)
            elif len(self.time_data) > 0:
                self.pwm_data.append(self.current_pwm)

    def update_gui(self):
        """Update GUI elements (labels) regularly"""
        # Update temperature display
        self.current_temp_label.setText(f"Current Temp: {self.current_temp:.1f} °C (Target: {self.current_target_temp:.1f} °C)")

        # Update PWM display
        self.current_pwm_label.setText(f"Current PWM: {self.current_pwm}%")

    def update_plot(self):
        """Update graph plots (less frequently to improve performance)"""
        # Only update if there's data
        if not self.time_data:
            return

        # Temperature graph
        self.temp_ax.clear()
        self.temp_ax.plot(self.time_data, self.temperature_data, 'r-', label='Current Temp')
        self.temp_ax.plot(self.time_data, self.target_temp_data, 'b--', label='Target Temp')
        self.temp_ax.set_ylabel('Temperature (°C)')
        self.temp_ax.set_title('Actuator Temperature Monitoring')
        self.temp_ax.legend(loc='upper right')
        self.temp_ax.grid(True)

        # PWM graph
        self.pwm_ax.clear()
        if self.pwm_data:
            time_data = self.time_data[:len(self.pwm_data)]
            self.pwm_ax.plot(time_data, self.pwm_data, 'g-', label='PWM Output')
            self.pwm_ax.set_ylim(0, 100)

        self.pwm_ax.set_xlabel('Time (seconds)')
        self.pwm_ax.set_ylabel('PWM (%)')
        self.pwm_ax.legend(loc='upper right')
        self.pwm_ax.grid(True)

        # Show last 60 seconds of data
        if len(self.time_data) > 1 and self.time_data[-1] - self.time_data[0] > 60:
            self.temp_ax.set_xlim(self.time_data[-1] - 60, self.time_data[-1])

        # Set temperature range
        if self.temperature_data and self.target_temp_data:
            min_temp = min(min(self.temperature_data), min(self.target_temp_data)) - 5
            max_temp = max(max(self.temperature_data), max(self.target_temp_data)) + 5
            self.temp_ax.set_ylim(min_temp, max_temp)

        # Update the canvas
        self.temp_fig.tight_layout()
        self.canvas.draw()

    def get_current_parameters(self):
        """Get current parameter values from ROS2 node"""
        # Check if parameter service is ready
        if not self.get_param_client.service_is_ready():
            self.status_label.setText("Status: Parameter service not ready")
            return

        # Create parameter request
        req = GetParameters.Request()
        req.names = ['target_temperature', 'kp', 'ki', 'max_pwm', 'min_pwm']

        self.status_label.setText("Status: Getting parameter values...")

        # Async call
        future = self.get_param_client.call_async(req)
        future.add_done_callback(self.get_parameters_callback)

    def get_parameters_callback(self, future):
        """Handle parameter query response"""
        try:
            response = future.result()

            # Parameter names in request order (matches response values order)
            param_names = ['target_temperature', 'kp', 'ki', 'max_pwm', 'min_pwm']

            # Process response
            for i, name in enumerate(param_names):
                if i < len(response.values):
                    if name == 'target_temperature':
                        self.current_target_temp = response.values[i].double_value
                        self.temp_slider.setValue(int(self.current_target_temp))
                        self.temp_spinbox.setValue(int(self.current_target_temp))

                    elif name == 'kp':
                        self.kp_value = response.values[i].double_value
                        self.kp_spinbox.setValue(self.kp_value)

                    elif name == 'ki':
                        self.ki_value = response.values[i].double_value
                        self.ki_spinbox.setValue(self.ki_value)

                    elif name == 'max_pwm':
                        self.max_pwm_value = response.values[i].double_value
                        self.max_pwm_spinbox.setValue(self.max_pwm_value)

                    elif name == 'min_pwm':
                        self.min_pwm_value = response.values[i].double_value
                        self.min_pwm_spinbox.setValue(self.min_pwm_value)

            self.status_label.setText("Status: Parameters updated successfully")

        except Exception as e:
            self.status_label.setText(f"Status: Failed to get parameters ({str(e)})")

    def apply_target_temperature(self):
        """Apply target temperature value"""
        # Get target temperature value
        target_temp = float(self.temp_spinbox.value())
        self.current_target_temp = target_temp

        # Set the parameter
        self.set_parameter('target_temperature', target_temp, "Target temperature")

    def apply_pi_parameters(self):
        """Apply PI controller parameters"""
        # Get parameter values
        kp = self.kp_spinbox.value()
        ki = self.ki_spinbox.value()

        # Set parameters
        self.set_parameter('kp', kp, "Proportional gain (Kp)")
        self.set_parameter('ki', ki, "Integral gain (Ki)")

        # Update internal variables
        self.kp_value = kp
        self.ki_value = ki

    def apply_output_limits(self):
        """Apply output limit values"""
        # Get parameter values
        max_pwm = self.max_pwm_spinbox.value()
        min_pwm = self.min_pwm_spinbox.value()

        # Set parameters
        self.set_parameter('max_pwm', max_pwm, "Maximum PWM")
        self.set_parameter('min_pwm', min_pwm, "Minimum PWM")

        # Update internal variables
        self.max_pwm_value = max_pwm
        self.min_pwm_value = min_pwm

    def set_parameter(self, name, value, display_name):
        """Set a ROS2 parameter"""
        # Check if parameter service is ready
        if not self.parameter_client.service_is_ready():
            self.status_label.setText(f"Status: Parameter service not ready")
            return

        # Create parameter change request
        req = SetParameters.Request()

        # Create parameter value
        param_value = ParameterValue()
        param_value.type = ParameterType.PARAMETER_DOUBLE
        param_value.double_value = float(value)

        # Create parameter object
        parameter = Parameter()
        parameter.name = name
        parameter.value = param_value

        req.parameters = [parameter]

        # Update status
        self.status_label.setText(f"Status: Changing {display_name}...")

        # Async call
        future = self.parameter_client.call_async(req)
        future.add_done_callback(
            lambda f: self.parameter_callback(f, display_name, value))

    def parameter_callback(self, future, display_name, value):
        """Handle parameter setting response"""
        try:
            response = future.result()
            if all(result.successful for result in response.results):
                self.status_label.setText(f"Status: Changed {display_name} to {value}")
            else:
                error_msgs = [result.reason for result in response.results if not result.successful]
                self.status_label.setText(f"Status: Failed to change {display_name} ({', '.join(error_msgs)})")
        except Exception as e:
            self.status_label.setText(f"Status: Error changing {display_name} ({str(e)})")

    def closeEvent(self, event):
        """Handle window close event"""
        self.gui_timer.stop()
        self.ros_timer.stop()
        self.graph_timer.stop()
        self.node.destroy_node()
        event.accept()

def main(args=None):
    app = QApplication(sys.argv)
    window = TemperatureControlWindow()
    window.show()

    try:
        ret = app.exec_()
    finally:
        rclpy.shutdown()
    return ret

if __name__ == '__main__':
    sys.exit(main())
