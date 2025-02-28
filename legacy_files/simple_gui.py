#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QLabel,
                           QSlider, QPushButton, QSpinBox, QWidget, QDoubleSpinBox,
                           QGroupBox, QGridLayout, QSplitter, QTextEdit)
from PyQt5.QtCore import Qt, QTimer, pyqtSlot
import pyqtgraph as pg
from wearable_robot_interfaces.msg import TemperatureData, ActuatorCommand
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
import csv
from datetime import datetime

# Global ROS2 node
global_node = None

class TemperatureControlGUI(QMainWindow):
    def __init__(self):
        super().__init__()

        # ROS2 node setup (global)
        global global_node
        if global_node is None:
            rclpy.init(args=None)
            global_node = rclpy.create_node('temperature_control_gui')
        self.node = global_node

        # Initialize parameter values
        self.current_target_temp = 50.0
        self.active_actuator = 5  # Default actuator index
        self.current_temp = 0.0
        self.current_pwm = 0
        self.kp_value = 2.0
        self.ki_value = 0.1
        self.max_pwm_value = 100.0
        self.min_pwm_value = 0.0

        # Setup UI
        self.setup_ui()

        # Data storage
        self.x_data = []  # Time data
        self.temp_data = []  # Temperature data
        self.target_data = []  # Target temperature data
        self.pwm_data = []  # PWM data

        # Starting time
        self.start_time = self.node.get_clock().now().seconds_nanoseconds()[0]

        # ROS2 subscriptions
        self.temp_subscription = self.node.create_subscription(
            TemperatureData,
            'temperature_data',
            self.temperature_callback,
            10)

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

        # Setup timer for ROS2 processing
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.process_ros_events)
        self.ros_timer.start(1)  # Process ROS events every 1ms

        # Get current parameters
        self.get_current_parameters()

    def setup_ui(self):
        """Setup the user interface"""
        self.setWindowTitle("Wearable Robot Temperature Control System")
        self.setGeometry(100, 100, 1200, 800)

        # Main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)

        # Graph widgets
        graph_group = QGroupBox("Real-time Monitoring")
        graph_layout = QVBoxLayout(graph_group)

        # Configure PyQtGraph
        pg.setConfigOptions(antialias=False)  # Disable antialiasing for better performance

        # Temperature graph
        self.temp_plot = pg.PlotWidget(title="Temperature")
        self.temp_plot.showGrid(x=False, y=True)
        self.temp_plot.setBackground('w')
        self.temp_plot.addLegend(size=(70, 30))
        self.temp_curve = self.temp_plot.plot(pen=pg.mkPen(width=2, color='r'), name="Current Temp")
        self.target_curve = self.temp_plot.plot(pen=pg.mkPen(width=2, color='b', style=Qt.DashLine), name="Target Temp")

        # PWM graph
        self.pwm_plot = pg.PlotWidget(title="PWM Output")
        self.pwm_plot.showGrid(x=False, y=True)
        self.pwm_plot.setBackground('w')
        self.pwm_plot.addLegend(size=(70, 30))
        self.pwm_curve = self.pwm_plot.plot(pen=pg.mkPen(width=2, color='g'), name="PWM")

        # Link X axes
        self.pwm_plot.setXLink(self.temp_plot)

        # Add to layout
        graph_layout.addWidget(self.temp_plot)
        graph_layout.addWidget(self.pwm_plot)

        # Controls panel
        controls_group = QGroupBox("Control Panel")
        controls_layout = QHBoxLayout(controls_group)

        # Left controls (target temperature)
        left_controls = QGroupBox("Temperature Setting")
        left_layout = QGridLayout(left_controls)

        # Target temperature slider and spinbox
        left_layout.addWidget(QLabel("Target Temperature (°C):"), 0, 0)

        self.temp_spinbox = QSpinBox()
        self.temp_spinbox.setRange(20, 80)
        self.temp_spinbox.setValue(int(self.current_target_temp))
        left_layout.addWidget(self.temp_spinbox, 0, 1)

        self.temp_slider = QSlider(Qt.Horizontal)
        self.temp_slider.setRange(20, 80)
        self.temp_slider.setValue(int(self.current_target_temp))
        self.temp_slider.setTickPosition(QSlider.TicksBelow)
        self.temp_slider.setTickInterval(5)
        left_layout.addWidget(self.temp_slider, 1, 0, 1, 2)

        # Connect slider and spinbox
        self.temp_slider.valueChanged.connect(self.temp_spinbox.setValue)
        self.temp_spinbox.valueChanged.connect(self.temp_slider.setValue)

        # Apply button
        self.apply_temp_btn = QPushButton("Apply Temperature Setting")
        self.apply_temp_btn.clicked.connect(self.apply_target_temperature)
        left_layout.addWidget(self.apply_temp_btn, 2, 0, 1, 2)

        # Middle controls (PI parameters)
        middle_controls = QGroupBox("PI Controller Settings")
        middle_layout = QGridLayout(middle_controls)

        # Kp parameter
        middle_layout.addWidget(QLabel("Kp:"), 0, 0)
        self.kp_spinbox = QDoubleSpinBox()
        self.kp_spinbox.setRange(0.1, 10.0)
        self.kp_spinbox.setValue(self.kp_value)
        self.kp_spinbox.setSingleStep(0.1)
        middle_layout.addWidget(self.kp_spinbox, 0, 1)

        # Ki parameter
        middle_layout.addWidget(QLabel("Ki:"), 1, 0)
        self.ki_spinbox = QDoubleSpinBox()
        self.ki_spinbox.setRange(0.01, 1.0)
        self.ki_spinbox.setValue(self.ki_value)
        self.ki_spinbox.setSingleStep(0.01)
        self.ki_spinbox.setDecimals(3)
        middle_layout.addWidget(self.ki_spinbox, 1, 1)

        # Apply PI parameters button
        self.apply_pi_btn = QPushButton("Apply PI Parameters")
        self.apply_pi_btn.clicked.connect(self.apply_pi_parameters)
        middle_layout.addWidget(self.apply_pi_btn, 2, 0, 1, 2)

        # Right controls (limits and status)
        right_controls = QGroupBox("System Controls")
        right_layout = QGridLayout(right_controls)

        # PWM limits
        right_layout.addWidget(QLabel("Max PWM (%):"), 0, 0)
        self.max_pwm_spinbox = QSpinBox()
        self.max_pwm_spinbox.setRange(0, 100)
        self.max_pwm_spinbox.setValue(int(self.max_pwm_value))
        right_layout.addWidget(self.max_pwm_spinbox, 0, 1)

        right_layout.addWidget(QLabel("Min PWM (%):"), 1, 0)
        self.min_pwm_spinbox = QSpinBox()
        self.min_pwm_spinbox.setRange(0, 50)
        self.min_pwm_spinbox.setValue(int(self.min_pwm_value))
        right_layout.addWidget(self.min_pwm_spinbox, 1, 1)

        # Apply limits button
        self.apply_limits_btn = QPushButton("Apply PWM Limits")
        self.apply_limits_btn.clicked.connect(self.apply_output_limits)
        right_layout.addWidget(self.apply_limits_btn, 2, 0, 1, 2)

        # Add controls to panel
        controls_layout.addWidget(left_controls)
        controls_layout.addWidget(middle_controls)
        controls_layout.addWidget(right_controls)

        # Status panel
        status_group = QGroupBox("System Status")
        status_layout = QHBoxLayout(status_group)

        # System status indicators
        self.current_temp_label = QLabel("Current Temperature: -- °C")
        self.current_temp_label.setStyleSheet("font-size: 14pt; font-weight: bold;")
        status_layout.addWidget(self.current_temp_label)

        self.current_pwm_label = QLabel("Current PWM: --%")
        self.current_pwm_label.setStyleSheet("font-size: 14pt;")
        status_layout.addWidget(self.current_pwm_label)

        self.status_label = QLabel("Status: Ready")
        self.status_label.setStyleSheet("font-size: 12pt;")
        status_layout.addWidget(self.status_label)

        # Data logging panel
        log_group = QGroupBox("Data Management")
        log_layout = QHBoxLayout(log_group)

        # Filename input
        log_layout.addWidget(QLabel("Filename:"))
        self.filename_input = QTextEdit()
        self.filename_input.setMaximumHeight(30)
        self.filename_input.setText("temp_data_")
        log_layout.addWidget(self.filename_input)

        # Data management buttons
        self.save_btn = QPushButton("Save Data")
        self.save_btn.clicked.connect(self.save_data)
        log_layout.addWidget(self.save_btn)

        self.clear_btn = QPushButton("Clear Data")
        self.clear_btn.clicked.connect(self.clear_data)
        log_layout.addWidget(self.clear_btn)

        self.start_btn = QPushButton("Start Graph")
        self.start_btn.clicked.connect(self.start_graph)
        log_layout.addWidget(self.start_btn)

        self.stop_btn = QPushButton("Stop Graph")
        self.stop_btn.clicked.connect(self.stop_graph)
        log_layout.addWidget(self.stop_btn)

        # Add all panels to main layout
        main_layout.addWidget(graph_group, 5)  # Graph takes 5/8 of space
        main_layout.addWidget(controls_group, 1)  # Controls take 1/8 of space
        main_layout.addWidget(status_group, 1)  # Status takes 1/8 of space
        main_layout.addWidget(log_group, 1)  # Logging takes 1/8 of space

        # Graph update timer (not started yet)
        self.graph_timer = QTimer()
        self.graph_timer.timeout.connect(self.update_graph)

        # Status update timer
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_status)
        self.status_timer.start(200)  # Update status every 200ms

    def process_ros_events(self):
        """Process ROS events (callbacks)"""
        rclpy.spin_once(self.node, timeout_sec=0)

    def temperature_callback(self, msg):
        """Handle temperature data"""
        if len(msg.temperature) > self.active_actuator:
            self.current_temp = msg.temperature[self.active_actuator]

            # Only append data when graph is running
            if hasattr(self, 'graph_running') and self.graph_running:
                current_time = self.node.get_clock().now().seconds_nanoseconds()[0] - self.start_time
                self.x_data.append(current_time)
                self.temp_data.append(self.current_temp)
                self.target_data.append(self.current_target_temp)

                # Limit data storage to prevent memory issues
                if len(self.x_data) > 1000:  # Keep only last 1000 points
                    self.x_data.pop(0)
                    self.temp_data.pop(0)
                    self.target_data.pop(0)
                    if len(self.pwm_data) > 0:
                        self.pwm_data.pop(0)

    def pwm_callback(self, msg):
        """Handle PWM data"""
        if len(msg.pwm) > self.active_actuator:
            self.current_pwm = msg.pwm[self.active_actuator]

            # Only append data when graph is running
            if hasattr(self, 'graph_running') and self.graph_running:
                if len(self.x_data) > len(self.pwm_data):
                    self.pwm_data.append(self.current_pwm)

    def start_graph(self):
        """Start graph updates"""
        self.graph_running = True
        self.start_time = self.node.get_clock().now().seconds_nanoseconds()[0]
        self.graph_timer.start(50)  # Update graph every 50ms (20Hz)
        self.status_label.setText("Status: Graph running")

    def stop_graph(self):
        """Stop graph updates"""
        self.graph_running = False
        self.graph_timer.stop()
        self.status_label.setText("Status: Graph stopped")

    def update_graph(self):
        """Update graph with current data"""
        if not self.x_data:  # No data yet
            return

        # Update temperature plot
        self.temp_curve.setData(x=self.x_data, y=self.temp_data)
        self.target_curve.setData(x=self.x_data, y=self.target_data)

        # Update PWM plot if data available
        if self.pwm_data:
            # Account for possible length mismatch
            if len(self.pwm_data) < len(self.x_data):
                x_data = self.x_data[:len(self.pwm_data)]
                self.pwm_curve.setData(x=x_data, y=self.pwm_data)
            else:
                self.pwm_curve.setData(x=self.x_data, y=self.pwm_data)

        # Auto-scroll to show last 60 seconds of data
        if len(self.x_data) > 1:
            latest_time = self.x_data[-1]
            view_min = max(0, latest_time - 60)
            self.temp_plot.setXRange(view_min, latest_time)

    def update_status(self):
        """Update status labels"""
        self.current_temp_label.setText(f"Current Temperature: {self.current_temp:.1f} °C")
        self.current_pwm_label.setText(f"Current PWM: {self.current_pwm}%")

    def save_data(self):
        """Save data to CSV file"""
        if not self.x_data:
            self.status_label.setText("Status: No data to save")
            return

        try:
            # Get filename from input
            filename_base = self.filename_input.toPlainText().strip()
            if not filename_base:
                filename_base = "temp_data_"

            # Add timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"{filename_base}{timestamp}.csv"

            # Create CSV
            with open(filename, 'w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["Time", "Temperature", "Target", "PWM"])

                # Write data, accounting for possibly different lengths
                max_len = len(self.x_data)
                for i in range(max_len):
                    row = [self.x_data[i], self.temp_data[i], self.target_data[i]]

                    # Add PWM if available for this index
                    if i < len(self.pwm_data):
                        row.append(self.pwm_data[i])
                    else:
                        row.append('')

                    writer.writerow(row)

            self.status_label.setText(f"Status: Data saved to {filename}")

        except Exception as e:
            self.status_label.setText(f"Status: Error saving data - {str(e)}")

    def clear_data(self):
        """Clear all stored data"""
        self.x_data = []
        self.temp_data = []
        self.target_data = []
        self.pwm_data = []

        # Reset plots
        self.temp_curve.setData([], [])
        self.target_curve.setData([], [])
        self.pwm_curve.setData([], [])

        self.status_label.setText("Status: Data cleared")

    def get_current_parameters(self):
        """Get current parameters from ROS2 node"""
        # Check if service is ready
        if not self.get_param_client.service_is_ready():
            self.status_label.setText("Status: Parameter service not ready")
            return

        # Create request
        req = GetParameters.Request()
        req.names = ['target_temperature', 'kp', 'ki', 'max_pwm', 'min_pwm']

        # Send request
        self.status_label.setText("Status: Getting parameters...")
        future = self.get_param_client.call_async(req)
        future.add_done_callback(self.get_parameters_callback)

    def get_parameters_callback(self, future):
        """Process parameter response"""
        try:
            response = future.result()

            # Process parameters
            param_names = ['target_temperature', 'kp', 'ki', 'max_pwm', 'min_pwm']
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
                        self.max_pwm_spinbox.setValue(int(self.max_pwm_value))
                    elif name == 'min_pwm':
                        self.min_pwm_value = response.values[i].double_value
                        self.min_pwm_spinbox.setValue(int(self.min_pwm_value))

            self.status_label.setText("Status: Parameters updated")

        except Exception as e:
            self.status_label.setText(f"Status: Error getting parameters - {str(e)}")

    def apply_target_temperature(self):
        """Apply target temperature setting"""
        target_temp = float(self.temp_spinbox.value())
        self.current_target_temp = target_temp
        self.set_parameter('target_temperature', target_temp, "Target temperature")

    def apply_pi_parameters(self):
        """Apply PI controller parameters"""
        kp = self.kp_spinbox.value()
        ki = self.ki_spinbox.value()

        self.set_parameter('kp', kp, "Kp")
        self.set_parameter('ki', ki, "Ki")

        self.kp_value = kp
        self.ki_value = ki

    def apply_output_limits(self):
        """Apply PWM output limits"""
        max_pwm = float(self.max_pwm_spinbox.value())
        min_pwm = float(self.min_pwm_spinbox.value())

        self.set_parameter('max_pwm', max_pwm, "Max PWM")
        self.set_parameter('min_pwm', min_pwm, "Min PWM")

        self.max_pwm_value = max_pwm
        self.min_pwm_value = min_pwm

    def set_parameter(self, name, value, display_name):
        """Set a parameter on the ROS2 node"""
        # Check if service is ready
        if not self.parameter_client.service_is_ready():
            self.status_label.setText(f"Status: Parameter service not ready")
            return

        # Create request
        req = SetParameters.Request()

        # Create parameter
        param_value = ParameterValue()
        param_value.type = ParameterType.PARAMETER_DOUBLE
        param_value.double_value = float(value)

        parameter = Parameter()
        parameter.name = name
        parameter.value = param_value

        req.parameters = [parameter]

        # Send request
        self.status_label.setText(f"Status: Setting {display_name}...")
        future = self.parameter_client.call_async(req)
        future.add_done_callback(
            lambda f: self.parameter_callback(f, display_name, value))

    def parameter_callback(self, future, display_name, value):
        """Process parameter setting response"""
        try:
            response = future.result()
            if all(result.successful for result in response.results):
                self.status_label.setText(f"Status: Set {display_name} to {value}")
            else:
                error_msgs = [result.reason for result in response.results if not result.successful]
                self.status_label.setText(f"Status: Failed to set {display_name} - {', '.join(error_msgs)}")
        except Exception as e:
            self.status_label.setText(f"Status: Error setting {display_name} - {str(e)}")

    def closeEvent(self, event):
        """Handle application close"""
        # Stop timers
        if hasattr(self, 'graph_timer') and self.graph_timer.isActive():
            self.graph_timer.stop()
        if hasattr(self, 'status_timer') and self.status_timer.isActive():
            self.status_timer.stop()
        if hasattr(self, 'ros_timer') and self.ros_timer.isActive():
            self.ros_timer.stop()

        # Note: We don't shut down the ROS node here since it's global
        event.accept()

def main(args=None):
    """Main function"""
    app = QApplication(sys.argv)
    window = TemperatureControlGUI()
    window.show()

    ret = app.exec_()

    # Shutdown ROS2 when application exits
    if global_node is not None:
        global_node.destroy_node()
    rclpy.shutdown()

    return ret

if __name__ == '__main__':
    sys.exit(main())
