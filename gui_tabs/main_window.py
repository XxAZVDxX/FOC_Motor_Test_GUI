# -*- coding: utf-8 -*-

import os
import math
import json
import time
from collections import deque

import numpy as np
import pyqtgraph as pg
from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QTabWidget,
                             QMessageBox, QFileDialog, QLabel, QApplication,
                             QFormLayout, QComboBox, QHBoxLayout, QLineEdit,
                             QPushButton, QDoubleSpinBox, QCheckBox, QSpinBox,
                             QGroupBox, QPlainTextEdit)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont

from protocol import CommandPacket
from comm_backend import SerialBackend, CANBackend, CAN_AVAILABLE
from widgets.motor_preview import MotorPreviewWidget
from widgets.imu_3d_widget import IMU3DWidget
from utils import compute_rotations_and_mod
from config_manager import ConfigManager

# ==================== 互补滤波器（带限幅和衰减） ====================
class ComplementaryFilter:
    def __init__(self, dt=0.02, alpha=0.92, max_delta_deg=5.0):
        self.dt = dt
        self.alpha = alpha
        self.max_delta_rad = math.radians(max_delta_deg)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    def update(self, gx, gy, gz, ax, ay, az):
        # 加速度计计算的姿态（弧度）
        acc_roll = math.atan2(ay, az)
        acc_pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
        # 陀螺仪积分，并限幅
        delta_r = math.radians(gx) * self.dt
        delta_p = math.radians(gy) * self.dt
        delta_y = math.radians(gz) * self.dt
        self.roll  += max(-self.max_delta_rad, min(self.max_delta_rad, delta_r))
        self.pitch += max(-self.max_delta_rad, min(self.max_delta_rad, delta_p))
        self.yaw   += max(-self.max_delta_rad, min(self.max_delta_rad, delta_y))
        # 互补滤波
        self.roll  = self.alpha * self.roll  + (1 - self.alpha) * acc_roll
        self.pitch = self.alpha * self.pitch + (1 - self.alpha) * acc_pitch
        # yaw 依赖陀螺仪，加轻微衰减防止长时间漂移
        self.yaw *= 0.9995
        # 转为四元数
        cy = math.cos(self.yaw * 0.5)
        sy = math.sin(self.yaw * 0.5)
        cp = math.cos(self.pitch * 0.5)
        sp = math.sin(self.pitch * 0.5)
        cr = math.cos(self.roll * 0.5)
        sr = math.sin(self.roll * 0.5)
        q = [
            cr*cp*cy + sr*sp*sy,
            sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy
        ]
        return q

TRIMESH_AVAILABLE = False
try:
    import trimesh
    TRIMESH_AVAILABLE = True
except ImportError:
    pass


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Motor Control GUI")
        self.setGeometry(100, 100, 1400, 900)

        self.comm_backend = None
        self.motor_id = None
        self.detected_ids = []
        self.poll_timer = QTimer()
        self.poll_timer.timeout.connect(self.poll_data)
        self.poll_enabled = False
        self.poll_type = "none"

        # 自动刷新（预览 + 电流）使用双定时器 + 忙标志
        self.preview_timer = QTimer()
        self.preview_timer.timeout.connect(self.request_next_preview)
        self.currents_timer = QTimer()
        self.currents_timer.timeout.connect(self.request_currents)
        self.auto_refresh_enabled = False
        self.is_busy = False
        self.busy_start = 0
        self.busy_timeout = 150
        self.preview_toggle = True

        self.gear_ratio_num = 1.0
        self.gear_ratio_den = 1.0
        self.last_position_deg = 0.0
        self.last_speed_rpm = 0.0

        self.data_history = {
            'time': deque(maxlen=500),
            'Ia': deque(maxlen=500),
            'Ib': deque(maxlen=500),
            'Ic': deque(maxlen=500),
            'Iq': deque(maxlen=500),
            'Id': deque(maxlen=500),
            'speed': deque(maxlen=500),
            'position': deque(maxlen=500),
        }
        self.plot_index = 0

        # IMU 相关 - 自动零偏校准
        self.imu_data = {'ax':0,'ay':0,'az':0,'gx':0,'gy':0,'gz':0,'temp':0}
        self.last_imu_time = time.time()
        self.imu_poll_timer = QTimer()
        self.imu_poll_timer.timeout.connect(self.request_imu_data)
        self.imu_poll_enabled = False
        self.filter = None
        self.calibrating_gyro = False
        self.gyro_bias = [0.0, 0.0, 0.0]
        self.calib_samples = 0
        self.calib_max_samples = 300          # 采集300个样本，约6-10秒
        self.calib_buffer = []

        self.config_manager = ConfigManager("./config")

        # 手动命令缓冲区
        self.manual_response_buffer = []
        self.manual_response_timer = QTimer()
        self.manual_response_timer.setInterval(100)
        self.manual_response_timer.timeout.connect(self.flush_manual_response)
        self.manual_response_timer.start()

        self.init_ui()

    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        tabs = QTabWidget()
        main_layout.addWidget(tabs)

        conn_tab = self.create_connection_tab()
        ctrl_tab = self.create_control_tab()
        pid_tab = self.create_pid_tab()
        data_tab = self.create_data_tab()
        limits_tab = self.create_limits_tab()
        manual_tab = self.create_manual_tab()
        imu_tab = self.create_imu_tab()

        tabs.addTab(conn_tab, "Connection")
        tabs.addTab(ctrl_tab, "Motor Control")
        tabs.addTab(pid_tab, "PID Tuning")
        tabs.addTab(data_tab, "Real-time Data")
        tabs.addTab(limits_tab, "Limits")
        tabs.addTab(manual_tab, "Manual")
        tabs.addTab(imu_tab, "IMU 3D")

        self.status_label = QLabel("Not connected")
        self.statusBar().addWidget(self.status_label)

    # ---------- 创建标签页的函数 ----------
    def create_connection_tab(self):
        widget = QWidget()
        layout = QFormLayout(widget)

        self.interface_combo = QComboBox()
        self.interface_combo.addItems(["Serial (UART/RS485)", "CAN"])
        self.serial_port_combo = QComboBox()
        self.refresh_ports_btn = QPushButton("Refresh")
        self.baudrate_combo = QComboBox()
        self.baudrate_combo.addItems(["9600","19200","38400","57600","115200","2000000"])
        self.can_channel_edit = QLineEdit("PCAN_USBBUS1")
        self.can_bustype_combo = QComboBox()
        self.can_bustype_combo.addItems(["pcan","socketcan","kvaser","ixxat","vector"])
        self.can_bitrate_edit = QLineEdit("500000")
        self.connect_btn = QPushButton("Connect")
        self.detect_btn = QPushButton("Detect Motor ID")
        self.detect_btn.setEnabled(False)
        self.motor_id_label = QLabel("None")

        layout.addRow("Interface:", self.interface_combo)
        port_layout = QHBoxLayout()
        port_layout.addWidget(self.serial_port_combo)
        port_layout.addWidget(self.refresh_ports_btn)
        layout.addRow("Serial Port:", port_layout)
        layout.addRow("Baudrate:", self.baudrate_combo)
        layout.addRow("CAN Channel:", self.can_channel_edit)
        layout.addRow("CAN Bustype:", self.can_bustype_combo)
        layout.addRow("CAN Bitrate:", self.can_bitrate_edit)
        layout.addRow(self.connect_btn)
        layout.addRow(self.detect_btn)
        layout.addRow("Detected Motor ID:", self.motor_id_label)

        self.interface_combo.currentTextChanged.connect(self.update_interface_visibility)
        self.refresh_ports_btn.clicked.connect(self.refresh_serial_ports)
        self.connect_btn.clicked.connect(self.toggle_connection)
        self.detect_btn.clicked.connect(self.detect_motor_id)
        self.update_interface_visibility()
        return widget

    def create_control_tab(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)

        config_group = QGroupBox("Configuration")
        cfg_vlayout = QVBoxLayout()

        # row 1: dropdown + buttons
        cfg_row1 = QHBoxLayout()
        self.config_combo = QComboBox()
        self.refresh_config_btn = QPushButton("Refresh")
        self.load_config_btn = QPushButton("Load")
        self.import_config_btn = QPushButton("Import…")
        self.save_as_btn = QPushButton("Save As…")
        cfg_row1.addWidget(QLabel("Config:"))
        cfg_row1.addWidget(self.config_combo, 1)
        cfg_row1.addWidget(self.refresh_config_btn)
        cfg_row1.addWidget(self.load_config_btn)
        cfg_row1.addWidget(self.import_config_btn)
        cfg_row1.addWidget(self.save_as_btn)
        cfg_vlayout.addLayout(cfg_row1)

        # row 2: status label
        cfg_row2 = QHBoxLayout()
        self.config_status_label = QLabel("No config loaded")
        self.config_status_label.setStyleSheet("color: gray; font-style: italic;")
        cfg_row2.addWidget(self.config_status_label)
        cfg_row2.addStretch()
        cfg_vlayout.addLayout(cfg_row2)

        config_group.setLayout(cfg_vlayout)
        layout.addWidget(config_group)

        self.load_config_list()
        self.refresh_config_btn.clicked.connect(self.load_config_list)
        self.load_config_btn.clicked.connect(self.on_load_config)
        self.import_config_btn.clicked.connect(self.on_import_config)
        self.save_as_btn.clicked.connect(self.on_save_config)

        id_group = QGroupBox("Motor Selection")
        id_layout = QHBoxLayout()
        self.motor_id_combo = QComboBox()
        self.motor_id_combo.addItem("None")
        id_layout.addWidget(QLabel("Motor ID:"))
        id_layout.addWidget(self.motor_id_combo)
        id_group.setLayout(id_layout)
        layout.addWidget(id_group)
        self.motor_id_combo.currentTextChanged.connect(self.on_motor_id_changed)

        mode_group = QGroupBox("Operating Mode")
        mode_layout = QFormLayout()
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["Stop","Self-test","Calibration","Open-loop",
                                  "Current loop","Speed loop","Position loop"])
        self.set_mode_btn = QPushButton("Set")
        self.get_mode_btn = QPushButton("Get")
        mode_layout.addRow("Mode:", self.mode_combo)
        btn_layout = QHBoxLayout()
        btn_layout.addWidget(self.set_mode_btn)
        btn_layout.addWidget(self.get_mode_btn)
        mode_layout.addRow(btn_layout)
        mode_group.setLayout(mode_layout)
        layout.addWidget(mode_group)

        param_group = QGroupBox("Motor Parameters")
        param_layout = QFormLayout()
        self.pole_pair_label = QLabel("---")
        self.offset_label = QLabel("---")
        self.encoder_dir_label = QLabel("---")
        get_params_btn = QPushButton("Get Parameters")
        param_layout.addRow("Pole Pairs:", self.pole_pair_label)
        param_layout.addRow("Zero Offset (°):", self.offset_label)
        param_layout.addRow("Encoder Direction:", self.encoder_dir_label)
        param_layout.addRow(get_params_btn)
        param_group.setLayout(param_layout)
        layout.addWidget(param_group)
        get_params_btn.clicked.connect(self.get_motor_parameters)

        target_group = QGroupBox("Target Values")
        target_layout = QFormLayout()
        self.target_iq = QDoubleSpinBox(); self.target_iq.setRange(-10,10); self.target_iq.setDecimals(3)
        self.target_id = QDoubleSpinBox(); self.target_id.setRange(-10,10); self.target_id.setDecimals(3)
        self.target_speed = QDoubleSpinBox(); self.target_speed.setRange(-5000,5000)
        self.target_position = QDoubleSpinBox(); self.target_position.setRange(-5000,5000)
        self.target_uq = QDoubleSpinBox(); self.target_uq.setRange(-6,6)
        self.target_ud = QDoubleSpinBox(); self.target_ud.setRange(-6,6)
        set_target_btn = QPushButton("Set All")
        get_target_btn = QPushButton("Get All")
        self.get_speed_btn = QPushButton("Get Speed")
        target_layout.addRow("Iq:", self.target_iq)
        target_layout.addRow("Id:", self.target_id)
        target_layout.addRow("Speed (rpm):", self.target_speed)
        target_layout.addRow("Position (deg):", self.target_position)
        target_layout.addRow("Uq:", self.target_uq)
        target_layout.addRow("Ud:", self.target_ud)
        btn_hlay = QHBoxLayout()
        btn_hlay.addWidget(set_target_btn)
        btn_hlay.addWidget(get_target_btn)
        btn_hlay.addWidget(self.get_speed_btn)
        target_layout.addRow(btn_hlay)
        target_group.setLayout(target_layout)
        layout.addWidget(target_group)
        set_target_btn.clicked.connect(self.set_targets)
        get_target_btn.clicked.connect(self.get_targets)
        self.get_speed_btn.clicked.connect(self.get_motor_speed)

        current_group = QGroupBox("Phase Currents")
        curr_layout = QFormLayout()
        self.label_Ia = QLabel("0.000 A"); self.label_Ia.setStyleSheet("color:#FF0000")
        self.label_Ib = QLabel("0.000 A"); self.label_Ib.setStyleSheet("color:#00AA00")
        self.label_Ic = QLabel("0.000 A"); self.label_Ic.setStyleSheet("color:#0000FF")
        curr_layout.addRow("Ia:", self.label_Ia)
        curr_layout.addRow("Ib:", self.label_Ib)
        curr_layout.addRow("Ic:", self.label_Ic)
        current_group.setLayout(curr_layout)
        layout.addWidget(current_group)

        auto_group = QGroupBox("Auto Refresh")
        auto_layout = QHBoxLayout()
        self.auto_refresh_cb = QCheckBox("Enable")
        self.auto_refresh_cb.setChecked(True)
        self.auto_refresh_cb.toggled.connect(self.toggle_auto_refresh)
        self.refresh_interval_spin = QSpinBox()
        self.refresh_interval_spin.setRange(100,5000)
        self.refresh_interval_spin.setValue(1000)
        self.refresh_interval_spin.setSuffix(" ms")
        auto_layout.addWidget(self.auto_refresh_cb)
        auto_layout.addWidget(QLabel("Interval:"))
        auto_layout.addWidget(self.refresh_interval_spin)
        self.preview_auto_cb = QCheckBox("Auto Refresh Preview")
        self.preview_auto_cb.setChecked(True)
        self.preview_auto_cb.toggled.connect(self.toggle_preview_auto_refresh)
        auto_layout.addWidget(self.preview_auto_cb)
        auto_group.setLayout(auto_layout)
        layout.addWidget(auto_group)

        preview_group = QGroupBox("Motor Preview")
        preview_layout = QVBoxLayout()
        gear_layout = QHBoxLayout()
        gear_layout.addWidget(QLabel("Gear Ratio:"))
        self.gear_ratio_edit = QLineEdit("1 : 1")
        self.gear_ratio_edit.textChanged.connect(self.update_gear_ratio)
        gear_layout.addWidget(self.gear_ratio_edit)
        preview_layout.addLayout(gear_layout)

        dial_layout = QHBoxLayout()
        self.motor_preview = MotorPreviewWidget()
        dial_layout.addWidget(self.motor_preview, 1)
        info_layout = QVBoxLayout()
        self.actual_angle_label = QLabel("Actual Angle: --- °")
        self.raw_angle_label = QLabel("Raw Motor Angle: --- °")
        self.total_rotations_label = QLabel("Total rotations: ---")
        self.mod_angle_label = QLabel("Mod angle (0-360°): ---")
        self.speed_label = QLabel("Motor Speed: --- rpm")
        self.speed_label.setFont(QFont("Arial", 10))

        info_layout.addWidget(self.actual_angle_label)
        info_layout.addWidget(self.raw_angle_label)
        info_layout.addWidget(self.total_rotations_label)
        info_layout.addWidget(self.mod_angle_label)
        info_layout.addWidget(self.speed_label)
        info_layout.addStretch()
        dial_layout.addLayout(info_layout, 0)

        preview_layout.addLayout(dial_layout)
        preview_group.setLayout(preview_layout)
        layout.addWidget(preview_group)

        self.set_mode_btn.clicked.connect(self.set_motor_mode)
        self.get_mode_btn.clicked.connect(self.get_motor_mode)
        return widget

    def create_pid_tab(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        self.pid_widgets = {}
        for name in ['Iq','Id','Speed','Position']:
            group = QGroupBox(f"{name} PID")
            form = QFormLayout()
            p = QDoubleSpinBox(); p.setRange(-1000,1000); p.setDecimals(6)
            i = QDoubleSpinBox(); i.setRange(-1000,1000); i.setDecimals(6)
            d = QDoubleSpinBox(); d.setRange(-1000,1000); d.setDecimals(6)
            set_btn = QPushButton("Set")
            get_btn = QPushButton("Get")
            btn_layout = QHBoxLayout()
            btn_layout.addWidget(set_btn)
            btn_layout.addWidget(get_btn)
            form.addRow("P:", p)
            form.addRow("I:", i)
            form.addRow("D:", d)
            form.addRow(btn_layout)
            group.setLayout(form)
            layout.addWidget(group)
            self.pid_widgets[name] = (p,i,d,set_btn,get_btn)
            set_btn.clicked.connect(lambda ch, n=name: self.set_pid(n))
            get_btn.clicked.connect(lambda ch, n=name: self.get_pid(n))
        return widget

    def create_data_tab(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        top_layout = QHBoxLayout()
        self.plot_combo = QComboBox()
        self.plot_combo.addItems(["IaIbIc","IqId","Speed","Position"])
        self.poll_checkbox = QCheckBox("Enable Polling")
        top_layout.addWidget(QLabel("Plot:"))
        top_layout.addWidget(self.plot_combo)
        top_layout.addWidget(self.poll_checkbox)
        layout.addLayout(top_layout)

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setLabel('left', 'Value')
        self.plot_widget.setLabel('bottom', 'Time (samples)')
        self.plot_widget.addLegend()
        self.plot_curves = {}
        layout.addWidget(self.plot_widget)

        save_btn = QPushButton("Save Data to CSV")
        save_btn.clicked.connect(self.save_data)
        layout.addWidget(save_btn)

        self.plot_combo.currentTextChanged.connect(self.change_plot_type)
        self.poll_checkbox.toggled.connect(self.toggle_polling)
        return widget

    def create_limits_tab(self):
        widget = QWidget()
        layout = QFormLayout(widget)
        self.limit_iq_max = QDoubleSpinBox(); self.limit_iq_max.setRange(-100,100)
        self.limit_iq_min = QDoubleSpinBox(); self.limit_iq_min.setRange(-100,100)
        self.limit_id_max = QDoubleSpinBox(); self.limit_id_max.setRange(-100,100)
        self.limit_id_min = QDoubleSpinBox(); self.limit_id_min.setRange(-100,100)
        self.limit_speed_max = QDoubleSpinBox(); self.limit_speed_max.setRange(-10000,10000)
        self.limit_speed_min = QDoubleSpinBox(); self.limit_speed_min.setRange(-10000,10000)
        self.limit_position_max = QDoubleSpinBox(); self.limit_position_max.setRange(-10000,10000)
        self.limit_position_min = QDoubleSpinBox(); self.limit_position_min.setRange(-10000,10000)
        layout.addRow("Iq max:", self.limit_iq_max)
        layout.addRow("Iq min:", self.limit_iq_min)
        layout.addRow("Id max:", self.limit_id_max)
        layout.addRow("Id min:", self.limit_id_min)
        layout.addRow("Speed max:", self.limit_speed_max)
        layout.addRow("Speed min:", self.limit_speed_min)
        layout.addRow("Position max:", self.limit_position_max)
        layout.addRow("Position min:", self.limit_position_min)
        set_btn = QPushButton("Set Limits")
        get_btn = QPushButton("Get Limits")
        layout.addRow(set_btn, get_btn)
        set_btn.clicked.connect(self.set_limits)
        get_btn.clicked.connect(self.get_limits)
        return widget

    def create_manual_tab(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        cmd_group = QGroupBox("Send Command (Hex)")
        cmd_layout = QVBoxLayout()
        self.manual_cmd_edit = QPlainTextEdit()
        self.manual_cmd_edit.setMaximumHeight(100)
        send_btn = QPushButton("Send")
        cmd_layout.addWidget(self.manual_cmd_edit)
        cmd_layout.addWidget(send_btn)
        cmd_group.setLayout(cmd_layout)
        layout.addWidget(cmd_group)

        resp_group = QGroupBox("Response (Raw Hex)")
        resp_layout = QVBoxLayout()
        self.manual_response_text = QPlainTextEdit()
        self.manual_response_text.setReadOnly(True)
        clear_btn = QPushButton("Clear")
        resp_layout.addWidget(self.manual_response_text)
        resp_layout.addWidget(clear_btn)
        resp_group.setLayout(resp_layout)
        layout.addWidget(resp_group)

        send_btn.clicked.connect(self.send_manual_command)
        clear_btn.clicked.connect(lambda: self.manual_response_text.clear())
        return widget

    def create_imu_tab(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)

        ctrl_layout = QHBoxLayout()
        self.imu_poll_cb = QCheckBox("Enable IMU Polling")
        self.imu_poll_interval = QSpinBox()
        self.imu_poll_interval.setRange(10,500)
        self.imu_poll_interval.setValue(50)
        ctrl_layout.addWidget(self.imu_poll_cb)
        ctrl_layout.addWidget(QLabel("Interval (ms):"))
        ctrl_layout.addWidget(self.imu_poll_interval)
        layout.addLayout(ctrl_layout)

        self.imu_3d_view = IMU3DWidget()
        layout.addWidget(self.imu_3d_view, stretch=2)

        model_layout = QHBoxLayout()
        self.model_combo = QComboBox()
        self.model_combo.addItem("Default Cube")
        refresh_model_btn = QPushButton("Refresh")
        browse_btn = QPushButton("Browse")
        reset_btn = QPushButton("Reset")
        model_layout.addWidget(QLabel("3D Model:"))
        model_layout.addWidget(self.model_combo)
        model_layout.addWidget(refresh_model_btn)
        model_layout.addWidget(browse_btn)
        model_layout.addWidget(reset_btn)
        layout.addLayout(model_layout)

        data_group = QGroupBox("IMU Data")
        data_layout = QHBoxLayout()
        left = QVBoxLayout(); left.addWidget(QLabel("Acc (g):"))
        self.label_ax = QLabel("ax: ---")
        self.label_ay = QLabel("ay: ---")
        self.label_az = QLabel("az: ---")
        left.addWidget(self.label_ax); left.addWidget(self.label_ay); left.addWidget(self.label_az)
        mid = QVBoxLayout(); mid.addWidget(QLabel("Gyro (dps):"))
        self.label_gx = QLabel("gx: ---"); self.label_gy = QLabel("gy: ---"); self.label_gz = QLabel("gz: ---")
        mid.addWidget(self.label_gx); mid.addWidget(self.label_gy); mid.addWidget(self.label_gz)
        right = QVBoxLayout(); right.addWidget(QLabel("Orientation (°):"))
        self.label_roll = QLabel("Roll: ---"); self.label_pitch = QLabel("Pitch: ---"); self.label_yaw = QLabel("Yaw: ---")
        right.addWidget(self.label_roll); right.addWidget(self.label_pitch); right.addWidget(self.label_yaw)
        data_layout.addLayout(left); data_layout.addLayout(mid); data_layout.addLayout(right)
        data_group.setLayout(data_layout)
        layout.addWidget(data_group)

        log_group = QGroupBox("IMU Data Log")
        log_layout = QVBoxLayout()
        self.imu_log_text = QPlainTextEdit()
        self.imu_log_text.setReadOnly(True)
        self.imu_log_text.setMaximumBlockCount(1000)
        self.imu_log_text.setFont(QFont("Courier New", 9))
        btn_layout = QHBoxLayout()
        self.clear_log_btn = QPushButton("Clear Log")
        self.save_log_btn = QPushButton("Save Log to CSV")
        self.logging_cb = QCheckBox("Auto Log")
        self.logging_cb.setChecked(True)
        btn_layout.addWidget(self.logging_cb)
        btn_layout.addWidget(self.clear_log_btn)
        btn_layout.addWidget(self.save_log_btn)
        log_layout.addWidget(self.imu_log_text)
        log_layout.addLayout(btn_layout)
        log_group.setLayout(log_layout)
        layout.addWidget(log_group)

        debug_group = QGroupBox("IMU Debug Data")
        debug_layout = QVBoxLayout()
        self.imu_debug_text = QPlainTextEdit()
        self.imu_debug_text.setReadOnly(True)
        self.imu_debug_text.setMaximumHeight(120)
        self.imu_debug_text.setFont(QFont("Courier New", 9))
        copy_btn = QPushButton("Copy Current IMU Data")
        copy_btn.clicked.connect(self.copy_imu_data)
        debug_layout.addWidget(self.imu_debug_text)
        debug_layout.addWidget(copy_btn)
        debug_group.setLayout(debug_layout)
        layout.addWidget(debug_group)

        self.imu_poll_cb.toggled.connect(self.toggle_imu_polling)
        self.imu_poll_interval.valueChanged.connect(self.update_imu_poll_interval)
        refresh_model_btn.clicked.connect(self.scan_asset_models)
        browse_btn.clicked.connect(self.browse_model_file)
        reset_btn.clicked.connect(self.reset_to_cube)
        self.clear_log_btn.clicked.connect(self.clear_imu_log)
        self.save_log_btn.clicked.connect(self.save_imu_log_to_csv)
        self.scan_asset_models()
        return widget

    # ---------- 通信和数据处理 ----------
    def update_interface_visibility(self):
        is_serial = self.interface_combo.currentText() == "Serial (UART/RS485)"
        self.serial_port_combo.setEnabled(is_serial)
        self.refresh_ports_btn.setEnabled(is_serial)
        self.baudrate_combo.setEnabled(is_serial)
        self.can_channel_edit.setEnabled(not is_serial)
        self.can_bustype_combo.setEnabled(not is_serial)
        self.can_bitrate_edit.setEnabled(not is_serial)

    def refresh_serial_ports(self):
        import serial.tools.list_ports
        current = self.serial_port_combo.currentText()
        self.serial_port_combo.clear()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.serial_port_combo.addItems(ports)
        if current in ports:
            self.serial_port_combo.setCurrentText(current)
        elif ports:
            self.serial_port_combo.setCurrentIndex(0)

    def toggle_connection(self):
        if self.comm_backend:
            self.disconnect()
        else:
            self.connect_device()

    def connect_device(self):
        if self.interface_combo.currentText() == "Serial (UART/RS485)":
            port = self.serial_port_combo.currentText()
            if not port:
                QMessageBox.warning(self,"Error","No serial port")
                return
            baud = int(self.baudrate_combo.currentText())
            backend = SerialBackend(port, baud)
        else:
            if not CAN_AVAILABLE:
                QMessageBox.critical(self,"Error","python-can not installed")
                return
            channel = self.can_channel_edit.text()
            bustype = self.can_bustype_combo.currentText()
            bitrate = int(self.can_bitrate_edit.text())
            backend = CANBackend(channel, bustype, bitrate)
        backend.packet_received.connect(self.on_packet_received)
        backend.raw_data_received.connect(self.on_raw_data_received)
        backend.error_occurred.connect(self.on_comm_error)
        backend.start()
        self.comm_backend = backend
        self.connect_btn.setText("Disconnect")
        self.detect_btn.setEnabled(True)
        self.status_label.setText("Connected")

        if self.preview_auto_cb.isChecked():
            self.start_auto_refresh()

    def disconnect(self):
        self.stop_auto_refresh()
        if self.comm_backend:
            try:
                self.comm_backend.packet_received.disconnect(self.on_packet_received)
                self.comm_backend.raw_data_received.disconnect(self.on_raw_data_received)
                self.comm_backend.error_occurred.disconnect(self.on_comm_error)
            except TypeError:
                pass
            self.comm_backend.stop()
            self.comm_backend = None
        self.connect_btn.setText("Connect")
        self.detect_btn.setEnabled(False)
        self.status_label.setText("Not connected")
        self.motor_id = None
        self.motor_id_label.setText("None")
        self.motor_id_combo.clear()
        self.motor_id_combo.addItem("None")
        self.imu_poll_cb.setChecked(False)

    def send_command(self, func2, func3, data1=0, data2=0, data3=0, data4=0, motor_id=0):
        if not self.comm_backend:
            return
        pkt = CommandPacket(func1=0x1A, func2=func2, func3=func3,
                            data1=data1, data2=data2, data3=data3, data4=data4,
                            motor_id=motor_id)
        self.comm_backend.send_packet(pkt)

    def on_packet_received(self, packet):
        if packet.func1 != 0x1A:
            return

        if packet.func2 in (0x30, 0x31, 0x32, 0x33) and self.auto_refresh_enabled:
            self.is_busy = False

        if packet.func2 == 0x00 and packet.func3 == 0x00:
            self.handle_detect_response(packet)
        elif packet.func2 == 0x01:
            self.handle_mode_response(packet)
        elif packet.func2 == 0x02:
            self.handle_pole_pair_response(packet)
        elif packet.func2 == 0x10:
            self.handle_pid_response("Iq", packet)
        elif packet.func2 == 0x11:
            self.handle_pid_response("Id", packet)
        elif packet.func2 == 0x12:
            self.handle_pid_response("Speed", packet)
        elif packet.func2 == 0x13:
            self.handle_pid_response("Position", packet)
        elif packet.func2 == 0x14:
            self.handle_limits_response(packet)
        elif packet.func2 == 0x20:
            self.handle_target_response("Iq", packet)
        elif packet.func2 == 0x21:
            self.handle_target_response("Id", packet)
        elif packet.func2 == 0x22:
            self.handle_target_response("Speed", packet)
        elif packet.func2 == 0x23:
            self.handle_target_response("Position", packet)
        elif packet.func2 == 0x24:
            self.handle_target_response("UqUd", packet)
        elif packet.func2 == 0x30:
            self.handle_iaibic(packet)
        elif packet.func2 == 0x31:
            self.handle_iqid(packet)
        elif packet.func2 == 0x32:
            self.handle_speed(packet)
        elif packet.func2 == 0x33:
            self.handle_position(packet)
        elif packet.func2 == 0x3D:
            self.decode_imu_packet(packet)

    def on_raw_data_received(self, data: bytes):
        hex_str = data.hex().upper()
        spaced = ' '.join(hex_str[i:i+2] for i in range(0, len(hex_str), 2))
        self.manual_response_buffer.append(f"[RX] {spaced}")

    def flush_manual_response(self):
        if not self.manual_response_buffer:
            return
        text = '\n'.join(self.manual_response_buffer)
        self.manual_response_text.appendPlainText(text)
        self.manual_response_buffer.clear()
        scrollbar = self.manual_response_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def detect_motor_id(self):
        self.send_command(0x00, 0x00, motor_id=0xFFFF)

    def handle_detect_response(self, packet):
        ids = []
        for i in range(1,5):
            val = getattr(packet, f'data{i}').as_uint32()
            if (val & 0xFFFF0000) == 0xFFFF0000:
                ids.append(val & 0xFFFF)
            if (val & 0x0000FFFF) != 0x0000FFFF:
                ids.append(val & 0xFFFF)
        if packet.motor_id != 0xFFFF:
            ids.append(packet.motor_id)
        ids = list(set(ids))
        self.detected_ids = ids
        self.motor_id_combo.clear()
        if ids:
            self.motor_id_combo.addItems([str(i) for i in ids])
            self.motor_id = ids[0]
            self.motor_id_label.setText(str(ids[0]))
            QMessageBox.information(self, "Detect", f"Detected IDs: {ids}")
        else:
            QMessageBox.warning(self, "Detect", "No motor found")

    def set_motor_mode(self):
        mode_map = {"Stop":0,"Self-test":1,"Calibration":2,"Open-loop":3,
                    "Current loop":4,"Speed loop":5,"Position loop":6}
        mode = mode_map[self.mode_combo.currentText()]
        mid = self.get_current_motor_id()
        if mid == 0:
            QMessageBox.warning(self,"Warning","No motor ID")
            return
        self.send_command(0x01, 0x01, data1=mode, motor_id=mid)

    def get_motor_mode(self):
        mid = self.get_current_motor_id()
        if mid == 0:
            return
        self.send_command(0x01, 0x00, motor_id=mid)

    def handle_mode_response(self, packet):
        mode = packet.data1.as_uint32()
        names = ["Stop","Self-test","Calibration","Open-loop",
                 "Current loop","Speed loop","Position loop"]
        if 0 <= mode < len(names):
            idx = self.mode_combo.findText(names[mode])
            if idx >=0:
                self.mode_combo.setCurrentIndex(idx)

    def get_motor_parameters(self):
        mid = self.get_current_motor_id()
        if mid == 0:
            return
        self.send_command(0x02, 0x00, motor_id=mid)

    def handle_pole_pair_response(self, packet):
        if packet.func3 == 0x00:
            self.pole_pair_label.setText(str(packet.data1.as_uint32()))
            self.offset_label.setText(f"{packet.data2.as_float():.3f}°")
            self.encoder_dir_label.setText(str(packet.data3.as_uint32()))

    def set_targets(self):
        mid = self.get_current_motor_id()
        if mid == 0:
            return
        self.send_command(0x20,0x01, data1=self.target_iq.value(), motor_id=mid)
        self.send_command(0x21,0x01, data1=self.target_id.value(), motor_id=mid)
        self.send_command(0x22,0x01, data1=self.target_speed.value(), motor_id=mid)
        self.send_command(0x23,0x01, data1=self.target_position.value(), motor_id=mid)
        self.send_command(0x24,0x01, data1=self.target_uq.value(), data2=self.target_ud.value(), motor_id=mid)

    def get_targets(self):
        mid = self.get_current_motor_id()
        if mid == 0:
            return
        self.send_command(0x20,0x00, motor_id=mid)
        self.send_command(0x21,0x00, motor_id=mid)
        self.send_command(0x22,0x00, motor_id=mid)
        self.send_command(0x23,0x00, motor_id=mid)
        self.send_command(0x24,0x00, motor_id=mid)

    def get_motor_speed(self):
        mid = self.get_current_motor_id()
        if mid == 0:
            QMessageBox.warning(self, "Warning", "No motor ID selected")
            return
        self.send_command(0x32, 0x00, motor_id=mid)

    def handle_target_response(self, name, packet):
        if name == "Iq":
            self.target_iq.setValue(packet.data1.as_float())
        elif name == "Id":
            self.target_id.setValue(packet.data1.as_float())
        elif name == "Speed":
            self.target_speed.setValue(packet.data1.as_float())
        elif name == "Position":
            self.target_position.setValue(packet.data1.as_float())
        elif name == "UqUd":
            self.target_uq.setValue(packet.data1.as_float())
            self.target_ud.setValue(packet.data2.as_float())

    def set_pid(self, name):
        mid = self.get_current_motor_id()
        if mid == 0:
            return
        func_map = {"Iq":0x10,"Id":0x11,"Speed":0x12,"Position":0x13}
        p,i,d,_,_ = self.pid_widgets[name]
        self.send_command(func_map[name],0x01, data1=p.value(), data2=i.value(), data3=d.value(), motor_id=mid)

    def get_pid(self, name):
        mid = self.get_current_motor_id()
        if mid == 0:
            return
        func_map = {"Iq":0x10,"Id":0x11,"Speed":0x12,"Position":0x13}
        self.send_command(func_map[name],0x00, motor_id=mid)

    def handle_pid_response(self, name, packet):
        p,i,d,_,_ = self.pid_widgets[name]
        p.setValue(packet.data1.as_float())
        i.setValue(packet.data2.as_float())
        d.setValue(packet.data3.as_float())

    def set_limits(self):
        mid = self.get_current_motor_id()
        if mid == 0:
            return
        self.send_command(0x14,0x01, data1=self.limit_iq_max.value(), data2=self.limit_iq_min.value(),
                          data3=self.limit_id_max.value(), data4=self.limit_id_min.value(), motor_id=mid)
        self.send_command(0x15,0x01, data1=self.limit_speed_max.value(), data2=self.limit_speed_min.value(), motor_id=mid)
        self.send_command(0x16,0x01, data1=self.limit_position_max.value(), data2=self.limit_position_min.value(), motor_id=mid)

    def get_limits(self):
        mid = self.get_current_motor_id()
        if mid == 0:
            return
        self.send_command(0x14,0x00, motor_id=mid)
        self.send_command(0x15,0x00, motor_id=mid)
        self.send_command(0x16,0x00, motor_id=mid)

    def handle_limits_response(self, packet):
        if packet.func2 == 0x14:
            self.limit_iq_max.setValue(packet.data1.as_float())
            self.limit_iq_min.setValue(packet.data2.as_float())
            self.limit_id_max.setValue(packet.data3.as_float())
            self.limit_id_min.setValue(packet.data4.as_float())
        elif packet.func2 == 0x15:
            self.limit_speed_max.setValue(packet.data1.as_float())
            self.limit_speed_min.setValue(packet.data2.as_float())
        elif packet.func2 == 0x16:
            self.limit_position_max.setValue(packet.data1.as_float())
            self.limit_position_min.setValue(packet.data2.as_float())

    def update_gear_ratio(self):
        text = self.gear_ratio_edit.text().strip()
        if ':' in text:
            parts = text.split(':')
            if len(parts)==2:
                try:
                    num = float(parts[0].strip())
                    den = float(parts[1].strip())
                    if den != 0:
                        self.gear_ratio_num = num
                        self.gear_ratio_den = den
                        self.update_preview_angle(self.last_position_deg)
                except:
                    pass

    def update_preview_angle(self, motor_position_deg):
        if self.gear_ratio_den != 0:
            actual_angle = motor_position_deg * (self.gear_ratio_num / self.gear_ratio_den)
        else:
            actual_angle = motor_position_deg
        self.last_position_deg = motor_position_deg
        self.motor_preview.set_angle(actual_angle)
        tot, mod = compute_rotations_and_mod(actual_angle)
        self.actual_angle_label.setText(f"Actual Angle: {actual_angle:.1f}°  (Rot: {tot}, Mod: {mod:.1f}°)")

    def toggle_polling(self, enabled):
        if enabled:
            if self.motor_id is None:
                QMessageBox.warning(self,"Polling","Detect motor ID first")
                self.poll_checkbox.setChecked(False)
                return
            self.poll_enabled = True
            self.poll_timer.start(50)
        else:
            self.poll_enabled = False
            self.poll_timer.stop()

    def change_plot_type(self, plot_type):
        self.poll_type = plot_type.lower()
        self.plot_widget.clear()
        self.plot_curves.clear()
        if self.poll_type == "iaibic":
            self.plot_curves['Ia'] = self.plot_widget.plot(pen='r', name='Ia')
            self.plot_curves['Ib'] = self.plot_widget.plot(pen='g', name='Ib')
            self.plot_curves['Ic'] = self.plot_widget.plot(pen='b', name='Ic')
        elif self.poll_type == "iqid":
            self.plot_curves['Iq'] = self.plot_widget.plot(pen='r', name='Iq')
            self.plot_curves['Id'] = self.plot_widget.plot(pen='b', name='Id')
        elif self.poll_type == "speed":
            self.plot_curves['speed'] = self.plot_widget.plot(pen='r', name='Speed')
        elif self.poll_type == "position":
            self.plot_curves['position'] = self.plot_widget.plot(pen='r', name='Position')

    def poll_data(self):
        if not self.poll_enabled or self.motor_id is None:
            return
        mid = self.motor_id
        if self.poll_type == "iaibic":
            self.send_command(0x30,0x00, motor_id=mid)
        elif self.poll_type == "iqid":
            self.send_command(0x31,0x00, motor_id=mid)
        elif self.poll_type == "speed":
            self.send_command(0x32,0x00, motor_id=mid)
        elif self.poll_type == "position":
            self.send_command(0x33,0x00, motor_id=mid)

    def handle_iaibic(self, packet):
        ia = packet.data1.as_float()
        ib = packet.data2.as_float()
        ic = packet.data3.as_float()
        self.label_Ia.setText(f"{ia:.3f} A")
        self.label_Ib.setText(f"{ib:.3f} A")
        self.label_Ic.setText(f"{ic:.3f} A")
        self.data_history['time'].append(self.plot_index)
        self.data_history['Ia'].append(ia)
        self.data_history['Ib'].append(ib)
        self.data_history['Ic'].append(ic)
        if self.poll_type == "iaibic":
            self.update_plot()

    def handle_iqid(self, packet):
        iq = packet.data1.as_float()
        id_ = packet.data2.as_float()
        self.data_history['time'].append(self.plot_index)
        self.data_history['Iq'].append(iq)
        self.data_history['Id'].append(id_)
        if self.poll_type == "iqid":
            self.update_plot()

    def handle_speed(self, packet):
        speed = packet.data1.as_float()
        self.last_speed_rpm = speed
        if hasattr(self, 'speed_label'):
            self.speed_label.setText(f"Motor Speed: {speed:.1f} rpm")
        self.data_history['time'].append(self.plot_index)
        self.data_history['speed'].append(speed)
        if self.poll_type == "speed":
            self.update_plot()

    def handle_position(self, packet):
        pos = packet.data1.as_float()
        tot, mod = compute_rotations_and_mod(pos)
        self.total_rotations_label.setText(f"Total rotations: {tot}")
        self.mod_angle_label.setText(f"Mod angle (0-360°): {mod:.2f}°")
        self.raw_angle_label.setText(f"Raw Motor Angle: {pos:.1f}°  (Rot: {tot}, Mod: {mod:.1f}°)")
        self.update_preview_angle(pos)
        self.data_history['time'].append(self.plot_index)
        self.data_history['position'].append(pos)
        if self.poll_type == "position":
            self.update_plot()

    def update_plot(self):
        time_arr = np.array(self.data_history['time'])
        if self.poll_type == "iaibic":
            self.plot_curves['Ia'].setData(time_arr, np.array(self.data_history['Ia']))
            self.plot_curves['Ib'].setData(time_arr, np.array(self.data_history['Ib']))
            self.plot_curves['Ic'].setData(time_arr, np.array(self.data_history['Ic']))
        elif self.poll_type == "iqid":
            self.plot_curves['Iq'].setData(time_arr, np.array(self.data_history['Iq']))
            self.plot_curves['Id'].setData(time_arr, np.array(self.data_history['Id']))
        elif self.poll_type == "speed":
            self.plot_curves['speed'].setData(time_arr, np.array(self.data_history['speed']))
        elif self.poll_type == "position":
            self.plot_curves['position'].setData(time_arr, np.array(self.data_history['position']))
        self.plot_index += 1

    def save_data(self):
        filename, _ = QFileDialog.getSaveFileName(self, "Save Data", "", "CSV Files (*.csv)")
        if filename:
            import csv
            max_len = max(len(self.data_history['time']),
                          len(self.data_history['Ia']), len(self.data_history['Ib']),
                          len(self.data_history['Ic']), len(self.data_history['Iq']),
                          len(self.data_history['Id']), len(self.data_history['speed']),
                          len(self.data_history['position']))
            with open(filename, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["Time","Ia","Ib","Ic","Iq","Id","Speed","Position"])
                for i in range(max_len):
                    row = [
                        self.data_history['time'][i] if i < len(self.data_history['time']) else "",
                        self.data_history['Ia'][i] if i < len(self.data_history['Ia']) else "",
                        self.data_history['Ib'][i] if i < len(self.data_history['Ib']) else "",
                        self.data_history['Ic'][i] if i < len(self.data_history['Ic']) else "",
                        self.data_history['Iq'][i] if i < len(self.data_history['Iq']) else "",
                        self.data_history['Id'][i] if i < len(self.data_history['Id']) else "",
                        self.data_history['speed'][i] if i < len(self.data_history['speed']) else "",
                        self.data_history['position'][i] if i < len(self.data_history['position']) else "",
                    ]
                    writer.writerow(row)

    def on_comm_error(self, msg):
        self.status_label.setText(f"Error: {msg}")
        QMessageBox.critical(self, "Comm Error", msg)
        self.disconnect()

    def get_current_motor_id(self):
        text = self.motor_id_combo.currentText()
        if text == "None" or not text:
            return 0
        try:
            return int(text)
        except:
            return 0

    def start_auto_refresh(self):
        if not self.comm_backend or self.auto_refresh_enabled:
            return
        self.auto_refresh_enabled = True
        self.is_busy = False
        self.preview_toggle = True
        self.preview_timer.start(50)
        self.currents_timer.start(300)
        self.request_next_preview()
        self.request_currents()

    def stop_auto_refresh(self):
        self.auto_refresh_enabled = False
        self.preview_timer.stop()
        self.currents_timer.stop()
        self.is_busy = False

    def request_next_preview(self):
        if not self.auto_refresh_enabled or not self.comm_backend:
            return
        mid = self.get_current_motor_id()
        if mid == 0:
            return
        if self.is_busy:
            if time.time() * 1000 - self.busy_start > self.busy_timeout:
                self.is_busy = False
            else:
                return
        if self.preview_toggle:
            self.send_command(0x33, 0x00, motor_id=mid)
        else:
            self.send_command(0x32, 0x00, motor_id=mid)
        self.preview_toggle = not self.preview_toggle
        self.is_busy = True
        self.busy_start = time.time() * 1000

    def request_currents(self):
        if not self.auto_refresh_enabled or not self.comm_backend:
            return
        mid = self.get_current_motor_id()
        if mid == 0:
            return
        if self.is_busy:
            return
        self.send_command(0x30, 0x00, motor_id=mid)
        self.send_command(0x31, 0x00, motor_id=mid)
        self.is_busy = True
        self.busy_start = time.time() * 1000

    def toggle_preview_auto_refresh(self, enabled):
        if enabled and self.comm_backend and self.get_current_motor_id() != 0:
            self.start_auto_refresh()
        else:
            self.stop_auto_refresh()

    def on_motor_id_changed(self):
        if hasattr(self, 'speed_label'):
            self.speed_label.setText("Motor Speed: --- rpm")
        if self.auto_refresh_enabled:
            self.is_busy = False

    def toggle_auto_refresh(self, enabled):
        if enabled:
            self.auto_refresh_timer = QTimer()
            self.auto_refresh_timer.timeout.connect(self.refresh_all_except_mode)
            self.auto_refresh_timer.start(self.refresh_interval_spin.value())
            self.refresh_all_except_mode()
        else:
            if hasattr(self, 'auto_refresh_timer'):
                self.auto_refresh_timer.stop()

    def refresh_all_except_mode(self):
        mid = self.get_current_motor_id()
        if mid == 0:
            return
        self.get_motor_parameters()
        self.get_targets()
        self.get_limits()
        self.get_pid("Iq")
        self.get_pid("Id")
        self.get_pid("Speed")
        self.get_pid("Position")

    # ==================== IMU 相关函数 ====================
    def request_imu_data(self):
        if not self.comm_backend or not self.imu_poll_enabled:
            return
        mid = self.get_current_motor_id()
        if mid == 0 and self.detected_ids:
            mid = self.detected_ids[0]
        if mid == 0:
            return
        self.send_command(0x3D, 0x00, motor_id=mid)

    def decode_imu_packet(self, packet):
        data1 = packet.data1.as_uint32()
        data2 = packet.data2.as_uint32()
        data3 = packet.data3.as_uint32()
        data4 = packet.data4.as_uint32()

        def to_int16(v):
            v = v & 0xFFFF
            return v - 0x10000 if v & 0x8000 else v

        ax_raw = to_int16(data1>>16)
        ay_raw = to_int16(data1 & 0xFFFF)
        az_raw = to_int16(data2>>16)
        gx_raw = to_int16(data2 & 0xFFFF)
        gy_raw = to_int16(data3>>16)
        gz_raw = to_int16(data3 & 0xFFFF)
        temp_raw = to_int16(data4>>16)

        self.ax_raw = ax_raw
        self.ay_raw = ay_raw
        self.az_raw = az_raw
        self.gx_raw = gx_raw
        self.gy_raw = gy_raw
        self.gz_raw = gz_raw

        # 量程系数：根据实际 LSM6DS3TR 配置 ±1000dps，加速度 ±4g
        # 若板端配置不同，请修改以下系数
        GYRO_SCALE = 0.0305   # 1000 dps / 32768
        ACC_SCALE = 0.000122  # 4g / 32768

        ax = ax_raw * ACC_SCALE
        ay = ay_raw * ACC_SCALE
        az = az_raw * ACC_SCALE
        gx = gx_raw * GYRO_SCALE
        gy = gy_raw * GYRO_SCALE
        gz = gz_raw * GYRO_SCALE
        temp = temp_raw / 256.0 + 25.0

        # 坐标系映射（若旋转方向不对，请调整此处）
        # 默认不映射，可根据需要取消注释
        # ax_gui = -ay
        # ay_gui =  az
        # az_gui = -ax
        # gx_gui = -gy
        # gy_gui =  gz
        # gz_gui = -gx
        ax_gui = ax
        ay_gui = ay
        az_gui = az
        gx_gui = gx
        gy_gui = gy
        gz_gui = gz

        self.imu_data.update({'ax':ax, 'ay':ay, 'az':az, 'gx':gx, 'gy':gy, 'gz':gz, 'temp':temp})
        self.label_ax.setText(f"ax: {ax:.3f} g")
        self.label_ay.setText(f"ay: {ay:.3f} g")
        self.label_az.setText(f"az: {az:.3f} g")
        self.label_gx.setText(f"gx: {gx:.1f} dps")
        self.label_gy.setText(f"gy: {gy:.1f} dps")
        self.label_gz.setText(f"gz: {gz:.1f} dps")

        # 自动零偏校准
        if self.calibrating_gyro:
            self.calib_buffer.append((gx_gui, gy_gui, gz_gui))
            self.calib_samples += 1
            if self.calib_samples >= self.calib_max_samples:
                avg_x = sum(v[0] for v in self.calib_buffer) / self.calib_samples
                avg_y = sum(v[1] for v in self.calib_buffer) / self.calib_samples
                avg_z = sum(v[2] for v in self.calib_buffer) / self.calib_samples
                self.gyro_bias = [avg_x, avg_y, avg_z]
                self.calibrating_gyro = False
                print(f"[IMU] Calibration done. Bias: {avg_x:.2f}, {avg_y:.2f}, {avg_z:.2f} dps")
                self.status_label.setText(f"IMU ready (bias: {avg_x:.1f}, {avg_y:.1f}, {avg_z:.1f})")
                self.filter = ComplementaryFilter(dt=0.02, alpha=0.92, max_delta_deg=5.0)
                self.calib_buffer.clear()
            else:
                self.status_label.setText(f"Calibrating IMU... {self.calib_samples}/{self.calib_max_samples}")
            return

        # 减去零偏
        gx_cal = gx_gui - self.gyro_bias[0]
        gy_cal = gy_gui - self.gyro_bias[1]
        gz_cal = gz_gui - self.gyro_bias[2]

        current_time = time.time()
        dt = current_time - self.last_imu_time
        if dt <= 0.001 or dt > 0.1:
            dt = 0.02
        self.last_imu_time = current_time

        if self.filter is None:
            return

        self.filter.dt = dt
        q = self.filter.update(gx_cal, gy_cal, gz_cal, ax_gui, ay_gui, az_gui)

        roll_deg = math.degrees(self.filter.roll)
        pitch_deg = math.degrees(self.filter.pitch)
        yaw_deg = math.degrees(self.filter.yaw)
        self.label_roll.setText(f"Roll: {roll_deg:.1f}°")
        self.label_pitch.setText(f"Pitch: {pitch_deg:.1f}°")
        self.label_yaw.setText(f"Yaw: {yaw_deg:.1f}°")

        self.imu_3d_view.set_orientation_quat(q)

        self.update_imu_debug_text()

        if self.logging_cb.isChecked():
            self.log_imu_data(ax, ay, az, gx, gy, gz, roll_deg, pitch_deg, yaw_deg)

    def update_imu_debug_text(self):
        if not hasattr(self, 'imu_debug_text'):
            return
        text = (f"Timestamp: {time.strftime('%H:%M:%S')}\n"
                f"Accel (g): ax={self.imu_data['ax']:.4f}, ay={self.imu_data['ay']:.4f}, az={self.imu_data['az']:.4f}\n"
                f"Gyro raw (dps): {self.imu_data['gx']:.1f}, {self.imu_data['gy']:.1f}, {self.imu_data['gz']:.1f}\n"
                f"Gyro bias (dps): {self.gyro_bias[0]:.1f}, {self.gyro_bias[1]:.1f}, {self.gyro_bias[2]:.1f}\n"
                f"Gyro cal (dps): {self.imu_data['gx']-self.gyro_bias[0]:.1f}, {self.imu_data['gy']-self.gyro_bias[1]:.1f}, {self.imu_data['gz']-self.gyro_bias[2]:.1f}\n"
                f"Temperature: {self.imu_data['temp']:.1f}°C\n"
                f"Orientation (deg): roll={self.label_roll.text().split(':')[1].strip()}, "
                f"pitch={self.label_pitch.text().split(':')[1].strip()}, "
                f"yaw={self.label_yaw.text().split(':')[1].strip()}\n"
                f"Raw LSBs: ax={getattr(self,'ax_raw',0)}, ay={getattr(self,'ay_raw',0)}, az={getattr(self,'az_raw',0)}, "
                f"gx={getattr(self,'gx_raw',0)}, gy={getattr(self,'gy_raw',0)}, gz={getattr(self,'gz_raw',0)}")
        self.imu_debug_text.setPlainText(text)

    def copy_imu_data(self):
        text = self.imu_debug_text.toPlainText()
        if text:
            QApplication.clipboard().setText(text)
            self.status_label.setText("IMU data copied to clipboard")

    def clear_imu_log(self):
        self.imu_log_text.clear()

    def save_imu_log_to_csv(self):
        filename, _ = QFileDialog.getSaveFileName(self, "Save IMU Log", "", "CSV Files (*.csv)")
        if filename:
            try:
                with open(filename, 'w') as f:
                    f.write(self.imu_log_text.toPlainText())
                QMessageBox.information(self, "Saved", f"Log saved to {filename}")
            except Exception as e:
                QMessageBox.critical(self, "Error", str(e))

    def log_imu_data(self, ax, ay, az, gx, gy, gz, roll, pitch, yaw):
        timestamp = time.strftime("%H:%M:%S") + f".{int(time.time()*1000)%1000:03d}"
        log_line = (f"{timestamp}, {ax:.3f}, {ay:.3f}, {az:.3f}, "
                    f"{gx:.1f}, {gy:.1f}, {gz:.1f}, "
                    f"{roll:.1f}, {pitch:.1f}, {yaw:.1f}")
        self.imu_log_text.appendPlainText(log_line)
        scrollbar = self.imu_log_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def toggle_imu_polling(self, enabled):
        self.imu_poll_enabled = enabled
        if enabled:
            self.calibrating_gyro = True
            self.calib_samples = 0
            self.calib_buffer = []
            self.gyro_bias = [0.0, 0.0, 0.0]
            self.filter = None
            self.status_label.setText("IMU calibrating... Keep device still")
            self.imu_poll_timer.start(self.imu_poll_interval.value())
            self.request_imu_data()
        else:
            self.imu_poll_timer.stop()
            self.calibrating_gyro = False
            self.status_label.setText("IMU polling stopped")

    def update_imu_poll_interval(self):
        if self.imu_poll_enabled:
            self.imu_poll_timer.start(self.imu_poll_interval.value())

    # ---------- 3D 模型相关 ----------
    def scan_asset_models(self):
        import glob
        self.model_combo.blockSignals(True)
        current = self.model_combo.currentText()
        self.model_combo.clear()
        self.model_combo.addItem("Default Cube")
        asset_dir = "./asset"
        if not os.path.exists(asset_dir):
            os.makedirs(asset_dir)
        for ext in ('.stl','.obj','.ply','.step','.stp'):
            for f in glob.glob(os.path.join(asset_dir, f"*{ext}")):
                self.model_combo.addItem(f)
        idx = self.model_combo.findText(current)
        if idx>=0:
            self.model_combo.setCurrentIndex(idx)
        self.model_combo.blockSignals(False)

    def browse_model_file(self):
        filepath, _ = QFileDialog.getOpenFileName(self, "Select 3D Model", "./asset", "3D Models (*.stl *.obj *.ply *.step *.stp)")
        if filepath:
            self.load_model_to_view(filepath)
            self.model_combo.addItem(filepath)
            self.model_combo.setCurrentText(filepath)

    def reset_to_cube(self):
        self.imu_3d_view.set_default_cube()
        self.model_combo.setCurrentText("Default Cube")

    def load_model_to_view(self, filepath):
        if not TRIMESH_AVAILABLE:
            QMessageBox.critical(self, "Missing Library", "trimesh not installed")
            return
        if not self.imu_3d_view.load_model_from_file(filepath):
            QMessageBox.warning(self, "Load Failed", f"Failed to load {filepath}")

    def send_manual_command(self):
        if not self.comm_backend:
            QMessageBox.warning(self, "Error", "Not connected")
            return
        hex_str = self.manual_cmd_edit.toPlainText().strip()
        if not hex_str:
            return
        hex_str = hex_str.replace(' ', '').replace('\n', '')
        try:
            data = bytes.fromhex(hex_str)
        except ValueError:
            QMessageBox.warning(self, "Error", "Invalid hex string")
            return
        self.comm_backend.send_raw(data)
        hex_repr = data.hex().upper()
        spaced = ' '.join(hex_repr[i:i+2] for i in range(0, len(hex_repr), 2))
        self.manual_response_buffer.append(f"[TX] {spaced}")

    def load_config_list(self):
        """Refresh the config dropdown from the config directory."""
        self.config_combo.clear()
        files = self.config_manager.list_configs()
        for f in files:
            self.config_combo.addItem(f)
        # auto-select the currently active config if it exists in the list
        if self.config_manager.current_name != "Unsaved":
            idx = self.config_combo.findText(self.config_manager.current_name)
            if idx >= 0:
                self.config_combo.setCurrentIndex(idx)
                return
        if files:
            self.config_combo.setCurrentIndex(0)

    def _update_config_status(self, success=True, message=None):
        """Update the config status label."""
        if message:
            text = message
            color = "green" if success else "red"
        else:
            name = self.config_manager.current_name
            text = f"Active: {name}"
            color = "green"
        self.config_status_label.setText(text)
        self.config_status_label.setStyleSheet(f"color: {color}; font-style: italic;")

    def on_load_config(self):
        """Load the selected config from the dropdown and apply to GUI."""
        file = self.config_combo.currentText()
        if not file:
            QMessageBox.warning(self, "No Config", "No config file selected.")
            return
        path = os.path.join(self.config_manager.config_dir, file)
        try:
            self.config_manager.load_and_apply(self, path)
            self._update_config_status(True)
            QMessageBox.information(self, "Config", f"Loaded: {file}")
        except Exception as e:
            self._update_config_status(False, f"Failed to load: {e}")
            QMessageBox.critical(self, "Error", str(e))

    def on_import_config(self):
        """Import a config file from anywhere on disk into the config dir,
        then load and apply it."""
        src, _ = QFileDialog.getOpenFileName(
            self, "Import Config", "",
            "JSON Files (*.json);;All Files (*)")
        if not src:
            return
        try:
            dst = self.config_manager.import_file(src)
            self.config_manager.load_and_apply(self, dst)
            self.load_config_list()
            # select the newly imported file
            imported_name = os.path.basename(dst)
            idx = self.config_combo.findText(imported_name)
            if idx >= 0:
                self.config_combo.setCurrentIndex(idx)
            self._update_config_status(True)
            QMessageBox.information(self, "Imported",
                                    f"Imported and loaded: {imported_name}")
        except Exception as e:
            self._update_config_status(False, f"Import failed: {e}")
            QMessageBox.critical(self, "Error", str(e))

    def on_save_config(self):
        """Save current GUI settings as a new config file, then set it as active."""
        default_dir = self.config_manager.config_dir
        filename, _ = QFileDialog.getSaveFileName(
            self, "Save Config As", default_dir, "JSON (*.json)")
        if not filename:
            return
        try:
            self.config_manager.save_current(self, filename)
            self.load_config_list()
            # auto-select the newly saved file
            saved_name = os.path.basename(filename)
            idx = self.config_combo.findText(saved_name)
            if idx >= 0:
                self.config_combo.setCurrentIndex(idx)
            self._update_config_status(True)
            QMessageBox.information(self, "Saved", f"Saved to {saved_name}")
        except Exception as e:
            self._update_config_status(False, f"Save failed: {e}")
            QMessageBox.critical(self, "Error", str(e))