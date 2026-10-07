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
                             QGroupBox, QPlainTextEdit, QAction, QActionGroup,
                             QScrollArea, QFrame)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont

import i18n
import settings
import theme
from i18n import tr

from protocol import CommandPacket
from comm_backend import SerialBackend, CANBackend, CAN_AVAILABLE
from widgets.motor_preview import MotorPreviewWidget
from widgets.imu_3d_widget import IMU3DWidget
from utils import compute_rotations_and_mod
from config_manager import ConfigManager

# 界面语言/主题切换时，这些字符串通过 tr() 取译文，键为英文原文
MODE_ITEMS = [
    (0, "Stop"),
    (1, "Self-test"),
    (2, "Calibration"),
    (3, "Open-loop"),
    (4, "Current loop"),
    (5, "Speed loop"),
    (6, "Position loop"),
]

INTERFACE_ITEMS = [
    ("serial", "Serial (UART/RS485)"),
    ("can", "CAN"),
]

DEFAULT_CUBE = "Default Cube"

TAB_KEYS = ["Connection", "Motor Control", "PID Tuning", "Real-time Data",
            "Limits", "Manual", "IMU 3D"]

# 各类轮询各自归属的标签页，只有该页可见时才允许请求数据
TAB_MOTOR_CONTROL = TAB_KEYS.index("Motor Control")
TAB_REALTIME_DATA = TAB_KEYS.index("Real-time Data")
TAB_IMU = TAB_KEYS.index("IMU 3D")

# ==================== 互补滤波器（带限幅和衰减） ====================
class ComplementaryFilter:
    def __init__(self, dt=0.02, alpha=0.92, max_rate_dps=1000.0, accel_gate_dps=20.0):
        self.dt = dt
        self.alpha = alpha
        # 积分限幅按角速度而不是按帧，换轮询周期时行为保持一致
        self.max_rate_rad = math.radians(max_rate_dps)
        # 只有角速度低于该阈值时才用加速度计修正姿态
        self.accel_gate_dps = accel_gate_dps
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    def update(self, gx, gy, gz, ax, ay, az):
        # 加速度计计算的姿态（弧度）
        acc_roll = math.atan2(ay, az)
        acc_pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
        # 陀螺仪积分，并限幅
        cap = self.max_rate_rad * self.dt
        delta_r = math.radians(gx) * self.dt
        delta_p = math.radians(gy) * self.dt
        delta_y = math.radians(gz) * self.dt
        self.roll  += max(-cap, min(cap, delta_r))
        self.pitch += max(-cap, min(cap, delta_p))
        self.yaw   += max(-cap, min(cap, delta_y))
        # 互补滤波：转动时加速度计读数含离心力，若每帧都融合会把姿态拉回，
        # 因此仅在角速度很低（接近静止）时才用它修正 roll/pitch
        if math.hypot(gx, gy) < self.accel_gate_dps:
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
    def __init__(self, initial_settings=None):
        super().__init__()
        if initial_settings is None:
            initial_settings = settings.load()
        self.current_theme = initial_settings["theme"]
        tr.set_language(initial_settings["language"])

        self.setWindowTitle(tr("Motor Control GUI"))
        self.setGeometry(100, 100, 1400, 900)
        self._clamp_to_screen()

        # 语言/主题切换时需要重新赋值的文本，集中保存便于 retranslate_ui()
        self._text_bindings = []
        self._combo_specs = {}

        self.comm_backend = None
        self.motor_id = None
        self.detected_ids = []
        # 下面几个 *_enabled 标志表示「当前是否真的在轮询」，而不是用户的勾选状态。
        # 勾选状态由各自的复选框保存，两者由 _apply_polling_gates() 统一同步，
        # 因此离开对应标签页时轮询会暂停，切回来再自动恢复。
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

        # 参数自动刷新（电机控制页），同样只在当前页可见时运行
        self.auto_refresh_timer = QTimer()
        self.auto_refresh_timer.timeout.connect(self.refresh_all_except_mode)
        self.auto_refresh_timer_running = False
        self.auto_refresh_interval_ms = 1000

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
        self.last_roll_deg = 0.0
        self.last_pitch_deg = 0.0
        self.last_yaw_deg = 0.0
        self.calib_samples = 0
        self.calib_max_samples = 100          # 100 个样本，约 2-5 秒
        self.calib_min_samples = 30           # 至少累计这么多静止样本才收敛
        self.calib_motion_dps = 8.0           # 超过该角速度视为在动，丢弃该样本
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
        self.create_menu_bar()

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(6)
        self.tabs = QTabWidget()
        main_layout.addWidget(self.tabs)

        # 内容过高的标签页放入滚动区域，避免小屏幕上控件不可达
        tab_factories = [
            (self.create_connection_tab, False),
            (self.create_control_tab, True),
            (self.create_pid_tab, True),
            (self.create_data_tab, False),
            (self.create_limits_tab, False),
            (self.create_manual_tab, False),
            (self.create_imu_tab, False),
        ]
        for i, (factory, force_scroll) in enumerate(tab_factories):
            self.tabs.addTab(self._wrap_scrollable(factory(), force_scroll), tr(TAB_KEYS[i]))

        # 切页时重新评估门控，只让当前标签页继续请求数据
        self.tabs.currentChanged.connect(self.on_tab_changed)

        self.status_label = QLabel(tr("Not connected"))
        self.statusBar().addWidget(self.status_label)

        self.apply_theme(self.current_theme)

    # ---------- 菜单栏 ----------
    def _wrap_scrollable(self, widget, force=False):
        """把过高的标签页放进滚动区域，保证小屏幕下也能访问全部控件。

        force=True 用于已知必然超高的标签页（其高度依赖运行时字体度量，
        不适合在布局前判断）。
        """
        if not force and widget.minimumSizeHint().height() <= 700:
            return widget
        scroll = QScrollArea()
        scroll.setWidget(widget)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.viewport().setAutoFillBackground(False)
        return scroll

    def _clamp_to_screen(self):
        """窗口按内容最小尺寸展开，屏幕装不下时自动收缩，避免标题栏跑出屏幕。"""
        screen = QApplication.primaryScreen()
        if screen is None:
            return
        avail = screen.availableGeometry()
        width = min(self.width(), avail.width())
        height = min(self.height(), avail.height())
        if width != self.width() or height != self.height():
            self.resize(width, height)
        self.move(max(avail.left(), min(self.x(), avail.right() - width)),
                  max(avail.top(), min(self.y(), avail.bottom() - height)))

    def create_menu_bar(self):
        view_menu = self.menuBar().addMenu(tr("&View"))
        self._view_menu = view_menu

        theme_menu = view_menu.addMenu(tr("&Theme"))
        self.theme_group = QActionGroup(self)
        self.theme_group.setExclusive(True)
        for code, name in theme.theme_names():
            action = QAction(tr(name), self, checkable=True)
            action.setData(code)
            action.setChecked(code == self.current_theme)
            action.triggered.connect(lambda _checked, c=code: self.set_theme(c))
            self.theme_group.addAction(action)
            theme_menu.addAction(action)
            self._text_bindings.append((action, name, "setText"))

        view_menu.addSeparator()

        lang_menu = view_menu.addMenu(tr("&Language"))
        self.lang_group = QActionGroup(self)
        self.lang_group.setExclusive(True)
        for code, name in i18n.LANGUAGES:
            action = QAction(name, self, checkable=True)
            action.setData(code)
            action.setChecked(code == tr.language)
            action.triggered.connect(lambda _checked, c=code: self.set_language(c))
            self.lang_group.addAction(action)
            lang_menu.addAction(action)

        help_menu = self.menuBar().addMenu(tr("&Help"))
        about_action = QAction(tr("About"), self)
        about_action.triggered.connect(self.show_about)
        help_menu.addAction(about_action)

        # 语言切换后需要重建菜单文本
        self._text_bindings.append((self.menuBar().actions()[0], "&View", "setText"))
        self._text_bindings.append((self.menuBar().actions()[1], "&Help", "setText"))
        self._theme_menu = theme_menu
        self._lang_menu = lang_menu
        self._about_action = about_action

    def show_about(self):
        QMessageBox.about(self, tr("About"), tr("About Text"))

    # ---------- 文本绑定 ----------
    def _label(self, key):
        """创建随语言切换自动更新的 QLabel。"""
        label = QLabel(tr(key))
        self._text_bindings.append((label, key, "setText"))
        return label

    def _bind_text(self, widget, key, setter="setText"):
        """注册一个在语言切换时需要重新设置文本的控件。"""
        getattr(widget, setter)(tr(key))
        self._text_bindings.append((widget, key, setter))
        return widget

    def _fill_combo(self, combo, entries, keep=True):
        """用 itemData 填充下拉框，逻辑判断只依赖 itemData 而非显示文本。"""
        current = combo.currentData() if keep else None
        combo.blockSignals(True)
        combo.clear()
        for data, key in entries:
            combo.addItem(tr(key), data)
        idx = combo.findData(current)
        combo.setCurrentIndex(idx if idx >= 0 else 0)
        combo.blockSignals(False)
        combo.setProperty("_i18n_entries", entries)

    # ---------- 主题 / 语言切换 ----------
    def apply_theme(self, theme_name):
        app = QApplication.instance()
        self.colors = theme.apply_theme(app, theme_name)
        theme.apply_plot_theme(self.plot_widget, theme_name)
        self.motor_preview.set_colors(self.colors)
        self.imu_3d_view.setBackgroundColor(self.colors["gl_bg"])
        self.imu_3d_view.update()
        self.label_Ia.setStyleSheet("color: %s;" % self.colors["phase_a"])
        self.label_Ib.setStyleSheet("color: %s;" % self.colors["phase_b"])
        self.label_Ic.setStyleSheet("color: %s;" % self.colors["phase_c"])
        self._refresh_status_color()
        if self.config_manager.current_path:
            self._update_config_status(True)
        else:
            self.config_status_label.setText(tr("No config loaded"))
            self.config_status_label.setStyleSheet(
                "color: %s; font-style: italic;" % self.colors["status_muted"])

    def _refresh_status_color(self):
        self.status_label.setStyleSheet(
            "color: %s; padding: 1px 4px;" % self.colors["status_muted"])

    def set_theme(self, theme_name):
        self.current_theme = theme_name
        self.apply_theme(theme_name)
        self._persist_settings()

    def set_language(self, language_code):
        if not tr.set_language(language_code):
            return
        self.retranslate_ui()
        self._persist_settings()

    def _persist_settings(self):
        settings.save({"theme": self.current_theme, "language": tr.language})

    def retranslate_ui(self):
        """语言切换后重新设置所有界面文本（不重建控件）。"""
        self.setWindowTitle(tr("Motor Control GUI"))
        self._theme_menu.setTitle(tr("&Theme"))
        self._lang_menu.setTitle(tr("&Language"))
        self._about_action.setText(tr("About"))
        for widget, key, setter in self._text_bindings:
            getattr(widget, setter)(tr(key))
        for i, key in enumerate(TAB_KEYS):
            self.tabs.setTabText(i, tr(key))
        for combo in self.findChildren(QComboBox):
            entries = combo.property("_i18n_entries")
            if entries:
                self._fill_combo(combo, entries)
                if combo is self.mode_combo:
                    self.handle_mode_response(combo.currentData())
        self.plot_widget.setLabel('left', tr("Value"))
        self.plot_widget.setLabel('bottom', tr("Time (samples)"))
        self.refresh_interval_spin.setSuffix(tr(" ms"))
        # 带 itemData 的下拉框：显示文本需要单独刷新
        if self.motor_id_combo.count():
            self.motor_id_combo.setItemText(0, tr("None"))
        cube_idx = self.model_combo.findData(DEFAULT_CUBE)
        if cube_idx >= 0:
            self.model_combo.setItemText(cube_idx, tr(DEFAULT_CUBE))

    # ---------- 创建标签页的函数 ----------
    def create_connection_tab(self):
        widget = QWidget()
        layout = QFormLayout(widget)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        self.interface_combo = QComboBox()
        self._fill_combo(self.interface_combo, INTERFACE_ITEMS, keep=False)
        self.serial_port_combo = QComboBox()
        self.refresh_ports_btn = self._bind_text(QPushButton(), "Refresh")
        self.baudrate_combo = QComboBox()
        self.baudrate_combo.addItems(["9600","19200","38400","57600","115200","2000000"])
        self.can_channel_edit = QLineEdit("PCAN_USBBUS1")
        self.can_bustype_combo = QComboBox()
        self.can_bustype_combo.addItems(["pcan","socketcan","kvaser","ixxat","vector"])
        self.can_bitrate_edit = QLineEdit("500000")
        self.connect_btn = self._bind_text(QPushButton(), "Connect")
        self.detect_btn = self._bind_text(QPushButton(), "Detect Motor ID")
        self.detect_btn.setEnabled(False)
        self.motor_id_label = self._label("None")

        layout.addRow(self._label("Interface:"), self.interface_combo)
        port_layout = QHBoxLayout()
        port_layout.addWidget(self.serial_port_combo)
        port_layout.addWidget(self.refresh_ports_btn)
        layout.addRow(self._label("Serial Port:"), port_layout)
        layout.addRow(self._label("Baudrate:"), self.baudrate_combo)
        layout.addRow(self._label("CAN Channel:"), self.can_channel_edit)
        layout.addRow(self._label("CAN Bustype:"), self.can_bustype_combo)
        layout.addRow(self._label("CAN Bitrate:"), self.can_bitrate_edit)
        layout.addRow(self.connect_btn)
        layout.addRow(self.detect_btn)
        layout.addRow(self._label("Detected Motor ID:"), self.motor_id_label)

        self.interface_combo.currentIndexChanged.connect(self.update_interface_visibility)
        self.refresh_ports_btn.clicked.connect(self.refresh_serial_ports)
        self.connect_btn.clicked.connect(self.toggle_connection)
        self.detect_btn.clicked.connect(self.detect_motor_id)
        self.update_interface_visibility()
        return widget

    def create_control_tab(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        config_group = QGroupBox(tr("Configuration"))
        self._text_bindings.append((config_group, "Configuration", "setTitle"))
        cfg_vlayout = QVBoxLayout()
        cfg_vlayout.setSpacing(6)

        # row 1: dropdown + buttons
        cfg_row1 = QHBoxLayout()
        cfg_row1.setSpacing(6)
        self.config_combo = QComboBox()
        self.refresh_config_btn = self._bind_text(QPushButton(), "Refresh")
        self.load_config_btn = self._bind_text(QPushButton(), "Load")
        self.import_config_btn = self._bind_text(QPushButton(), "Import…")
        self.save_as_btn = self._bind_text(QPushButton(), "Save As…")
        cfg_row1.addWidget(self._label("Config:"))
        cfg_row1.addWidget(self.config_combo, 1)
        cfg_row1.addWidget(self.refresh_config_btn)
        cfg_row1.addWidget(self.load_config_btn)
        cfg_row1.addWidget(self.import_config_btn)
        cfg_row1.addWidget(self.save_as_btn)
        cfg_vlayout.addLayout(cfg_row1)

        # row 2: status label
        cfg_row2 = QHBoxLayout()
        self.config_status_label = QLabel(tr("No config loaded"))
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

        id_group = QGroupBox(tr("Motor Selection"))
        self._text_bindings.append((id_group, "Motor Selection", "setTitle"))
        id_layout = QHBoxLayout()
        id_layout.setSpacing(6)
        self.motor_id_combo = QComboBox()
        self.motor_id_combo.addItem(tr("None"), None)
        id_layout.addWidget(self._label("Motor ID:"))
        id_layout.addWidget(self.motor_id_combo)
        id_group.setLayout(id_layout)
        layout.addWidget(id_group)
        self.motor_id_combo.currentIndexChanged.connect(self.on_motor_id_changed)

        mode_group = QGroupBox(tr("Operating Mode"))
        self._text_bindings.append((mode_group, "Operating Mode", "setTitle"))
        mode_layout = QFormLayout()
        mode_layout.setSpacing(6)
        self.mode_combo = QComboBox()
        self._fill_combo(self.mode_combo, MODE_ITEMS, keep=False)
        self.set_mode_btn = self._bind_text(QPushButton(), "Set")
        self.get_mode_btn = self._bind_text(QPushButton(), "Get")
        mode_layout.addRow(self._label("Mode:"), self.mode_combo)
        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(6)
        btn_layout.addWidget(self.set_mode_btn)
        btn_layout.addWidget(self.get_mode_btn)
        mode_layout.addRow(btn_layout)
        mode_group.setLayout(mode_layout)
        layout.addWidget(mode_group)

        param_group = QGroupBox(tr("Motor Parameters"))
        self._text_bindings.append((param_group, "Motor Parameters", "setTitle"))
        param_layout = QFormLayout()
        param_layout.setSpacing(6)
        self.pole_pair_label = QLabel("---")
        self.offset_label = QLabel("---")
        self.encoder_dir_label = QLabel("---")
        get_params_btn = self._bind_text(QPushButton(), "Get Parameters")
        param_layout.addRow(self._label("Pole Pairs:"), self.pole_pair_label)
        param_layout.addRow(self._label("Zero Offset (°):"), self.offset_label)
        param_layout.addRow(self._label("Encoder Direction:"), self.encoder_dir_label)
        param_layout.addRow(get_params_btn)
        param_group.setLayout(param_layout)
        layout.addWidget(param_group)
        get_params_btn.clicked.connect(self.get_motor_parameters)

        target_group = QGroupBox(tr("Target Values"))
        self._text_bindings.append((target_group, "Target Values", "setTitle"))
        target_layout = QFormLayout()
        target_layout.setSpacing(6)
        self.target_iq = QDoubleSpinBox(); self.target_iq.setRange(-10,10); self.target_iq.setDecimals(3)
        self.target_id = QDoubleSpinBox(); self.target_id.setRange(-10,10); self.target_id.setDecimals(3)
        self.target_speed = QDoubleSpinBox(); self.target_speed.setRange(-5000,5000)
        self.target_position = QDoubleSpinBox(); self.target_position.setRange(-5000,5000)
        self.target_uq = QDoubleSpinBox(); self.target_uq.setRange(-6,6)
        self.target_ud = QDoubleSpinBox(); self.target_ud.setRange(-6,6)
        set_target_btn = self._bind_text(QPushButton(), "Set All")
        get_target_btn = self._bind_text(QPushButton(), "Get All")
        self.get_speed_btn = self._bind_text(QPushButton(), "Get Speed")
        target_layout.addRow(self._label("Iq:"), self.target_iq)
        target_layout.addRow(self._label("Id:"), self.target_id)
        target_layout.addRow(self._label("Speed (rpm):"), self.target_speed)
        target_layout.addRow(self._label("Position (deg):"), self.target_position)
        target_layout.addRow(self._label("Uq:"), self.target_uq)
        target_layout.addRow(self._label("Ud:"), self.target_ud)
        btn_hlay = QHBoxLayout()
        btn_hlay.setSpacing(6)
        btn_hlay.addWidget(set_target_btn)
        btn_hlay.addWidget(get_target_btn)
        btn_hlay.addWidget(self.get_speed_btn)
        target_layout.addRow(btn_hlay)
        target_group.setLayout(target_layout)
        layout.addWidget(target_group)
        set_target_btn.clicked.connect(self.set_targets)
        get_target_btn.clicked.connect(self.get_targets)
        self.get_speed_btn.clicked.connect(self.get_motor_speed)

        current_group = QGroupBox(tr("Phase Currents"))
        self._text_bindings.append((current_group, "Phase Currents", "setTitle"))
        curr_layout = QFormLayout()
        curr_layout.setSpacing(6)
        self.label_Ia = QLabel("0.000 A")
        self.label_Ib = QLabel("0.000 A")
        self.label_Ic = QLabel("0.000 A")
        curr_layout.addRow(self._label("Ia:"), self.label_Ia)
        curr_layout.addRow(self._label("Ib:"), self.label_Ib)
        curr_layout.addRow(self._label("Ic:"), self.label_Ic)
        current_group.setLayout(curr_layout)
        layout.addWidget(current_group)

        auto_group = QGroupBox(tr("Auto Refresh"))
        self._text_bindings.append((auto_group, "Auto Refresh", "setTitle"))
        auto_layout = QHBoxLayout()
        auto_layout.setSpacing(6)
        self.auto_refresh_cb = QCheckBox(tr("Enable"))
        self._text_bindings.append((self.auto_refresh_cb, "Enable", "setText"))
        self.auto_refresh_cb.setChecked(True)
        self.auto_refresh_cb.toggled.connect(self.toggle_auto_refresh)
        self.refresh_interval_spin = QSpinBox()
        self.refresh_interval_spin.setRange(100,5000)
        self.refresh_interval_spin.setValue(1000)
        self.refresh_interval_spin.setSuffix(tr(" ms"))
        self.refresh_interval_spin.valueChanged.connect(self.on_auto_refresh_interval_changed)
        auto_layout.addWidget(self.auto_refresh_cb)
        auto_layout.addWidget(self._label("Interval:"))
        auto_layout.addWidget(self.refresh_interval_spin)
        self.preview_auto_cb = QCheckBox(tr("Auto Refresh Preview"))
        self._text_bindings.append((self.preview_auto_cb, "Auto Refresh Preview", "setText"))
        self.preview_auto_cb.setChecked(True)
        self.preview_auto_cb.toggled.connect(self.toggle_preview_auto_refresh)
        auto_layout.addWidget(self.preview_auto_cb)
        auto_group.setLayout(auto_layout)
        layout.addWidget(auto_group)

        preview_group = QGroupBox(tr("Motor Preview"))
        self._text_bindings.append((preview_group, "Motor Preview", "setTitle"))
        preview_layout = QVBoxLayout()
        preview_layout.setSpacing(6)
        gear_layout = QHBoxLayout()
        gear_layout.setSpacing(6)
        gear_layout.addWidget(self._label("Gear Ratio:"))
        self.gear_ratio_edit = QLineEdit("1 : 1")
        self.gear_ratio_edit.textChanged.connect(self.update_gear_ratio)
        gear_layout.addWidget(self.gear_ratio_edit)
        preview_layout.addLayout(gear_layout)

        dial_layout = QHBoxLayout()
        dial_layout.setSpacing(6)
        self.motor_preview = MotorPreviewWidget()
        dial_layout.addWidget(self.motor_preview, 1)
        info_layout = QVBoxLayout()
        info_layout.setSpacing(4)
        self.actual_angle_label = self._label("Actual Angle: --- °")
        self.raw_angle_label = self._label("Raw Motor Angle: --- °")
        self.total_rotations_label = self._label("Total rotations: ---")
        self.mod_angle_label = self._label("Mod angle (0-360°): ---")
        self.speed_label = self._label("Motor Speed: --- rpm")
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
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        self.pid_widgets = {}
        for name in ['Iq','Id','Speed','Position']:
            group = QGroupBox(tr("{} PID").format(name))
            form = QFormLayout()
            form.setSpacing(6)
            p = QDoubleSpinBox(); p.setRange(-1000,1000); p.setDecimals(6)
            i = QDoubleSpinBox(); i.setRange(-1000,1000); i.setDecimals(6)
            d = QDoubleSpinBox(); d.setRange(-1000,1000); d.setDecimals(6)
            set_btn = self._bind_text(QPushButton(), "Set")
            get_btn = self._bind_text(QPushButton(), "Get")
            btn_layout = QHBoxLayout()
            btn_layout.setSpacing(6)
            btn_layout.addWidget(set_btn)
            btn_layout.addWidget(get_btn)
            form.addRow(self._label("P:"), p)
            form.addRow(self._label("I:"), i)
            form.addRow(self._label("D:"), d)
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
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        top_layout = QHBoxLayout()
        top_layout.setSpacing(6)
        self.plot_combo = QComboBox()
        self.plot_combo.addItems(["IaIbIc","IqId","Speed","Position"])
        self.poll_checkbox = QCheckBox(tr("Enable Polling"))
        self._text_bindings.append((self.poll_checkbox, "Enable Polling", "setText"))
        top_layout.addWidget(self._label("Plot:"))
        top_layout.addWidget(self.plot_combo)
        top_layout.addWidget(self.poll_checkbox)
        layout.addLayout(top_layout)

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setLabel('left', tr("Value"))
        self.plot_widget.setLabel('bottom', tr("Time (samples)"))
        self.plot_widget.addLegend()
        self.plot_curves = {}
        layout.addWidget(self.plot_widget)

        save_btn = self._bind_text(QPushButton(), "Save Data to CSV")
        save_btn.clicked.connect(self.save_data)
        layout.addWidget(save_btn)

        self.plot_combo.currentTextChanged.connect(self.change_plot_type)
        self.poll_checkbox.toggled.connect(self.toggle_polling)
        return widget

    def create_limits_tab(self):
        widget = QWidget()
        layout = QFormLayout(widget)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        self.limit_iq_max = QDoubleSpinBox(); self.limit_iq_max.setRange(-100,100)
        self.limit_iq_min = QDoubleSpinBox(); self.limit_iq_min.setRange(-100,100)
        self.limit_id_max = QDoubleSpinBox(); self.limit_id_max.setRange(-100,100)
        self.limit_id_min = QDoubleSpinBox(); self.limit_id_min.setRange(-100,100)
        self.limit_speed_max = QDoubleSpinBox(); self.limit_speed_max.setRange(-10000,10000)
        self.limit_speed_min = QDoubleSpinBox(); self.limit_speed_min.setRange(-10000,10000)
        self.limit_position_max = QDoubleSpinBox(); self.limit_position_max.setRange(-10000,10000)
        self.limit_position_min = QDoubleSpinBox(); self.limit_position_min.setRange(-10000,10000)
        layout.addRow(self._label("Iq max:"), self.limit_iq_max)
        layout.addRow(self._label("Iq min:"), self.limit_iq_min)
        layout.addRow(self._label("Id max:"), self.limit_id_max)
        layout.addRow(self._label("Id min:"), self.limit_id_min)
        layout.addRow(self._label("Speed max:"), self.limit_speed_max)
        layout.addRow(self._label("Speed min:"), self.limit_speed_min)
        layout.addRow(self._label("Position max:"), self.limit_position_max)
        layout.addRow(self._label("Position min:"), self.limit_position_min)
        set_btn = self._bind_text(QPushButton(), "Set Limits")
        get_btn = self._bind_text(QPushButton(), "Get Limits")
        layout.addRow(set_btn, get_btn)
        set_btn.clicked.connect(self.set_limits)
        get_btn.clicked.connect(self.get_limits)
        return widget

    def create_manual_tab(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        cmd_group = QGroupBox(tr("Send Command (Hex)"))
        self._text_bindings.append((cmd_group, "Send Command (Hex)", "setTitle"))
        cmd_layout = QVBoxLayout()
        cmd_layout.setSpacing(6)
        self.manual_cmd_edit = QPlainTextEdit()
        self.manual_cmd_edit.setMaximumHeight(100)
        send_btn = self._bind_text(QPushButton(), "Send")
        cmd_layout.addWidget(self.manual_cmd_edit)
        cmd_layout.addWidget(send_btn)
        cmd_group.setLayout(cmd_layout)
        layout.addWidget(cmd_group)

        resp_group = QGroupBox(tr("Response (Raw Hex)"))
        self._text_bindings.append((resp_group, "Response (Raw Hex)", "setTitle"))
        resp_layout = QVBoxLayout()
        resp_layout.setSpacing(6)
        self.manual_response_text = QPlainTextEdit()
        self.manual_response_text.setReadOnly(True)
        clear_btn = self._bind_text(QPushButton(), "Clear")
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
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        ctrl_layout = QHBoxLayout()
        ctrl_layout.setSpacing(6)
        self.imu_poll_cb = QCheckBox(tr("Enable IMU Polling"))
        self._text_bindings.append((self.imu_poll_cb, "Enable IMU Polling", "setText"))
        self.imu_poll_interval = QSpinBox()
        self.imu_poll_interval.setRange(10,500)
        self.imu_poll_interval.setValue(50)
        ctrl_layout.addWidget(self.imu_poll_cb)
        ctrl_layout.addWidget(self._label("Interval (ms):"))
        ctrl_layout.addWidget(self.imu_poll_interval)
        layout.addLayout(ctrl_layout)

        self.imu_3d_view = IMU3DWidget()
        layout.addWidget(self.imu_3d_view, stretch=2)

        model_layout = QHBoxLayout()
        model_layout.setSpacing(6)
        self.model_combo = QComboBox()
        self.model_combo.addItem(tr(DEFAULT_CUBE), DEFAULT_CUBE)
        refresh_model_btn = self._bind_text(QPushButton(), "Refresh")
        browse_btn = self._bind_text(QPushButton(), "Browse")
        reset_btn = self._bind_text(QPushButton(), "Reset")
        model_layout.addWidget(self._label("3D Model:"))
        model_layout.addWidget(self.model_combo)
        model_layout.addWidget(refresh_model_btn)
        model_layout.addWidget(browse_btn)
        model_layout.addWidget(reset_btn)
        layout.addLayout(model_layout)

        data_group = QGroupBox(tr("IMU Data"))
        self._text_bindings.append((data_group, "IMU Data", "setTitle"))
        data_layout = QHBoxLayout()
        data_layout.setSpacing(8)
        left = QVBoxLayout(); left.addWidget(self._label("Acc (g):"))
        self.label_ax = QLabel("ax: ---")
        self.label_ay = QLabel("ay: ---")
        self.label_az = QLabel("az: ---")
        left.addWidget(self.label_ax); left.addWidget(self.label_ay); left.addWidget(self.label_az)
        mid = QVBoxLayout(); mid.addWidget(self._label("Gyro (dps):"))
        self.label_gx = QLabel("gx: ---"); self.label_gy = QLabel("gy: ---"); self.label_gz = QLabel("gz: ---")
        mid.addWidget(self.label_gx); mid.addWidget(self.label_gy); mid.addWidget(self.label_gz)
        right = QVBoxLayout(); right.addWidget(self._label("Orientation (°):"))
        self.label_roll = QLabel(tr("Roll: ---")); self.label_pitch = QLabel(tr("Pitch: ---")); self.label_yaw = QLabel(tr("Yaw: ---"))
        self._text_bindings.append((self.label_roll, "Roll: ---", "setText"))
        self._text_bindings.append((self.label_pitch, "Pitch: ---", "setText"))
        self._text_bindings.append((self.label_yaw, "Yaw: ---", "setText"))
        right.addWidget(self.label_roll); right.addWidget(self.label_pitch); right.addWidget(self.label_yaw)
        data_layout.addLayout(left); data_layout.addLayout(mid); data_layout.addLayout(right)
        data_group.setLayout(data_layout)
        layout.addWidget(data_group)

        log_group = QGroupBox(tr("IMU Data Log"))
        self._text_bindings.append((log_group, "IMU Data Log", "setTitle"))
        log_layout = QVBoxLayout()
        log_layout.setSpacing(6)
        self.imu_log_text = QPlainTextEdit()
        self.imu_log_text.setReadOnly(True)
        self.imu_log_text.setMaximumBlockCount(1000)
        self.imu_log_text.setFont(QFont("Courier New", 9))
        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(6)
        self.clear_log_btn = self._bind_text(QPushButton(), "Clear Log")
        self.save_log_btn = self._bind_text(QPushButton(), "Save Log to CSV")
        self.logging_cb = QCheckBox(tr("Auto Log"))
        self._text_bindings.append((self.logging_cb, "Auto Log", "setText"))
        self.logging_cb.setChecked(True)
        btn_layout.addWidget(self.logging_cb)
        btn_layout.addWidget(self.clear_log_btn)
        btn_layout.addWidget(self.save_log_btn)
        log_layout.addWidget(self.imu_log_text)
        log_layout.addLayout(btn_layout)
        log_group.setLayout(log_layout)
        layout.addWidget(log_group)

        debug_group = QGroupBox(tr("IMU Debug Data"))
        self._text_bindings.append((debug_group, "IMU Debug Data", "setTitle"))
        debug_layout = QVBoxLayout()
        debug_layout.setSpacing(6)
        self.imu_debug_text = QPlainTextEdit()
        self.imu_debug_text.setReadOnly(True)
        self.imu_debug_text.setMaximumHeight(120)
        self.imu_debug_text.setFont(QFont("Courier New", 9))
        copy_btn = self._bind_text(QPushButton(), "Copy Current IMU Data")
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
        is_serial = self.interface_combo.currentData() == "serial"
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
        if self.interface_combo.currentData() == "serial":
            port = self.serial_port_combo.currentText()
            if not port:
                QMessageBox.warning(self, tr("Error"), tr("No serial port"))
                return
            baud = int(self.baudrate_combo.currentText())
            backend = SerialBackend(port, baud)
        else:
            if not CAN_AVAILABLE:
                QMessageBox.critical(self, tr("Error"), tr("python-can not installed"))
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
        self.connect_btn.setText(tr("Disconnect"))
        self.detect_btn.setEnabled(True)
        self.status_label.setText(tr("Connected"))
        self._refresh_status_color()

        # 只恢复当前可见标签页的轮询，其余等待用户切过去
        self._apply_polling_gates()

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
        # 先停掉 IMU 轮询，避免它随后覆盖"Not connected"状态文字
        self.imu_poll_cb.setChecked(False)
        self.connect_btn.setText(tr("Connect"))
        self.detect_btn.setEnabled(False)
        self.status_label.setText(tr("Not connected"))
        self._refresh_status_color()
        self.motor_id = None
        self.motor_id_label.setText(tr("None"))
        self.motor_id_combo.clear()
        self.motor_id_combo.addItem(tr("None"), None)
        self._apply_polling_gates()

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
            self.handle_mode_response(packet.data1.as_uint32())
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
            for i in ids:
                self.motor_id_combo.addItem(str(i), i)
            self.motor_id = ids[0]
            self.motor_id_label.setText(str(ids[0]))
            QMessageBox.information(self, tr("Detect"),
                                    tr("Detected IDs: {}").format(ids))
        else:
            QMessageBox.warning(self, tr("Detect"), tr("No motor found"))

    def set_motor_mode(self):
        mode = self.mode_combo.currentData()
        if mode is None:
            return
        mid = self.get_current_motor_id()
        if mid == 0:
            QMessageBox.warning(self, tr("Warning"), tr("No motor ID"))
            return
        self.send_command(0x01, 0x01, data1=mode, motor_id=mid)

    def get_motor_mode(self):
        mid = self.get_current_motor_id()
        if mid == 0:
            return
        self.send_command(0x01, 0x00, motor_id=mid)

    def handle_mode_response(self, mode):
        """Select the combo entry matching the mode code (itemData based, language independent)."""
        if mode is None:
            return
        idx = self.mode_combo.findData(mode)
        if idx >= 0:
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
            QMessageBox.warning(self, tr("Warning"), tr("No motor ID selected"))
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
        self.actual_angle_label.setText(
            tr("Actual Angle: {}°  (Rot: {}, Mod: {}°)").format(actual_angle, tot, mod))

    def toggle_polling(self, enabled):
        if enabled and self.motor_id is None:
            QMessageBox.warning(self, tr("Polling"), tr("Detect motor ID first"))
            self.poll_checkbox.setChecked(False)
            return
        # 复选框只表达用户意图，实际启停交给门控判断当前页是否可见
        self._apply_polling_gates()

    def on_tab_changed(self, index):
        # 切换标签页时重新评估：只有当前可见页对应的轮询才继续请求数据
        self._apply_polling_gates()

    def _apply_polling_gates(self):
        """按"用户意图 + 当前可见标签页 + 连接状态"统一启停各类轮询。

        每个 *_enabled 标志只表示"此刻真的在轮询"，用户勾选状态始终以复选框为准，
        这样切回标签页或重新勾选时都能恢复到之前的状态。
        """
        connected = self.comm_backend is not None
        index = self.tabs.currentIndex()
        motor_id = self.get_current_motor_id()

        # 1. 实时数据页的曲线轮询
        real_time_ok = (index == TAB_REALTIME_DATA and connected
                        and self.poll_checkbox.isChecked() and self.motor_id is not None)
        if real_time_ok and not self.poll_enabled:
            self.poll_enabled = True
            self.poll_timer.start(50)
        elif not real_time_ok and self.poll_enabled:
            self.poll_enabled = False
            self.poll_timer.stop()

        # 2/3. 电机控制页的预览与相电流轮询
        preview_ok = (index == TAB_MOTOR_CONTROL and connected and motor_id != 0
                      and self.preview_auto_cb.isChecked())
        if preview_ok and not self.auto_refresh_enabled:
            self.start_auto_refresh()
        elif not preview_ok and self.auto_refresh_enabled:
            self.stop_auto_refresh()

        # 暂停/恢复都可能留下未完成的请求，清掉以免恢复后卡住
        if not preview_ok:
            self.is_busy = False

        # 4. 电机控制页的参数自动刷新
        self._sync_auto_refresh_timer(
            index == TAB_MOTOR_CONTROL and connected and motor_id != 0
            and self.auto_refresh_cb.isChecked())

        # 5. IMU 3D 页的数据轮询
        imu_ok = (index == TAB_IMU and connected and self.imu_poll_cb.isChecked())
        if imu_ok and not self.imu_poll_enabled:
            self.imu_poll_enabled = True
            # 恢复轮询时重新打时间戳，避免把暂停时长算进积分
            self.last_imu_time = time.time()
            self.imu_poll_timer.start(self.imu_poll_interval.value())
            self.request_imu_data()
        elif not imu_ok and self.imu_poll_enabled:
            # 只暂停，保留校准结果，切回来可以直接继续
            self.imu_poll_enabled = False
            self.imu_poll_timer.stop()

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
            self.speed_label.setText(tr("Motor Speed: {} rpm").format(round(speed, 1)))
        self.data_history['time'].append(self.plot_index)
        self.data_history['speed'].append(speed)
        if self.poll_type == "speed":
            self.update_plot()

    def handle_position(self, packet):
        pos = packet.data1.as_float()
        tot, mod = compute_rotations_and_mod(pos)
        self.total_rotations_label.setText(tr("Total rotations: {}").format(tot))
        self.mod_angle_label.setText(tr("Mod angle (0-360°): {}°").format(round(mod, 2)))
        self.raw_angle_label.setText(
            tr("Raw Motor Angle: {}°  (Rot: {}, Mod: {}°)").format(round(pos, 1), tot, round(mod, 1)))
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
        self.status_label.setText(tr("Error: {}").format(msg))
        self._refresh_status_color()
        QMessageBox.critical(self, tr("Comm Error"), msg)
        self.disconnect()

    def get_current_motor_id(self):
        data = self.motor_id_combo.currentData()
        if data is None:
            return 0
        try:
            return int(data)
        except (TypeError, ValueError):
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
        # 只记录意图，实际启停由门控按当前可见标签页决定
        self._apply_polling_gates()

    def on_motor_id_changed(self):
        if hasattr(self, 'speed_label'):
            self.speed_label.setText(tr("Motor Speed: --- rpm"))
        if self.auto_refresh_enabled:
            self.is_busy = False
        self._apply_polling_gates()

    def toggle_auto_refresh(self, enabled):
        # 只记录意图，实际启停由门控按当前可见标签页决定
        self._apply_polling_gates()

    def _sync_auto_refresh_timer(self, should_run):
        if should_run:
            self.auto_refresh_timer.start(self.auto_refresh_interval_ms)
            if not self.auto_refresh_timer_running:
                # 仅在由停转启时补一次参数刷新，避免与定时器重复请求
                self.auto_refresh_timer_running = True
                self.refresh_all_except_mode()
        else:
            self.auto_refresh_timer.stop()
            self.auto_refresh_timer_running = False

    def on_auto_refresh_interval_changed(self, value):
        self.auto_refresh_interval_ms = value
        if self.auto_refresh_timer.isActive():
            self.auto_refresh_timer.start(value)

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

        # 自动零偏校准：只在设备静止时采样，避免把转动当成零偏
        if self.calibrating_gyro:
            self.calib_samples += 1
            if math.hypot(gx_gui, gy_gui, gz_gui) < self.calib_motion_dps:
                self.calib_buffer.append((gx_gui, gy_gui, gz_gui))
            enough_samples = len(self.calib_buffer) >= self.calib_min_samples
            if enough_samples or self.calib_samples >= self.calib_max_samples:
                n = len(self.calib_buffer)
                if n:
                    avg_x = sum(v[0] for v in self.calib_buffer) / n
                    avg_y = sum(v[1] for v in self.calib_buffer) / n
                    avg_z = sum(v[2] for v in self.calib_buffer) / n
                else:
                    avg_x = avg_y = avg_z = 0.0
                self.gyro_bias = [avg_x, avg_y, avg_z]
                self.calibrating_gyro = False
                print(f"[IMU] Calibration done. Bias: {avg_x:.2f}, {avg_y:.2f}, {avg_z:.2f} dps")
                self.status_label.setText(
                    tr("IMU ready (bias: {}, {}, {})").format(
                        round(avg_x, 1), round(avg_y, 1), round(avg_z, 1)))
                self.calib_buffer.clear()
            else:
                self.status_label.setText(
                    tr("Calibrating IMU... {}/{}").format(
                        len(self.calib_buffer), self.calib_min_samples))

        # 减去零偏
        gx_cal = gx_gui - self.gyro_bias[0]
        gy_cal = gy_gui - self.gyro_bias[1]
        gz_cal = gz_gui - self.gyro_bias[2]

        current_time = time.time()
        dt = current_time - self.last_imu_time
        # 单次积分跨度上限 0.1s，超出部分不再累积，避免丢包后姿态跳变
        dt = max(0.001, min(0.1, dt))
        self.last_imu_time = current_time

        # 校准期间也持续积分，3D 模型从第一帧起就能跟随转动
        if self.filter is None:
            self.filter = ComplementaryFilter(dt=0.02, alpha=0.92)

        self.filter.dt = dt
        q = self.filter.update(gx_cal, gy_cal, gz_cal, ax_gui, ay_gui, az_gui)

        roll_deg = math.degrees(self.filter.roll)
        pitch_deg = math.degrees(self.filter.pitch)
        yaw_deg = math.degrees(self.filter.yaw)
        self.last_roll_deg = roll_deg
        self.last_pitch_deg = pitch_deg
        self.last_yaw_deg = yaw_deg
        self.label_roll.setText(tr("Roll: {}°").format(round(roll_deg, 1)))
        self.label_pitch.setText(tr("Pitch: {}°").format(round(pitch_deg, 1)))
        self.label_yaw.setText(tr("Yaw: {}°").format(round(yaw_deg, 1)))

        self.imu_3d_view.set_orientation_quat(q)

        self.update_imu_debug_text()

        if self.logging_cb.isChecked():
            self.log_imu_data(ax, ay, az, gx, gy, gz, roll_deg, pitch_deg, yaw_deg)

    def update_imu_debug_text(self):
        if not hasattr(self, 'imu_debug_text'):
            return

        def line(label, value):
            # 译文本身可能已带冒号（含全角），避免出现重复或半角混排
            label = tr(label)
            sep = "" if label.endswith((':', '：')) else ":"
            return label + sep + " " + value + "\n"

        text = (line("Timestamp:", time.strftime('%H:%M:%S'))
                + line("Accel (g):", "ax={:.4f}, ay={:.4f}, az={:.4f}".format(
                    self.imu_data['ax'], self.imu_data['ay'], self.imu_data['az']))
                + line("Gyro raw (dps):", "{:.1f}, {:.1f}, {:.1f}".format(
                    self.imu_data['gx'], self.imu_data['gy'], self.imu_data['gz']))
                + line("Gyro bias (dps):", "{:.1f}, {:.1f}, {:.1f}".format(
                    self.gyro_bias[0], self.gyro_bias[1], self.gyro_bias[2]))
                + line("Gyro cal (dps):", "{:.1f}, {:.1f}, {:.1f}".format(
                    self.imu_data['gx'] - self.gyro_bias[0],
                    self.imu_data['gy'] - self.gyro_bias[1],
                    self.imu_data['gz'] - self.gyro_bias[2]))
                + line("Temperature:", "{:.1f}°C".format(self.imu_data['temp']))
                + line("Orientation (deg):", "roll={:.1f}, pitch={:.1f}, yaw={:.1f}".format(
                    self.last_roll_deg, self.last_pitch_deg, self.last_yaw_deg))
                + line("Raw LSBs:", "ax={}, ay={}, az={}, gx={}, gy={}, gz={}".format(
                    getattr(self, 'ax_raw', 0), getattr(self, 'ay_raw', 0), getattr(self, 'az_raw', 0),
                    getattr(self, 'gx_raw', 0), getattr(self, 'gy_raw', 0), getattr(self, 'gz_raw', 0))))
        self.imu_debug_text.setPlainText(text.rstrip("\n"))

    def copy_imu_data(self):
        text = self.imu_debug_text.toPlainText()
        if text:
            QApplication.clipboard().setText(text)
            self.status_label.setText(tr("IMU data copied to clipboard"))

    def clear_imu_log(self):
        self.imu_log_text.clear()

    def save_imu_log_to_csv(self):
        filename, _ = QFileDialog.getSaveFileName(self, tr("Save IMU Log"), "", tr("CSV Files (*.csv)"))
        if filename:
            try:
                with open(filename, 'w') as f:
                    f.write(self.imu_log_text.toPlainText())
                QMessageBox.information(self, tr("Saved"), tr("Log saved to {}").format(filename))
            except Exception as e:
                QMessageBox.critical(self, tr("Error"), str(e))

    def log_imu_data(self, ax, ay, az, gx, gy, gz, roll, pitch, yaw):
        timestamp = time.strftime("%H:%M:%S") + f".{int(time.time()*1000)%1000:03d}"
        log_line = (f"{timestamp}, {ax:.3f}, {ay:.3f}, {az:.3f}, "
                    f"{gx:.1f}, {gy:.1f}, {gz:.1f}, "
                    f"{roll:.1f}, {pitch:.1f}, {yaw:.1f}")
        self.imu_log_text.appendPlainText(log_line)
        scrollbar = self.imu_log_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def toggle_imu_polling(self, enabled):
        # enabled 表示用户意图；真正是否轮询由 _apply_polling_gates() 决定
        if enabled:
            self.calibrating_gyro = True
            self.calib_samples = 0
            self.calib_buffer = []
            self.gyro_bias = [0.0, 0.0, 0.0]
            # 姿态从零开始，但保留 filter 实例，使模型在校准期间即可跟随转动
            self.filter = ComplementaryFilter(dt=0.02, alpha=0.92)
            self.last_imu_time = time.time()
            self.status_label.setText(tr("IMU calibrating... Keep device still"))
        else:
            self.calibrating_gyro = False
            self.status_label.setText(tr("IMU polling stopped"))
        self._apply_polling_gates()

    def update_imu_poll_interval(self):
        if self.imu_poll_timer.isActive():
            self.imu_poll_timer.start(self.imu_poll_interval.value())

    # ---------- 3D 模型相关 ----------
    def scan_asset_models(self):
        import glob
        self.model_combo.blockSignals(True)
        current = self.model_combo.currentData()
        self.model_combo.clear()
        self.model_combo.addItem(tr(DEFAULT_CUBE), DEFAULT_CUBE)
        asset_dir = "./asset"
        if not os.path.exists(asset_dir):
            os.makedirs(asset_dir)
        for ext in ('.stl','.obj','.ply','.step','.stp'):
            for f in glob.glob(os.path.join(asset_dir, f"*{ext}")):
                self.model_combo.addItem(f, f)
        idx = self.model_combo.findData(current)
        if idx>=0:
            self.model_combo.setCurrentIndex(idx)
        self.model_combo.blockSignals(False)

    def browse_model_file(self):
        filepath, _ = QFileDialog.getOpenFileName(self, tr("Select 3D Model"), "./asset", tr("3D Models (*.stl *.obj *.ply *.step *.stp)"))
        if filepath:
            self.load_model_to_view(filepath)
            self.model_combo.addItem(filepath, filepath)
            self.model_combo.setCurrentIndex(self.model_combo.findData(filepath))

    def reset_to_cube(self):
        self.imu_3d_view.set_default_cube()
        idx = self.model_combo.findData(DEFAULT_CUBE)
        if idx >= 0:
            self.model_combo.setCurrentIndex(idx)

    def load_model_to_view(self, filepath):
        if not TRIMESH_AVAILABLE:
            QMessageBox.critical(self, tr("Missing Library"), tr("trimesh not installed"))
            return
        if not self.imu_3d_view.load_model_from_file(filepath):
            QMessageBox.warning(self, tr("Load Failed"), tr("Failed to load {}").format(filepath))

    def send_manual_command(self):
        if not self.comm_backend:
            QMessageBox.warning(self, tr("Error"), tr("Not connected"))
            return
        hex_str = self.manual_cmd_edit.toPlainText().strip()
        if not hex_str:
            return
        hex_str = hex_str.replace(' ', '').replace('\n', '')
        try:
            data = bytes.fromhex(hex_str)
        except ValueError:
            QMessageBox.warning(self, tr("Error"), tr("Invalid hex string"))
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
            color = self.colors["status_ok"] if success else self.colors["status_error"]
        else:
            name = self.config_manager.current_name
            text = tr("Active: {}").format(name)
            color = self.colors["status_ok"]
        self.config_status_label.setText(text)
        self.config_status_label.setStyleSheet(f"color: {color}; font-style: italic;")

    def on_load_config(self):
        """Load the selected config from the dropdown and apply to GUI."""
        file = self.config_combo.currentText()
        if not file:
            QMessageBox.warning(self, tr("No Config"), tr("No config file selected."))
            return
        path = os.path.join(self.config_manager.config_dir, file)
        try:
            self.config_manager.load_and_apply(self, path)
            self._update_config_status(True)
            QMessageBox.information(self, tr("Config"), tr("Loaded: {}").format(file))
        except Exception as e:
            self._update_config_status(False, tr("Failed to load: {}").format(e))
            QMessageBox.critical(self, tr("Error"), str(e))

    def on_import_config(self):
        """Import a config file from anywhere on disk into the config dir,
        then load and apply it."""
        src, _ = QFileDialog.getOpenFileName(
            self, tr("Import Config"), "",
            tr("JSON Files (*.json);;All Files (*)"))
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
            QMessageBox.information(self, tr("Imported"),
                                    tr("Imported and loaded: {}").format(imported_name))
        except Exception as e:
            self._update_config_status(False, tr("Import failed: {}").format(e))
            QMessageBox.critical(self, tr("Error"), str(e))

    def on_save_config(self):
        """Save current GUI settings as a new config file, then set it as active."""
        default_dir = self.config_manager.config_dir
        filename, _ = QFileDialog.getSaveFileName(
            self, tr("Save Config As"), default_dir, tr("JSON (*.json)"))
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
            QMessageBox.information(self, tr("Saved"), tr("Saved to {}").format(saved_name))
        except Exception as e:
            self._update_config_status(False, tr("Save failed: {}").format(e))
            QMessageBox.critical(self, "Error", str(e))