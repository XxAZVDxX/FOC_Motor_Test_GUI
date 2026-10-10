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
                             QScrollArea, QFrame, QGridLayout, QSplitter,
                             QListWidget)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont, QKeySequence

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

MODE_STOP = 0
MODE_OPEN_LOOP = 3
MODE_CURRENT_LOOP = 4
MODE_SPEED_LOOP = 5
MODE_POSITION_LOOP = 6

# 每种控制模式实际生效的目标值，用于在界面上标出当前起作用的那一项
MODE_TARGET_KEYS = {
    MODE_OPEN_LOOP: ("Uq:", "Ud:"),
    MODE_CURRENT_LOOP: ("Iq:", "Id:"),
    MODE_SPEED_LOOP: ("Speed (rpm):",),
    MODE_POSITION_LOOP: ("Position (deg):",),
}

INTERFACE_ITEMS = [
    ("serial", "Serial (UART/RS485)"),
    ("can", "CAN"),
]

DEFAULT_CUBE = "Default Cube"

# 串口列表自动刷新间隔（毫秒）与电机 ID 检测超时（毫秒）
PORT_REFRESH_INTERVAL_MS = 1500
DETECT_TIMEOUT_MS = 3000
# 模式切换与目标值之间需要留出间隔：固件切换闭环要花时间（初始化电流环、
# 复位速度环积分器、读取编码器），紧接着下发目标值会被当成上个模式的残留指令。
MODE_SWITCH_SETTLE_MS = 120

# 速度回读系数的默认值。
# 修复前的固件 motor_aim_speed_param() 回读的是内部值 aim_speed（rpm/9.5238095），
# 所以 Get 回来的速度只有实际值的 1/9.5238095，需要乘 9.5238095 补回来。
# Nebula_st_mdk 已改成回读 aim_speeed_rpm（就是 rpm），新固件下这个系数应该是 1.0。
# 两种固件都可能遇到，所以这里只当默认值用，实际值由 _detect_speed_scale()
# 拿“刚下发过的速度”和回读值对比后自动校准，不必手工改。
FIRMWARE_SPEED_READBACK_SCALE = 9.5238095
# 新旧固件下回读值/下发值的比值（近似）
_FW_RATIO_OLD = 1.0 / FIRMWARE_SPEED_READBACK_SCALE
_FW_RATIO_NEW = 1.0

TAB_KEYS = ["Connection", "Motor Control", "PID Tuning", "Real-time Data",
            "Limits", "Manual", "IMU 3D"]

# 各类轮询各自归属的标签页，只有该页可见时才允许请求数据
TAB_CONNECTION = TAB_KEYS.index("Connection")
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

    def update(self, gx, gy, gz, ax, ay, az, accel_usable=True, gyro_trustworthy=True):
        # 加速度计计算的姿态（弧度）
        acc_roll = math.atan2(ay, az)
        acc_pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
        # 陀螺仪积分，并限幅
        cap = self.max_rate_rad * self.dt
        delta_r = math.radians(gx) * self.dt
        delta_p = math.radians(gy) * self.dt
        delta_y = math.radians(gz) * self.dt
        # gyro_trustworthy 为 False 表示该帧的角速度与加速度计互相矛盾（物理上
        # 不可能同时成立），几乎可以肯定是串口坏帧，此时宁可整帧冻住姿态：
        # 不积分等于本帧引入 0 误差，随后几帧的加速度计修正会把它拉回正确位置。
        # 注意 yaw 不受此门限约束——纯偏航转动不会改变重力方向，无法用加速度计
        # 校验，若一并冻结会把真实的偏航削掉。
        if gyro_trustworthy:
            self.roll  += max(-cap, min(cap, delta_r))
            self.pitch += max(-cap, min(cap, delta_p))
        self.yaw   += max(-cap, min(cap, delta_y))
        # 互补滤波：转动时加速度计读数含离心力，若每帧都融合会把姿态拉回，
        # 因此仅在角速度很低（接近静止）时才用它修正 roll/pitch。
        # 这里按合矢量判断，避免某个轴单独偏大时门限被绕过。
        # accel_usable 由调用方给出：合矢量偏离 1g 太多时该帧的 acc_roll/acc_pitch
        # 没有意义（坏帧会算出 ±90° 的假姿态），必须禁止融合。
        # 坏帧额外“强制打开”修正：坏帧的虚假角速度往往同时超过 accel_gate_dps，
        # 若照常判断，最需要被修正的那一帧反而会被门限挡住。
        if accel_usable and (not gyro_trustworthy
                             or math.sqrt(gx*gx + gy*gy + gz*gz) < self.accel_gate_dps):
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
        # 视图缩放百分比：整份样式表的字体/间距都按它等比缩放
        self.zoom = theme.clamp_zoom(initial_settings.get("zoom", theme.DEFAULT_ZOOM))
        # 上次用过的速度/位置指令，跨会话保留，打开就能直接再发一次
        self.last_speed_cmd = float(initial_settings.get("speed_cmd", 0.0) or 0.0)
        self.last_position_cmd = float(initial_settings.get("position_cmd", 0.0) or 0.0)
        # 速度/位置输入框当前是否“由用户掌管”。
        # 只要用户敲过值、或者刚点过 Set 发过指令，这里就是 True，
        # 轮询回读一律不许改写输入框——否则下一次 Set 发的就不是用户想要的值。
        # 只有显式点 Get All 时才清成 False，允许回读把固件里的值填进来。
        self.target_edited = {"Speed (rpm):": False, "Position (deg):": False}
        # 上一次真正下发过的速度/位置，用来判断回读值的单位是否合理
        self.last_sent_speed = None
        self.last_sent_position = None
        # 当前判断出的固件速度回读系数，由 _detect_speed_scale() 自动校准
        self.firmware_speed_scale = FIRMWARE_SPEED_READBACK_SCALE
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
        self._last_bus_voltage = None

        # 参数自动刷新（电机控制页），同样只在当前页可见时运行
        self.auto_refresh_timer = QTimer()
        self.auto_refresh_timer.timeout.connect(self.refresh_all_except_mode)
        self.auto_refresh_timer_running = False
        self.auto_refresh_interval_ms = 1000

        # 串口列表自动刷新：只在连接页可见且未连接时运行
        self.port_refresh_timer = QTimer()
        self.port_refresh_timer.timeout.connect(self.refresh_serial_ports)
        self.port_refresh_timer.setInterval(PORT_REFRESH_INTERVAL_MS)

        # 电机 ID 检测看门狗：广播后若超时仍无应答则提示用户
        self.detect_pending = False
        self.detect_auto = False
        self.detect_timeout_timer = QTimer()
        self.detect_timeout_timer.setSingleShot(True)
        self.detect_timeout_timer.setInterval(DETECT_TIMEOUT_MS)
        self.detect_timeout_timer.timeout.connect(self.on_detect_timeout)

        # 延迟下发目标值：先切模式，等固件就绪后再发目标值
        self.pending_target_timer = QTimer()
        self.pending_target_timer.setSingleShot(True)
        self.pending_target_timer.setInterval(MODE_SWITCH_SETTLE_MS)
        self.pending_target_timer.timeout.connect(self._flush_pending_target)
        self.pending_target = None

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
        # 零偏大小没有上限（有些模块静态偏移就有几十 dps），因此不能用绝对阈值
        # 判断“是否在动”，改为看样本是否偏离已采集样本的中位数。
        self.calib_motion_dps = 8.0           # 偏离中位数的容忍量，超过则丢弃该样本
        self.calib_buffer = []
        self.gyro_deadband_dps = 0.5          # 校准后残余的微小读数直接归零，避免长期漂移
        # 偶发的读数尖峰（实测 gz 会瞬间跳到 416 dps，前后帧却只有 -0.4 dps），
        # 积分后会带来几度到几十度的虚假偏航。用 5 点中值滤波消除尖峰：
        # 要污染中位数需要连续 3 帧坏数据，而真实转动是连续的多帧信号，
        # 只会被延迟 2 帧（40ms），肉眼不可见。
        self.gyro_history = []
        self.gyro_outlier_dps = 50.0          # 中值与原值差异超过该值即判定为尖峰
        # 加速度计合矢量必须接近 1g 才可信。实测静止帧为 0.977~1.071g，而坏帧
        # 会落到 0.09g 或 3.2g，由此算出的 acc_roll/acc_pitch 是 ±90° 之类的假姿态；
        # 若让它们进入互补滤波，姿态会被瞬间拉偏再慢慢爬回来，即肉眼看 到的“抖动”。
        self.accel_norm_min = 0.75
        self.accel_norm_max = 1.25

        # 仅凭合矢量还不够：有些坏帧的加速度计数值仍在 0.75~1.25g 内，但陀螺仪
        # 读数被线路干扰成几百 dps。这种帧积分一次就能推走十几到几十度（实测单帧
        # 最大 56°，因为 dt 上限 0.1s），随后几秒才慢慢爬回来，即“抖一下又回正”。
        # 因此再做一次运动学一致性校验：陀螺仪预测的重力方向变化必须与加速度计
        # 实测的变化相符。两者互相矛盾时该帧角速度不可信，冻结 roll/pitch 积分。
        # 实测合法转动（400 dps 以内）残差不超过 ~91 dps，而全部 8 个已知坏帧都
        # 在 450 dps 以上，留有约 5 倍余量。
        self.consist_tol_deg = 60.0           # 残差超过该值判为该帧不一致
        self.consist_min_dps = 40.0           # 角速度太小时残差被噪声放大，不做判断
        self.consist_nbad = 1                 # 连续多少帧不一致才判定为坏帧
        self.freeze_bad_accel = True          # 加速度计失真的帧是否一并冻结角速度积分
        self.imu_prev_unit = None             # 上一帧归一化后的重力方向
        self.imu_prev_good = False            # 上一帧加速度计是否可用
        self.imu_bad_streak = 0               # 连续不一致帧数

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

        # 内容过高的标签页放入滚动区域，避免小屏幕上控件不可达。
        # force=True 用于高度依赖运行时字体度量、无法在布局前判断的标签页。
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
        # 启动时先刷新一次串口列表，让用户一打开就能看到端口
        self.refresh_serial_ports()

        self.status_label = QLabel(tr("Not connected"))
        self.statusBar().addWidget(self.status_label)

        # 缩放控件固定在状态栏右侧，配合 View 菜单的快捷键一起使用
        self.zoom_label = self._bind_text(QLabel(), "Zoom:")
        self.zoom_combo = QComboBox()
        for level in theme.ZOOM_LEVELS:
            self.zoom_combo.addItem("%d%%" % level, level)
        self.zoom_combo.setCurrentIndex(self.zoom_combo.findData(self.zoom))
        self.zoom_combo.activated.connect(
            lambda _index: self.set_zoom(self.zoom_combo.currentData()))
        self.statusBar().addPermanentWidget(self.zoom_label)
        self.statusBar().addPermanentWidget(self.zoom_combo)

        self.apply_theme(self.current_theme)
        # 首次评估轮询门控：连接页的串口列表自动刷新也随之启动
        self._apply_polling_gates()

    # ---------- 菜单栏 ----------
    def _wrap_scrollable(self, widget, force=False):
        """把过大的标签页放进滚动区域，保证小屏幕/高缩放下也能访问全部控件。

        force=True 用于已知必然超高的标签页（其高度依赖运行时字体度量，
        不适合在布局前判断）。缩放到很大时宽度也会超过屏幕，同样需要滚动。
        """
        hint = widget.minimumSizeHint()
        if not force and hint.height() <= 700 and hint.width() <= 700:
            return widget
        scroll = QScrollArea()
        scroll.setWidget(widget)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
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

        # 缩放：快捷键在 macOS 上由 Qt 自动映射为 Command +/-/0
        zoom_in_action = QAction(tr("Zoom In"), self)
        zoom_in_action.setShortcuts([QKeySequence("Ctrl++"), QKeySequence("Ctrl+=")])
        zoom_in_action.triggered.connect(self.zoom_in)
        view_menu.addAction(zoom_in_action)

        zoom_out_action = QAction(tr("Zoom Out"), self)
        zoom_out_action.setShortcuts([QKeySequence("Ctrl+-"), QKeySequence("Ctrl+_")])
        zoom_out_action.triggered.connect(self.zoom_out)
        view_menu.addAction(zoom_out_action)

        zoom_reset_action = QAction(tr("Reset Zoom"), self)
        zoom_reset_action.setShortcut(QKeySequence("Ctrl+0"))
        zoom_reset_action.triggered.connect(self.reset_zoom)
        view_menu.addAction(zoom_reset_action)

        self._text_bindings.append((zoom_in_action, "Zoom In", "setText"))
        self._text_bindings.append((zoom_out_action, "Zoom Out", "setText"))
        self._text_bindings.append((zoom_reset_action, "Reset Zoom", "setText"))

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
        self.colors = theme.apply_theme(app, theme_name, self.zoom)
        theme.apply_plot_theme(self.plot_widget, theme_name)
        theme.apply_plot_zoom(self.plot_widget, self.zoom)
        self._apply_zoom_to_widgets()
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

    # ---------- 视图缩放 ----------
    def set_zoom(self, percent):
        """按百分比等比缩放整个界面（样式表 + pyqtgraph + 少量硬编码尺寸）。"""
        new_zoom = theme.clamp_zoom(percent)
        if new_zoom == self.zoom:
            return
        self.zoom = new_zoom
        self.apply_theme(self.current_theme)
        if self.zoom_combo.currentData() != new_zoom:
            self.zoom_combo.setCurrentIndex(self.zoom_combo.findData(new_zoom))
        # 缩放后最小尺寸会变化，等布局完成后重新把窗口收回屏幕内
        QTimer.singleShot(0, self._clamp_to_screen)
        self._persist_settings()

    def zoom_in(self):
        self.set_zoom(self.zoom + theme.ZOOM_STEP)

    def zoom_out(self):
        self.set_zoom(self.zoom - theme.ZOOM_STEP)

    def reset_zoom(self):
        self.set_zoom(theme.DEFAULT_ZOOM)

    def _apply_zoom_to_widgets(self):
        """缩放样式表管不到的硬编码尺寸（表盘、3D 视图、绘图控件等）。"""
        factor = self.zoom / 100.0
        preview_side = int(round(340 * factor))
        self.motor_preview.setMinimumSize(int(round(200 * factor)), int(round(200 * factor)))
        self.motor_preview.setMaximumSize(preview_side, preview_side)
        self.imu_3d_view.setMinimumSize(int(round(360 * factor)), int(round(320 * factor)))
        self.model_combo.setMinimumWidth(int(round(160 * factor)))
        self.plot_widget.setMinimumHeight(int(round(260 * factor)))
        self.serial_port_list.setMinimumHeight(int(round(72 * factor)))
        self.serial_port_list.setMaximumHeight(int(round(96 * factor)))
        for spin, _btn in self.target_set_btns.values():
            spin.setMinimumWidth(int(round(110 * factor)))

    def set_language(self, language_code):
        if not tr.set_language(language_code):
            return
        self.retranslate_ui()
        self._persist_settings()

    def _persist_settings(self):
        # 与已有设置合并，避免只写入部分键时丢掉其它偏好
        values = settings.load()
        values["theme"] = self.current_theme
        values["language"] = tr.language
        values["zoom"] = self.zoom
        values["speed_cmd"] = self.last_speed_cmd
        values["position_cmd"] = self.last_position_cmd
        settings.save(values)

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
        # 缩放下拉框的条目是百分比数字，_fill_combo 会清空它，需要重新填充
        self.zoom_combo.blockSignals(True)
        self.zoom_combo.clear()
        for level in theme.ZOOM_LEVELS:
            self.zoom_combo.addItem("%d%%" % level, level)
        self.zoom_combo.setCurrentIndex(self.zoom_combo.findData(self.zoom))
        self.zoom_combo.blockSignals(False)
        self.plot_widget.setLabel('left', tr("Value"))
        self.plot_widget.setLabel('bottom', tr("Time (samples)"))
        self.refresh_interval_spin.setSuffix(tr(" ms"))
        # 详情页签的标题不参与 _text_bindings，需要单独刷新
        self.imu_detail_tabs.setTabText(0, tr("IMU Debug Data"))
        self.imu_detail_tabs.setTabText(1, tr("IMU Data Log"))
        # 带 itemData 的下拉框：显示文本需要单独刷新
        if self.motor_id_combo.count():
            self.motor_id_combo.setItemText(0, tr("None"))
        cube_idx = self.model_combo.findData(DEFAULT_CUBE)
        if cube_idx >= 0:
            self.model_combo.setItemText(cube_idx, tr(DEFAULT_CUBE))
        self.update_active_target_highlight()

    # ---------- 创建标签页的函数 ----------
    def create_connection_tab(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        # 接口选择放在最上面，串口/CAN 分组框随之整体切换
        top_bar = QHBoxLayout()
        top_bar.setSpacing(6)
        top_bar.addWidget(self._label("Interface:"))
        self.interface_combo = QComboBox()
        self._fill_combo(self.interface_combo, INTERFACE_ITEMS, keep=False)
        top_bar.addWidget(self.interface_combo)
        top_bar.addStretch(1)
        layout.addLayout(top_bar)

        # ── 串口分组：端口列表 + 刷新按钮同一行标题栏，列表本身压到最低高度 ──
        self.serial_group = QGroupBox(tr("Serial (UART/RS485)"))
        self._text_bindings.append((self.serial_group, "Serial (UART/RS485)", "setTitle"))
        serial_grid = QGridLayout(self.serial_group)
        serial_grid.setContentsMargins(8, 8, 8, 8)
        serial_grid.setHorizontalSpacing(8)
        serial_grid.setVerticalSpacing(6)

        # 串口用列表而不是下拉框：新出现的端口插入到列表顶部，方便一眼看到
        self.serial_port_list = QListWidget()
        self.serial_port_list.setMinimumHeight(72)
        self.serial_port_list.setMaximumHeight(96)
        self.serial_port_list.setSelectionMode(QListWidget.SingleSelection)
        self.auto_refresh_ports_cb = self._bind_text(QCheckBox(), "Auto-refresh ports")
        self.auto_refresh_ports_cb.setChecked(True)
        self.refresh_ports_btn = self._bind_text(QPushButton(), "Refresh")
        self.baudrate_combo = QComboBox()
        self.baudrate_combo.addItems(["9600","19200","38400","57600","115200","2000000"])

        port_bar = QHBoxLayout()
        port_bar.setSpacing(6)
        port_bar.addWidget(self._label("Serial Port:"))
        port_bar.addStretch(1)
        port_bar.addWidget(self.auto_refresh_ports_cb)
        port_bar.addWidget(self.refresh_ports_btn)
        serial_grid.addLayout(port_bar, 0, 0, 1, 2)
        serial_grid.addWidget(self.serial_port_list, 1, 0, 1, 2)
        serial_grid.addWidget(self._label("Baudrate:"), 2, 0)
        serial_grid.addWidget(self.baudrate_combo, 2, 1)
        serial_grid.setColumnStretch(1, 1)
        layout.addWidget(self.serial_group)

        # ── CAN 分组 ─────────────────────────────────────────────────
        self.can_group = QGroupBox(tr("CAN"))
        self._text_bindings.append((self.can_group, "CAN", "setTitle"))
        can_grid = QGridLayout(self.can_group)
        can_grid.setContentsMargins(8, 8, 8, 8)
        can_grid.setHorizontalSpacing(8)
        can_grid.setVerticalSpacing(6)

        self.can_channel_edit = QLineEdit("PCAN_USBBUS1")
        self.can_bustype_combo = QComboBox()
        self.can_bustype_combo.addItems(["pcan","socketcan","kvaser","ixxat","vector"])
        self.can_bitrate_edit = QLineEdit("500000")
        can_grid.addWidget(self._label("CAN Channel:"), 0, 0)
        can_grid.addWidget(self.can_channel_edit, 0, 1)
        can_grid.addWidget(self._label("CAN Bustype:"), 1, 0)
        can_grid.addWidget(self.can_bustype_combo, 1, 1)
        can_grid.addWidget(self._label("CAN Bitrate:"), 2, 0)
        can_grid.addWidget(self.can_bitrate_edit, 2, 1)
        can_grid.setColumnStretch(1, 1)
        layout.addWidget(self.can_group)

        # ── 连接操作与检测结果显示 ───────────────────────────────────
        action_bar = QHBoxLayout()
        action_bar.setSpacing(6)
        self.connect_btn = self._bind_text(QPushButton(), "Connect")
        self.detect_btn = self._bind_text(QPushButton(), "Detect Motor ID")
        self.detect_btn.setEnabled(False)
        self.motor_id_label = self._label("None")
        action_bar.addWidget(self.connect_btn)
        action_bar.addWidget(self.detect_btn)
        action_bar.addStretch(1)
        action_bar.addWidget(self._label("Detected Motor ID:"))
        action_bar.addWidget(self.motor_id_label)
        layout.addLayout(action_bar)
        layout.addStretch(1)

        self.interface_combo.currentIndexChanged.connect(self.update_interface_visibility)
        self.refresh_ports_btn.clicked.connect(self.refresh_serial_ports)
        self.auto_refresh_ports_cb.toggled.connect(self._on_auto_refresh_ports_toggled)
        self.connect_btn.clicked.connect(self.toggle_connection)
        # 手动检测需要弹窗确认，用 lambda 固定 auto=False，避免误收 clicked 的 bool
        self.detect_btn.clicked.connect(lambda _checked=False: self.detect_motor_id(auto=False))
        self.refresh_serial_ports()
        self.update_interface_visibility()
        return widget

    def create_control_tab(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        # 八个分组框改用两列网格，避免纵向堆叠浪费屏幕高度
        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(6)

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
        grid.addWidget(config_group, 0, 0, 1, 2)

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
        grid.addWidget(id_group, 1, 1)
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
        grid.addWidget(mode_group, 1, 0)

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
        grid.addWidget(param_group, 2, 0)
        get_params_btn.clicked.connect(self.get_motor_parameters)

        # 手动写入极对数/零偏/方向。
        # 自动校准偶尔会算错（比如机械行程绕圈），以前算错就只能改固件重新编译烧录，
        # 现在可以直接在这里填正确值下发，固件会立刻存进 Flash，掉电也不丢。
        set_param_group = QGroupBox(tr("Set Motor Parameters"))
        self._text_bindings.append((set_param_group, "Set Motor Parameters", "setTitle"))
        set_param_layout = QFormLayout()
        set_param_layout.setSpacing(6)
        self.set_pole_pair_spin = QSpinBox()
        self.set_pole_pair_spin.setRange(1, 64)
        self.set_pole_pair_spin.setValue(7)
        self.set_offset_spin = QDoubleSpinBox()
        self.set_offset_spin.setRange(0.0, 359.999)
        self.set_offset_spin.setDecimals(3)
        self.set_offset_spin.setValue(0.0)
        self.set_dir_combo = QComboBox()
        self._fill_combo(self.set_dir_combo, [(1, "Enable"), (0, "Disable")], keep=False)
        set_param_layout.addRow(self._label("Pole Pairs:"), self.set_pole_pair_spin)
        set_param_layout.addRow(self._label("Zero Offset (°):"), self.set_offset_spin)
        set_param_layout.addRow(self._label("Encoder Direction:"), self.set_dir_combo)
        set_param_btn = self._bind_text(QPushButton(), "Set")
        set_param_layout.addRow(set_param_btn)

        clear_cal_btn = self._bind_text(QPushButton(), "Clear Calibration")
        set_param_layout.addRow(clear_cal_btn)
        set_param_group.setLayout(set_param_layout)
        grid.addWidget(set_param_group, 3, 0)

        set_param_btn.clicked.connect(self.set_motor_parameters)
        clear_cal_btn.clicked.connect(self.clear_calibration)

        target_group = QGroupBox(tr("Target Values"))
        self._text_bindings.append((target_group, "Target Values", "setTitle"))
        target_layout = QGridLayout()
        target_layout.setHorizontalSpacing(8)
        target_layout.setVerticalSpacing(4)
        self.target_iq = QDoubleSpinBox(); self.target_iq.setRange(-10,10); self.target_iq.setDecimals(3)
        self.target_id = QDoubleSpinBox(); self.target_id.setRange(-10,10); self.target_id.setDecimals(3)
        # 速度和位置要能直接敲入指令值，所以放宽上限并保留三位小数
        self.target_speed = QDoubleSpinBox()
        self.target_speed.setRange(-settings.CMD_LIMIT, settings.CMD_LIMIT)
        self.target_speed.setDecimals(3)
        self.target_speed.setSingleStep(10)
        self.target_position = QDoubleSpinBox()
        self.target_position.setRange(-settings.CMD_LIMIT, settings.CMD_LIMIT)
        self.target_position.setDecimals(3)
        self.target_position.setSingleStep(1)
        self.target_uq = QDoubleSpinBox(); self.target_uq.setRange(-6,6)
        self.target_ud = QDoubleSpinBox(); self.target_ud.setRange(-6,6)
        # 每项后面各带一个 Set 按钮：只发这一项，不再连带其它目标值
        self.target_set_btns = {}
        self.target_labels = {}
        targets = (("Iq:", self.target_iq), ("Id:", self.target_id),
                   ("Speed (rpm):", self.target_speed),
                   ("Position (deg):", self.target_position),
                   ("Uq:", self.target_uq), ("Ud:", self.target_ud))
        for idx, (key, spin) in enumerate(targets):
            row, col = divmod(idx, 2)
            base = col * 3
            label = self._label(key)
            spin.setMinimumWidth(int(round(110 * self.zoom / 100.0)))
            set_btn = self._bind_text(QPushButton(), "Set")
            set_btn.clicked.connect(lambda _=False, k=key: self.set_target_value(k))
            target_layout.addWidget(label, row, base)
            target_layout.addWidget(spin, row, base + 1)
            target_layout.addWidget(set_btn, row, base + 2)
            self.target_labels[key] = label
            self.target_set_btns[key] = (spin, set_btn)
        # 提示：单项 Set 的行为，以及速度/位置会自动切换工作模式
        self.target_hint = self._label(
            "Each Set button sends only that value; Speed/Position also switch the mode")
        target_layout.addWidget(self.target_hint, 3, 0, 1, 6)
        btn_hlay = QHBoxLayout()
        btn_hlay.setSpacing(6)
        set_target_btn = self._bind_text(QPushButton(), "Set All")
        get_target_btn = self._bind_text(QPushButton(), "Get All")
        self.get_speed_btn = self._bind_text(QPushButton(), "Get Speed")
        self.stop_motor_btn = self._bind_text(QPushButton(), "Stop")
        btn_hlay.addWidget(set_target_btn)
        btn_hlay.addWidget(get_target_btn)
        btn_hlay.addWidget(self.get_speed_btn)
        btn_hlay.addWidget(self.stop_motor_btn)
        target_layout.addLayout(btn_hlay, 4, 0, 1, 6)
        target_group.setLayout(target_layout)
        grid.addWidget(target_group, 2, 1)
        set_target_btn.clicked.connect(self.set_targets)
        get_target_btn.clicked.connect(lambda: self.get_targets(user_initiated=True))
        self.get_speed_btn.clicked.connect(self.get_motor_speed)
        self.stop_motor_btn.clicked.connect(self.stop_motor)
        # 恢复上次用过的速度/位置指令
        self.target_speed.setValue(self.last_speed_cmd)
        self.target_position.setValue(self.last_position_cmd)
        # 用户手动改值就标记为“已编辑”，轮询回读不再覆盖；点 Set/Get 时清除
        self.target_speed.valueChanged.connect(
            lambda _=0.0: self._mark_target_edited("Speed (rpm):"))
        self.target_position.valueChanged.connect(
            lambda _=0.0: self._mark_target_edited("Position (deg):"))

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
        grid.addWidget(current_group, 4, 0)

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
        grid.addWidget(auto_group, 3, 1, 2, 1)

        preview_group = QGroupBox(tr("Motor Preview"))
        self._text_bindings.append((preview_group, "Motor Preview", "setTitle"))
        # 表盘居中放大，读数在下方并排成两列
        preview_layout = QVBoxLayout()
        preview_layout.setSpacing(6)
        gear_layout = QHBoxLayout()
        gear_layout.setSpacing(6)
        gear_layout.addWidget(self._label("Gear Ratio:"))
        self.gear_ratio_edit = QLineEdit("1 : 1")
        self.gear_ratio_edit.textChanged.connect(self.update_gear_ratio)
        gear_layout.addWidget(self.gear_ratio_edit)
        preview_layout.addLayout(gear_layout)

        self.motor_preview = MotorPreviewWidget()
        preview_layout.addWidget(self.motor_preview, 0, Qt.AlignHCenter)

        # 读数排成两列放在表盘下方，不再用竖排长文本撑宽整个标签页
        info_layout = QGridLayout()
        info_layout.setHorizontalSpacing(10)
        info_layout.setVerticalSpacing(2)
        self.actual_angle_label = self._label("Actual Angle: --- °")
        self.raw_angle_label = self._label("Raw Motor Angle: --- °")
        self.total_rotations_label = self._label("Total rotations: ---")
        self.mod_angle_label = self._label("Mod angle (0-360°): ---")
        self.speed_label = self._label("Motor Speed: --- rpm")
        self.speed_label.setFont(QFont("Arial", 10))
        info_layout.addWidget(self.actual_angle_label, 0, 0)
        info_layout.addWidget(self.raw_angle_label, 0, 1)
        info_layout.addWidget(self.total_rotations_label, 1, 0)
        info_layout.addWidget(self.mod_angle_label, 1, 1)
        info_layout.addWidget(self.speed_label, 2, 0)

        preview_layout.addLayout(info_layout)
        preview_layout.addStretch(1)
        preview_group.setLayout(preview_layout)
        grid.addWidget(preview_group, 5, 0, 1, 2)

        # 母线(电源)电压：PA7 经 18k/1k 分压采样，只测量显示，改 Udc 需要手动点按钮。
        # 采样有噪声，直接拿它自动改 Udc 会让 SVPWM 增益 K 跟着抖，所以做成手动下发。
        bus_group = QGroupBox(tr("Bus Voltage (PA7)"))
        self._text_bindings.append((bus_group, "Bus Voltage (PA7)", "setTitle"))
        bus_layout = QFormLayout()
        bus_layout.setSpacing(6)
        self.bus_voltage_label = self._label("Measured: --- V")
        self.udc_label = self._label("Active Udc: --- V")
        self.bus_adc_label = self._label("ADC: ---")

        bus_btn_row = QHBoxLayout()
        bus_btn_row.setSpacing(6)
        read_bus_btn = self._bind_text(QPushButton(), "Read Bus Voltage")
        apply_bus_btn = self._bind_text(QPushButton(), "Apply as Udc")
        apply_bus_btn.setToolTip(tr(
            "Use the last measured value as Udc; K is recomputed on the firmware"))
        bus_btn_row.addWidget(read_bus_btn)
        bus_btn_row.addWidget(apply_bus_btn)
        bus_btn_row.addStretch(1)

        # 手动写入窗口：实测值和想用的值不一致时可以自己填
        self.set_udc_spin = QDoubleSpinBox()
        self.set_udc_spin.setRange(6.0, 60.0)
        self.set_udc_spin.setDecimals(2)
        self.set_udc_spin.setSingleStep(0.5)
        self.set_udc_spin.setValue(12.0)
        set_udc_btn = self._bind_text(QPushButton(), "Set")
        udc_row = QHBoxLayout()
        udc_row.setSpacing(6)
        udc_row.addWidget(self.set_udc_spin)
        udc_row.addWidget(set_udc_btn)
        udc_row.addStretch(1)

        bus_layout.addRow(self.bus_voltage_label)
        bus_layout.addRow(self.udc_label)
        bus_layout.addRow(self.bus_adc_label)
        bus_layout.addRow(bus_btn_row)
        bus_layout.addRow(self._label("Set Udc (V):"), udc_row)
        bus_group.setLayout(bus_layout)
        grid.addWidget(bus_group, 6, 0, 1, 2)

        # 显示区比输入区更值得占用多余宽度，行 2/3 也允许拉伸
        grid.setColumnStretch(0, 3)
        grid.setColumnStretch(1, 2)
        grid.setRowStretch(2, 1)
        grid.setRowStretch(3, 0)
        grid.setRowStretch(4, 0)
        grid.setRowStretch(5, 2)
        layout.addLayout(grid)

        self.read_bus_btn = read_bus_btn
        self.apply_bus_btn = apply_bus_btn
        self.set_udc_btn = set_udc_btn
        read_bus_btn.clicked.connect(self.read_bus_voltage)
        apply_bus_btn.clicked.connect(self.apply_measured_as_udc)
        set_udc_btn.clicked.connect(self.set_udc_value)

        self.set_mode_btn.clicked.connect(self.set_motor_mode)
        self.get_mode_btn.clicked.connect(self.get_motor_mode)
        return widget

    def create_pid_tab(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        self.pid_widgets = {}
        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(6)
        # 四个 PID 分组排列成 2×2，而不是纵向堆叠
        for idx, name in enumerate(['Iq','Id','Speed','Position']):
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
            row, col = divmod(idx, 2)
            grid.addWidget(group, row, col)
            self.pid_widgets[name] = (p,i,d,set_btn,get_btn)
            set_btn.clicked.connect(lambda ch, n=name: self.set_pid(n))
            get_btn.clicked.connect(lambda ch, n=name: self.get_pid(n))
        layout.addLayout(grid)
        layout.addStretch(1)
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
        layout = QGridLayout(widget)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setHorizontalSpacing(8)
        layout.setVerticalSpacing(4)
        self.limit_iq_max = QDoubleSpinBox(); self.limit_iq_max.setRange(-100,100)
        self.limit_iq_min = QDoubleSpinBox(); self.limit_iq_min.setRange(-100,100)
        self.limit_id_max = QDoubleSpinBox(); self.limit_id_max.setRange(-100,100)
        self.limit_id_min = QDoubleSpinBox(); self.limit_id_min.setRange(-100,100)
        self.limit_speed_max = QDoubleSpinBox(); self.limit_speed_max.setRange(-10000,10000)
        self.limit_speed_min = QDoubleSpinBox(); self.limit_speed_min.setRange(-10000,10000)
        self.limit_position_max = QDoubleSpinBox(); self.limit_position_max.setRange(-10000,10000)
        self.limit_position_min = QDoubleSpinBox(); self.limit_position_min.setRange(-10000,10000)
        # 上限/下限并排放置，8 行压缩为 4 行
        limit_rows = (
            ("Iq max:", self.limit_iq_max, "Iq min:", self.limit_iq_min),
            ("Id max:", self.limit_id_max, "Id min:", self.limit_id_min),
            ("Speed max:", self.limit_speed_max, "Speed min:", self.limit_speed_min),
            ("Position max:", self.limit_position_max,
             "Position min:", self.limit_position_min),
        )
        for row, (max_key, max_spin, min_key, min_spin) in enumerate(limit_rows):
            layout.addWidget(self._label(max_key), row, 0)
            layout.addWidget(max_spin, row, 1)
            layout.addWidget(self._label(min_key), row, 2)
            layout.addWidget(min_spin, row, 3)
        set_btn = self._bind_text(QPushButton(), "Set Limits")
        get_btn = self._bind_text(QPushButton(), "Get Limits")
        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(6)
        btn_layout.addWidget(set_btn)
        btn_layout.addWidget(get_btn)
        btn_layout.addStretch(1)
        layout.addLayout(btn_layout, 4, 0, 1, 4)
        layout.setRowStretch(5, 1)
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
        send_btn = self._bind_text(QPushButton(), "Send")
        cmd_layout.addWidget(self.manual_cmd_edit)
        cmd_layout.addWidget(send_btn)
        cmd_group.setLayout(cmd_layout)

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

        # 两个输入框改为可拖动分栏，响应区能随窗口一起变高
        self.manual_splitter = QSplitter(Qt.Vertical)
        self.manual_splitter.addWidget(cmd_group)
        self.manual_splitter.addWidget(resp_group)
        self.manual_splitter.setChildrenCollapsible(False)
        self.manual_splitter.setStretchFactor(0, 1)
        self.manual_splitter.setStretchFactor(1, 2)
        self.manual_splitter.setSizes([220, 440])
        layout.addWidget(self.manual_splitter)

        send_btn.clicked.connect(self.send_manual_command)
        clear_btn.clicked.connect(lambda: self.manual_response_text.clear())
        return widget

    def create_imu_tab(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        # ── 顶部控制条：轮询开关一行，3D 模型选择一行。
        # 拆成两行是为了压住最小宽度——八个控件挤在一行需要 1363px，
        # 会把整个窗口撑到 1399px 宽。
        poll_bar = QHBoxLayout()
        poll_bar.setSpacing(6)
        self.imu_poll_cb = QCheckBox(tr("Enable IMU Polling"))
        self._text_bindings.append((self.imu_poll_cb, "Enable IMU Polling", "setText"))
        self.imu_poll_interval = QSpinBox()
        self.imu_poll_interval.setRange(10, 500)
        self.imu_poll_interval.setValue(50)
        poll_bar.addWidget(self.imu_poll_cb)
        poll_bar.addWidget(self._label("Interval (ms):"))
        poll_bar.addWidget(self.imu_poll_interval)
        poll_bar.addStretch(1)
        layout.addLayout(poll_bar)

        model_bar = QHBoxLayout()
        model_bar.setSpacing(6)
        self.model_combo = QComboBox()
        self.model_combo.addItem(tr(DEFAULT_CUBE), DEFAULT_CUBE)
        refresh_model_btn = self._bind_text(QPushButton(), "Refresh")
        browse_btn = self._bind_text(QPushButton(), "Browse")
        reset_btn = self._bind_text(QPushButton(), "Reset")
        model_bar.addWidget(self._label("3D Model:"))
        self.model_combo.setMinimumWidth(160)
        model_bar.addWidget(self.model_combo, stretch=1)
        model_bar.addWidget(refresh_model_btn)
        model_bar.addWidget(browse_btn)
        model_bar.addWidget(reset_btn)
        layout.addLayout(model_bar)

        # ── 主体：3D 视图在左，读数/日志在右侧可拖动分栏
        self.imu_3d_view = IMU3DWidget()
        self.imu_3d_view.setMinimumSize(360, 320)

        side = QWidget()
        side_layout = QVBoxLayout(side)
        side_layout.setContentsMargins(0, 0, 0, 0)
        side_layout.setSpacing(6)

        data_group = QGroupBox(tr("IMU Data"))
        self._text_bindings.append((data_group, "IMU Data", "setTitle"))
        data_layout = QGridLayout()
        data_layout.setHorizontalSpacing(10)
        data_layout.setVerticalSpacing(2)
        self.label_ax = QLabel("ax: ---")
        self.label_ay = QLabel("ay: ---")
        self.label_az = QLabel("az: ---")
        self.label_gx = QLabel("gx: ---")
        self.label_gy = QLabel("gy: ---")
        self.label_gz = QLabel("gz: ---")
        self.label_roll = QLabel(tr("Roll: ---"))
        self.label_pitch = QLabel(tr("Pitch: ---"))
        self.label_yaw = QLabel(tr("Yaw: ---"))
        self._text_bindings.append((self.label_roll, "Roll: ---", "setText"))
        self._text_bindings.append((self.label_pitch, "Pitch: ---", "setText"))
        self._text_bindings.append((self.label_yaw, "Yaw: ---", "setText"))

        data_layout.addWidget(self._label("Acc (g):"), 0, 0)
        data_layout.addWidget(self._label("Gyro (dps):"), 0, 1)
        data_layout.addWidget(self._label("Orientation (°):"), 0, 2)
        for row, labels in enumerate(
                ((self.label_ax, self.label_gx, self.label_roll),
                 (self.label_ay, self.label_gy, self.label_pitch),
                 (self.label_az, self.label_gz, self.label_yaw)), start=1):
            for col, label in enumerate(labels):
                data_layout.addWidget(label, row, col)
        data_layout.setColumnStretch(3, 1)
        data_group.setLayout(data_layout)
        side_layout.addWidget(data_group)

        # 调试信息与数据记录共用一组页签，避免两块文本框各占一半高度
        detail_tabs = QTabWidget()
        detail_tabs.setDocumentMode(True)

        debug_page = QWidget()
        debug_layout = QVBoxLayout(debug_page)
        debug_layout.setContentsMargins(6, 6, 6, 6)
        debug_layout.setSpacing(6)
        self.imu_debug_text = QPlainTextEdit()
        self.imu_debug_text.setReadOnly(True)
        self.imu_debug_text.setFont(QFont("Courier New", 9))
        copy_btn = self._bind_text(QPushButton(), "Copy Current IMU Data")
        copy_btn.clicked.connect(self.copy_imu_data)
        debug_layout.addWidget(self.imu_debug_text)
        debug_layout.addWidget(copy_btn)
        detail_tabs.addTab(debug_page, tr("IMU Debug Data"))

        log_page = QWidget()
        log_layout = QVBoxLayout(log_page)
        log_layout.setContentsMargins(6, 6, 6, 6)
        log_layout.setSpacing(6)
        self.imu_log_text = QPlainTextEdit()
        self.imu_log_text.setReadOnly(True)
        self.imu_log_text.setMaximumBlockCount(1000)
        self.imu_log_text.setFont(QFont("Courier New", 9))
        log_btn_layout = QHBoxLayout()
        log_btn_layout.setSpacing(6)
        self.clear_log_btn = self._bind_text(QPushButton(), "Clear Log")
        self.save_log_btn = self._bind_text(QPushButton(), "Save Log to CSV")
        self.logging_cb = QCheckBox(tr("Auto Log"))
        self._text_bindings.append((self.logging_cb, "Auto Log", "setText"))
        self.logging_cb.setChecked(True)
        log_btn_layout.addWidget(self.logging_cb)
        log_btn_layout.addWidget(self.clear_log_btn)
        log_btn_layout.addWidget(self.save_log_btn)
        log_btn_layout.addStretch(1)
        log_layout.addWidget(self.imu_log_text)
        log_layout.addLayout(log_btn_layout)
        detail_tabs.addTab(log_page, tr("IMU Data Log"))

        self.imu_detail_tabs = detail_tabs
        side_layout.addWidget(detail_tabs, stretch=1)

        self.imu_splitter = QSplitter(Qt.Horizontal)
        self.imu_splitter.addWidget(self.imu_3d_view)
        self.imu_splitter.addWidget(side)
        self.imu_splitter.setChildrenCollapsible(False)
        self.imu_splitter.setStretchFactor(0, 3)
        self.imu_splitter.setStretchFactor(1, 2)
        self.imu_splitter.setSizes([860, 520])
        layout.addWidget(self.imu_splitter, stretch=1)

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
        # 只显示当前接口的分组框，避免另一组控件白占高度
        self.serial_group.setVisible(is_serial)
        self.can_group.setVisible(not is_serial)
        self.serial_port_list.setEnabled(is_serial)
        self.refresh_ports_btn.setEnabled(is_serial)
        self.auto_refresh_ports_cb.setEnabled(is_serial)
        self.baudrate_combo.setEnabled(is_serial)
        self.can_channel_edit.setEnabled(not is_serial)
        self.can_bustype_combo.setEnabled(not is_serial)
        self.can_bitrate_edit.setEnabled(not is_serial)
        self._update_port_refresh_timer()

    def _on_auto_refresh_ports_toggled(self, _checked):
        self._update_port_refresh_timer()

    def _update_port_refresh_timer(self):
        """仅在连接页可见、未连接且用户勾选时轮询串口列表。"""
        should_run = (self.tabs.currentIndex() == TAB_CONNECTION
                      and self.comm_backend is None
                      and self.auto_refresh_ports_cb.isChecked())
        if should_run and not self.port_refresh_timer.isActive():
            self.refresh_serial_ports()
            self.port_refresh_timer.start()
        elif not should_run and self.port_refresh_timer.isActive():
            self.port_refresh_timer.stop()

    def selected_serial_port(self):
        item = self.serial_port_list.currentItem()
        return item.text() if item is not None else ""

    def refresh_serial_ports(self):
        """增量刷新串口列表：新端口插到顶部，已有的端口保持原位和选中状态。"""
        try:
            import serial.tools.list_ports
            ports = [p.device for p in serial.tools.list_ports.comports()]
        except Exception:
            return

        selected = self.selected_serial_port()
        existing = [self.serial_port_list.item(i).text()
                    for i in range(self.serial_port_list.count())]

        # 拔掉的端口移除
        for i in range(self.serial_port_list.count() - 1, -1, -1):
            if self.serial_port_list.item(i).text() not in ports:
                self.serial_port_list.takeItem(i)

        # 新出现的端口按枚举顺序插到顶部（最后插入的排最前）
        for port in ports:
            if port not in existing:
                self.serial_port_list.insertItem(0, port)

        if selected:
            for i in range(self.serial_port_list.count()):
                if self.serial_port_list.item(i).text() == selected:
                    self.serial_port_list.setCurrentRow(i)
                    return
        if self.serial_port_list.count() and self.serial_port_list.currentRow() < 0:
            self.serial_port_list.setCurrentRow(0)

    def toggle_connection(self):
        if self.comm_backend:
            self.disconnect()
        else:
            self.connect_device()

    def connect_device(self):
        if self.interface_combo.currentData() == "serial":
            port = self.selected_serial_port()
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

        # 连上后立刻自动广播一次电机 ID 检测，省去手动点按钮
        self.detect_motor_id(auto=True)

    def disconnect(self):
        self.stop_auto_refresh()
        self.detect_timeout_timer.stop()
        self.detect_pending = False
        self.pending_target_timer.stop()
        self.pending_target = None
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
        self.detected_ids = []
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

        if packet.func2 in (0x30, 0x31, 0x32, 0x33, 0x39) and self.auto_refresh_enabled:
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
        elif packet.func2 == 0x39:
            self.handle_bus_voltage_response(packet)
        elif packet.func2 == 0x3B:
            self.handle_udc_response(packet)
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

    def detect_motor_id(self, auto=False):
        if not self.comm_backend:
            return
        if self.detect_pending:
            # 上一次广播还没等到应答，不重复发送
            return
        self.detect_pending = True
        self.detect_auto = auto
        self.detect_btn.setEnabled(False)
        self.status_label.setText(tr("Searching for motor ID..."))
        self._refresh_status_color()
        self.send_command(0x00, 0x00, motor_id=0xFFFF)
        self.detect_timeout_timer.start(DETECT_TIMEOUT_MS)

    def on_detect_timeout(self):
        """广播后超时仍无应答：提示用户并恢复按钮。"""
        if not self.detect_pending:
            return
        self.detect_pending = False
        self.detect_auto = False
        if self.comm_backend is not None:
            self.detect_btn.setEnabled(True)
        if self.detected_ids:
            self.status_label.setText(tr("Connected"))
        else:
            self.status_label.setText(tr("Motor ID not detected"))
        self._refresh_status_color()
        QMessageBox.warning(self, tr("Detect"), tr("Motor ID not detected"))

    def _finish_detect(self):
        self.detect_timeout_timer.stop()
        auto = self.detect_auto
        self.detect_pending = False
        self.detect_auto = False
        if self.comm_backend is not None:
            self.detect_btn.setEnabled(True)
        return auto

    def handle_detect_response(self, packet):
        auto = self._finish_detect()
        ids = []
        for i in range(1,5):
            val = getattr(packet, f'data{i}').as_uint32()
            if (val & 0xFFFF0000) == 0xFFFF0000:
                ids.append(val & 0xFFFF)
            if (val & 0x0000FFFF) != 0x0000FFFF:
                ids.append(val & 0xFFFF)
        if packet.motor_id != 0xFFFF:
            ids.append(packet.motor_id)
        ids = [i for i in dict.fromkeys(ids) if i != 0xFFFF]
        self.detected_ids = ids
        self.motor_id_combo.clear()
        connected = self.comm_backend is not None
        if ids:
            for i in ids:
                self.motor_id_combo.addItem(str(i), i)
            self.motor_id = ids[0]
            self.motor_id_label.setText(str(ids[0]))
            if connected:
                self.status_label.setText(tr("Connected"))
                self._refresh_status_color()
            # 连接后自动检测不再弹窗打扰，手动检测仍然给出确认
            if not auto:
                QMessageBox.information(self, tr("Detect"),
                                        tr("Detected IDs: {}").format(ids))
        else:
            if connected:
                self.status_label.setText(tr("Motor ID not detected"))
                self._refresh_status_color()
            if not auto:
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
        self.update_active_target_highlight()

    def update_active_target_highlight(self):
        """把当前工作模式下真正生效的目标值加粗，其余保持常规字重。"""
        active = MODE_TARGET_KEYS.get(self.mode_combo.currentData(), ())
        font = self.target_hint.font()
        for key, label in self.target_labels.items():
            bold = key in active
            if label.font().bold() != bold:
                new_font = QFont(font)
                new_font.setBold(bold)
                label.setFont(new_font)
            spin, set_btn = self.target_set_btns[key]
            tip = tr("Active target for the current mode") if bold else tr(
                "Not used by the current mode")
            spin.setToolTip(tip)
            set_btn.setToolTip(tip)

    def get_motor_parameters(self):
        mid = self.get_current_motor_id()
        if mid == 0:
            return
        self.send_command(0x02, 0x00, motor_id=mid)

    # ---------- 母线电压 / Udc ----------
    def read_bus_voltage(self, quiet=False):
        """主动读一次母线电压。quiet=True 时用于自动轮询，不弹提示。"""
        mid = self.get_current_motor_id()
        if mid == 0:
            if not quiet:
                QMessageBox.warning(self, tr("Warning"), tr("No motor ID selected"))
            return
        self.send_command(0x39, 0x00, motor_id=mid)

    def handle_bus_voltage_response(self, packet):
        """固件应答 0x39：data1=实测电压, data2=当前 Udc, data3=K, data4=原始 ADC 码。"""
        voltage = packet.data1.as_float()
        udc = packet.data2.as_float()
        adc = int(round(packet.data4.as_float()))
        if voltage < 0.0:
            # 固件侧 ADC 启动或转换失败返回 -1.0，别把负值当成真电压显示
            self.bus_voltage_label.setText(tr("Measured: --- V"))
            self.status_label.setText(tr("Bus voltage read failed"))
            return
        self.bus_voltage_label.setText(
            tr("Measured: {v} V").format(v=f"{voltage:.2f}"))
        self.bus_adc_label.setText(tr("ADC: {n}").format(n=adc))
        self.udc_label.setText(tr("Active Udc: {v} V").format(v=f"{udc:.2f}"))
        self._last_bus_voltage = voltage

    def handle_udc_response(self, packet):
        """固件应答 0x3B：data1=当前 Udc, data2=当前 K（写入后回读也是这个格式）。"""
        udc = packet.data1.as_float()
        self.udc_label.setText(tr("Active Udc: {v} V").format(v=f"{udc:.2f}"))
        self.set_udc_spin.setValue(max(6.0, min(60.0, udc)))

    def apply_measured_as_udc(self):
        """把最后一次实测值填进输入框。不直接下发——下发必须再点一次 Set。"""
        measured = getattr(self, "_last_bus_voltage", None)
        if measured is None:
            QMessageBox.warning(self, tr("Warning"),
                                tr("Read the bus voltage first"))
            return
        self.set_udc_spin.setValue(max(6.0, min(60.0, measured)))
        self.status_label.setText(
            tr("Measured voltage copied to Udc box; press Set to write"))

    def set_udc_value(self):
        """手动写入 Udc，固件会同步重算 K = sqrt(3)*Ts/Udc。"""
        mid = self.get_current_motor_id()
        if mid == 0:
            QMessageBox.warning(self, tr("Warning"), tr("No motor ID selected"))
            return
        udc = float(self.set_udc_spin.value())
        self.send_command(0x3B, 0x01, udc, motor_id=mid)
        self.status_label.setText(tr("Udc set to {v} V").format(v=f"{udc:.2f}"))

    def handle_pole_pair_response(self, packet):
        # func3=0x00 是读取应答，0x01 是写入后的回读应答，两种都要刷新界面
        if packet.func3 in (0x00, 0x01):
            pole_pair = packet.data1.as_uint32()
            offset = packet.data2.as_float()
            direction = packet.data3.as_uint32()
            self.pole_pair_label.setText(str(pole_pair))
            self.offset_label.setText(f"{offset:.3f}°")
            self.encoder_dir_label.setText(str(direction))
            # 顺手填进写入框，用户想改哪一项就在哪一项上改，不用重复输入
            self.set_pole_pair_spin.setValue(max(1, min(64, int(pole_pair))))
            self.set_offset_spin.setValue(max(0.0, min(359.999, offset)))
            self._fill_combo(self.set_dir_combo, [(1, "Enable"), (0, "Disable")], keep=False)
            idx = self.set_dir_combo.findData(1 if direction else 0)
            if idx >= 0:
                self.set_dir_combo.setCurrentIndex(idx)

    def set_motor_parameters(self):
        """手动写入极对数/零点偏移/编码器方向，写入 0x02 且 func3=0x01。

        固件收到后立刻写 Flash（掉电保留），所以按一次就够，不需要重复烧录。
        """
        mid = self.get_current_motor_id()
        if mid == 0:
            QMessageBox.warning(self, tr("Warning"), tr("No motor ID selected"))
            return
        pole_pair = int(self.set_pole_pair_spin.value())
        offset = float(self.set_offset_spin.value())
        direction = int(self.set_dir_combo.currentData() or 0)
        self.send_command(0x02, 0x01, pole_pair, offset, direction, motor_id=mid)
        self.status_label.setText(
            f"{tr('Set Motor Parameters')}: PP={pole_pair}, "
            f"Offset={offset:.3f}°, Dir={direction}")

    def clear_calibration(self):
        """清掉 Flash 里保存的校准记录，恢复固件自带的默认参数。

        校准把人搞糊涂的时候（比如装机后读数明显不合理）用这个回到出厂状态，
        然后重新走一遍 Calibration 模式即可。
        """
        mid = self.get_current_motor_id()
        if mid == 0:
            QMessageBox.warning(self, tr("Warning"), tr("No motor ID selected"))
            return
        # 0x02 / func3=0x02：擦掉校准页，改回编译期默认值（7 / 1 / 301.464°）
        self.send_command(0x02, 0x02, motor_id=mid)
        self.pole_pair_label.setText("7")
        self.offset_label.setText("301.464°")
        self.encoder_dir_label.setText("1")
        self.set_pole_pair_spin.setValue(7)
        self.set_offset_spin.setValue(301.464)
        idx = self.set_dir_combo.findData(1)
        if idx >= 0:
            self.set_dir_combo.setCurrentIndex(idx)
        self.status_label.setText(tr("Clear Calibration"))

    def set_targets(self):
        mid = self.get_current_motor_id()
        if mid == 0:
            QMessageBox.warning(self, tr("Warning"), tr("No motor ID"))
            return
        self.send_command(0x20,0x01, data1=self.target_iq.value(), motor_id=mid)
        self.send_command(0x21,0x01, data1=self.target_id.value(), motor_id=mid)
        self.send_command(0x22,0x01, data1=self.target_speed.value(), motor_id=mid)
        self.send_command(0x23,0x01, data1=self.target_position.value(), motor_id=mid)
        self.send_command(0x24,0x01, data1=self.target_uq.value(), data2=self.target_ud.value(), motor_id=mid)
        # 输入框里现在存的是“刚下发的指令”，不能被轮询回读改写，
        # 否则下一次 Set 发的就不是用户填的值了。
        self.last_sent_speed = self.target_speed.value()
        self.last_sent_position = self.target_position.value()
        self._mark_target_edited("Speed (rpm):")
        self._mark_target_edited("Position (deg):")

    def _target_command(self, key):
        """返回某个目标值的 (功能码, 取值函数, 需要的数据字个数)。"""
        if key == "Iq:":
            return 0x20, lambda: self.target_iq.value(), 1
        if key == "Id:":
            return 0x21, lambda: self.target_id.value(), 1
        if key == "Speed (rpm):":
            return 0x22, lambda: self.target_speed.value(), 1
        if key == "Position (deg):":
            return 0x23, lambda: self.target_position.value(), 1
        if key == "Uq:":
            return 0x24, lambda: self.target_uq.value(), 2
        if key == "Ud:":
            # Ud 与 Uq 共用 0x24，两个数据字必须一起发
            return 0x24, lambda: self.target_ud.value(), 2
        return None, None, 0

    def set_target_value(self, key):
        """只发送一个目标值；速度和位置会顺带把工作模式切到对应闭环。"""
        func2, getter, words = self._target_command(key)
        if func2 is None:
            return
        mid = self.get_current_motor_id()
        if mid == 0:
            QMessageBox.warning(self, tr("Warning"), tr("No motor ID"))
            return
        if key == "Speed (rpm):":
            self.last_speed_cmd = self.target_speed.value()
            self.last_sent_speed = self.target_speed.value()
            self._persist_settings()
            # 发出去之后输入框里存的就是指令本身，锁定它不让轮询回读改写
            self._mark_target_edited(key)
        elif key == "Position (deg):":
            self.last_position_cmd = self.target_position.value()
            self.last_sent_position = self.target_position.value()
            self._persist_settings()
            self._mark_target_edited(key)
        target_mode = {"Speed (rpm):": MODE_SPEED_LOOP,
                       "Position (deg):": MODE_POSITION_LOOP}.get(key)
        # 需要先切模式时，目标值必须等模式生效后再发（见 _switch_mode_for_target）
        if target_mode is not None and self.mode_combo.currentData() != target_mode:
            self.pending_target = (func2, getter, words, mid)
            self._switch_mode_for_target(target_mode, mid)
            return
        self._send_target(func2, getter, words, mid)

    def _send_target(self, func2, getter, words, mid):
        """真正把目标值写下去；words == 2 表示 Uq/Ud 共用一个功能码。"""
        if words == 2:
            self.send_command(func2, 0x01, data1=self.target_uq.value(),
                              data2=self.target_ud.value(), motor_id=mid)
        else:
            self.send_command(func2, 0x01, data1=getter(), motor_id=mid)

    def _flush_pending_target(self):
        """模式切换的等待时间到了，补发之前挂起的目标值。"""
        pending, self.pending_target = self.pending_target, None
        if pending is None or not self.comm_backend:
            return
        func2, getter, words, mid = pending
        # 期间用户可能换了电机，按当时的电机号发送
        self._send_target(func2, getter, words, self.get_current_motor_id() or mid)

    def _switch_mode_for_target(self, mode, mid):
        """把电机切到指定闭环模式；模式设置与目标值之间保持间隔。

        模式指令和速度/位置指令挨着发出去时，固件往往还在执行模式切换，
        新目标值会被当作切换过程中的扰动，表现为电机左右抖动。
        调用方需先确认当前模式与 mode 不同，并且已设置好 pending_target。
        """
        self.send_command(0x01, 0x01, data1=mode, motor_id=mid)
        self.handle_mode_response(mode)
        if self.pending_target is not None:
            self.pending_target_timer.start()

    def stop_motor(self):
        mid = self.get_current_motor_id()
        if mid == 0:
            QMessageBox.warning(self, tr("Warning"), tr("No motor ID"))
            return
        self.pending_target_timer.stop()
        self.pending_target = None
        self.send_command(0x01, 0x01, data1=MODE_STOP, motor_id=mid)
        self.handle_mode_response(MODE_STOP)

    def get_targets(self, user_initiated=False):
        """读回目标值。

        user_initiated 只有点“Get All”按钮时才为 True。自动轮询每秒也会调这里，
        那时绝不能清“已编辑”标记——否则用户刚填的值会在 1 秒内被回读覆盖掉，
        表现就是“要填好几次才能设置成功”。
        """
        mid = self.get_current_motor_id()
        if mid == 0:
            return
        self.send_command(0x20,0x00, motor_id=mid)
        self.send_command(0x21,0x00, motor_id=mid)
        self.send_command(0x22,0x00, motor_id=mid)
        self.send_command(0x23,0x00, motor_id=mid)
        self.send_command(0x24,0x00, motor_id=mid)
        if user_initiated:
            # 显式刷新：以固件状态为准，清掉“已编辑”标记，回读才允许改写输入框
            self._clear_target_edited("Speed (rpm):")
            self._clear_target_edited("Position (deg):")

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
            self._apply_target_readback("Speed (rpm):", self.target_speed, packet.data1.as_float())
        elif name == "Position":
            self._apply_target_readback("Position (deg):", self.target_position, packet.data1.as_float())
        elif name == "UqUd":
            self.target_uq.setValue(packet.data1.as_float())
            self.target_ud.setValue(packet.data2.as_float())

    def _mark_target_edited(self, key):
        """标记这个输入框由用户掌管（敲过值，或刚发过 Set）。

        标记为 True 后，自动轮询的回读一律不写回输入框，
        保证框里始终是用户填的指令，下一次 Set 发的值不会被悄悄改掉。
        """
        self.target_edited[key] = True

    def _clear_target_edited(self, key):
        """交还控制权：允许回读把固件里的值填进输入框（只在显式 Get 时调用）。"""
        self.target_edited[key] = False

    def _detect_speed_scale(self, raw_value):
        """自动判断固件速度回读用的是哪种单位。

        刚下发过速度时，拿回读值和下发值比一下：

        - 比值 ≈ 1          → 固件回读的就是 rpm（已修复的固件）
        - 比值 ≈ 1/9.5238095 → 固件回读的是内部 rad/s 值（未修复的固件）

        这样无论板子上烧的是哪个版本，显示都对，不用手改常量。
        比值落在两者之间（比如被限幅截过）时不乱猜，沿用上次的判断。
        """
        sent = self.last_sent_speed
        if sent is None:
            # 本次会话还没发过速度，用上次会话存下来的指令当参照
            sent = self.last_speed_cmd
        if not sent or abs(sent) < 1e-6 or abs(raw_value) < 1e-6:
            return
        ratio = raw_value / sent
        tol = 0.05
        if abs(ratio - _FW_RATIO_NEW) < tol:
            self.firmware_speed_scale = 1.0
        elif abs(ratio - _FW_RATIO_OLD) < tol:
            self.firmware_speed_scale = FIRMWARE_SPEED_READBACK_SCALE

    def _apply_target_readback(self, key, spin, raw_value):
        """把固件回读的目标值写回输入框。

        只在用户没有掌管这个框时才会真正写入（见 _mark_target_edited）：
        自动轮询碰不到用户填的值，只有显式点 Get All 才会刷新显示。
        速度要乘自动校准出来的 firmware_speed_scale，位置原样写回。
        """
        if key == "Speed (rpm):":
            self._detect_speed_scale(raw_value)
        if self.target_edited.get(key):
            return
        # 光标还在框里说明正在输入，这时候回写会把用户敲一半的值顶掉
        if spin.hasFocus() and spin.text() != "":
            return
        value = raw_value
        if key == "Speed (rpm):":
            value = raw_value * self.firmware_speed_scale
        spin.blockSignals(True)
        spin.setValue(value)
        spin.blockSignals(False)

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
        # 连接页的串口列表只在可见时轮询（_apply_polling_gates 已处理）

    def _apply_polling_gates(self):
        """按"用户意图 + 当前可见标签页 + 连接状态"统一启停各类轮询。

        每个 *_enabled 标志只表示"此刻真的在轮询"，用户勾选状态始终以复选框为准，
        这样切回标签页或重新勾选时都能恢复到之前的状态。
        """
        connected = self.comm_backend is not None
        index = self.tabs.currentIndex()
        motor_id = self.get_current_motor_id()

        # 0. 连接页的串口列表自动刷新
        self._update_port_refresh_timer()

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
        # 母线电压也顺手读一次，方便观察电源是否掉压
        self.read_bus_voltage(quiet=True)

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
        # 若板端配置不同，请修改以下系数（量程偏大/偏小会让静止读数偏离 0）
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

        # 先做中值滤波，保证零偏不被尖峰污染，也不会把尖峰积分成虚假偏航
        gx_gui, gy_gui, gz_gui = self._filter_gyro_median(gx_gui, gy_gui, gz_gui)

        # 加速度计合矢量校验：坏帧（az 被读成 0、或数值异常）算出的 acc_roll /
        # acc_pitch 毫无意义，必须整帧跳过互补滤波，否则姿态会被瞬间拉偏再慢慢
        # 爬回原位，看起来就是“抖一下又回正”。
        accel_norm = math.sqrt(ax_gui * ax_gui + ay_gui * ay_gui + az_gui * az_gui)
        accel_usable = accel_norm > 0 and (
            self.accel_norm_min <= accel_norm <= self.accel_norm_max)
        if accel_norm > 0 and not accel_usable:
            print("[IMU] Bad accel frame skipped: |a| = {:.3f} g".format(accel_norm))
            self.status_label.setText(tr("IMU bad accel frame skipped"))
        # 自动零偏校准：零偏本身可能很大（实测静态偏移可达 ~37 dps），所以不能
        # 用“角速度绝对值”判断设备是否在动，否则静止样本会被全部丢弃、零偏退化为 0。
        # 改为判断样本是否偏离已采集样本的中位数：静止读数彼此接近，转动读数会明显偏离。
        # 零偏取中位数而非均值，偶发的读数尖峰（日志中确实存在）不会污染结果。
        # 坏帧既不参与校准，也不阻止校准收敛（否则坏帧会让校准永远差几个样本）。
        if self.calibrating_gyro and accel_usable:
            self.calib_samples += 1
            sample = (gx_gui, gy_gui, gz_gui)
            moving = False
            if self.calib_buffer:
                med = self._median_axes(self.calib_buffer)
                deviation = math.sqrt(sum((sample[i] - med[i]) ** 2 for i in range(3)))
                moving = deviation >= self.calib_motion_dps
            if not moving:
                self.calib_buffer.append(sample)
            if (len(self.calib_buffer) >= self.calib_min_samples
                    or self.calib_samples >= self.calib_max_samples):
                # 收尾时优先用采到的静止样本；一个都没采到就退回当前读数
                self.gyro_bias = self._median_axes(self.calib_buffer or [sample])
                self.calibrating_gyro = False
                print("[IMU] Calibration done. Bias: {:.2f}, {:.2f}, {:.2f} dps".format(
                    self.gyro_bias[0], self.gyro_bias[1], self.gyro_bias[2]))
                self.status_label.setText(
                    tr("IMU ready (bias: {}, {}, {})").format(
                        round(self.gyro_bias[0], 1), round(self.gyro_bias[1], 1),
                        round(self.gyro_bias[2], 1)))
                self.calib_buffer.clear()
            else:
                self.status_label.setText(
                    tr("Calibrating IMU... {}/{}").format(
                        len(self.calib_buffer), self.calib_min_samples))

        # 校准期间用已采样本的中位数即时扣除：避免在拿到零偏之前把静态偏移
        # 积分成虚假偏航（校准结束后 gyro_bias 本身就是这个中位数，数值连续）。
        # 还没采到样本时直接以当前读数为零偏，第一帧起就不会虚假转动。
        if self.calibrating_gyro:
            bias = self._median_axes(self.calib_buffer) if self.calib_buffer \
                else [gx_gui, gy_gui, gz_gui]
        else:
            bias = self.gyro_bias

        # 减去零偏，并消掉残余的微小读数
        gx_cal = self._apply_gyro_deadband(gx_gui - bias[0])
        gy_cal = self._apply_gyro_deadband(gy_gui - bias[1])
        gz_cal = self._apply_gyro_deadband(gz_gui - bias[2])

        current_time = time.time()
        dt = current_time - self.last_imu_time
        # 单次积分跨度上限 0.1s，超出部分不再累积，避免丢包后姿态跳变
        dt = max(0.001, min(0.1, dt))
        self.last_imu_time = current_time

        # 运动学一致性校验：陀螺仪预测的重力方向变化 vs 加速度计实测变化。
        # 只有本帧和上一帧的加速度计都可信、且角速度足够大时才判断——静止时
        # d(ĝ)/dt 与 ω×ĝ 都接近 0，残差完全被噪声支配，判断没有意义
        # （实测静止噪声到 0.05g 都不会误触发，因为前置了 consist_min_dps）。
        gyro_trustworthy = True
        if (not self.calibrating_gyro) and accel_usable and self.imu_prev_good \
                and self.imu_prev_unit is not None:
            g_cal = (gx_cal, gy_cal, gz_cal)
            if math.sqrt(g_cal[0]**2 + g_cal[1]**2 + g_cal[2]**2) >= self.consist_min_dps:
                cur_unit = (ax_gui / accel_norm, ay_gui / accel_norm, az_gui / accel_norm)
                gp = self.imu_prev_unit
                dg = [(cur_unit[i] - gp[i]) / dt for i in range(3)]
                # ω 用校准后的角速度（弧度/秒），单位与 dg 一致
                omega = [math.radians(v) for v in g_cal]
                gpx, gpy, gpz = gp
                cross = (omega[1] * gpz - omega[2] * gpy,
                         omega[2] * gpx - omega[0] * gpz,
                         omega[0] * gpy - omega[1] * gpx)
                # 传感器坐标轴与推导姿态的符号约定未完全确定，两种符号都试、取更接近的，
                # 否则正常转动会被误判为坏帧
                d1 = math.sqrt(sum((dg[i] - cross[i]) ** 2 for i in range(3)))
                d2 = math.sqrt(sum((dg[i] + cross[i]) ** 2 for i in range(3)))
                residual_deg = math.degrees(min(d1, d2))
                if residual_deg > self.consist_tol_deg:
                    self.imu_bad_streak += 1
                    if self.imu_bad_streak >= self.consist_nbad:
                        gyro_trustworthy = False
                        print("[IMU] Inconsistent gyro frame skipped: "
                              "residual = {:.0f} dps, |g| = {:.0f} dps".format(
                                  residual_deg,
                                  math.sqrt(g_cal[0]**2 + g_cal[1]**2 + g_cal[2]**2)))
                        self.status_label.setText(tr("IMU inconsistent gyro frame skipped"))
                else:
                    self.imu_bad_streak = 0
        # 加速度计不可用的帧同样不能积分：这类帧的角速度往往一起被干扰成几百 dps，
        # 却又无法做一致性校验（没有可信的重力方向），只能保守冻结。
        if (not self.calibrating_gyro) and not accel_usable and self.freeze_bad_accel:
            gyro_trustworthy = False
        # 记录本帧状态供下一帧比较
        if accel_usable and accel_norm > 1e-9:
            self.imu_prev_unit = (ax_gui / accel_norm, ay_gui / accel_norm, az_gui / accel_norm)
            self.imu_prev_good = True
        else:
            self.imu_prev_good = False

        # 校准期间也持续积分，3D 模型从第一帧起就能跟随转动
        if self.filter is None:
            self.filter = ComplementaryFilter(dt=0.02, alpha=0.92)

        self.filter.dt = dt
        q = self.filter.update(gx_cal, gy_cal, gz_cal, ax_gui, ay_gui, az_gui,
                               accel_usable, gyro_trustworthy)

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

    def _filter_gyro_median(self, gx, gy, gz):
        # 5 点中值滤波：逐轴取最近五帧的中位数。连续 2 帧以内的尖峰会被完全
        # 消除，连续的真实转动不受影响（稳态下中值即原值，仅延迟 2 帧）。
        self.gyro_history.append((gx, gy, gz))
        if len(self.gyro_history) > 5:
            del self.gyro_history[0]
        if len(self.gyro_history) < 5:
            return gx, gy, gz
        out = []
        for i in range(3):
            col = sorted(s[i] for s in self.gyro_history)
            out.append(col[2])
        if max(abs(out[i] - (gx, gy, gz)[i]) for i in range(3)) >= self.gyro_outlier_dps:
            print("[IMU] Outlier frame filtered: raw ({:.1f}, {:.1f}, {:.1f}) dps".format(gx, gy, gz))
            self.status_label.setText(tr("IMU outlier frame discarded"))
        return out[0], out[1], out[2]

    @staticmethod
    def _median_axes(samples):
        # 每个轴单独取中位数：对偶发的读数尖峰不敏感
        n = len(samples)
        med = []
        for i in range(3):
            col = sorted(s[i] for s in samples)
            med.append(col[n // 2] if n % 2 else 0.5 * (col[n // 2 - 1] + col[n // 2]))
        return med

    def _apply_gyro_deadband(self, value):
        # 零偏校准后的残差通常只有零点几 dps，直接积分会缓慢累积成漂移
        if abs(value) < self.gyro_deadband_dps:
            return 0.0
        return value

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
        self._imu_log_header_written = False

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
        if not getattr(self, '_imu_log_header_written', False):
            self.imu_log_text.appendPlainText(
                "timestamp, ax(g), ay(g), az(g), gx(dps), gy(dps), gz(dps), "
                "roll(deg), pitch(deg), yaw(deg)")
            self._imu_log_header_written = True
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
            self.gyro_history = []
            self.gyro_bias = [0.0, 0.0, 0.0]
            # 一致性校验的跨帧状态必须清空，否则会拿上一次运行的旧方向做比较
            self.imu_prev_unit = None
            self.imu_prev_good = False
            self.imu_bad_streak = 0
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