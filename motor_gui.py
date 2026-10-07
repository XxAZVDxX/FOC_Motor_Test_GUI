#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import struct
import queue
import time
from collections import deque
from datetime import datetime
import numpy as np
import math
import glob

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QTabWidget, QGroupBox, QFormLayout,
                             QLabel, QLineEdit, QPushButton, QComboBox,
                             QDoubleSpinBox, QCheckBox, QMessageBox, QFileDialog,
                             QTextEdit, QPlainTextEdit, QSpinBox, QSlider)
from PyQt5.QtCore import QThread, pyqtSignal, QTimer, Qt, QRectF
from PyQt5.QtGui import QPainter, QPen, QBrush, QColor, QFont, QPolygonF
import pyqtgraph as pg
import serial
import serial.tools.list_ports

# 3D support
from pyqtgraph.opengl import GLViewWidget, GLMeshItem, MeshData, GLGridItem
import pyqtgraph.opengl as gl

# 3D model loading (optional)
try:
    import trimesh
    TRIMESH_AVAILABLE = True
except ImportError:
    TRIMESH_AVAILABLE = False

# CAN support (optional)
try:
    import can
    CAN_AVAILABLE = True
except ImportError:
    CAN_AVAILABLE = False


# ----------------------------------------------------------------------
# Protocol constants
# ----------------------------------------------------------------------
PACKAGE_SIZE = 24
HEAD = 0xDE
TAIL = 0xED


# ----------------------------------------------------------------------
# Helper classes for protocol data conversion
# ----------------------------------------------------------------------
class FloatUint32IntChar:
    def __init__(self, value=0):
        self.data = bytearray(4)
        self.set_value(value)

    def set_value(self, value):
        if isinstance(value, float):
            self.data = struct.pack('<f', value)
        elif isinstance(value, int):
            self.data = struct.pack('<I', value)
        else:
            self.data = value

    def as_float(self):
        return struct.unpack('<f', self.data)[0]

    def as_uint32(self):
        return struct.unpack('<I', self.data)[0]

    def as_int32(self):
        return struct.unpack('<i', self.data)[0]

    def get_bytes(self):
        return bytes(self.data)


class CommandPacket:
    def __init__(self, func1=0x1A, func2=0, func3=0,
                 data1=0, data2=0, data3=0, data4=0, motor_id=0, sender=0):
        self.head = HEAD
        self.func1 = func1
        self.func2 = func2
        self.func3 = func3
        self.data1 = FloatUint32IntChar(data1)
        self.data2 = FloatUint32IntChar(data2)
        self.data3 = FloatUint32IntChar(data3)
        self.data4 = FloatUint32IntChar(data4)
        self.motor_id = motor_id
        self.sender = sender
        self.tail = TAIL

    def build(self):
        packet = bytearray()
        packet.append(self.head)
        packet.append(self.func1)
        packet.append(self.func2)
        packet.append(self.func3)
        packet.extend(self.data1.get_bytes())
        packet.extend(self.data2.get_bytes())
        packet.extend(self.data3.get_bytes())
        packet.extend(self.data4.get_bytes())
        packet.extend(struct.pack('>H', self.motor_id))
        packet.append(self.sender)
        packet.append(self.tail)
        return bytes(packet)

    @staticmethod
    def parse(data):
        if len(data) != PACKAGE_SIZE or data[0] != HEAD or data[-1] != TAIL:
            return None
        p = CommandPacket()
        p.head = data[0]
        p.func1 = data[1]
        p.func2 = data[2]
        p.func3 = data[3]
        p.data1 = FloatUint32IntChar(data[4:8])
        p.data2 = FloatUint32IntChar(data[8:12])
        p.data3 = FloatUint32IntChar(data[12:16])
        p.data4 = FloatUint32IntChar(data[16:20])
        p.motor_id = struct.unpack('>H', data[20:22])[0]
        p.sender = data[22]
        p.tail = data[23]
        return p


# ----------------------------------------------------------------------
# Communication backend (abstract)
# ----------------------------------------------------------------------
class CommBackend(QThread):
    packet_received = pyqtSignal(object)
    raw_data_received = pyqtSignal(bytes)
    error_occurred = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.running = False
        self.tx_queue = queue.Queue()
        self.raw_tx_queue = queue.Queue()

    def send_packet(self, packet):
        self.tx_queue.put(packet)

    def send_raw(self, data: bytes):
        self.raw_tx_queue.put(data)

    def stop(self):
        self.running = False
        self.wait()


class SerialBackend(CommBackend):
    def __init__(self, port, baudrate, timeout=0.1):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None

    def run(self):
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            self.running = True
        except Exception as e:
            self.error_occurred.emit(f"Failed to open serial port: {e}")
            return

        buffer = bytearray()
        while self.running:
            try:
                while True:
                    pkt = self.tx_queue.get_nowait()
                    data = pkt.build()
                    self.serial.write(data)
            except queue.Empty:
                pass
            try:
                while True:
                    raw_data = self.raw_tx_queue.get_nowait()
                    self.serial.write(raw_data)
                    self.raw_data_received.emit(raw_data)
            except queue.Empty:
                pass

            try:
                read_data = self.serial.read(1)
                if read_data:
                    buffer.extend(read_data)
                    self.raw_data_received.emit(read_data)
                    while len(buffer) >= PACKAGE_SIZE:
                        if buffer[0] == HEAD:
                            if buffer[PACKAGE_SIZE-1] == TAIL:
                                packet = CommandPacket.parse(buffer[:PACKAGE_SIZE])
                                if packet:
                                    self.packet_received.emit(packet)
                                buffer = buffer[PACKAGE_SIZE:]
                            else:
                                buffer = buffer[1:]
                        else:
                            buffer = buffer[1:]
            except Exception as e:
                self.error_occurred.emit(f"Serial read error: {e}")
                time.sleep(0.01)

        if self.serial:
            self.serial.close()


class CANBackend(CommBackend):
    def __init__(self, channel, bustype='pcan', bitrate=500000):
        super().__init__()
        self.channel = channel
        self.bustype = bustype
        self.bitrate = bitrate
        self.bus = None

    def run(self):
        if not CAN_AVAILABLE:
            self.error_occurred.emit("python-can not installed. CAN not available.")
            return
        try:
            self.bus = can.interface.Bus(channel=self.channel, bustype=self.bustype, bitrate=self.bitrate)
            self.running = True
        except Exception as e:
            self.error_occurred.emit(f"Failed to open CAN bus: {e}")
            return

        buffer = bytearray()
        expected_seq = 0
        while self.running:
            try:
                while True:
                    pkt = self.tx_queue.get_nowait()
                    data = pkt.build()
                    for seq in range(3):
                        frame_data = bytearray(8)
                        frame_data[0] = seq
                        start = seq * 7
                        for i in range(7):
                            if start + i < len(data):
                                frame_data[1 + i] = data[start + i]
                        msg = can.Message(arbitration_id=0x123, data=frame_data, is_extended_id=True)
                        self.bus.send(msg)
            except queue.Empty:
                pass
            try:
                while True:
                    raw_data = self.raw_tx_queue.get_nowait()
                    for seq in range(3):
                        frame_data = bytearray(8)
                        frame_data[0] = seq
                        start = seq * 7
                        for i in range(7):
                            if start + i < len(raw_data):
                                frame_data[1 + i] = raw_data[start + i]
                        msg = can.Message(arbitration_id=0x123, data=frame_data, is_extended_id=True)
                        self.bus.send(msg)
                    self.raw_data_received.emit(raw_data)
            except queue.Empty:
                pass

            msg = self.bus.recv(0.01)
            if msg:
                if msg.arbitration_id == 0x123 and len(msg.data) == 8:
                    seq = msg.data[0]
                    if seq == 0:
                        buffer = bytearray()
                        expected_seq = 0
                    if seq == expected_seq and expected_seq < 3:
                        buffer.extend(msg.data[1:])
                        expected_seq += 1
                        if expected_seq == 3 and len(buffer) >= PACKAGE_SIZE:
                            self.raw_data_received.emit(buffer[:PACKAGE_SIZE])
                            packet = CommandPacket.parse(buffer[:PACKAGE_SIZE])
                            if packet:
                                self.packet_received.emit(packet)
                            buffer = bytearray()
                            expected_seq = 0

        if self.bus:
            self.bus.shutdown()


# ----------------------------------------------------------------------
# Motor Preview Widget (圆形仪表盘)
# ----------------------------------------------------------------------
class MotorPreviewWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.current_angle = 0.0
        self.setMinimumSize(200, 200)
        self.setMaximumSize(300, 300)

    def set_angle(self, angle_deg):
        self.current_angle = angle_deg % 360.0
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        rect = self.rect()
        side = min(rect.width(), rect.height())
        painter.setViewport((rect.width() - side) // 2, (rect.height() - side) // 2, side, side)
        painter.setWindow(-100, -100, 200, 200)

        painter.setPen(QPen(QColor(80, 80, 80), 2))
        painter.setBrush(QBrush(QColor(240, 240, 240)))
        painter.drawEllipse(-90, -90, 180, 180)

        font = QFont("Arial", 8)
        painter.setFont(font)
        for angle in range(0, 360, 30):
            rad = math.radians(angle)
            x1 = 85 * math.cos(rad)
            y1 = 85 * math.sin(rad)
            x2 = 75 * math.cos(rad)
            y2 = 75 * math.sin(rad)
            painter.drawLine(int(x1), int(y1), int(x2), int(y2))
            if angle % 90 == 0:
                num_angle = angle
                tx = 65 * math.cos(rad)
                ty = 65 * math.sin(rad)
                painter.drawText(int(tx) - 5, int(ty) - 5, 10, 10, Qt.AlignCenter, str(num_angle))

        rad = math.radians(self.current_angle)
        pointer = QPolygonF()
        pointer.append(QPointF(0, 0))
        pointer.append(QPointF(-8, -20))
        pointer.append(QPointF(0, -70))
        pointer.append(QPointF(8, -20))
        transform = painter.transform()
        painter.translate(0, 0)
        painter.rotate(self.current_angle)
        painter.setBrush(QBrush(QColor(200, 50, 50)))
        painter.setPen(QPen(Qt.black, 1))
        painter.drawPolygon(pointer)
        painter.setTransform(transform)

        painter.setBrush(QBrush(Qt.black))
        painter.drawEllipse(-5, -5, 10, 10)


# ----------------------------------------------------------------------
# IMU 3D Cube Widget using OpenGL with model loading support
# ----------------------------------------------------------------------
class IMU3DWidget(GLViewWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setBackgroundColor('k')
        self.setCameraPosition(distance=3)
        grid = GLGridItem()
        grid.scale(1, 1, 1)
        self.addItem(grid)

        # Current 3D model item
        self.current_mesh_item = None
        # Default cube
        self.set_default_cube()

    def set_default_cube(self):
        """Create and show the default colored cube."""
        if self.current_mesh_item:
            self.removeItem(self.current_mesh_item)

        vertices = np.array([
            [ 0.5,  0.5,  0.5], [ 0.5,  0.5, -0.5], [ 0.5, -0.5,  0.5], [ 0.5, -0.5, -0.5],
            [-0.5,  0.5,  0.5], [-0.5,  0.5, -0.5], [-0.5, -0.5,  0.5], [-0.5, -0.5, -0.5]
        ])
        faces = np.array([
            [0,1,3], [0,3,2], [4,6,7], [4,7,5],
            [0,4,5], [0,5,1], [2,3,7], [2,7,6],
            [0,2,6], [0,6,4], [1,5,7], [1,7,3]
        ])
        colors = np.array([
            [1,0,0,1], [1,0,0,1], [0,1,0,1], [0,1,0,1],
            [0,0,1,1], [0,0,1,1], [1,1,0,1], [1,1,0,1],
            [1,0,1,1], [1,0,1,1], [0,1,1,1], [0,1,1,1]
        ])
        meshdata = MeshData(vertexes=vertices, faces=faces, faceColors=colors)
        self.current_mesh_item = GLMeshItem(meshdata=meshdata, smooth=False, drawEdges=True, edgeColor=(1,1,1,1))
        self.addItem(self.current_mesh_item)

    def load_model_from_file(self, filepath):
        """Load a 3D model from file (STL, OBJ, PLY, STEP, etc.) using trimesh."""
        if not TRIMESH_AVAILABLE:
            QMessageBox.critical(None, "Error", "trimesh library not installed. Cannot load custom model.\n"
                                                "Please install: pip install trimesh")
            return False

        if not os.path.exists(filepath):
            QMessageBox.critical(None, "Error", f"File not found: {filepath}")
            return False

        try:
            mesh = trimesh.load(filepath, force='mesh')
            if mesh is None or len(mesh.vertices) == 0:
                raise ValueError("Empty mesh")

            # Ensure triangles
            if not isinstance(mesh.faces, np.ndarray) or len(mesh.faces) == 0:
                mesh = trimesh.Trimesh(vertices=mesh.vertices, faces=mesh.faces)

            # Convert to pyqtgraph MeshData
            vertices = mesh.vertices
            faces = mesh.faces
            # Compute face colors from vertex colors if available
            if hasattr(mesh.visual, 'vertex_colors') and mesh.visual.vertex_colors is not None:
                vc = mesh.visual.vertex_colors
                if vc.shape[1] == 3:
                    vc = np.hstack((vc, np.ones((vc.shape[0], 1))))  # add alpha
                # Average vertex colors per face
                face_colors = vc[faces].mean(axis=1)
            else:
                # Default gray color with random variation
                face_colors = np.ones((len(faces), 4)) * [0.7, 0.7, 0.7, 1.0]

            meshdata = MeshData(vertexes=vertices, faces=faces, faceColors=face_colors)

            # Remove old model and add new
            if self.current_mesh_item:
                self.removeItem(self.current_mesh_item)

            self.current_mesh_item = GLMeshItem(meshdata=meshdata, smooth=True, drawEdges=False)
            self.addItem(self.current_mesh_item)

            # --- 修复相机位置 ---
            bounds = mesh.bounds  # (min_point, max_point)
            center = (bounds[0] + bounds[1]) / 2.0
            # 转换为 pyqtgraph 可接受的对象
            center_vec = pg.Vector(center[0], center[1], center[2])
            # 模型尺寸
            size = bounds[1] - bounds[0]
            # 取最大轴的长度，乘系数作为相机距离
            distance = max(size) * 2.0
            self.setCameraPosition(distance=distance, pos=center_vec)
            return True

        except Exception as e:
            QMessageBox.critical(None, "Load Model Error", f"Failed to load {filepath}:\n{str(e)}")
            return False

    def set_orientation(self, roll_deg, pitch_deg, yaw_deg):
        """Apply rotation to the current model."""
        if self.current_mesh_item:
            self.current_mesh_item.resetTransform()
            self.current_mesh_item.rotate(yaw_deg, 0, 0, 1)
            self.current_mesh_item.rotate(pitch_deg, 0, 1, 0)
            self.current_mesh_item.rotate(roll_deg, 1, 0, 0)


# ----------------------------------------------------------------------
# Main GUI
# ----------------------------------------------------------------------
class MotorGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Motor Control GUI with IMU 3D Display")
        self.setGeometry(100, 100, 1400, 900)

        self.comm_backend = None
        self.motor_id = None
        self.detected_ids = []
        self.poll_timer = QTimer()
        self.poll_timer.timeout.connect(self.poll_data)
        self.poll_enabled = False
        self.poll_type = "none"
        self.mode_verify_retries = 0
        self.expected_mode = None
        self.is_calibration_mode = False

        self.gear_ratio_num = 1.0
        self.gear_ratio_den = 1.0

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
        self.last_position_deg = 0.0

        # IMU data
        self.imu_data = {
            'ax': 0.0, 'ay': 0.0, 'az': 0.0,
            'gx': 0.0, 'gy': 0.0, 'gz': 0.0,
            'temp': 0.0,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0
        }
        self.last_imu_time = time.time()
        self.complementary_alpha = 0.98

        self.serial_refresh_timer = QTimer()
        self.serial_refresh_timer.timeout.connect(self.refresh_serial_ports_auto)
        self.serial_refresh_timer.setInterval(500)
        self.serial_refresh_timer.start()

        self.manual_response_timer = QTimer()
        self.manual_response_timer.setInterval(100)
        self.manual_response_timer.timeout.connect(self.flush_manual_response)
        self.manual_response_buffer = []
        self.manual_response_max_lines = 200

        self.init_ui()

        self.auto_refresh_timer = QTimer()
        self.auto_refresh_timer.timeout.connect(self.refresh_all_except_mode)

        self.imu_poll_timer = QTimer()
        self.imu_poll_timer.timeout.connect(self.request_imu_data)
        self.imu_poll_enabled = False

    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)

        tabs = QTabWidget()
        main_layout.addWidget(tabs)

        conn_tab = QWidget()
        tabs.addTab(conn_tab, "Connection")
        self.init_connection_tab(conn_tab)

        ctrl_tab = QWidget()
        tabs.addTab(ctrl_tab, "Motor Control")
        self.init_control_tab(ctrl_tab)

        pid_tab = QWidget()
        tabs.addTab(pid_tab, "PID Tuning")
        self.init_pid_tab(pid_tab)

        data_tab = QWidget()
        tabs.addTab(data_tab, "Real-time Data")
        self.init_data_tab(data_tab)

        limits_tab = QWidget()
        tabs.addTab(limits_tab, "Limits")
        self.init_limits_tab(limits_tab)

        log_tab = QWidget()
        tabs.addTab(log_tab, "Communication Log")
        self.init_log_tab(log_tab)

        manual_tab = QWidget()
        tabs.addTab(manual_tab, "Manual Command")
        self.init_manual_tab(manual_tab)

        imu_tab = QWidget()
        tabs.addTab(imu_tab, "IMU 3D Display")
        self.init_imu_tab(imu_tab)

        self.status_bar = self.statusBar()
        self.status_label = QLabel("Not connected")
        self.status_bar.addWidget(self.status_label)

        self.manual_response_timer.start()

    def init_connection_tab(self, parent):
        layout = QFormLayout(parent)

        self.interface_combo = QComboBox()
        self.interface_combo.addItems(["Serial (UART/RS485)", "CAN"])
        self.interface_combo.currentTextChanged.connect(self.on_interface_changed)
        layout.addRow("Interface:", self.interface_combo)

        serial_port_layout = QHBoxLayout()
        self.serial_port_combo = QComboBox()
        self.refresh_ports_btn = QPushButton("Refresh")
        self.refresh_ports_btn.clicked.connect(self.refresh_serial_ports_manual)
        serial_port_layout.addWidget(self.serial_port_combo)
        serial_port_layout.addWidget(self.refresh_ports_btn)
        layout.addRow("Serial Port:", serial_port_layout)

        self.baudrate_combo = QComboBox()
        self.baudrate_combo.addItems(["9600", "19200", "38400", "57600", "115200", "2000000"])
        self.baudrate_combo.setCurrentText("115200")
        layout.addRow("Baudrate:", self.baudrate_combo)

        self.can_channel_edit = QLineEdit("PCAN_USBBUS1")
        self.can_bustype_combo = QComboBox()
        self.can_bustype_combo.addItems(["pcan", "socketcan", "kvaser", "ixxat", "vector"])
        layout.addRow("CAN Channel:", self.can_channel_edit)
        layout.addRow("CAN Bustype:", self.can_bustype_combo)
        self.can_bitrate_edit = QLineEdit("500000")
        layout.addRow("CAN Bitrate:", self.can_bitrate_edit)

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        layout.addRow(self.connect_btn)

        self.detect_btn = QPushButton("Detect Motor ID")
        self.detect_btn.clicked.connect(self.detect_motor_id)
        self.detect_btn.setEnabled(False)
        layout.addRow(self.detect_btn)

        self.motor_id_label = QLabel("None")
        layout.addRow("Detected Motor ID:", self.motor_id_label)

        self.update_interface_visibility()

    def init_control_tab(self, parent):
        layout = QVBoxLayout(parent)

        id_group = QGroupBox("Motor Selection")
        id_layout = QHBoxLayout()
        self.motor_id_combo = QComboBox()
        self.motor_id_combo.addItem("None")
        id_layout.addWidget(QLabel("Motor ID:"))
        id_layout.addWidget(self.motor_id_combo)
        id_layout.addStretch()
        id_group.setLayout(id_layout)
        layout.addWidget(id_group)

        self.motor_id_combo.currentTextChanged.connect(self.on_motor_id_changed)

        mode_group = QGroupBox("Operating Mode")
        mode_layout = QFormLayout()
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["Stop", "Self-test", "Calibration", "Open-loop",
                                  "Current loop", "Speed loop", "Position loop"])
        mode_layout.addRow("Mode:", self.mode_combo)
        self.set_mode_btn = QPushButton("Set Mode")
        self.get_mode_btn = QPushButton("Get Mode")
        mode_btns = QHBoxLayout()
        mode_btns.addWidget(self.set_mode_btn)
        mode_btns.addWidget(self.get_mode_btn)
        mode_layout.addRow(mode_btns)
        mode_group.setLayout(mode_layout)
        layout.addWidget(mode_group)

        param_group = QGroupBox("Motor Parameters")
        param_layout = QFormLayout()
        self.pole_pair_label = QLabel("---")
        self.offset_label = QLabel("---")
        self.encoder_dir_label = QLabel("---")
        param_layout.addRow("Pole Pairs:", self.pole_pair_label)
        param_layout.addRow("Zero Offset (°):", self.offset_label)
        param_layout.addRow("Encoder Direction:", self.encoder_dir_label)
        get_params_btn = QPushButton("Get Pole Pairs & Offset")
        get_params_btn.clicked.connect(self.get_motor_parameters)
        param_layout.addRow(get_params_btn)
        param_group.setLayout(param_layout)
        layout.addWidget(param_group)

        target_group = QGroupBox("Target Values")
        target_layout = QFormLayout()
        self.target_iq = QDoubleSpinBox()
        self.target_iq.setRange(-10, 10)
        self.target_iq.setDecimals(3)
        self.target_id = QDoubleSpinBox()
        self.target_id.setRange(-10, 10)
        self.target_id.setDecimals(3)
        self.target_speed = QDoubleSpinBox()
        self.target_speed.setRange(-5000, 5000)
        self.target_position = QDoubleSpinBox()
        self.target_position.setRange(-5000, 5000)
        self.target_uq = QDoubleSpinBox()
        self.target_uq.setRange(-6, 6)
        self.target_ud = QDoubleSpinBox()
        self.target_ud.setRange(-6, 6)

        target_layout.addRow("Iq target:", self.target_iq)
        target_layout.addRow("Id target:", self.target_id)
        target_layout.addRow("Speed target (rpm):", self.target_speed)
        target_layout.addRow("Position target (deg):", self.target_position)
        target_layout.addRow("Uq target (open-loop):", self.target_uq)
        target_layout.addRow("Ud target (open-loop):", self.target_ud)

        set_target_btn = QPushButton("Set Targets")
        get_target_btn = QPushButton("Get Targets")
        target_btns = QHBoxLayout()
        target_btns.addWidget(set_target_btn)
        target_btns.addWidget(get_target_btn)
        target_layout.addRow(target_btns)
        target_group.setLayout(target_layout)
        layout.addWidget(target_group)

        # 三相电流实时显示
        current_group = QGroupBox("Phase Currents (Ia, Ib, Ic)")
        current_layout = QFormLayout()
        self.label_Ia = QLabel("0.000 A")
        self.label_Ib = QLabel("0.000 A")
        self.label_Ic = QLabel("0.000 A")
        self.label_Ia.setStyleSheet("font-weight: bold; color: #FF0000;")
        self.label_Ib.setStyleSheet("font-weight: bold; color: #00AA00;")
        self.label_Ic.setStyleSheet("font-weight: bold; color: #0000FF;")
        current_layout.addRow("Ia:", self.label_Ia)
        current_layout.addRow("Ib:", self.label_Ib)
        current_layout.addRow("Ic:", self.label_Ic)
        current_group.setLayout(current_layout)
        layout.addWidget(current_group)

        auto_group = QGroupBox("Auto Refresh Parameters")
        auto_layout = QHBoxLayout()
        self.auto_refresh_checkbox = QCheckBox("Enable Auto Refresh")
        self.auto_refresh_checkbox.setChecked(True)
        self.auto_refresh_checkbox.toggled.connect(self.toggle_auto_refresh)
        auto_layout.addWidget(self.auto_refresh_checkbox)
        auto_layout.addWidget(QLabel("Interval (ms):"))
        self.refresh_interval_spin = QSpinBox()
        self.refresh_interval_spin.setRange(100, 5000)
        self.refresh_interval_spin.setValue(1000)
        self.refresh_interval_spin.setSuffix(" ms")
        self.refresh_interval_spin.valueChanged.connect(self.update_refresh_interval)
        auto_layout.addWidget(self.refresh_interval_spin)
        auto_layout.addStretch()
        auto_group.setLayout(auto_layout)
        layout.addWidget(auto_group)

        preview_group = QGroupBox("Motor Preview")
        preview_layout = QVBoxLayout()
        gear_layout = QHBoxLayout()
        gear_layout.addWidget(QLabel("Gear Ratio:"))
        self.gear_ratio_edit = QLineEdit("1 : 1")
        self.gear_ratio_edit.setMaximumWidth(120)
        self.gear_ratio_edit.textChanged.connect(self.update_gear_ratio)
        gear_layout.addWidget(self.gear_ratio_edit)
        gear_layout.addStretch()
        preview_layout.addLayout(gear_layout)

        dial_layout = QHBoxLayout()
        self.motor_preview = MotorPreviewWidget()
        dial_layout.addWidget(self.motor_preview, 1)

        angle_info_layout = QVBoxLayout()
        self.actual_angle_label = QLabel("Actual Angle: --- °")
        self.actual_angle_label.setFont(QFont("Arial", 12))
        angle_info_layout.addWidget(self.actual_angle_label)
        angle_info_layout.addStretch()
        dial_layout.addLayout(angle_info_layout, 0)

        preview_layout.addLayout(dial_layout)
        preview_group.setLayout(preview_layout)
        layout.addWidget(preview_group)

        self.set_mode_btn.clicked.connect(self.set_motor_mode)
        self.get_mode_btn.clicked.connect(self.get_motor_mode)
        set_target_btn.clicked.connect(self.set_targets)
        get_target_btn.clicked.connect(self.get_targets)

    def init_pid_tab(self, parent):
        layout = QVBoxLayout(parent)
        self.pid_widgets = {}
        for name in ['Iq', 'Id', 'Speed', 'Position']:
            group = QGroupBox(f"{name} PID")
            form = QFormLayout()
            p_spin = QDoubleSpinBox()
            p_spin.setRange(-1000, 1000)
            p_spin.setDecimals(6)
            i_spin = QDoubleSpinBox()
            i_spin.setRange(-1000, 1000)
            i_spin.setDecimals(6)
            d_spin = QDoubleSpinBox()
            d_spin.setRange(-1000, 1000)
            d_spin.setDecimals(6)
            form.addRow("P:", p_spin)
            form.addRow("I:", i_spin)
            form.addRow("D:", d_spin)
            set_btn = QPushButton("Set")
            get_btn = QPushButton("Get")
            btn_layout = QHBoxLayout()
            btn_layout.addWidget(set_btn)
            btn_layout.addWidget(get_btn)
            form.addRow(btn_layout)
            group.setLayout(form)
            layout.addWidget(group)
            self.pid_widgets[name] = (p_spin, i_spin, d_spin, set_btn, get_btn)
            set_btn.clicked.connect(lambda checked, n=name: self.set_pid(n))
            get_btn.clicked.connect(lambda checked, n=name: self.get_pid(n))

    def init_data_tab(self, parent):
        layout = QVBoxLayout(parent)
        plot_sel_layout = QHBoxLayout()
        self.plot_combo = QComboBox()
        self.plot_combo.addItems(["IaIbIc", "IqId", "Speed", "Position"])
        self.plot_combo.currentTextChanged.connect(self.change_plot_type)
        self.poll_checkbox = QCheckBox("Enable Polling")
        self.poll_checkbox.toggled.connect(self.toggle_polling)
        plot_sel_layout.addWidget(QLabel("Plot:"))
        plot_sel_layout.addWidget(self.plot_combo)
        plot_sel_layout.addWidget(self.poll_checkbox)
        layout.addLayout(plot_sel_layout)

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setLabel('left', 'Value')
        self.plot_widget.setLabel('bottom', 'Time (samples)')
        self.plot_widget.addLegend()
        self.plot_curves = {}
        layout.addWidget(self.plot_widget)

        save_btn = QPushButton("Save Data to CSV")
        save_btn.clicked.connect(self.save_data)
        layout.addWidget(save_btn)

    def init_limits_tab(self, parent):
        layout = QFormLayout(parent)
        self.limit_iq_max = QDoubleSpinBox()
        self.limit_iq_max.setRange(-100, 100)
        self.limit_iq_min = QDoubleSpinBox()
        self.limit_iq_min.setRange(-100, 100)
        self.limit_id_max = QDoubleSpinBox()
        self.limit_id_max.setRange(-100, 100)
        self.limit_id_min = QDoubleSpinBox()
        self.limit_id_min.setRange(-100, 100)
        self.limit_speed_max = QDoubleSpinBox()
        self.limit_speed_max.setRange(-10000, 10000)
        self.limit_speed_min = QDoubleSpinBox()
        self.limit_speed_min.setRange(-10000, 10000)
        self.limit_position_max = QDoubleSpinBox()
        self.limit_position_max.setRange(-10000, 10000)
        self.limit_position_min = QDoubleSpinBox()
        self.limit_position_min.setRange(-10000, 10000)

        layout.addRow("Iq max:", self.limit_iq_max)
        layout.addRow("Iq min:", self.limit_iq_min)
        layout.addRow("Id max:", self.limit_id_max)
        layout.addRow("Id min:", self.limit_id_min)
        layout.addRow("Speed max:", self.limit_speed_max)
        layout.addRow("Speed min:", self.limit_speed_min)
        layout.addRow("Position max:", self.limit_position_max)
        layout.addRow("Position min:", self.limit_position_min)

        set_limits_btn = QPushButton("Set Limits")
        get_limits_btn = QPushButton("Get Limits")
        layout.addRow(set_limits_btn, get_limits_btn)
        set_limits_btn.clicked.connect(self.set_limits)
        get_limits_btn.clicked.connect(self.get_limits)

    def init_log_tab(self, parent):
        layout = QVBoxLayout(parent)
        self.log_text = QPlainTextEdit()
        self.log_text.setReadOnly(True)
        font = QFont("Courier New")
        self.log_text.setFont(font)
        self.log_text.setMaximumBlockCount(500)
        layout.addWidget(self.log_text)
        clear_btn = QPushButton("Clear Log")
        clear_btn.clicked.connect(self.clear_log)
        layout.addWidget(clear_btn)

    def init_manual_tab(self, parent):
        layout = QVBoxLayout(parent)
        cmd_group = QGroupBox("Send Command (Hex)")
        cmd_layout = QVBoxLayout()
        self.manual_cmd_edit = QPlainTextEdit()
        self.manual_cmd_edit.setPlaceholderText("Enter hex bytes separated by space, e.g.: DE 1A 01 01 02 00 00 00 ... ED")
        self.manual_cmd_edit.setMaximumHeight(100)
        cmd_layout.addWidget(self.manual_cmd_edit)
        send_btn = QPushButton("Send Command")
        send_btn.clicked.connect(self.send_manual_command)
        cmd_layout.addWidget(send_btn)
        cmd_group.setLayout(cmd_layout)
        layout.addWidget(cmd_group)

        resp_group = QGroupBox("Response (Raw Hex)")
        resp_layout = QVBoxLayout()
        self.manual_response_text = QPlainTextEdit()
        self.manual_response_text.setReadOnly(True)
        font = QFont("Courier New")
        self.manual_response_text.setFont(font)
        self.manual_response_text.setMaximumBlockCount(self.manual_response_max_lines)
        resp_layout.addWidget(self.manual_response_text)
        clear_resp_btn = QPushButton("Clear Response")
        clear_resp_btn.clicked.connect(self.clear_manual_response)
        resp_layout.addWidget(clear_resp_btn)
        resp_group.setLayout(resp_layout)
        layout.addWidget(resp_group)

        info_label = QLabel("Note: For CAN, the command must be exactly 24 bytes (protocol packet). "
                            "It will be automatically split into 3 CAN frames.")
        info_label.setWordWrap(True)
        layout.addWidget(info_label)

    # ------------------------------------------------------------------
    # IMU 标签页，新增模型选择功能
    # ------------------------------------------------------------------
    def init_imu_tab(self, parent):
        layout = QVBoxLayout(parent)

        # 控制行：IMU 轮询控制
        control_layout = QHBoxLayout()
        self.imu_poll_checkbox = QCheckBox("Enable IMU Polling")
        self.imu_poll_checkbox.toggled.connect(self.toggle_imu_polling)
        control_layout.addWidget(self.imu_poll_checkbox)
        control_layout.addWidget(QLabel("Poll Interval (ms):"))
        self.imu_poll_interval = QSpinBox()
        self.imu_poll_interval.setRange(10, 500)
        self.imu_poll_interval.setValue(50)
        self.imu_poll_interval.setSuffix(" ms")
        self.imu_poll_interval.valueChanged.connect(self.update_imu_poll_interval)
        control_layout.addWidget(self.imu_poll_interval)
        control_layout.addStretch()
        layout.addLayout(control_layout)

        # 3D 视图
        self.imu_3d_view = IMU3DWidget()
        layout.addWidget(self.imu_3d_view, stretch=2)

        # 模型选择行
        model_layout = QHBoxLayout()
        model_layout.addWidget(QLabel("3D Model:"))
        self.model_combo = QComboBox()
        self.model_combo.addItem("Default Cube")
        self.refresh_model_btn = QPushButton("Refresh List")
        self.browse_model_btn = QPushButton("Browse File...")
        self.reset_cube_btn = QPushButton("Reset to Cube")
        model_layout.addWidget(self.model_combo)
        model_layout.addWidget(self.refresh_model_btn)
        model_layout.addWidget(self.browse_model_btn)
        model_layout.addWidget(self.reset_cube_btn)
        layout.addLayout(model_layout)

        # 扫描 /asset 下的模型
        self.scan_asset_models()
        self.model_combo.currentTextChanged.connect(self.on_model_selected)
        self.refresh_model_btn.clicked.connect(self.scan_asset_models)
        self.browse_model_btn.clicked.connect(self.browse_model_file)
        self.reset_cube_btn.clicked.connect(self.reset_to_cube)

        # IMU 数据显示
        data_group = QGroupBox("IMU Data & Angles")
        data_layout = QHBoxLayout()

        left_col = QVBoxLayout()
        left_col.addWidget(QLabel("Accelerometer (g):"))
        self.label_ax = QLabel("ax: ---")
        self.label_ay = QLabel("ay: ---")
        self.label_az = QLabel("az: ---")
        left_col.addWidget(self.label_ax)
        left_col.addWidget(self.label_ay)
        left_col.addWidget(self.label_az)
        data_layout.addLayout(left_col)

        mid_col = QVBoxLayout()
        mid_col.addWidget(QLabel("Gyroscope (dps):"))
        self.label_gx = QLabel("gx: ---")
        self.label_gy = QLabel("gy: ---")
        self.label_gz = QLabel("gz: ---")
        mid_col.addWidget(self.label_gx)
        mid_col.addWidget(self.label_gy)
        mid_col.addWidget(self.label_gz)
        data_layout.addLayout(mid_col)

        right_col = QVBoxLayout()
        right_col.addWidget(QLabel("Orientation (deg):"))
        self.label_roll = QLabel("Roll: ---")
        self.label_pitch = QLabel("Pitch: ---")
        self.label_yaw = QLabel("Yaw: ---")
        right_col.addWidget(self.label_roll)
        right_col.addWidget(self.label_pitch)
        right_col.addWidget(self.label_yaw)
        data_layout.addLayout(right_col)

        data_group.setLayout(data_layout)
        layout.addWidget(data_group)

        info = QLabel("姿态解算：互补滤波（Acc + Gyro）\nYaw 仅由陀螺仪积分，会随时间漂移。\n"
                      "3D模型支持 STL/OBJ/PLY/STEP 等格式，需要安装 trimesh 库。SLDASM 请先转换为 STEP/STL。")
        info.setWordWrap(True)
        layout.addWidget(info)

    def scan_asset_models(self):
        """扫描 ./asset 文件夹，将支持的模型文件添加到下拉列表"""
        self.model_combo.blockSignals(True)
        current = self.model_combo.currentText()
        self.model_combo.clear()
        self.model_combo.addItem("Default Cube")

        asset_dir = "./asset"
        if not os.path.exists(asset_dir):
            os.makedirs(asset_dir, exist_ok=True)

        supported_ext = ('.stl', '.obj', '.ply', '.step', '.stp')
        model_files = []
        for ext in supported_ext:
            model_files.extend(glob.glob(os.path.join(asset_dir, f"*{ext}")))
            model_files.extend(glob.glob(os.path.join(asset_dir, f"*{ext.upper()}")))

        for f in model_files:
            self.model_combo.addItem(f)

        # 尝试恢复之前选中的项
        idx = self.model_combo.findText(current)
        if idx >= 0:
            self.model_combo.setCurrentIndex(idx)
        self.model_combo.blockSignals(False)

    def on_model_selected(self, model_path):
        """当下拉列表选择模型时加载"""
        if model_path == "Default Cube":
            self.reset_to_cube()
        else:
            if os.path.exists(model_path):
                self.load_model_to_view(model_path)
            else:
                QMessageBox.warning(self, "Model Error", f"File not found: {model_path}")

    def browse_model_file(self):
        """打开文件对话框选择模型文件"""
        filter_str = "3D Models (*.stl *.obj *.ply *.step *.stp);;All Files (*.*)"
        filepath, _ = QFileDialog.getOpenFileName(self, "Select 3D Model", "./asset", filter_str)
        if filepath:
            # 若文件在 asset 目录下，刷新列表并选中
            asset_dir = os.path.abspath("./asset")
            if os.path.dirname(filepath) == asset_dir:
                self.scan_asset_models()
                idx = self.model_combo.findText(filepath)
                if idx >= 0:
                    self.model_combo.setCurrentIndex(idx)
                else:
                    self.model_combo.addItem(filepath)
                    self.model_combo.setCurrentText(filepath)
            else:
                # 直接加载外部文件
                self.load_model_to_view(filepath)
                # 可选：临时添加到下拉列表
                self.model_combo.addItem(filepath)
                self.model_combo.setCurrentText(filepath)

    def reset_to_cube(self):
        """重置为默认立方体"""
        self.imu_3d_view.set_default_cube()
        if self.model_combo.currentText() != "Default Cube":
            self.model_combo.setCurrentText("Default Cube")

    def load_model_to_view(self, filepath):
        """使用 IMU3DWidget 加载模型"""
        if not TRIMESH_AVAILABLE:
            QMessageBox.critical(self, "Missing Library",
                                 "trimesh is not installed.\n"
                                 "Please run: pip install trimesh")
            return
        success = self.imu_3d_view.load_model_from_file(filepath)
        if not success:
            QMessageBox.warning(self, "Load Failed", f"Could not load model:\n{filepath}\n"
                                                     "Falling back to default cube.")
            self.imu_3d_view.set_default_cube()
            # 将下拉列表重置为 Default Cube
            if self.model_combo.currentText() != "Default Cube":
                self.model_combo.setCurrentText("Default Cube")

    # ------------------------------------------------------------------
    # IMU 数据处理与姿态解算 (原有)
    # ------------------------------------------------------------------
    def request_imu_data(self):
        if not self.comm_backend or not self.imu_poll_enabled:
            return
        mid = self.get_current_motor_id()
        if mid == 0:
            if self.detected_ids:
                mid = self.detected_ids[0]
            else:
                return
        self.send_command(0x3D, 0x00, motor_id=mid)

    def decode_imu_packet(self, packet):
        data1 = packet.data1.as_uint32()
        data2 = packet.data2.as_uint32()
        data3 = packet.data3.as_uint32()
        data4 = packet.data4.as_uint32()

        def to_int16(value):
            value = value & 0xFFFF
            return value - 0x10000 if value & 0x8000 else value

        ax_raw = to_int16(data1 >> 16)
        ay_raw = to_int16(data1 & 0xFFFF)
        az_raw = to_int16(data2 >> 16)
        gx_raw = to_int16(data2 & 0xFFFF)
        gy_raw = to_int16(data3 >> 16)
        gz_raw = to_int16(data3 & 0xFFFF)
        temp_raw = to_int16(data4 >> 16)

        ACCEL_SCALE = 0.000122
        GYRO_SCALE = 0.035
        TEMP_SCALE = 1/256.0

        self.imu_data['ax'] = ax_raw * ACCEL_SCALE
        self.imu_data['ay'] = ay_raw * ACCEL_SCALE
        self.imu_data['az'] = az_raw * ACCEL_SCALE
        self.imu_data['gx'] = gx_raw * GYRO_SCALE
        self.imu_data['gy'] = gy_raw * GYRO_SCALE
        self.imu_data['gz'] = gz_raw * GYRO_SCALE
        self.imu_data['temp'] = temp_raw * TEMP_SCALE + 25.0

        self.label_ax.setText(f"ax: {self.imu_data['ax']:.3f} g")
        self.label_ay.setText(f"ay: {self.imu_data['ay']:.3f} g")
        self.label_az.setText(f"az: {self.imu_data['az']:.3f} g")
        self.label_gx.setText(f"gx: {self.imu_data['gx']:.1f} dps")
        self.label_gy.setText(f"gy: {self.imu_data['gy']:.1f} dps")
        self.label_gz.setText(f"gz: {self.imu_data['gz']:.1f} dps")

        self.update_orientation()

    def update_orientation(self):
        dt = time.time() - self.last_imu_time
        if dt <= 0 or dt > 0.1:
            dt = 0.02
        self.last_imu_time = time.time()

        ax, ay, az = self.imu_data['ax'], self.imu_data['ay'], self.imu_data['az']
        gx, gy, gz = self.imu_data['gx'], self.imu_data['gy'], self.imu_data['gz']

        alpha_lpf = 0.2
        if not hasattr(self, 'ax_filt'):
            self.ax_filt, self.ay_filt, self.az_filt = ax, ay, az
        self.ax_filt = alpha_lpf * ax + (1 - alpha_lpf) * self.ax_filt
        self.ay_filt = alpha_lpf * ay + (1 - alpha_lpf) * self.ay_filt
        self.az_filt = alpha_lpf * az + (1 - alpha_lpf) * self.az_filt

        norm = math.sqrt(self.ax_filt**2 + self.ay_filt**2 + self.az_filt**2)
        if norm > 1e-6:
            ax_n = self.ax_filt / norm
            ay_n = self.ay_filt / norm
            az_n = self.az_filt / norm
        else:
            ax_n, ay_n, az_n = 0, 0, 1

        q_acc = [1.0, 0.0, 0.0, 0.0]
        v_ref = [0, 0, 1]
        v_meas = [ax_n, ay_n, az_n]
        dot = v_ref[0]*v_meas[0] + v_ref[1]*v_meas[1] + v_ref[2]*v_meas[2]
        if dot > 0.99999:
            q_acc = [1.0, 0.0, 0.0, 0.0]
        elif dot < -0.99999:
            q_acc = [0.0, 1.0, 0.0, 0.0]
        else:
            cross = [ v_ref[1]*v_meas[2] - v_ref[2]*v_meas[1],
                      v_ref[2]*v_meas[0] - v_ref[0]*v_meas[2],
                      v_ref[0]*v_meas[1] - v_ref[1]*v_meas[0] ]
            w = math.sqrt((1.0 + dot) * 2.0)
            inv_w = 1.0 / w
            q_acc = [w * 0.5, cross[0] * inv_w, cross[1] * inv_w, cross[2] * inv_w]

        if not hasattr(self, 'q'):
            self.q = [1.0, 0.0, 0.0, 0.0]
        gx_rad = math.radians(gx)
        gy_rad = math.radians(gy)
        gz_rad = math.radians(gz)
        omega_norm = math.sqrt(gx_rad**2 + gy_rad**2 + gz_rad**2)
        if omega_norm > 1e-6:
            theta = omega_norm * dt
            half_theta = theta * 0.5
            sin_half = math.sin(half_theta)
            cos_half = math.cos(half_theta)
            ux = gx_rad / omega_norm
            uy = gy_rad / omega_norm
            uz = gz_rad / omega_norm
            q_gyro = [cos_half, ux * sin_half, uy * sin_half, uz * sin_half]
            q_new = [
                self.q[0]*q_gyro[0] - self.q[1]*q_gyro[1] - self.q[2]*q_gyro[2] - self.q[3]*q_gyro[3],
                self.q[0]*q_gyro[1] + self.q[1]*q_gyro[0] + self.q[2]*q_gyro[3] - self.q[3]*q_gyro[2],
                self.q[0]*q_gyro[2] - self.q[1]*q_gyro[3] + self.q[2]*q_gyro[0] + self.q[3]*q_gyro[1],
                self.q[0]*q_gyro[3] + self.q[1]*q_gyro[2] - self.q[2]*q_gyro[1] + self.q[3]*q_gyro[0]
            ]
        else:
            q_new = self.q[:]

        alpha = 0.96
        q_fused = [
            alpha * q_new[0] + (1-alpha) * q_acc[0],
            alpha * q_new[1] + (1-alpha) * q_acc[1],
            alpha * q_new[2] + (1-alpha) * q_acc[2],
            alpha * q_new[3] + (1-alpha) * q_acc[3]
        ]
        norm_q = math.sqrt(q_fused[0]**2 + q_fused[1]**2 + q_fused[2]**2 + q_fused[3]**2)
        if norm_q > 1e-6:
            self.q = [x / norm_q for x in q_fused]
        else:
            self.q = [1.0, 0.0, 0.0, 0.0]

        q0, q1, q2, q3 = self.q
        roll  = math.atan2(2.0*(q0*q1 + q2*q3), 1.0 - 2.0*(q1*q1 + q2*q2)) * 180.0 / math.pi
        pitch = math.asin(2.0*(q0*q2 - q3*q1)) * 180.0 / math.pi
        yaw   = math.atan2(2.0*(q0*q3 + q1*q2), 1.0 - 2.0*(q2*q2 + q3*q3)) * 180.0 / math.pi

        self.imu_data['roll'] = roll
        self.imu_data['pitch'] = pitch
        self.imu_data['yaw'] = yaw

        self.label_roll.setText(f"Roll: {roll:.1f}°")
        self.label_pitch.setText(f"Pitch: {pitch:.1f}°")
        self.label_yaw.setText(f"Yaw: {yaw:.1f}°")

        self.imu_3d_view.set_orientation(roll, pitch, yaw)

    def toggle_imu_polling(self, enabled):
        self.imu_poll_enabled = enabled
        if enabled:
            self.imu_poll_timer.start(self.imu_poll_interval.value())
            self.request_imu_data()
        else:
            self.imu_poll_timer.stop()

    def update_imu_poll_interval(self):
        if self.imu_poll_enabled:
            self.imu_poll_timer.start(self.imu_poll_interval.value())

    # ------------------------------------------------------------------
    # Communication helpers (原有，未改动)
    # ------------------------------------------------------------------
    def on_interface_changed(self, text):
        self.update_interface_visibility()

    def update_interface_visibility(self):
        is_serial = self.interface_combo.currentText() == "Serial (UART/RS485)"
        self.serial_port_combo.setEnabled(is_serial)
        self.refresh_ports_btn.setEnabled(is_serial)
        self.baudrate_combo.setEnabled(is_serial)
        self.can_channel_edit.setEnabled(not is_serial)
        self.can_bustype_combo.setEnabled(not is_serial)
        self.can_bitrate_edit.setEnabled(not is_serial)

        if is_serial and self.comm_backend is None:
            if not self.serial_refresh_timer.isActive():
                self.serial_refresh_timer.start()
        else:
            if self.serial_refresh_timer.isActive():
                self.serial_refresh_timer.stop()

    def refresh_serial_ports_manual(self):
        self.refresh_serial_ports_auto()

    def refresh_serial_ports_auto(self):
        if self.comm_backend is not None:
            return
        if self.interface_combo.currentText() != "Serial (UART/RS485)":
            return
        current_port = self.serial_port_combo.currentText()
        self.serial_port_combo.clear()
        ports = serial.tools.list_ports.comports()
        port_list = [port.device for port in ports]
        self.serial_port_combo.addItems(port_list)
        if current_port in port_list:
            self.serial_port_combo.setCurrentText(current_port)
        elif port_list:
            self.serial_port_combo.setCurrentIndex(0)

    def toggle_connection(self):
        if self.comm_backend is not None:
            self.disconnect()
        else:
            self.connect_device()

    def connect_device(self):
        if self.interface_combo.currentText() == "Serial (UART/RS485)":
            port = self.serial_port_combo.currentText()
            if not port:
                QMessageBox.warning(self, "Error", "No serial port selected")
                return
            baud = int(self.baudrate_combo.currentText())
            backend = SerialBackend(port, baud)
        else:
            if not CAN_AVAILABLE:
                QMessageBox.critical(self, "Error", "python-can not installed")
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
        self.status_label.setText(f"Connected to {self.interface_combo.currentText()}")
        if self.serial_refresh_timer.isActive():
            self.serial_refresh_timer.stop()

    def disconnect(self):
        if self.comm_backend:
            self.comm_backend.stop()
            self.comm_backend = None
        self.connect_btn.setText("Connect")
        self.detect_btn.setEnabled(False)
        self.status_label.setText("Not connected")
        self.motor_id = None
        self.motor_id_label.setText("None")
        self.motor_id_combo.clear()
        self.motor_id_combo.addItem("None")
        self.imu_poll_checkbox.setChecked(False)
        if self.interface_combo.currentText() == "Serial (UART/RS485)":
            if not self.serial_refresh_timer.isActive():
                self.serial_refresh_timer.start()

    def log_message(self, direction, func2, func3, motor_id, data1=None, data2=None, data3=None, data4=None):
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        msg = f"[{timestamp}] [{direction}] func2=0x{func2:02X} func3=0x{func3:02X} motor_id={motor_id}"
        if data1 is not None:
            msg += f" data1={data1}"
        if data2 is not None:
            msg += f" data2={data2}"
        if data3 is not None:
            msg += f" data3={data3}"
        if data4 is not None:
            msg += f" data4={data4}"
        self.log_text.appendPlainText(msg)

    def clear_log(self):
        self.log_text.clear()

    def send_command(self, func2, func3, data1=0, data2=0, data3=0, data4=0, motor_id=0):
        if not self.comm_backend:
            QMessageBox.warning(self, "Error", "Not connected")
            return
        pkt = CommandPacket(func1=0x1A, func2=func2, func3=func3,
                            data1=data1, data2=data2, data3=data3, data4=data4,
                            motor_id=motor_id, sender=0)
        self.log_message("TX", func2, func3, motor_id, data1, data2, data3, data4)
        self.comm_backend.send_packet(pkt)

    def send_manual_command(self):
        if not self.comm_backend:
            QMessageBox.warning(self, "Error", "Not connected")
            return
        hex_str = self.manual_cmd_edit.toPlainText().strip()
        if not hex_str:
            QMessageBox.warning(self, "Warning", "Please enter a hex command")
            return
        hex_str = hex_str.replace(' ', '').replace('\n', '').replace('\r', '')
        try:
            cmd_bytes = bytes.fromhex(hex_str)
        except ValueError as e:
            QMessageBox.warning(self, "Error", f"Invalid hex string: {e}")
            return
        if len(cmd_bytes) == 0:
            QMessageBox.warning(self, "Error", "Empty command")
            return
        if len(cmd_bytes) > 256:
            reply = QMessageBox.question(self, "Confirm", f"Command length {len(cmd_bytes)} bytes. Send anyway?",
                                         QMessageBox.Yes | QMessageBox.No)
            if reply != QMessageBox.Yes:
                return
        self.comm_backend.send_raw(cmd_bytes)
        self.manual_response_buffer.append(f"\n[TX] {cmd_bytes.hex().upper()}")

    def on_raw_data_received(self, data: bytes):
        hex_str = data.hex().upper()
        spaced = ' '.join(hex_str[i:i+2] for i in range(0, len(hex_str), 2))
        self.manual_response_buffer.append(f"[RX] {spaced}")

    def flush_manual_response(self):
        if not self.manual_response_buffer:
            return
        text_to_add = '\n'.join(self.manual_response_buffer)
        self.manual_response_text.appendPlainText(text_to_add)
        self.manual_response_buffer.clear()
        scrollbar = self.manual_response_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def clear_manual_response(self):
        self.manual_response_text.clear()
        self.manual_response_buffer.clear()

    def on_packet_received(self, packet):
        if packet.func1 != 0x1A:
            return
        self.log_message("RX", packet.func2, packet.func3, packet.motor_id,
                         packet.data1.as_float(), packet.data2.as_float(),
                         packet.data3.as_float(), packet.data4.as_float())

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

        if hasattr(self, 'mode_verify_active') and self.mode_verify_active and not self.is_calibration_mode:
            if packet.func2 == 0x01 and packet.func3 == 0x00:
                current_mode = packet.data1.as_uint32()
                if current_mode == self.expected_mode:
                    self.status_label.setText(f"Mode successfully set to {self.mode_combo.currentText()}")
                    self.mode_verify_active = False
                    if hasattr(self, 'mode_verify_timer'):
                        self.mode_verify_timer.stop()
                else:
                    self.mode_verify_retries -= 1
                    if self.mode_verify_retries <= 0:
                        self.status_label.setText(f"Warning: Mode switch failed. Current mode is {self.get_mode_name(current_mode)}")
                        QMessageBox.warning(self, "Mode Switch",
                            f"Failed to switch to {self.get_mode_name(self.expected_mode)}.\n"
                            f"Current mode: {self.get_mode_name(current_mode)}\n"
                            "Please check motor status or firmware.")
                        self.mode_verify_active = False
                        if hasattr(self, 'mode_verify_timer'):
                            self.mode_verify_timer.stop()

    def get_mode_name(self, mode_val):
        names = ["Stop", "Self-test", "Calibration", "Open-loop",
                 "Current loop", "Speed loop", "Position loop"]
        if 0 <= mode_val < len(names):
            return names[mode_val]
        return "Unknown"

    # 电机命令实现
    def detect_motor_id(self):
        self.send_command(0x00, 0x00, motor_id=0xFFFF)

    def handle_detect_response(self, packet):
        ids = []
        for i in range(1, 5):
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
            QMessageBox.information(self, "Motor Detection", f"Detected motor IDs: {ids}")
        else:
            QMessageBox.warning(self, "Motor Detection", "No motor found")

    def set_motor_mode(self):
        mode_map = {"Stop":0, "Self-test":1, "Calibration":2, "Open-loop":3,
                    "Current loop":4, "Speed loop":5, "Position loop":6}
        expected_mode = mode_map[self.mode_combo.currentText()]
        mid = self.get_current_motor_id()
        if mid == 0:
            QMessageBox.warning(self, "Warning", "Please detect and select a motor ID first.")
            return
        self.send_command(0x01, 0x01, data1=expected_mode, motor_id=mid)
        self.status_label.setText(f"Setting mode to {self.mode_combo.currentText()}...")
        self.expected_mode = expected_mode
        if self.mode_combo.currentText() == "Calibration":
            self.is_calibration_mode = True
            self.mode_verify_active = False
            if hasattr(self, 'mode_verify_timer'):
                self.mode_verify_timer.stop()
            self.status_label.setText("Calibration mode set. You can now read parameters.")
            return
        else:
            self.is_calibration_mode = False
            self.mode_verify_retries = 3
            self.mode_verify_active = True
            if not hasattr(self, 'mode_verify_timer'):
                self.mode_verify_timer = QTimer()
                self.mode_verify_timer.timeout.connect(self.verify_mode_step)
            self.mode_verify_timer.start(500)

    def verify_mode_step(self):
        if not self.mode_verify_active:
            if hasattr(self, 'mode_verify_timer'):
                self.mode_verify_timer.stop()
            return
        self.get_motor_mode()

    def get_motor_mode(self):
        mid = self.get_current_motor_id()
        if mid == 0:
            QMessageBox.warning(self, "Warning", "Please detect and select a motor ID first.")
            return
        self.send_command(0x01, 0x00, motor_id=mid)

    def handle_mode_response(self, packet):
        mode = packet.data1.as_uint32()
        mode_names = ["Stop","Self-test","Calibration","Open-loop",
                      "Current loop","Speed loop","Position loop"]
        if 0 <= mode < len(mode_names):
            idx = self.mode_combo.findText(mode_names[mode])
            if idx >= 0:
                self.mode_combo.setCurrentIndex(idx)
        else:
            print(f"Invalid mode value: {mode}")

    def get_motor_parameters(self):
        mid = self.get_current_motor_id()
        if mid == 0:
            QMessageBox.warning(self, "Warning", "Please detect and select a motor ID first.")
            return
        self.send_command(0x02, 0x00, motor_id=mid)

    def handle_pole_pair_response(self, packet):
        if packet.func3 == 0x00:
            pole_pairs = packet.data1.as_uint32()
            offset_angle = packet.data2.as_float()
            encoder_dir = packet.data3.as_uint32()
            self.pole_pair_label.setText(str(pole_pairs))
            self.offset_label.setText(f"{offset_angle:.3f}°")
            self.encoder_dir_label.setText(str(encoder_dir))
            self.status_label.setText(f"Motor parameters: {pole_pairs} poles, offset {offset_angle:.2f}°, direction {encoder_dir}")

    def set_targets(self):
        mid = self.get_current_motor_id()
        if mid == 0:
            QMessageBox.warning(self, "Warning", "Please detect and select a motor ID first.")
            return
        self.send_command(0x20,0x01, data1=self.target_iq.value(), motor_id=mid)
        self.send_command(0x21,0x01, data1=self.target_id.value(), motor_id=mid)
        self.send_command(0x22,0x01, data1=self.target_speed.value(), motor_id=mid)
        self.send_command(0x23,0x01, data1=self.target_position.value(), motor_id=mid)
        self.send_command(0x24,0x01, data1=self.target_uq.value(), data2=self.target_ud.value(), motor_id=mid)

    def get_targets(self):
        mid = self.get_current_motor_id()
        if mid == 0:
            QMessageBox.warning(self, "Warning", "Please detect and select a motor ID first.")
            return
        self.send_command(0x20,0x00, motor_id=mid)
        self.send_command(0x21,0x00, motor_id=mid)
        self.send_command(0x22,0x00, motor_id=mid)
        self.send_command(0x23,0x00, motor_id=mid)
        self.send_command(0x24,0x00, motor_id=mid)

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
            QMessageBox.warning(self, "Warning", "Please detect and select a motor ID first.")
            return
        func2_map = {"Iq":0x10, "Id":0x11, "Speed":0x12, "Position":0x13}
        p,i,d,_,_ = self.pid_widgets[name]
        self.send_command(func2_map[name],0x01, data1=p.value(), data2=i.value(), data3=d.value(), motor_id=mid)

    def get_pid(self, name):
        mid = self.get_current_motor_id()
        if mid == 0:
            QMessageBox.warning(self, "Warning", "Please detect and select a motor ID first.")
            return
        func2_map = {"Iq":0x10, "Id":0x11, "Speed":0x12, "Position":0x13}
        self.send_command(func2_map[name],0x00, motor_id=mid)

    def handle_pid_response(self, name, packet):
        p,i,d,_,_ = self.pid_widgets[name]
        p.setValue(packet.data1.as_float())
        i.setValue(packet.data2.as_float())
        d.setValue(packet.data3.as_float())

    def set_limits(self):
        mid = self.get_current_motor_id()
        if mid == 0:
            QMessageBox.warning(self, "Warning", "Please detect and select a motor ID first.")
            return
        self.send_command(0x14,0x01, data1=self.limit_iq_max.value(), data2=self.limit_iq_min.value(),
                          data3=self.limit_id_max.value(), data4=self.limit_id_min.value(), motor_id=mid)
        self.send_command(0x15,0x01, data1=self.limit_speed_max.value(), data2=self.limit_speed_min.value(), motor_id=mid)
        self.send_command(0x16,0x01, data1=self.limit_position_max.value(), data2=self.limit_position_min.value(), motor_id=mid)

    def get_limits(self):
        mid = self.get_current_motor_id()
        if mid == 0:
            QMessageBox.warning(self, "Warning", "Please detect and select a motor ID first.")
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

    # 实时数据轮询
    def update_gear_ratio(self):
        text = self.gear_ratio_edit.text().strip()
        if ':' in text:
            parts = text.split(':')
            if len(parts) == 2:
                try:
                    num = float(parts[0].strip())
                    den = float(parts[1].strip())
                    if den != 0:
                        self.gear_ratio_num = num
                        self.gear_ratio_den = den
                        self.update_preview_angle(self.last_position_deg)
                except ValueError:
                    pass

    def update_preview_angle(self, motor_position_deg):
        if self.gear_ratio_den != 0:
            actual_angle = motor_position_deg * (self.gear_ratio_num / self.gear_ratio_den)
        else:
            actual_angle = motor_position_deg
        self.last_position_deg = motor_position_deg
        self.motor_preview.set_angle(actual_angle)
        self.actual_angle_label.setText(f"Actual Angle: {actual_angle:.1f}°")

    def toggle_polling(self, enabled):
        if enabled:
            if self.motor_id is None:
                QMessageBox.warning(self, "Polling", "Detect motor ID first")
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

    # 处理 0x30 响应，更新电流数值显示和绘图
    def handle_iaibic(self, packet):
        ia = packet.data1.as_float()
        ib = packet.data2.as_float()
        ic = packet.data3.as_float()
        # 更新 Control 标签页中的实时电流显示
        self.label_Ia.setText(f"{ia:.3f} A")
        self.label_Ib.setText(f"{ib:.3f} A")
        self.label_Ic.setText(f"{ic:.3f} A")
        # 更新数据历史
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
        self.data_history['time'].append(self.plot_index)
        self.data_history['speed'].append(speed)
        if self.poll_type == "speed":
            self.update_plot()

    def handle_position(self, packet):
        pos = packet.data1.as_float()
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
        QMessageBox.critical(self, "Communication Error", msg)
        self.disconnect()

    def get_current_motor_id(self):
        text = self.motor_id_combo.currentText()
        if text == "None" or not text:
            return 0
        try:
            return int(text)
        except ValueError:
            return 0

    def toggle_auto_refresh(self, enabled):
        if enabled:
            self.auto_refresh_timer.start(self.refresh_interval_spin.value())
            self.refresh_all_except_mode()
        else:
            self.auto_refresh_timer.stop()

    def update_refresh_interval(self):
        if self.auto_refresh_checkbox.isChecked():
            self.auto_refresh_timer.start(self.refresh_interval_spin.value())

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

    def on_motor_id_changed(self):
        if self.auto_refresh_checkbox.isChecked():
            self.refresh_all_except_mode()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MotorGUI()
    window.show()
    sys.exit(app.exec_())