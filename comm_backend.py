# -*- coding: utf-8 -*-

import queue
import time
import serial
import serial.tools.list_ports
from PyQt5.QtCore import QThread, pyqtSignal
from protocol import PACKAGE_SIZE, HEAD, TAIL, CommandPacket

try:
    import can
    CAN_AVAILABLE = True
except ImportError:
    CAN_AVAILABLE = False


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
            # 发送队列 (CommandPacket)
            try:
                while True:
                    pkt = self.tx_queue.get_nowait()
                    self.serial.write(pkt.build())
            except queue.Empty:
                pass

            # 发送原始字节
            try:
                while True:
                    raw_data = self.raw_tx_queue.get_nowait()
                    self.serial.write(raw_data)
                    self.raw_data_received.emit(raw_data)  # 可选：回显
            except queue.Empty:
                pass

            # 接收数据
            try:
                read_data = self.serial.read(1)
                if read_data:
                    buffer.extend(read_data)
                    while len(buffer) >= PACKAGE_SIZE:
                        head_idx = -1
                        for i in range(len(buffer) - PACKAGE_SIZE + 1):
                            if buffer[i] == HEAD:
                                head_idx = i
                                break
                        if head_idx == -1:
                            buffer.clear()
                            break
                        if head_idx > 0:
                            buffer = buffer[head_idx:]
                        if len(buffer) < PACKAGE_SIZE:
                            break
                        if buffer[PACKAGE_SIZE - 1] == TAIL:
                            packet_data = buffer[:PACKAGE_SIZE]
                            packet = CommandPacket.parse(packet_data)
                            if packet:
                                self.packet_received.emit(packet)
                            buffer = buffer[PACKAGE_SIZE:]
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
            # 发送 CommandPacket
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

            # 发送原始字节 (同样拆分为3帧)
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

            # 接收 CAN 消息
            msg = self.bus.recv(0.01)
            if msg and msg.arbitration_id == 0x123 and len(msg.data) == 8:
                seq = msg.data[0]
                if seq == 0:
                    buffer = bytearray()
                    expected_seq = 0
                if seq == expected_seq and expected_seq < 3:
                    buffer.extend(msg.data[1:])
                    expected_seq += 1
                    if expected_seq == 3 and len(buffer) >= PACKAGE_SIZE:
                        # 尝试解析为协议包
                        packet = CommandPacket.parse(buffer[:PACKAGE_SIZE])
                        if packet:
                            self.packet_received.emit(packet)
                        else:
                            # 不是有效协议包，发射原始数据
                            self.raw_data_received.emit(buffer[:PACKAGE_SIZE])
                        buffer = bytearray()
                        expected_seq = 0

        if self.bus:
            self.bus.shutdown()