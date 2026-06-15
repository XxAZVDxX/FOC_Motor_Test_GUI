# -*- coding: utf-8 -*-

import struct

PACKAGE_SIZE = 24
HEAD = 0xDE
TAIL = 0xED


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