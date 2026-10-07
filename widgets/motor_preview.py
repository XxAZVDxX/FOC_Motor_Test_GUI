# -*- coding: utf-8 -*-

import math
from PyQt5.QtWidgets import QWidget, QSizePolicy
from PyQt5.QtCore import Qt, QPointF, QSize
from PyQt5.QtGui import QPainter, QPen, QBrush, QColor, QFont, QPolygonF


class MotorPreviewWidget(QWidget):
    # 主题配色缺省值，保证未调用 set_colors() 时也能正常绘制
    DEFAULT_COLORS = {
        "preview_edge": "#505050",
        "preview_face": "#f0f0f0",
        "preview_pointer": "#c83232",
        "preview_hub": "#000000",
        "preview_text": "#404040",
    }

    def __init__(self, parent=None):
        super().__init__(parent)
        self.current_angle = 0.0
        self.colors = dict(self.DEFAULT_COLORS)
        self.setMinimumSize(200, 200)
        # 原来 300px 的硬上限让表盘偏小；放宽到 340px 让它随标签页一起变大。
        # paintEvent 会把绘制区居中成正方形，所以容器不是正方形也不会变形。
        self.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        self.setMaximumSize(340, 340)

    def sizeHint(self):
        return QSize(320, 320)

    def set_colors(self, colors):
        """应用主题配色（来自 theme.palette()）。"""
        for key, value in self.DEFAULT_COLORS.items():
            self.colors[key] = colors.get(key, value)
        self.update()

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

        edge = QColor(self.colors["preview_edge"])
        face = QColor(self.colors["preview_face"])
        pointer_color = QColor(self.colors["preview_pointer"])
        hub = QColor(self.colors["preview_hub"])

        painter.setPen(QPen(edge, 2))
        painter.setBrush(QBrush(face))
        painter.drawEllipse(-90, -90, 180, 180)

        font = QFont("Arial", 8)
        painter.setFont(font)
        painter.setPen(QPen(QColor(self.colors["preview_text"]), 1))
        for angle in range(0, 360, 30):
            rad = math.radians(angle)
            x1 = 85 * math.cos(rad)
            y1 = 85 * math.sin(rad)
            x2 = 75 * math.cos(rad)
            y2 = 75 * math.sin(rad)
            painter.drawLine(int(x1), int(y1), int(x2), int(y2))
            if angle % 90 == 0:
                tx = 65 * math.cos(rad)
                ty = 65 * math.sin(rad)
                painter.drawText(int(tx) - 5, int(ty) - 5, 10, 10, Qt.AlignCenter, str(angle))

        rad = math.radians(self.current_angle)
        pointer = QPolygonF()
        pointer.append(QPointF(0, 0))
        pointer.append(QPointF(-8, -20))
        pointer.append(QPointF(0, -70))
        pointer.append(QPointF(8, -20))
        painter.translate(0, 0)
        painter.rotate(self.current_angle)
        painter.setBrush(QBrush(pointer_color))
        painter.setPen(QPen(edge, 1))
        painter.drawPolygon(pointer)
        painter.rotate(-self.current_angle)

        painter.setBrush(QBrush(hub))
        painter.setPen(QPen(edge, 1))
        painter.drawEllipse(-5, -5, 10, 10)