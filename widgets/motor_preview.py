# -*- coding: utf-8 -*-

import math
from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import Qt, QPointF
from PyQt5.QtGui import QPainter, QPen, QBrush, QColor, QFont, QPolygonF


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
        painter.setBrush(QBrush(QColor(200, 50, 50)))
        painter.setPen(QPen(Qt.black, 1))
        painter.drawPolygon(pointer)
        painter.rotate(-self.current_angle)

        painter.setBrush(QBrush(Qt.black))
        painter.drawEllipse(-5, -5, 10, 10)