"""Минималистичная визуализация облака точек 2D-лидара в координатах робота."""
from __future__ import annotations

import math
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QColor, QPainter, QPen, QFont
from PyQt6.QtWidgets import QWidget

from lidar import LidarFrame


class LidarView(QWidget):
    """Вид сверху: направление вперёд — вверх, индекс 0 по умолчанию смотрит вперёд."""

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setMinimumSize(360, 360)
        self.setAutoFillBackground(True)
        self._ranges: tuple[int, ...] = ()
        self._trajectory: tuple[tuple[float, float], ...] = ()
        self._target: tuple[float, float] | None = None
        self._timestamp: str | None = None
        # Максимальная дальность — радиус от робота, а не диаметр области.
        # 16 м — масштаб ближе к размеру полигона 16 x 10 м; дальность задаётся из GUI.
        self._max_range_mm = 4000
        # Шасси ID=1/ID=2 по QuickStart: 60 см ширина, 90 см длина.
        self._robot_width_mm = 600
        self._robot_length_mm = 900
        self._angle_offset_deg = 180.0  # пакет лидара ориентирован задом наперёд
        self._frame_count = 0
        self._last_source = ""
        self._refresh_timer = QTimer(self)
        self._refresh_timer.setInterval(200)  # визуализация около 5 кадров/с
        self._refresh_timer.timeout.connect(self.update)
        self._refresh_timer.start()

    def set_frame(self, frame: LidarFrame) -> None:
        self._ranges = frame.ranges_mm
        self._timestamp = frame.timestamp
        self._frame_count += 1
        self._last_source = f"{frame.source_host}:{frame.source_port}" if frame.source_host else ""

    @property
    def ranges_mm(self) -> tuple[int, ...]:
        return self._ranges

    def set_target(self, point) -> None:
        self._target = None if point is None else (float(point[0]), float(point[1]))
        self.update()

    def set_trajectory(self, points) -> None:
        self._trajectory = tuple(points)
        self.update()

    def set_max_range(self, value_mm: int) -> None:
        self._max_range_mm = max(500, int(value_mm))
        self.update()

    def set_angle_offset(self, degrees: float) -> None:
        self._angle_offset_deg = float(degrees)
        self.update()

    @property
    def measurement_count(self) -> int:
        return len(self._ranges)

    @property
    def frame_count(self) -> int:
        return self._frame_count

    def paintEvent(self, event) -> None:  # noqa: N802 - Qt API name
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        painter.fillRect(self.rect(), QColor("#101820"))

        w, h = self.width(), self.height()
        cx, cy = w / 2.0, h / 2.0 + 8
        radius = max(20.0, min(w, h) * 0.47)
        max_range = float(self._max_range_mm)
        scale = radius / max_range

        grid_pen = QPen(QColor("#344955"), 1)
        painter.setPen(grid_pen)
        for fraction in (0.25, 0.5, 0.75, 1.0):
            r = radius * fraction
            painter.drawEllipse(int(cx-r), int(cy-r), int(2*r), int(2*r))
        painter.drawLine(int(cx-radius), int(cy), int(cx+radius), int(cy))
        painter.drawLine(int(cx), int(cy-radius), int(cx), int(cy+radius))

        if self._ranges:
            point_color = QColor("#ff6b6b")
            painter.setPen(QPen(point_color, 1.5))
            painter.setBrush(point_color)
            count = len(self._ranges)
            offset = math.radians(self._angle_offset_deg)
            for index, distance_mm in enumerate(self._ranges):
                if distance_mm <= 0 or distance_mm > max_range:
                    continue
                angle = (2.0 * math.pi * index / count) + offset
                # Угол 0 направлен вверх; положительный угол идёт по часовой стрелке.
                x = cx + math.sin(angle) * distance_mm * scale
                y = cy - math.cos(angle) * distance_mm * scale
                painter.drawPoint(int(x), int(y))

        # Выбранная DWA-траектория в локальных координатах (right, forward).
        if self._trajectory:
            painter.setPen(QPen(QColor("#4ade80"), 2.5))
            previous = (cx, cy)
            for right_m, forward_m in self._trajectory:
                current = (cx + right_m * 1000 * scale, cy - forward_m * 1000 * scale)
                painter.drawLine(int(previous[0]), int(previous[1]), int(current[0]), int(current[1]))
                previous = current

        # Текущая цель маршрута — синяя метка в координатах робота.
        if self._target is not None:
            tx, ty = self._target
            px, py = cx + tx * 1000 * scale, cy - ty * 1000 * scale
            painter.setPen(QPen(QColor("#60a5fa"), 2))
            painter.setBrush(QColor(96, 165, 250, 70))
            painter.drawEllipse(int(px - 6), int(py - 6), 12, 12)
            painter.drawLine(int(cx), int(cy), int(px), int(py))

        # Корпус робота ID=1/ID=2: 0,60 x 0,90 м по QuickStart.
        # Центр корпуса совпадает с центром локальной системы координат; перед — вверх.
        body_w = max(1.0, self._robot_width_mm * scale)
        body_h = max(1.0, self._robot_length_mm * scale)
        body_x = cx - body_w / 2.0
        body_y = cy - body_h / 2.0
        painter.setPen(QPen(QColor("#4fc3f7"), 2))
        painter.setBrush(QColor("#243746"))
        painter.drawRoundedRect(
            int(body_x), int(body_y), max(1, int(body_w)), max(1, int(body_h)), 2, 2
        )
        # Короткая стрелка внутри корпуса обозначает переднюю сторону.
        painter.setPen(QPen(QColor("#e7eef2"), 2))
        painter.drawLine(int(cx), int(cy - body_h * 0.12), int(cx), int(body_y + body_h * 0.2))
        painter.drawLine(
            int(cx), int(body_y + body_h * 0.2),
            int(cx - body_w * 0.18), int(body_y + body_h * 0.34)
        )
        painter.drawLine(
            int(cx), int(body_y + body_h * 0.2),
            int(cx + body_w * 0.18), int(body_y + body_h * 0.34)
        )

        painter.setPen(QColor("#e7eef2"))
        painter.setFont(QFont("Sans Serif", 9))
        if self._ranges:
            info = f"Кадр: {self._frame_count} | измерений: {len(self._ranges)}"
            if self._timestamp:
                info += f" | {self._timestamp}"
        else:
            info = "Ожидание данных лидара…"
        painter.drawText(10, 18, info)
        painter.end()
