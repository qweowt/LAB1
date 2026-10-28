"""PyQt6-интерфейс управления, телеметрии и визуализации 2D-лидара."""
from __future__ import annotations

from datetime import datetime
from pathlib import Path
import csv
import math

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QLabel, QLineEdit, QPushButton, QSpinBox,
    QSlider, QDoubleSpinBox, QGridLayout, QVBoxLayout, QHBoxLayout,
    QGroupBox, QFormLayout, QPlainTextEdit, QTableWidget,
    QTableWidgetItem, QMessageBox, QSplitter, QFileDialog,
)

from robot import Robot
from robot_data import Telemetry
from udp import UDPConnection
from lidar import LidarConnection, LidarFrame
from lidar_view import LidarView
from dwa import choose_dwa_command, lidar_obstacles


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("IoTRobotWorld — управление и телеметрия")
        self.resize(1450, 900)
        self.robot = Robot()
        self.udp = UDPConnection()
        self.lidar = LidarConnection()
        self._connected = False
        self._lidar_connected = False
        self.route_points: list[tuple[float, float]] = []
        self.current_target_index = 0
        self._auto_mode = False
        self._auto_speed_mps = 0.0
        self._last_dwa_result = None
        self._build_ui()
        self._control_timer = QTimer(self)
        self._control_timer.setInterval(200)
        self._control_timer.timeout.connect(self._auto_control_tick)
        self._set_connected(False)

    def _build_ui(self) -> None:
        root = QWidget()
        self.setCentralWidget(root)
        main_layout = QVBoxLayout(root)

        connection_group = QGroupBox("UDP-подключение")
        connection_form = QGridLayout(connection_group)
        self.host_input = QLineEdit("127.0.0.1")
        self.remote_port_input = QSpinBox()
        self.remote_port_input.setRange(1, 65535)
        self.remote_port_input.setValue(8888)
        self.local_port_input = QSpinBox()
        self.local_port_input.setRange(1, 65535)
        self.local_port_input.setValue(7777)
        self.connect_button = QPushButton("Подключиться")
        self.disconnect_button = QPushButton("Отключиться")
        connection_form.addWidget(QLabel("IP симулятора:"), 0, 0)
        connection_form.addWidget(self.host_input, 0, 1)
        connection_form.addWidget(QLabel("Порт симулятора:"), 0, 2)
        connection_form.addWidget(self.remote_port_input, 0, 3)
        connection_form.addWidget(QLabel("Локальный порт:"), 0, 4)
        connection_form.addWidget(self.local_port_input, 0, 5)
        connection_form.addWidget(self.connect_button, 0, 6)
        connection_form.addWidget(self.disconnect_button, 0, 7)
        self.status_label = QLabel("Отключено")
        self.status_label.setStyleSheet("font-weight: bold;")
        connection_form.addWidget(self.status_label, 1, 0, 1, 8)
        main_layout.addWidget(connection_group)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        main_layout.addWidget(splitter, 1)

        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)

        control_group = QGroupBox("Ручная команда")
        control_layout = QFormLayout(control_group)
        self.f_spin = QSpinBox()
        self.f_spin.setRange(-100, 100)
        self.f_spin.setSuffix(" %")
        self.f_slider = QSlider(Qt.Orientation.Horizontal)
        self.f_slider.setRange(-100, 100)
        self.f_slider.setValue(0)
        self.f_spin.valueChanged.connect(self.f_slider.setValue)
        self.f_slider.valueChanged.connect(self.f_spin.setValue)
        f_row = QWidget()
        f_row_layout = QHBoxLayout(f_row)
        f_row_layout.setContentsMargins(0, 0, 0, 0)
        f_row_layout.addWidget(self.f_slider, 1)
        f_row_layout.addWidget(self.f_spin)
        control_layout.addRow("F — продольная тяга:", f_row)

        self.b_spin = QSpinBox()
        self.b_spin.setRange(-100, 100)
        self.b_spin.setSuffix(" ед.")
        self.b_slider = QSlider(Qt.Orientation.Horizontal)
        self.b_slider.setRange(-100, 100)
        self.b_slider.setValue(0)
        self.b_spin.valueChanged.connect(self.b_slider.setValue)
        self.b_slider.valueChanged.connect(self.b_spin.setValue)
        b_row = QWidget()
        b_row_layout = QHBoxLayout(b_row)
        b_row_layout.setContentsMargins(0, 0, 0, 0)
        b_row_layout.addWidget(self.b_slider, 1)
        b_row_layout.addWidget(self.b_spin)
        control_layout.addRow("B — параметр B:", b_row)

        self.send_button = QPushButton("Отправить команду")
        self.stop_button = QPushButton("СТОП")
        self.stop_button.setStyleSheet("font-weight: bold;")
        self.auto_button = QPushButton("Автоматический режим: ВЫКЛ")
        self.auto_button.setCheckable(True)
        command_buttons = QHBoxLayout()
        command_buttons.addWidget(self.send_button)
        command_buttons.addWidget(self.stop_button)
        command_buttons.addWidget(self.auto_button)
        control_layout.addRow(command_buttons)
        self.max_speed_spin = QDoubleSpinBox()
        self.max_speed_spin.setRange(0.02, 0.20)
        self.max_speed_spin.setSingleStep(0.02)
        self.max_speed_spin.setValue(0.20)
        self.max_speed_spin.setSuffix(" м/с")
        control_layout.addRow("Максимальная скорость:", self.max_speed_spin)
        self.auto_state_label = QLabel("Автоматический режим выключен")
        self.auto_state_label.setWordWrap(True)
        control_layout.addRow("DWA:", self.auto_state_label)
        self.last_command_label = QLabel("Последняя команда: —")
        self.last_command_label.setWordWrap(True)
        control_layout.addRow(self.last_command_label)
        left_layout.addWidget(control_group)

        notes = QLabel(
            "Поля F/B отправляются как значения протокола IoTRobotWorld. "
            "Для шасси ID=1 перед испытаниями нужно сверить трактовку B "
            "с настройками/версией симулятора. До проверки держите F=0 и B=0."
        )
        notes.setWordWrap(True)
        notes.setStyleSheet("color: #8a5a00;")
        left_layout.addWidget(notes)

        route_group = QGroupBox("Маршрут (CurrentRoute.txt / CSV)")
        route_layout = QVBoxLayout(route_group)
        route_buttons = QHBoxLayout()
        self.load_route_button = QPushButton("Загрузить CSV…")
        self.reset_route_button = QPushButton("Сбросить цель")
        route_buttons.addWidget(self.load_route_button)
        route_buttons.addWidget(self.reset_route_button)
        route_layout.addLayout(route_buttons)
        self.route_status_label = QLabel("Маршрут не загружен")
        route_layout.addWidget(self.route_status_label)
        self.route_table = QTableWidget(0, 3)
        self.route_table.setHorizontalHeaderLabels(["№", "X, м", "Y, м"])
        self.route_table.verticalHeader().setVisible(False)
        self.route_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.route_table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.route_table.horizontalHeader().setStretchLastSection(True)
        self.route_table.setMinimumHeight(190)
        route_layout.addWidget(self.route_table)
        left_layout.addWidget(route_group, 1)
        left_layout.addStretch(1)

        telemetry_group = QGroupBox("Телеметрия")
        telemetry_layout = QVBoxLayout(telemetry_group)
        self.telemetry_table = QTableWidget(0, 2)
        self.telemetry_table.setHorizontalHeaderLabels(["Поле", "Значение"])
        self.telemetry_table.verticalHeader().setVisible(False)
        self.telemetry_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.telemetry_table.setSelectionMode(QTableWidget.SelectionMode.NoSelection)
        self.telemetry_table.horizontalHeader().setStretchLastSection(True)
        self.telemetry_table.setMinimumWidth(280)
        self.telemetry_keys = [
            ("n", "Номер/состояние n"), ("c", "Код объекта c"),
            ("b", "Бампер b"), ("le", "Энкодер слева le"),
            ("re", "Энкодер справа re"), ("az", "Азимут az"),
            ("x", "Координата X (long/x)"), ("y", "Координата Y (latt/y)"),
            ("course", "Курс (course)"), ("sp", "Скорость sp"),
            *[(f"d{i}", f"Дальномер d{i}") for i in range(8)],
            *[(f"l{i}", f"Датчик линии l{i}") for i in range(5)],
        ]
        self.telemetry_table.setRowCount(len(self.telemetry_keys))
        self.telemetry_items: dict[str, QTableWidgetItem] = {}
        for row, (key, label) in enumerate(self.telemetry_keys):
            self.telemetry_table.setItem(row, 0, QTableWidgetItem(label))
            value_item = QTableWidgetItem("—")
            self.telemetry_table.setItem(row, 1, value_item)
            self.telemetry_items[key] = value_item
        telemetry_layout.addWidget(self.telemetry_table)
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)

        lidar_group = QGroupBox("2D-лидар")
        lidar_layout = QVBoxLayout(lidar_group)
        lidar_controls = QHBoxLayout()
        self.lidar_port_input = QSpinBox()
        self.lidar_port_input.setRange(1, 65535)
        self.lidar_port_input.setValue(2368)
        self.lidar_connect_button = QPushButton("Подключить лидар")
        self.lidar_disconnect_button = QPushButton("Отключить")
        self.lidar_disconnect_button.setEnabled(False)
        self.lidar_status_label = QLabel("Лидар отключён")
        lidar_controls.addWidget(QLabel("Локальный UDP-порт:"))
        lidar_controls.addWidget(self.lidar_port_input)
        lidar_controls.addWidget(self.lidar_connect_button)
        lidar_controls.addWidget(self.lidar_disconnect_button)
        lidar_layout.addLayout(lidar_controls)

        lidar_view_controls = QHBoxLayout()
        self.lidar_range_spin = QSpinBox()
        self.lidar_range_spin.setRange(500, 30000)
        self.lidar_range_spin.setSingleStep(500)
        self.lidar_range_spin.setValue(4000)
        self.lidar_range_spin.setSuffix(" мм")
        self.lidar_angle_spin = QDoubleSpinBox()
        self.lidar_angle_spin.setRange(-360.0, 360.0)
        self.lidar_angle_spin.setSingleStep(5.0)
        self.lidar_angle_spin.setValue(180.0)
        self.lidar_angle_spin.setSuffix("°")
        lidar_view_controls.addWidget(QLabel("Макс. дальность (радиус):"))
        lidar_view_controls.addWidget(self.lidar_range_spin)
        lidar_view_controls.addSpacing(12)
        lidar_view_controls.addWidget(QLabel("Угол смещения:"))
        lidar_view_controls.addWidget(self.lidar_angle_spin)
        lidar_view_controls.addStretch(1)
        lidar_layout.addLayout(lidar_view_controls)
        self.lidar_view = LidarView()
        lidar_layout.addWidget(self.lidar_view, 1)
        lidar_layout.addWidget(self.lidar_status_label)
        right_layout.addWidget(lidar_group, 3)
        right_layout.addWidget(telemetry_group, 2)

        log_group = QGroupBox("Журнал обмена")
        log_layout = QVBoxLayout(log_group)
        self.log_output = QPlainTextEdit()
        self.log_output.setReadOnly(True)
        self.log_output.document().setMaximumBlockCount(1500)
        log_layout.addWidget(self.log_output)
        clear_log_button = QPushButton("Очистить журнал")
        clear_log_button.clicked.connect(self.log_output.clear)
        log_layout.addWidget(clear_log_button)
        right_layout.addWidget(log_group, 1)

        splitter.addWidget(left_panel)
        splitter.addWidget(right_panel)
        splitter.setSizes([420, 650])

        self.connect_button.clicked.connect(self._connect_udp)
        self.disconnect_button.clicked.connect(self._disconnect_udp)
        self.send_button.clicked.connect(self._send_command)
        self.stop_button.clicked.connect(self._send_stop)
        self.auto_button.toggled.connect(self._set_auto_mode)
        self.lidar_connect_button.clicked.connect(self._connect_lidar)
        self.lidar_disconnect_button.clicked.connect(self._disconnect_lidar)
        self.lidar_range_spin.valueChanged.connect(self.lidar_view.set_max_range)
        self.lidar_angle_spin.valueChanged.connect(self.lidar_view.set_angle_offset)
        self.load_route_button.clicked.connect(self._choose_route_file)
        self.reset_route_button.clicked.connect(self._reset_route_target)
        # Загружаем маршрут рядом с main.py, если файл присутствует.
        default_route = Path(__file__).resolve().with_name("CurrentRoute.txt")
        if default_route.exists():
            self._load_route_file(str(default_route))

    def _choose_route_file(self) -> None:
        filename, _ = QFileDialog.getOpenFileName(
            self, "Загрузить маршрут", str(Path.cwd()),
            "Маршрут CSV/TXT (*.txt *.csv);;Все файлы (*)"
        )
        if filename:
            self._load_route_file(filename)

    def _load_route_file(self, filename: str) -> None:
        points: list[tuple[float, float]] = []
        try:
            with open(filename, "r", encoding="utf-8-sig", newline="") as file:
                for line_no, line in enumerate(file, start=1):
                    line = line.strip()
                    if not line or line.startswith("#"):
                        continue
                    # Поддерживаем CSV с разделителем запятая или точка с запятой.
                    delimiter = ";" if ";" in line and "," not in line else ","
                    cells = next(csv.reader([line], delimiter=delimiter))
                    if len(cells) != 2:
                        raise ValueError(f"строка {line_no}: ожидались две координаты X,Y")
                    x, y = (float(cell.strip().replace(",", ".")) for cell in cells)
                    if not (math.isfinite(x) and math.isfinite(y)):
                        raise ValueError(f"строка {line_no}: координаты должны быть конечными числами")
                    points.append((x, y))
        except (OSError, ValueError, csv.Error) as exc:
            QMessageBox.warning(self, "Ошибка маршрута", f"Не удалось загрузить маршрут:\n{exc}")
            self._log(f"Ошибка загрузки маршрута {filename}: {exc}")
            return
        if not points:
            QMessageBox.warning(self, "Пустой маршрут", "В файле не найдено ни одной точки X,Y.")
            return
        if self._auto_mode:
            self._set_auto_mode(False)
        self.route_points = points
        self.current_target_index = 0
        self.route_table.setRowCount(len(points))
        for i, (x, y) in enumerate(points):
            self.route_table.setItem(i, 0, QTableWidgetItem(str(i + 1)))
            self.route_table.setItem(i, 1, QTableWidgetItem(f"{x:.3f}"))
            self.route_table.setItem(i, 2, QTableWidgetItem(f"{y:.3f}"))
        self.route_table.selectRow(0)
        self.route_status_label.setText(f"Точек: {len(points)} | цель: 1/{len(points)}")
        self._log(f"Загружен маршрут: {filename}, точек: {len(points)}")

    def _reset_route_target(self) -> None:
        self._auto_speed_mps = 0.0
        self.lidar_view.set_trajectory(())
        if not self.route_points:
            return
        self.current_target_index = 0
        self.route_table.selectRow(0)
        self.route_status_label.setText(f"Точек: {len(self.route_points)} | цель: 1/{len(self.route_points)}")
        self._log("Текущая цель маршрута сброшена на точку 1")

    def _update_route_progress(self, x: float | None, y: float | None) -> None:
        if not self.route_points or x is None or y is None:
            return
        # Переход к следующей цели при расстоянии <= 1.5 м, как в задании.
        while self.current_target_index < len(self.route_points):
            tx, ty = self.route_points[self.current_target_index]
            distance = math.hypot(tx - x, ty - y)
            if distance > 1.5:
                break
            if self.current_target_index == len(self.route_points) - 1:
                self.route_status_label.setText(f"Маршрут завершён | точек: {len(self.route_points)}")
                self.route_table.selectRow(self.current_target_index)
                if self._auto_mode:
                    self._set_auto_mode(False)
                    self._send_stop()
                    QMessageBox.information(self, "Маршрут завершён", "Робот достиг последней точки маршрута.")
                return
            self.current_target_index += 1
            self.route_table.selectRow(self.current_target_index)
        tx, ty = self.route_points[self.current_target_index]
        distance = math.hypot(tx - x, ty - y)
        self.route_status_label.setText(
            f"Цель {self.current_target_index + 1}/{len(self.route_points)}: "
            f"({tx:.2f}, {ty:.2f}) м | до цели: {distance:.2f} м"
        )

    def _log(self, message: str) -> None:
        stamp = datetime.now().strftime("%H:%M:%S")
        self.log_output.appendPlainText(f"[{stamp}] {message}")

    def _set_connected(self, connected: bool, message: str | None = None) -> None:
        self._connected = connected
        self.connect_button.setEnabled(not connected)
        self.disconnect_button.setEnabled(connected)
        self.send_button.setEnabled(connected)
        self.stop_button.setEnabled(connected)
        self.host_input.setEnabled(not connected)
        self.remote_port_input.setEnabled(not connected)
        self.local_port_input.setEnabled(not connected)
        if message:
            self.status_label.setText(message)
        else:
            self.status_label.setText("UDP подключён" if connected else "Отключено")

    def _connect_udp(self) -> None:
        host = self.host_input.text().strip()
        if not host:
            QMessageBox.warning(self, "Нет IP", "Укажите IP-адрес симулятора.")
            return
        try:
            receiver = self.udp.start(
                host,
                self.remote_port_input.value(),
                self.local_port_input.value(),
                parent=self,
            )
        except RuntimeError as exc:
            QMessageBox.warning(self, "UDP", str(exc))
            return
        receiver.telemetry_received.connect(self._on_telemetry)
        receiver.log_message.connect(self._log)
        receiver.connection_state.connect(self._on_connection_state)
        receiver.start()
        self._set_connected(True, "Запуск UDP…")
        self._log(
            f"Запуск: симулятор {host}:{self.remote_port_input.value()}, "
            f"локальный порт {self.local_port_input.value()}"
        )

    def _disconnect_udp(self) -> None:
        # Перед закрытием соединения отправляем нулевую команду, если сокет активен.
        if self._connected:
            self._send_stop()
        self.udp.stop()
        self._set_connected(False, "Отключено")
        self._log("UDP отключён")

    def _on_connection_state(self, active: bool, message: str) -> None:
        self.status_label.setText(message)
        if not active and self._connected and message.startswith("Не удалось"):
            self._set_connected(False, message)


    def _connect_lidar(self) -> None:
        try:
            receiver = self.lidar.start(self.lidar_port_input.value(), parent=self)
        except RuntimeError as exc:
            QMessageBox.warning(self, "Лидар", str(exc))
            return
        receiver.frame_received.connect(self._on_lidar_frame)
        receiver.log_message.connect(self._log)
        receiver.connection_state.connect(self._on_lidar_state)
        receiver.start()
        self._lidar_connected = True
        self.lidar_connect_button.setEnabled(False)
        self.lidar_disconnect_button.setEnabled(True)
        self.lidar_port_input.setEnabled(False)
        self.lidar_status_label.setText(f"Ожидание пакетов на порту {self.lidar_port_input.value()}…")
        self._log(f"Запуск приёмника лидара на локальном UDP-порту {self.lidar_port_input.value()}")

    def _disconnect_lidar(self) -> None:
        self.lidar.stop()
        self._lidar_connected = False
        self.lidar_connect_button.setEnabled(True)
        self.lidar_disconnect_button.setEnabled(False)
        self.lidar_port_input.setEnabled(True)
        self.lidar_status_label.setText("Лидар отключён")
        self._log("Приём лидара отключён")

    def _on_lidar_state(self, active: bool, message: str) -> None:
        self.lidar_status_label.setText(message)
        if not active and message.startswith("Не удалось"):
            self._lidar_connected = False
            self.lidar_connect_button.setEnabled(True)
            self.lidar_disconnect_button.setEnabled(False)
            self.lidar_port_input.setEnabled(True)

    def _on_lidar_frame(self, frame: LidarFrame) -> None:
        self.lidar_view.set_frame(frame)
        stamp = frame.timestamp or "без временной метки"
        self.lidar_status_label.setText(
            f"Кадров: {self.lidar_view.frame_count} | измерений: {len(frame.ranges_mm)} | {stamp}"
        )

    def _on_telemetry(self, data: dict) -> None:
        telemetry = self.robot.update_telemetry(data)
        values: dict[str, object] = {
            "n": telemetry.n, "c": telemetry.c, "b": telemetry.b,
            "le": telemetry.le, "re": telemetry.re, "az": telemetry.az,
            "x": telemetry.x, "y": telemetry.y, "course": telemetry.course,
            "sp": telemetry.sp,
        }
        values.update({f"d{i}": telemetry.distances[i] for i in range(8)})
        values.update({f"l{i}": telemetry.line_sensors[i] for i in range(5)})
        for key, value in values.items():
            self.telemetry_items[key].setText("—" if value is None else str(value))
        self._update_route_progress(telemetry.x, telemetry.y)
        # Любое ненулевое значение bumper считается контактом: аварийный останов.
        if self._auto_mode and telemetry.b not in (None, 0):
            self._set_auto_mode(False)
            self._send_stop()
            self.auto_state_label.setText("АВАРИЙНАЯ ОСТАНОВКА: сработал бампер")
            QMessageBox.warning(self, "Аварийная остановка", "Бампер сработал. Робот остановлен.")

    def _set_auto_mode(self, enabled: bool) -> None:
        enabled = bool(enabled)
        if enabled and not self.route_points:
            self.auto_button.blockSignals(True)
            self.auto_button.setChecked(False)
            self.auto_button.blockSignals(False)
            QMessageBox.warning(self, "Нет маршрута", "Сначала загрузите CurrentRoute.txt/CSV.")
            return
        if enabled and not self._connected:
            self.auto_button.blockSignals(True)
            self.auto_button.setChecked(False)
            self.auto_button.blockSignals(False)
            QMessageBox.warning(self, "Нет UDP", "Сначала подключитесь к симулятору.")
            return
        self._auto_mode = enabled
        self.auto_button.setText("Автоматический режим: ВКЛ" if enabled else "Автоматический режим: ВЫКЛ")
        self.auto_state_label.setText("DWA управляет движением" if enabled else "Автоматический режим выключен")
        if enabled:
            self._auto_speed_mps = 0.0
            self._control_timer.start()
            self._log("Автоматический режим DWA включён")
        else:
            self._control_timer.stop()
            self.lidar_view.set_trajectory(())
            self.lidar_view.set_target(None)
            self._log("Автоматический режим выключен")

    def _auto_control_tick(self) -> None:
        if not self._auto_mode or not self._connected:
            return
        telemetry = self.robot.telemetry
        # В некоторых конфигурациях симулятора поле course отсутствует,
        # но передаётся az (азимут). Предпочитаем course, используем az как резерв.
        heading_deg = telemetry.course if telemetry.course is not None else telemetry.az
        if telemetry.x is None or telemetry.y is None or heading_deg is None:
            missing = []
            if telemetry.x is None:
                missing.append("X/long")
            if telemetry.y is None:
                missing.append("Y/latt")
            if heading_deg is None:
                missing.append("course/az")
            self.auto_state_label.setText("Ожидание телеметрии: " + ", ".join(missing))
            return
        if not self.route_points or self.current_target_index >= len(self.route_points):
            self._set_auto_mode(False)
            self._send_stop()
            return
        if telemetry.b not in (None, 0):
            self._set_auto_mode(False)
            self._send_stop()
            self.auto_state_label.setText("Аварийная остановка: бампер")
            return
        if self.lidar_view.measurement_count == 0:
            self._auto_speed_mps = 0.0
            self._send(self.robot.make_command(f=0, b=0))
            self.auto_state_label.setText("Ожидание первого кадра лидара — движение заблокировано")
            return
        tx, ty = self.route_points[self.current_target_index]
        dx, dy = tx - telemetry.x, ty - telemetry.y
        course = math.radians(heading_deg)  # 0° north, clockwise; fallback az
        # Перевод в локальные координаты: вправо, вперёд.
        goal_right = dx * math.cos(course) - dy * math.sin(course)
        goal_forward = dx * math.sin(course) + dy * math.cos(course)
        obstacles = lidar_obstacles(
            self.lidar_view.ranges_mm,
            self.lidar_angle_spin.value(),
            self.lidar_range_spin.value(),
        )
        result = choose_dwa_command(
            (goal_right, goal_forward), obstacles, self._auto_speed_mps,
            max_speed=self.max_speed_spin.value(), max_accel=0.20,
        )
        self._last_dwa_result = result
        self.lidar_view.set_target((goal_right, goal_forward))
        self.lidar_view.set_trajectory(result.trajectory)
        if result.blocked:
            self._auto_speed_mps = 0.0
            self._send(self.robot.make_command(f=0, b=0))
            self.auto_state_label.setText("Препятствие на всех траекториях — остановка")
            return
        self._auto_speed_mps = result.speed_mps
        # Интерфейсный протокол использует F/B в процентах/единицах. Здесь B
        # трактуется как руление для автомобильного шасси; проверьте знак на малой скорости.
        f_cmd = int(round(100 * result.speed_mps / max(0.02, self.max_speed_spin.value())))
        b_cmd = int(round(100 * result.steering_deg / 30.0))
        self._send(self.robot.make_command(f=f_cmd, b=b_cmd))
        self.auto_state_label.setText(
            f"DWA: v={result.speed_mps:.2f} м/с, руль={result.steering_deg:+.0f}°; "
            f"F={f_cmd}, B={b_cmd}"
        )

    def _send_command(self) -> None:
        if self._auto_mode:
            self._set_auto_mode(False)
        command = self.robot.make_command(f=self.f_spin.value(), b=self.b_spin.value())
        self._send(command)

    def _send_stop(self) -> None:
        if self._auto_mode:
            self._set_auto_mode(False)
        self._auto_speed_mps = 0.0
        self.lidar_view.set_trajectory(())
        self.lidar_view.set_target(None)
        self.f_spin.setValue(0)
        self.b_spin.setValue(0)
        self._send(self.robot.stop_command())

    def _send(self, command) -> None:
        payload = command.to_json_bytes()
        if self.udp.send(payload):
            self.last_command_label.setText(
                "Последняя команда: " + payload.decode("utf-8").strip()
            )

    def closeEvent(self, event) -> None:
        if self._connected:
            self._send_stop()
        self.udp.stop()
        self.lidar.stop()
        event.accept()
