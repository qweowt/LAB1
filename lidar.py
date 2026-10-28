"""Приём и разбор текстовых кадров 2D-лидара IoTRobotWorld по UDP."""
from __future__ import annotations

import socket
import threading

from PyQt6.QtCore import QThread, pyqtSignal

from lidar_protocol import LidarFrame, parse_lidar_frame


class LidarReceiver(QThread):
    """Независимый UDP-приёмник лидарных пакетов."""

    frame_received = pyqtSignal(object)
    log_message = pyqtSignal(str)
    connection_state = pyqtSignal(bool, str)

    def __init__(self, local_port: int, parent=None) -> None:
        super().__init__(parent)
        self.local_port = int(local_port)
        self._stop_event = threading.Event()
        self._socket: socket.socket | None = None

    def run(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket = sock
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("0.0.0.0", self.local_port))
            sock.settimeout(0.25)
            self.connection_state.emit(True, f"Лидар слушает UDP-порт {self.local_port}")
            self.log_message.emit(f"Приём лидара запущен на 0.0.0.0:{self.local_port}")
            while not self._stop_event.is_set():
                try:
                    packet, address = sock.recvfrom(65535)
                except socket.timeout:
                    continue
                except OSError as exc:
                    if not self._stop_event.is_set():
                        self.log_message.emit(f"Ошибка приёма лидара: {exc}")
                    break
                try:
                    frame = parse_lidar_frame(packet, address[0], address[1])
                except ValueError as exc:
                    snippet = packet[:160].decode("utf-8", errors="replace").strip()
                    self.log_message.emit(f"Пакет лидара не распознан ({address[0]}:{address[1]}): {exc}; {snippet!r}")
                    continue
                self.frame_received.emit(frame)
        except OSError as exc:
            self.connection_state.emit(False, f"Не удалось открыть порт лидара {self.local_port}: {exc}")
            self.log_message.emit(f"Ошибка сокета лидара: {exc}")
        finally:
            try:
                sock.close()
            except OSError:
                pass
            self._socket = None
            self.connection_state.emit(False, "Приём лидара остановлен")

    def stop(self) -> None:
        self._stop_event.set()
        self.wait(1500)


class LidarConnection:
    """Фасад для запуска/остановки отдельного потока лидара."""

    def __init__(self) -> None:
        self.receiver: LidarReceiver | None = None

    def start(self, local_port: int, parent=None) -> LidarReceiver:
        if self.receiver is not None and self.receiver.isRunning():
            raise RuntimeError("Приём лидара уже запущен")
        self.receiver = LidarReceiver(local_port, parent)
        return self.receiver

    def stop(self) -> None:
        if self.receiver is not None:
            self.receiver.stop()
            self.receiver = None
