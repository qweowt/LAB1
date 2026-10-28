"""UDP-обмен с IoTRobotWorld.

Симулятор регулярно присылает телеметрию на локальный порт приложения.
Команды отправляются на IP/порт симулятора. В конце пакета добавляется '\\n'.
"""
from __future__ import annotations

import json
import socket
import threading
from typing import Any

from PyQt6.QtCore import QThread, pyqtSignal


class UDPReceiver(QThread):
    telemetry_received = pyqtSignal(dict)
    log_message = pyqtSignal(str)
    connection_state = pyqtSignal(bool, str)

    def __init__(
        self,
        remote_host: str,
        remote_port: int,
        local_port: int,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self.remote_host = remote_host
        self.remote_port = remote_port
        self.local_port = local_port
        self._socket: socket.socket | None = None
        self._send_lock = threading.Lock()
        self._stop_event = threading.Event()

    def run(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket = sock
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("0.0.0.0", self.local_port))
            sock.settimeout(0.25)
            self.connection_state.emit(
                True,
                f"UDP готов: локальный порт {self.local_port}; адрес назначения "
                f"{self.remote_host}:{self.remote_port}",
            )
            self.log_message.emit(f"Приём UDP запущен на 0.0.0.0:{self.local_port}")
            while not self._stop_event.is_set():
                try:
                    packet, address = sock.recvfrom(65535)
                except socket.timeout:
                    continue
                except OSError as exc:
                    if not self._stop_event.is_set():
                        self.log_message.emit(f"Ошибка приёма UDP: {exc}")
                    break

                text = packet.decode("utf-8", errors="replace").strip()
                if not text:
                    continue
                # Обычно пакет содержит один JSON. На случай нескольких
                # строк в одном пакете обрабатываем каждую отдельно.
                for line in text.splitlines():
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        message: Any = json.loads(line)
                    except json.JSONDecodeError:
                        self.log_message.emit(
                            f"Получен не JSON от {address[0]}:{address[1]}: {line[:180]}"
                        )
                        continue
                    if isinstance(message, dict):
                        self.telemetry_received.emit(message)
                        self.log_message.emit(
                            f"RX {address[0]}:{address[1]}  {json.dumps(message, ensure_ascii=False)}"
                        )
                    else:
                        self.log_message.emit(f"Получен JSON не в виде объекта: {type(message).__name__}")
        except OSError as exc:
            self.connection_state.emit(False, f"Не удалось открыть UDP-порт {self.local_port}: {exc}")
            self.log_message.emit(f"Ошибка UDP-сокета: {exc}")
        finally:
            try:
                sock.close()
            except OSError:
                pass
            self._socket = None
            self.connection_state.emit(False, "UDP-соединение остановлено")

    def send_packet(self, payload: bytes) -> bool:
        with self._send_lock:
            sock = self._socket
            if sock is None or not self.isRunning():
                self.log_message.emit("Команда не отправлена: UDP-приёмник не запущен")
                return False
            try:
                sock.sendto(payload, (self.remote_host, self.remote_port))
            except OSError as exc:
                self.log_message.emit(f"Ошибка отправки UDP: {exc}")
                return False
        self.log_message.emit(f"TX {self.remote_host}:{self.remote_port}  {payload.decode('utf-8').strip()}")
        return True

    def stop(self) -> None:
        self._stop_event.set()
        # Таймаут recvfrom гарантирует выход из цикла без принудительного
        # закрытия сокета из другого потока.
        self.wait(1500)


class UDPConnection:
    """Небольшой фасад, который удобно использовать из GUI."""

    def __init__(self) -> None:
        self.receiver: UDPReceiver | None = None

    def start(self, host: str, remote_port: int, local_port: int, parent=None) -> UDPReceiver:
        if self.receiver is not None and self.receiver.isRunning():
            raise RuntimeError("UDP уже запущен")
        # Возвращаем ещё не запущенный поток, чтобы GUI успел подключить
        # сигналы и не потерял раннее сообщение об ошибке bind().
        self.receiver = UDPReceiver(host, remote_port, local_port, parent)
        return self.receiver

    def send(self, payload: bytes) -> bool:
        if self.receiver is None:
            return False
        return self.receiver.send_packet(payload)

    def stop(self) -> None:
        if self.receiver is not None:
            self.receiver.stop()
            self.receiver = None
