"""Протокол текстовых кадров 2D-лидара IoTRobotWorld (без зависимости от Qt)."""
from __future__ import annotations

import re
from dataclasses import dataclass

_TIMESTAMP_RE = re.compile(r"^\s*(\d{2}:\d{2}:\d{2}\.\d+)\s*>\s*(.*)$", re.DOTALL)


@dataclass(frozen=True)
class LidarFrame:
    timestamp: str | None
    ranges_mm: tuple[int, ...]
    source_host: str
    source_port: int


def parse_lidar_frame(packet: bytes, source_host: str = "", source_port: int = 0) -> LidarFrame:
    """Разбирает один кадр вида ``hh:mm:ss.nn> d0 d1 ...``.

    Временная метка допускает дробную часть любой длины; значения измерений
    целочисленные и приходят в сантиметрах; внутри приложения переводятся в миллиметры. Без метки пакет также допустим.
    """
    text = packet.decode("utf-8", errors="replace").strip().strip("\x00")
    if not text:
        raise ValueError("Пустой пакет лидара")

    match = _TIMESTAMP_RE.match(text)
    timestamp = None
    payload = text
    if match:
        timestamp, payload = match.group(1), match.group(2)
    else:
        # Некоторые конфигурации симулятора присылают только разделитель
        # кадра, без временной метки: "> 500 500 501 ...".
        # Удаляем ведущий разделитель, но не скрываем другие ошибки формата.
        if text.startswith(">"):
            payload = text[1:].strip()
        elif ">" in text:
            prefix, rest = text.split(">", 1)
            if ":" in prefix:
                raise ValueError(f"Некорректная временная метка: {prefix!r}")
            # Если перед разделителем нет временной метки, считаем его
            # служебным префиксом только когда до него нет полезных данных.
            if not prefix.strip():
                payload = rest.strip()

    values: list[int] = []
    for token in payload.split():
        try:
            values.append(int(token.rstrip(",;")))
        except ValueError as exc:
            raise ValueError(f"Некорректное значение дальности: {token!r}") from exc

    if not values:
        raise ValueError("В пакете лидара нет измерений")
    if len(values) > 360:
        raise ValueError(f"Ожидалось не более 360 измерений, получено {len(values)}")
    if any(value < 0 for value in values):
        raise ValueError("Дальность не может быть отрицательной")

    return LidarFrame(timestamp, tuple(values), source_host, source_port)
