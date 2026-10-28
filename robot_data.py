"""Модели данных телеметрии и команды IoTRobotWorld."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any
import json


def _first(data: dict[str, Any], *keys: str, default: Any = None) -> Any:
    """Возвращает первое присутствующее поле из списка возможных алиасов."""
    for key in keys:
        if key in data and data[key] is not None:
            return data[key]
    return default


def _as_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _as_int(value: Any, default: int = 0) -> int:
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return default


@dataclass
class Telemetry:
    """Нормализованное представление сообщения мониторинга."""

    n: int | None = None
    s: int | None = None
    c: int | None = None
    le: int | None = None
    re: int | None = None
    sa: int | None = None
    az: float | None = None
    b: int | None = None
    distances: list[int | None] = field(default_factory=lambda: [None] * 8)
    line_sensors: list[int | None] = field(default_factory=lambda: [None] * 5)
    sp: float | None = None
    x: float | None = None
    y: float | None = None
    course: float | None = None
    raw: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "Telemetry":
        # В разных конфигурациях/версиях симулятора встречаются разные имена
        # навигационных полей. Поддерживаем обе группы алиасов.
        distances = [
            _as_int(data[f"d{i}"]) if f"d{i}" in data and data[f"d{i}"] is not None else None
            for i in range(8)
        ]
        line_sensors = [
            _as_int(data[f"l{i}"]) if f"l{i}" in data and data[f"l{i}"] is not None else None
            for i in range(5)
        ]
        return cls(
            n=_as_int(data["n"]) if "n" in data else None,
            s=_as_int(data["s"]) if "s" in data else None,
            c=_as_int(data["c"]) if "c" in data else None,
            le=_as_int(data["le"]) if "le" in data else None,
            re=_as_int(data["re"]) if "re" in data else None,
            sa=_as_int(data["sa"]) if "sa" in data else None,
            az=_as_float(data["az"]) if "az" in data else None,
            b=_as_int(data["b"]) if "b" in data else None,
            distances=distances,
            line_sensors=line_sensors,
            sp=_as_float(data["sp"]) if "sp" in data else None,
            x=_as_float(_first(data, "long", "x", "X")) if _first(data, "long", "x", "X") is not None else None,
            y=_as_float(_first(data, "latt", "lat", "y", "Y")) if _first(data, "latt", "lat", "y", "Y") is not None else None,
            course=_as_float(_first(data, "course", "heading", "theta")) if _first(data, "course", "heading", "theta") is not None else None,
            raw=data.copy(),
        )

    def display_value(self, name: str) -> str:
        value = getattr(self, name, None)
        return "—" if value is None else str(value)


@dataclass
class RobotCommand:
    """Команда в формате протокола IoTRobotWorld.

    F и B оставлены в терминах протокола. Для автомобильного шасси ID=1
    необходимо подтвердить в используемой версии симулятора, как именно
    параметр B интерпретируется: QuickStart описывает его как баланс тяги,
    а задание лабораторной — управление углом руления.
    """

    N: int = 1
    M: int = 0
    F: int = 0
    B: int = 0
    T: int = 0
    L0: int = 0
    L1: int = 0
    L2: int = 0
    L3: int = 0
    L4: int = 0
    TEXT: str = ""

    def to_json_bytes(self) -> bytes:
        payload = {
            "N": int(self.N),
            "M": int(self.M),
            "F": int(self.F),
            "B": int(self.B),
            "T": int(self.T),
        }
        # Вспомогательные параметры линии и TEXT добавляем только при
        # необходимости, чтобы не отправлять лишние поля в каждом пакете.
        if any((self.L0, self.L1, self.L2, self.L3, self.L4)):
            payload.update({
                "L0": int(self.L0), "L1": int(self.L1), "L2": int(self.L2),
                "L3": int(self.L3), "L4": int(self.L4),
            })
        if self.TEXT:
            payload["TEXT"] = self.TEXT
        return (json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n").encode("utf-8")
