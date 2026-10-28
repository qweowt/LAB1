"""Высокоуровневая модель робота. Алгоритм движения добавим на следующем этапе."""
from __future__ import annotations

from robot_data import RobotCommand, Telemetry


class Robot:
    def __init__(self) -> None:
        self.telemetry = Telemetry()
        self.command_number = 0

    def update_telemetry(self, data: dict) -> Telemetry:
        self.telemetry = Telemetry.from_dict(data)
        return self.telemetry

    def make_command(self, f: int = 0, b: int = 0, m: int = 0, t: int = 0, text: str = "") -> RobotCommand:
        self.command_number += 1
        return RobotCommand(N=self.command_number, M=m, F=f, B=b, T=t, TEXT=text)

    def stop_command(self) -> RobotCommand:
        # По QuickStart команда остановки: M=0, F=0, B=0, T=0.
        return self.make_command(f=0, b=0, m=0, t=0)
