"""Точка входа приложения управления роботом IoTRobotWorld."""
import sys

from PyQt6.QtWidgets import QApplication

from gui import MainWindow


def main() -> int:
    app = QApplication(sys.argv)
    app.setApplicationName("IoTRobotWorld Controller")
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
