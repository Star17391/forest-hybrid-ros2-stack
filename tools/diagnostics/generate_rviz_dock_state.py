#!/usr/bin/env python3
"""Gera QMainWindow State hex para RViz: dock esquerdo estreito + Time em baixo."""
from __future__ import annotations

import sys

from PyQt5.QtCore import QByteArray, Qt
from PyQt5.QtWidgets import QApplication, QDockWidget, QMainWindow, QWidget


def main() -> int:
    app = QApplication(sys.argv)
    win = QMainWindow()
    win.resize(1920, 1040)

    left = QDockWidget("Displays", win)
    left.setWidget(QWidget())
    left.setMinimumWidth(180)
    left.setMaximumWidth(280)
    win.addDockWidget(Qt.LeftDockWidgetArea, left)

    bottom = QDockWidget("Time", win)
    bottom.setWidget(QWidget())
    bottom.setMaximumHeight(120)
    win.addDockWidget(Qt.BottomDockWidgetArea, bottom)

    central = QWidget()
    win.setCentralWidget(central)

    win.showMaximized()
    app.processEvents()

    state = win.saveState().toHex().data().decode("ascii")
    print(state)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
