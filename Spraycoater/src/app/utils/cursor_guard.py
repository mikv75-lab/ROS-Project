# -*- coding: utf-8 -*-
from __future__ import annotations
from contextlib import contextmanager
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QApplication, QWidget

@contextmanager
def busy_cursor(disable: QWidget | None = None):
    """
    Setzt den globalen Mauszeiger auf 'Warten' und (optional) deaktiviert ein Widget.
    Stellt in jedem Fall den alten Zustand wieder her (auch bei Exceptions).
    """
    try:
        QApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
        if disable is not None:
            disable.setEnabled(False)
        yield
    finally:
        if disable is not None:
            disable.setEnabled(True)
        QApplication.restoreOverrideCursor()
