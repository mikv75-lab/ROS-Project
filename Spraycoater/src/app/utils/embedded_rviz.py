# -*- coding: utf-8 -*-
from __future__ import annotations

from contextlib import contextmanager
from typing import Optional

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QApplication, QWidget


@contextmanager
def busy_cursor(disable: Optional[QWidget] = None):
    """
    Setzt den globalen Mauszeiger auf 'Warten' und (optional) deaktiviert ein Widget.
    - verschachtelbar (nested busy_cursor)
    - stellt vorherigen Enabled-Zustand wieder her
    - robust bei Exceptions
    """
    prev_enabled = None
    did_override = False
    try:
        # Cursor: nur dann new override setzen, wenn noch keiner aktiv ist
        if QApplication.overrideCursor() is None:
            QApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
            did_override = True

        if disable is not None:
            prev_enabled = disable.isEnabled()
            disable.setEnabled(False)

        yield

    finally:
        if disable is not None and prev_enabled is not None:
            disable.setEnabled(prev_enabled)

        # Cursor nur zurücksetzen, wenn wir ihn gesetzt haben
        if did_override:
            # Safety: restore nur wenn noch ein override da ist
            if QApplication.overrideCursor() is not None:
                QApplication.restoreOverrideCursor()
