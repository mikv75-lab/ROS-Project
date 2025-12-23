# Spraycoater/src/app/tabs/system/system_tab.py
# -*- coding: utf-8 -*-
from __future__ import annotations

import logging
from pathlib import Path

from PyQt6 import uic
from PyQt6.QtWidgets import QWidget

_LOG = logging.getLogger("tabs.system")


def _find_project_root(start: Path) -> Path:
    """
    Sucht nach dem Projekt-Root (Ordner, der 'resource' und 'src' enthält),
    ausgehend von einem Startpfad (typisch: diese Datei).

    Vorteil:
    - funktioniert auch, wenn sich die Datei-Struktur minimal ändert
    - bricht sauber ab, statt falsche Pfade zu liefern
    """
    for p in [start, *start.parents]:
        if (p / "resource").is_dir() and (p / "src").is_dir():
            return p
    raise RuntimeError(
        f"Projekt-Root nicht gefunden ab: {start}. "
        "Erwartet Ordnerstruktur mit 'resource/' und 'src/'."
    )


def _ui_path(project_root: Path, filename: str) -> Path:
    """
    UI-Dateien liegen in:
      resource/ui/tabs/system/<filename>
    """
    return project_root / "resource" / "ui" / "tabs" / "system" / filename


class SystemTab(QWidget):
    def __init__(self, *, ctx, parent=None) -> None:
        super().__init__(parent)
        self.ctx = ctx

        here = Path(__file__).resolve()
        project_root = _find_project_root(here.parent)

        ui_file = _ui_path(project_root, "system_tab.ui")

        if not ui_file.is_file():
            # Nicht "return" → das verschleiert Fehler und du wunderst dich später,
            # warum Widgets fehlen oder Signale nicht gehen.
            msg = f"SystemTab UI nicht gefunden: {ui_file}"
            _LOG.error(msg)
            raise RuntimeError(msg)

        try:
            # uic.loadUi akzeptiert auch str; Path ist ok, aber ich konvertiere bewusst:
            uic.loadUi(str(ui_file), self)
        except Exception as e:
            _LOG.exception("uic.loadUi failed for %s", ui_file)
            raise RuntimeError(f"uic.loadUi failed for {ui_file}: {e}") from e
