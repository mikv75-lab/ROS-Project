# -*- coding: utf-8 -*-
from __future__ import annotations
import logging
from typing import Optional

from PyQt6.QtWidgets import QWidget, QVBoxLayout

import pyvista as pv

_LOG = logging.getLogger("app.tabs.recipe.preview.interactor")

try:
    from pyvistaqt import QtInteractor  # type: ignore
except Exception as e:
    QtInteractor = None  # type: ignore
    _LOG.error("pyvistaqt.QtInteractor nicht verfügbar: %s", e)


class InteractorHost:
    """
    Sorgt robust dafür, dass ein QtInteractor im gegebenen Qt-Container montiert ist.
    - Erstellt bei Bedarf einen neuen QtInteractor.
    - Erzwingt ein QVBoxLayout am container_widget.
    - Entfernt alte Kinderwidgets im Container, bevor der Interactor hinzugefügt wird.
    - Zeigt Interactor/Container/Parent an und rendert.
    """

    def __init__(self, parent_widget: QWidget, container_widget: QWidget):
        self.parent_widget = parent_widget
        self.container_widget = container_widget
        self.ia: Optional[QtInteractor] = None  # type: ignore[name-defined]
        self._mounted = False

    # ------- interne Helfer -------------------------------------------------
    def _ensure_container_layout(self) -> QVBoxLayout:
        lay = self.container_widget.layout()
        if not isinstance(lay, QVBoxLayout):
            lay = QVBoxLayout(self.container_widget)
            lay.setContentsMargins(0, 0, 0, 0)
            lay.setSpacing(0)
            self.container_widget.setLayout(lay)
        return lay  # type: ignore[return-value]

    def _clear_layout_items(self, lay: QVBoxLayout) -> None:
        try:
            while lay.count():
                item = lay.takeAt(0)
                w = item.widget()
                if w is not None:
                    w.setParent(None)
                    w.deleteLater()
        except Exception:
            _LOG.exception("Layout-Räumung fehlgeschlagen")

    # ------- öffentlich -----------------------------------------------------
    def ensure(self) -> bool:
        """Stellt sicher, dass self.ia existiert und im container_widget montiert ist."""
        if QtInteractor is None:
            _LOG.error("QtInteractor fehlt – ist pyvistaqt installiert?")
            return False

        try:
            lay = self._ensure_container_layout()

            # Interactor einmalig erzeugen
            if self.ia is None:
                self.ia = QtInteractor(self.container_widget)
                # Optionale, sanfte Defaults
                try:
                    pv.global_theme.window_size = None
                except Exception:
                    pass

            # Falls noch nicht montiert: Container leeren und Interactor hinzufügen
            if not self._mounted:
                self._clear_layout_items(lay)
                lay.addWidget(self.ia)
                self._mounted = True

            # Sichtbar machen
            try:
                self.ia.show()
                self.container_widget.show()
                self.parent_widget.show()
            except Exception:
                _LOG.exception("show() auf Interactor/Container/Parent fehlgeschlagen")

            # Minimalrender / Clipping
            try:
                if hasattr(self.ia, "reset_camera_clipping_range"):
                    self.ia.reset_camera_clipping_range()
                self.ia.render()
            except Exception:
                _LOG.exception("Erstrendern fehlgeschlagen")

            return True
        except Exception:
            _LOG.exception("InteractorHost.ensure() failed")
            return False

    def detach(self) -> None:
        """Interactor aus dem Container lösen und entsorgen (optional nutzbar)."""
        if self.ia is None:
            return
        try:
            lay = self.container_widget.layout()
            if lay is not None:
                lay.removeWidget(self.ia)
            self.ia.setParent(None)
            self.ia.deleteLater()
        except Exception:
            _LOG.exception("Interactor detach/delete failed")
        finally:
            self.ia = None
            self._mounted = False
