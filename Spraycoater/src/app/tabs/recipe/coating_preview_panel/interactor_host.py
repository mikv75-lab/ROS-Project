# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Optional
from PyQt6.QtCore import QObject, QEvent
from PyQt6.QtWidgets import QWidget

try:
    from pyvistaqt import QtInteractor  # type: ignore
except Exception:  # bewusst kurz – Fehler darf nach oben gehen, wenn lib fehlt
    QtInteractor = None  # type: ignore

class InteractorHost(QObject):
    """Mountet einen QtInteractor OHNE Layout und hält ihn auf volle Größe."""
    def __init__(self, parent_widget: QWidget, container_widget: QWidget):
        super().__init__(parent_widget)
        self.parent_widget = parent_widget
        self.container_widget = container_widget
        self.ia: Optional[QtInteractor] = None  # type: ignore[name-defined]

    def ensure(self) -> bool:
        if self.ia is None:
            self.ia = QtInteractor(self.container_widget)  # type: ignore[call-arg]
            self.ia.setObjectName("pyvista_interactor")
            self.ia.setGeometry(self.container_widget.rect())
            self.container_widget.installEventFilter(self)  # resize mitführen
        self.container_widget.show(); self.parent_widget.show()
        self.ia.show(); self.ia.render()  # type: ignore[union-attr]
        return True

    def eventFilter(self, obj, ev):
        if obj is self.container_widget and ev.type() == QEvent.Resize and self.ia:
            self.ia.setGeometry(self.container_widget.rect())
        return super().eventFilter(obj, ev)

    def detach(self) -> None:
        if self.ia:
            self.container_widget.removeEventFilter(self)
            self.ia.setParent(None)
            self.ia.deleteLater()
            self.ia = None
