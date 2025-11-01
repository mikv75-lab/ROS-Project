# -*- coding: utf-8 -*-
from __future__ import annotations
import logging
from typing import Optional, Any
from PyQt6.QtWidgets import QVBoxLayout, QWidget

_LOG = logging.getLogger("app.tabs.recipe.preview.interactor")

try:
    from pyvistaqt import QtInteractor as _QtInteractor
except Exception:
    _QtInteractor = None


class InteractorHost:
    """Verantwortlich für das Finden/Anhängen/Absichern des QtInteractor."""
    def __init__(self, panel: QWidget, host_widget: QWidget):
        self._panel = panel
        self._host = host_widget
        self._ia: Optional[Any] = None

    @property
    def ia(self) -> Optional[Any]:
        return self._ia

    def ensure(self) -> bool:
        if self._ia is not None:
            return True
        ia = self._resolve_via_parents()
        if ia is not None:
            self._ia = ia
            return True
        return False

    def _resolve_via_parents(self) -> Optional[Any]:
        cand_attr_names = ("previewPlot", "plotter", "interactor", "plot")
        p = self._panel
        while p is not None:
            for name in cand_attr_names:
                if hasattr(p, name):
                    obj = getattr(p, name)
                    if obj is None:
                        continue
                    if all(hasattr(obj, m) for m in ("add_mesh", "view_xy", "render")):
                        if _QtInteractor is None or isinstance(obj, _QtInteractor) or hasattr(obj, "camera"):
                            return obj
            p = p.parent()
        return None

    def attach(self, interactor: Any) -> None:
        if interactor is None:
            raise ValueError("InteractorHost.attach: interactor is None")
        self._ia = interactor
        ly = self._host.layout()
        if ly is None:
            ly = QVBoxLayout(self._host)
            ly.setContentsMargins(0, 0, 0, 0)
            ly.setSpacing(0)
        self._ia.setParent(self._host)
        try:
            ly.addWidget(self._ia)
        except Exception:
            pass
        self._ia.setEnabled(True)
        self._ia.show()
        self._ia.update()
        try:
            self._ia.render()
        except Exception:
            pass
