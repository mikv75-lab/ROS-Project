# -*- coding: utf-8 -*-
from __future__ import annotations

from typing import Optional, Callable, Tuple, Any

from PyQt6.QtWidgets import QWidget, QGroupBox, QHBoxLayout, QPushButton, QSizePolicy

from .views_3d.view_controller_3d import ViewController3D

Bounds = Tuple[float, float, float, float, float, float]


class Views3DBox(QGroupBox):
    def __init__(
        self,
        *,
        interactor_getter: Callable[[], Any],
        render_callable: Callable[..., None],
        bounds_getter: Callable[[], Bounds],
        substrate_bounds_getter: Optional[Callable[[], Optional[Bounds]]] = None,
        cam_pad: float = 1.2,
        iso_extra_zoom: float = 1.30,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__("3D Views", parent)

        self._vc = ViewController3D(
            interactor_getter=interactor_getter,
            render_callable=render_callable,
            bounds_getter=bounds_getter,
            substrate_bounds_getter=substrate_bounds_getter,
            cam_pad=cam_pad,
            iso_extra_zoom=iso_extra_zoom,
        )

        lay = QHBoxLayout(self)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setSpacing(6)

        def mk(text: str, fn):
            b = QPushButton(text, self)
            b.clicked.connect(fn)
            b.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
            lay.addWidget(b)

        mk("Iso", self._vc.view_isometric)
        mk("Top", self._vc.view_top)
        mk("Front", self._vc.view_front)
        mk("Back", self._vc.view_back)
        mk("Left", self._vc.view_left)
        mk("Right", self._vc.view_right)

        sp = self.sizePolicy()
        sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp.setVerticalPolicy(QSizePolicy.Policy.Preferred)
        self.setSizePolicy(sp)
