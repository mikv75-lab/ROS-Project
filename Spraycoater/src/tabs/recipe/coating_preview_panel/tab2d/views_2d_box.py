# -*- coding: utf-8 -*-
# File: src/tabs/recipe/coating_preview_panel/tab2d/views_2d_box.py
from __future__ import annotations

from typing import Callable, Optional

from PyQt6.QtWidgets import (
    QWidget,
    QGroupBox,
    QHBoxLayout,
    QPushButton,
    QSizePolicy,
)


class Views2DBox(QGroupBox):
    """
    Strict 2D views box: plane buttons only.
    """

    def __init__(
        self,
        *,
        switch_2d: Callable[[str], None],
        refresh_callable: Callable[[], None],
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__("2D Views", parent)

        self._switch_2d = switch_2d
        self._refresh = refresh_callable

        lay = QHBoxLayout(self)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setSpacing(6)

        def mk(label: str, plane: str) -> QPushButton:
            b = QPushButton(label, self)
            b.clicked.connect(lambda *_: self._switch_2d(plane))
            return b

        lay.addWidget(mk("Top", "top"))
        lay.addWidget(mk("Front", "front"))
        lay.addWidget(mk("Back", "back"))
        lay.addWidget(mk("Left", "left"))
        lay.addWidget(mk("Right", "right"))

        lay.addStretch(1)

        sp = self.sizePolicy()
        sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp.setVerticalPolicy(QSizePolicy.Policy.Preferred)
        self.setSizePolicy(sp)
