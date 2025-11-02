# -*- coding: utf-8 -*-
from __future__ import annotations
import logging
from typing import Callable, Optional

_LOG = logging.getLogger("app.tabs.recipe.preview.views")


class ViewController:
    """Kamera-Views + Button-Wiring (3D & 2D)."""

    def __init__(self, interactor_getter: Callable[[], object], render_callable: Callable[..., None]):
        self._get_ia = interactor_getter
        self._render_after = render_callable

    # ---------- Low-level 3D view ops ----------
    def view_isometric(self):
        ia = self._get_ia()
        if ia is None:
            return
        try:
            ia.view_isometric()
        except Exception:
            _LOG.exception("view_isometric failed")

    def view_top(self):
        ia = self._get_ia()
        if ia is None:
            return
        try:
            ia.view_xy()
        except Exception:
            _LOG.exception("view_top failed")

    def view_front(self):
        ia = self._get_ia()
        if ia is None:
            return
        try:
            ia.view_yz()
        except Exception:
            _LOG.exception("view_front failed")

    def view_left(self):
        ia = self._get_ia()
        if ia is None:
            return
        try:
            ia.view_xz()
        except Exception:
            _LOG.exception("view_left failed")

    def view_right(self):
        ia = self._get_ia()
        if ia is None:
            return
        try:
            ia.view_xz()
            try:
                ia.camera.azimuth(180)
            except Exception:
                pass
        except Exception:
            _LOG.exception("view_right failed")

    def view_back(self):
        ia = self._get_ia()
        if ia is None:
            return
        try:
            ia.view_yz()
            try:
                ia.camera.azimuth(180)
            except Exception:
                pass
        except Exception:
            _LOG.exception("view_back failed")

    # ---------- Wiring helpers ----------
    def _call3d(self, fn: Callable[[], None], on_3d: Optional[Callable[[], None]] = None):
        try:
            if on_3d:
                on_3d()
        except Exception:
            _LOG.exception("on_3d callback failed")
        try:
            fn()
        except Exception:
            _LOG.exception("3D view call failed")
        try:
            self._render_after(reset_camera=False)
        except Exception:
            _LOG.exception("Render after 3D view failed")

    def wire_buttons_3d(
        self,
        *,
        btn_iso=None,
        btn_top=None,
        btn_front=None,
        btn_back=None,
        btn_left=None,
        btn_right=None,
        on_3d: Optional[Callable[[], None]] = None,
    ):
        if btn_iso:   btn_iso.clicked.connect(lambda: self._call3d(self.view_isometric, on_3d))
        if btn_top:   btn_top.clicked.connect(lambda: self._call3d(self.view_top, on_3d))
        if btn_front: btn_front.clicked.connect(lambda: self._call3d(self.view_front, on_3d))
        if btn_back:  btn_back.clicked.connect(lambda: self._call3d(self.view_back, on_3d))
        if btn_left:  btn_left.clicked.connect(lambda: self._call3d(self.view_left, on_3d))
        if btn_right: btn_right.clicked.connect(lambda: self._call3d(self.view_right, on_3d))

    # ---------- 2D switch wiring ----------
    def _call2d(self, plane: str, switch: Callable[[str], None]):
        try:
            switch(plane)
        except Exception:
            _LOG.exception("2D switch failed: %s", plane)

    def wire_buttons_2d(
        self,
        *,
        btn_top=None,
        btn_front=None,
        btn_left=None,
        btn_right=None,
        btn_back=None,
        switcher: Callable[[str], None] | None = None,
        on_change: Callable[[str], None] | None = None,
    ):
        # Kompatibilität: akzeptiere switcher oder on_change
        switch = switcher or on_change
        if switch is None:
            raise TypeError("wire_buttons_2d() erwartet 'switcher' oder 'on_change' Callback")
        if btn_top:   btn_top.clicked.connect(  lambda: self._call2d("top",   switch))
        if btn_front: btn_front.clicked.connect(lambda: self._call2d("front", switch))
        if btn_left:  btn_left.clicked.connect( lambda: self._call2d("left",  switch))
        if btn_right: btn_right.clicked.connect(lambda: self._call2d("right", switch))
        if btn_back:  btn_back.clicked.connect( lambda: self._call2d("back",  switch))
