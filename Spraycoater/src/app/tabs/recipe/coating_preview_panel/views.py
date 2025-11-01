# -*- coding: utf-8 -*-
from __future__ import annotations
import logging
from typing import Callable, Optional

_LOG = logging.getLogger("app.tabs.recipe.preview.views")


class ViewController:
    """Kamera-Views + Button-Wiring."""
    def __init__(self, interactor_getter: callable, render_callable: callable):
        self._get_ia = interactor_getter
        self._render_after = render_callable

    def view_isometric(self):
        ia = self._get_ia()
        if ia is None: return
        try: ia.view_isometric()
        except Exception: pass

    def view_top(self):
        ia = self._get_ia()
        if ia is None: return
        try: ia.view_xy()
        except Exception: pass

    def view_front(self):
        ia = self._get_ia()
        if ia is None: return
        try: ia.view_yz()
        except Exception: pass

    def view_left(self):
        ia = self._get_ia()
        if ia is None: return
        try: ia.view_xz()
        except Exception: pass

    def view_right(self):
        ia = self._get_ia()
        if ia is None: return
        try:
            ia.view_xz()
            try: ia.camera.azimuth(180)
            except Exception: pass
        except Exception:
            pass

    def view_back(self):
        ia = self._get_ia()
        if ia is None: return
        try:
            ia.view_yz()
            try: ia.camera.azimuth(180)
            except Exception: pass
        except Exception:
            pass

    def wire_buttons(self, *, btn_iso=None, btn_top=None, btn_front=None, btn_left=None, btn_right=None, btn_back=None):
        if btn_iso:   btn_iso.clicked.connect(lambda: self._call(self.view_isometric))
        if btn_top:   btn_top.clicked.connect(lambda: self._call(self.view_top))
        if btn_front: btn_front.clicked.connect(lambda: self._call(self.view_front))
        if btn_left:  btn_left.clicked.connect(lambda: self._call(self.view_left))
        if btn_right: btn_right.clicked.connect(lambda: self._call(self.view_right))
        if btn_back:  btn_back.clicked.connect(lambda: self._call(self.view_back))

    def _call(self, fn: Callable[[], None]):
        try: fn()
        except Exception: _LOG.exception("View call failed")
        try: self._render_after(reset_camera=False)
        except Exception: _LOG.exception("Render after view failed")
