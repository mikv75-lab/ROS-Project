# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Optional, Callable, Tuple
import logging

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QGroupBox, QWidget, QFormLayout, QHBoxLayout, QPushButton,
    QSizePolicy, QLabel
)

_LOG = logging.getLogger("app.tabs.recipe.preview.views")

# ---------------------------------------------------------------------------
# Gemeinsamer Kamera-Controller nur für die 3D-Ansicht (PyVista)
# ---------------------------------------------------------------------------
class ViewController3D:
    """Kamera-Views + Low-level Kamera-Calls für PyVista mit robustem Snap."""

    def __init__(
        self,
        interactor_getter: Callable[[], object],
        render_callable: Callable[..., None],
        bounds_getter: Optional[Callable[[], Tuple[float, float, float, float, float, float]]] = None,
        cam_pad: float = 1.6,
    ):
        self._get_ia = interactor_getter
        self._render_after = render_callable
        self._get_bounds = bounds_getter
        self._cam_pad = float(cam_pad)

    def _camera_snap(self, *, reset_cam: bool = False):
        ia = self._get_ia()
        if ia is None:
            return
        try:
            if hasattr(ia, "reset_camera_clipping_range"):
                ia.reset_camera_clipping_range()
            if reset_cam and self._get_bounds is not None:
                b = self._get_bounds()
                if b is not None:
                    try:
                        ia.reset_camera(bounds=b)
                    except Exception:
                        ia.reset_camera()
            ia.render()
        except Exception:
            _LOG.exception("_camera_snap failed")

    # --- Orientierungen ---
    def view_isometric(self):
        ia = self._get_ia()
        if ia is None:
            return
        try:
            ia.view_isometric()
        except Exception:
            _LOG.exception("view_isometric failed")
        self._camera_snap(reset_cam=False)

    def view_top(self):
        ia = self._get_ia()
        if ia is None:
            return
        try:
            ia.view_xy()
        except Exception:
            _LOG.exception("view_top failed")
        self._camera_snap(reset_cam=False)

    def view_front(self):
        ia = self._get_ia()
        if ia is None:
            return
        try:
            ia.view_yz()
        except Exception:
            _LOG.exception("view_front failed")
        self._camera_snap(reset_cam=False)

    def view_left(self):
        ia = self._get_ia()
        if ia is None:
            return
        try:
            ia.view_xz()
        except Exception:
            _LOG.exception("view_left failed")
        self._camera_snap(reset_cam=False)

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
        self._camera_snap(reset_cam=False)

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
        self._camera_snap(reset_cam=False)


def _set_policy(w: QWidget,
                *,
                h: QSizePolicy.Policy = QSizePolicy.Policy.Expanding,
                v: QSizePolicy.Policy = QSizePolicy.Policy.Preferred) -> None:
    sp = w.sizePolicy()
    sp.setHorizontalPolicy(h)
    sp.setVerticalPolicy(v)
    w.setSizePolicy(sp)


# ---------------------------------------------------------------------------
# 2D-View-Box (nur Matplotlib-Ebene schalten)
# ---------------------------------------------------------------------------
class Views2DBox(QGroupBox):
    """
    2D-Controls (Matplotlib): Top / Front / Back / Left / Right
    Ruft ausschließlich switch_2d(plane).
    """
    def __init__(self, *, switch_2d: Callable[[str], None], parent: Optional[QWidget] = None):
        super().__init__("2D View", parent)
        self._switch_2d = switch_2d

        form = QFormLayout(self)
        form.setContentsMargins(8, 8, 8, 8)
        form.setHorizontalSpacing(8)
        form.setVerticalSpacing(4)
        form.setLabelAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        form.setFormAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        form.setFieldGrowthPolicy(QFormLayout.FieldGrowthPolicy.ExpandingFieldsGrow)

        row = QWidget(self)
        lay = QHBoxLayout(row)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(6)

        self.btnTop   = QPushButton("Top", self)
        self.btnFront = QPushButton("Front", self)
        self.btnBack  = QPushButton("Back", self)
        self.btnLeft  = QPushButton("Left", self)
        self.btnRight = QPushButton("Right", self)

        for b in (self.btnTop, self.btnFront, self.btnBack, self.btnLeft, self.btnRight):
            b.setAutoDefault(False)
            lay.addWidget(b)

        form.addRow(QLabel("Plane", self), row)

        # Wiring
        self.btnTop.clicked.connect(  lambda: self._switch_2d("top"))
        self.btnFront.clicked.connect(lambda: self._switch_2d("front"))
        self.btnBack.clicked.connect( lambda: self._switch_2d("back"))
        self.btnLeft.clicked.connect( lambda: self._switch_2d("left"))
        self.btnRight.clicked.connect(lambda: self._switch_2d("right"))


# ---------------------------------------------------------------------------
# 3D-View-Box (PyVista-Kamera steuern)
# ---------------------------------------------------------------------------
class Views3DBox(QGroupBox):
    """
    3D-Controls (PyVista): Iso / Top / Front / Back / Left / Right
    Nutzt ViewController3D intern. Keine Panel-Logik.
    """
    def __init__(
        self,
        *,
        interactor_getter: Callable[[], object],
        render_callable: Callable[..., None],
        bounds_getter: Optional[Callable[[], Tuple[float, float, float, float, float, float]]] = None,
        cam_pad: float = 1.6,
        parent: Optional[QWidget] = None,
    ):
        super().__init__("3D View", parent)

        self.views = ViewController3D(
            interactor_getter=interactor_getter,
            render_callable=render_callable,
            bounds_getter=bounds_getter,
            cam_pad=cam_pad,
        )

        form = QFormLayout(self)
        form.setContentsMargins(8, 8, 8, 8)
        form.setHorizontalSpacing(8)
        form.setVerticalSpacing(4)
        form.setLabelAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        form.setFormAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        form.setFieldGrowthPolicy(QFormLayout.FieldGrowthPolicy.ExpandingFieldsGrow)

        row = QWidget(self)
        lay = QHBoxLayout(row)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(6)

        self.btnIso   = QPushButton("Iso", self)
        self.btnTop   = QPushButton("Top", self)
        self.btnFront = QPushButton("Front", self)
        self.btnBack  = QPushButton("Back", self)
        self.btnLeft  = QPushButton("Left", self)
        self.btnRight = QPushButton("Right", self)

        for b in (self.btnIso, self.btnTop, self.btnFront, self.btnBack, self.btnLeft, self.btnRight):
            b.setAutoDefault(False)
            lay.addWidget(b)

        form.addRow(QLabel("Camera", self), row)

        # Wiring
        self.btnIso.clicked.connect(  self.views.view_isometric)
        self.btnTop.clicked.connect(  self.views.view_top)
        self.btnFront.clicked.connect(self.views.view_front)
        self.btnBack.clicked.connect( self.views.view_back)
        self.btnLeft.clicked.connect( self.views.view_left)
        self.btnRight.clicked.connect(self.views.view_right)
