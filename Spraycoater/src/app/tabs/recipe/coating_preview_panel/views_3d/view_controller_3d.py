# -*- coding: utf-8 -*-
from __future__ import annotations

from typing import Optional, Callable, Tuple
import logging

_LOG = logging.getLogger("app.tabs.recipe.preview.views.controller3d")

Bounds = Tuple[float, float, float, float, float, float]


def _pad_bounds(b: Bounds, pad: float) -> Bounds:
    """Skaliert Bounds um ihren Mittelpunkt (pad>1.0 = mehr Luft)."""
    xmin, xmax, ymin, ymax, zmin, zmax = map(float, b)
    cx, cy, cz = (0.5 * (xmin + xmax), 0.5 * (ymin + ymax), 0.5 * (zmin + zmax))
    sx = max(xmax - xmin, 1e-6)
    sy = max(ymax - ymin, 1e-6)
    sz = max(zmax - zmin, 1e-6)
    k = float(pad)
    return (
        cx - 0.5 * sx * k, cx + 0.5 * sx * k,
        cy - 0.5 * sy * k, cy + 0.5 * sy * k,
        cz - 0.5 * sz * k, cz + 0.5 * sz * k,
    )


class ViewController3D:
    """
    Kamera-Controller für PyVista-QtInteractor.
    - Snap/Reset bevorzugt auf Substrat-Bounds (falls vorhanden), sonst Scene-Bounds.
    - Optionaler Extra-Zoom nur für Isometric-View.
    """

    def __init__(
        self,
        interactor_getter: Callable[[], object],
        render_callable: Callable[..., None],
        bounds_getter: Optional[Callable[[], Bounds]] = None,
        cam_pad: float = 1.2,
        *,
        substrate_bounds_getter: Optional[Callable[[], Optional[Bounds]]] = None,
        zoom_after_reset: float = 1.12,
        iso_extra_zoom: float = 1.30,
    ):
        self._get_ia = interactor_getter
        self._render_after = render_callable  # bewusst nicht genutzt (ia.render() reicht hier)
        self._get_bounds = bounds_getter
        self._get_sub_bounds = substrate_bounds_getter
        self._cam_pad = float(cam_pad)
        self._zoom = float(zoom_after_reset)
        self._iso_extra_zoom = float(iso_extra_zoom)

    # ------------------- Internals -------------------

    def _target_bounds(self) -> Optional[Bounds]:
        """Substrat-Bounds bevorzugen, sonst Gesamt-Bounds."""
        try:
            if self._get_sub_bounds:
                b = self._get_sub_bounds()
                if b:
                    return b
        except Exception:
            _LOG.exception("substrate_bounds_getter failed")

        try:
            return self._get_bounds() if self._get_bounds else None
        except Exception:
            _LOG.exception("bounds_getter failed")
            return None

    def _snap_on_target(self, *, extra_zoom: float = 1.0) -> None:
        """Reset/Center Kamera auf Target-Bounds + Zoom, dann rendern."""
        ia = self._get_ia()
        if ia is None:
            return
        try:
            if hasattr(ia, "reset_camera_clipping_range"):
                ia.reset_camera_clipping_range()

            b = self._target_bounds()
            if b is not None:
                try:
                    ia.reset_camera(bounds=_pad_bounds(b, self._cam_pad))
                except Exception:
                    ia.reset_camera()
            else:
                ia.reset_camera()

            if self._zoom and self._zoom != 1.0:
                try:
                    ia.camera.zoom(self._zoom)
                except Exception:
                    pass

            if extra_zoom and extra_zoom != 1.0:
                try:
                    ia.camera.zoom(extra_zoom)
                except Exception:
                    pass

            ia.render()
        except Exception:
            _LOG.exception("_snap_on_target failed")

    # ------------------- Views -------------------

    def view_isometric(self) -> None:
        ia = self._get_ia()
        if ia is None:
            return
        try:
            ia.view_isometric()
        except Exception:
            _LOG.exception("view_isometric failed")
        self._snap_on_target(extra_zoom=self._iso_extra_zoom)

    def view_top(self) -> None:
        ia = self._get_ia()
        if ia is None:
            return
        try:
            ia.view_xy()
        except Exception:
            _LOG.exception("view_top failed")
        self._snap_on_target()

    def view_front(self) -> None:
        ia = self._get_ia()
        if ia is None:
            return
        try:
            ia.view_yz()
        except Exception:
            _LOG.exception("view_front failed")
        self._snap_on_target()

    def view_left(self) -> None:
        ia = self._get_ia()
        if ia is None:
            return
        try:
            ia.view_xz()
        except Exception:
            _LOG.exception("view_left failed")
        self._snap_on_target()

    def view_right(self) -> None:
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
        self._snap_on_target()

    def view_back(self) -> None:
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
        self._snap_on_target()
