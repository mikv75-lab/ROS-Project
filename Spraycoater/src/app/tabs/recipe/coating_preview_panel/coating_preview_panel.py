# -*- coding: utf-8 -*-
from __future__ import annotations
import logging
from typing import Optional, Any, Iterable

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QSizePolicy
from pyvistaqt import QtInteractor

from .preview import PreviewEngine
from .mesh_utils import (
    get_mount_scene_offset_from_key,
    apply_transform,
    load_substrate_mesh_from_key,
    place_substrate_on_mount,
    load_mount_mesh_from_key,
)

_LOG = logging.getLogger("app.tabs.recipe.coating_preview_panel")


class CoatingPreviewPanel(QWidget):
    """
    3D-Preview (PyVistaQt) für Mount + genau EIN Substrat.
    Erwartet im Recipe:
      - recipe.substrate (str, erforderlich)
      - recipe.substrate_mount (str, erforderlich)
    """
    readyChanged = pyqtSignal(bool, str)

    def __init__(self, *, ctx, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx

        # Layout + Plotter
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)

        self._pv = QtInteractor(self)
        self._pv.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        root.addWidget(self._pv)

        self._engine: PreviewEngine = PreviewEngine(self._pv)

        # Startzustand
        try:
            self._engine.clear_scene()
            self._engine.view_iso()
        except Exception:
            pass

    # ---------- Public API ----------
    def render_from_model(self, model: Any, sides: Iterable[str]) -> None:
        """Rendert Mount + Substrat aus dem Recipe (sides wird ignoriert)."""
        mount_key = self._extract(model, "substrate_mount")
        if not isinstance(mount_key, str) or not mount_key.strip():
            msg = "Recipe benötigt ein gültiges 'substrate_mount'."
            _LOG.error(msg); self.readyChanged.emit(False, msg); return
        mount_key = mount_key.strip()

        substrate_key = self._extract(model, "substrate")
        if not isinstance(substrate_key, str) or not substrate_key.strip():
            msg = "Recipe benötigt ein gültiges 'substrate'."
            _LOG.error(msg); self.readyChanged.emit(False, msg); return
        substrate_key = substrate_key.strip()

        self.render_mount_and_substrate(substrate_key=substrate_key, mount_key=mount_key)

    def render_mount_and_substrate(
        self,
        *,
        substrate_key: str,
        mount_key: str,
        mount_color: str = "lightgray",
        substrate_color: str = "orange",
        substrate_opacity: float = 0.85,
        show_edges: bool = False,
    ) -> None:
        """Mount + Substrat synchron darstellen (Offset strikt aus substrate_mounts.yaml)."""
        try:
            self._engine.clear_scene()

            # Mount
            mount_mesh = load_mount_mesh_from_key(self.ctx, mount_key)
            xyz_mm, rpy_deg = get_mount_scene_offset_from_key(self.ctx, mount_key)
            mount_scene = apply_transform(mount_mesh, translate_mm=xyz_mm, rpy_deg=rpy_deg)
            self._engine.add_mesh(mount_scene, color=mount_color, opacity=1.0, show_edges=show_edges)

            # Substrat (Platzierung relativ zu mount_key)
            sub_mesh = load_substrate_mesh_from_key(self.ctx, substrate_key)
            sub_scene = place_substrate_on_mount(self.ctx, sub_mesh, mount_key=mount_key)
            self._engine.add_mesh(sub_scene, color=substrate_color, opacity=substrate_opacity, show_edges=show_edges)

            self._engine.view_iso()
            self.readyChanged.emit(True, "ok")

        except Exception as e:
            _LOG.error("Rendern fehlgeschlagen: %s", e, exc_info=True)
            self._engine.view_iso()
            self.readyChanged.emit(False, str(e))

    # ---------- Helper ----------
    @staticmethod
    def _extract(model: Any, key: str):
        if hasattr(model, key):
            return getattr(model, key)
        if isinstance(model, dict):
            return model.get(key)
        return None
