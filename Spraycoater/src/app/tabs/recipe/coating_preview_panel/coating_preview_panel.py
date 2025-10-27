# -*- coding: utf-8 -*-
from __future__ import annotations
import logging
from typing import Optional, Any, Iterable

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QSizePolicy

from pyvistaqt import QtInteractor

from .preview import PreviewEngine
from .mesh_utils import (
    load_active_mount_mesh,
    get_active_mount_scene_offset,
    get_mount_scene_offset_from_key,
    apply_transform,
    load_substrate_mesh_from_key,
    place_substrate_on_mount,
    load_mount_mesh_from_key,
)

_LOG = logging.getLogger("app.tabs.recipe.coating_preview_panel")


class CoatingPreviewPanel(QWidget):
    """
    Minimaler 3D-Preview (PyVistaQt) für Mount + Substrat.
    Nutzt ausschließlich Felder des Recipe-Objekts:
      - recipe.substrate ODER recipe.substrates[0]
      - recipe.substrate_mount (optional)
    API: render_from_model(recipe, sides)
    """
    readyChanged = pyqtSignal(bool, str)

    def __init__(self, *, ctx, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx

        # Layout + QtInteractor
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)

        self._pv = QtInteractor(self)  # QtInteractor ist selbst der Plotter
        self._pv.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        root.addWidget(self._pv)

        # PreviewEngine arbeitet direkt auf dem QtInteractor
        self._engine: PreviewEngine = PreviewEngine(self._pv)

    # ---------- Public API ----------
    def render_from_model(self, model: Any, sides: Iterable[str]) -> None:
        """
        Rendert NUR Mount + Substrat anhand des Recipe-Objekts.
        'sides' wird aktuell ignoriert.
        """
        # 1) Mount-Key optional aus dem Recipe
        mount_key = self._extract(model, "substrate_mount")

        # 2) Substrat-Key required
        substrate_key = self._resolve_substrate_from_recipe(model)
        if not substrate_key:
            msg = "Recipe enthält kein 'substrate' und keine nichtleere 'substrates'-Liste."
            _LOG.error(msg)
            self.readyChanged.emit(False, msg)
            return

        # 3) Optional aktiven Mount im Context setzen (wenn API vorhanden)
        if mount_key and hasattr(self.ctx, "set_active_mount") and callable(getattr(self.ctx, "set_active_mount")):
            try:
                self.ctx.set_active_mount(mount_key)
            except Exception as e:
                _LOG.warning("set_active_mount('%s') schlug fehl: %s", mount_key, e)

        # 4) Rendern
        self.render_mount_and_substrate(substrate_key=substrate_key, mount_key=mount_key)

    def render_mount_and_substrate(
        self,
        *,
        substrate_key: str,
        mount_key: Optional[str] = None,
        mount_color: str = "lightgray",
        substrate_color: str = "orange",
        substrate_opacity: float = 0.85,
        show_edges: bool = False,
    ) -> None:
        """Mount + Substrat synchron in der Szene darstellen."""
        try:
            # Szene säubern
            self._engine.clear_scene()

            # --- Mount laden + platzieren ---
            if mount_key:
                mount_mesh = load_mount_mesh_from_key(self.ctx, mount_key)
                xyz_mm, rpy_deg = get_mount_scene_offset_from_key(self.ctx, mount_key)
            else:
                mount_mesh = load_active_mount_mesh(self.ctx)
                xyz_mm, rpy_deg = get_active_mount_scene_offset(self.ctx)

            mount_scene = apply_transform(mount_mesh, translate_mm=xyz_mm, rpy_deg=rpy_deg)
            self._engine.add_mesh(
                mount_scene,
                color=mount_color,
                opacity=1.0,
                show_edges=show_edges,
            )

            # --- Substrat laden + auf Mount platzieren ---
            sub_mesh = load_substrate_mesh_from_key(self.ctx, substrate_key)
            sub_scene = place_substrate_on_mount(self.ctx, sub_mesh)
            self._engine.add_mesh(
                sub_scene,
                color=substrate_color,
                opacity=substrate_opacity,
                show_edges=show_edges,
            )

            # Kamera
            self._engine.view_iso()

            self.readyChanged.emit(True, "ok")

        except Exception as e:
            _LOG.error("Rendern fehlgeschlagen: %s", e, exc_info=True)
            self.readyChanged.emit(False, str(e))

    # ---------- Helpers ----------
    def _resolve_substrate_from_recipe(self, model: Any) -> Optional[str]:
        """Nur Recipe-Felder: substrate oder erstes aus substrates."""
        # 1) Recipe.substrate
        sub = self._extract(model, "substrate")
        if isinstance(sub, str) and sub.strip():
            return sub.strip()

        # 2) Recipe.substrates[0]
        subs = self._extract(model, "substrates")
        if isinstance(subs, (list, tuple)) and subs:
            first = subs[0]
            if isinstance(first, str) and first.strip():
                return first.strip()

        return None

    @staticmethod
    def _extract(model: Any, key: str):
        """Unterstützt sowohl dataclass-Objekt als auch dict."""
        if hasattr(model, key):
            return getattr(model, key)
        if isinstance(model, dict):
            return model.get(key)
        return None
