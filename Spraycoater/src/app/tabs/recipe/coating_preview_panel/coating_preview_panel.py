# Spraycoater/src/app/tabs/recipe/coating_preview_panel/coating_preview_panel.py
from __future__ import annotations
import os
import logging
from typing import Optional, Any

from PyQt6 import uic
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QPushButton

import pyvista as pv
from pyvistaqt import QtInteractor

_LOG = logging.getLogger("app.tabs.recipe.preview")


def _project_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))
    return os.path.abspath(os.path.join(here, "..", "..", "..", "..", ".."))


def _ui_path(filename: str) -> str:
    return os.path.join(_project_root(), "resource", "ui", "tabs", "recipe", filename)


class CoatingPreviewPanel(QWidget):
    """
    Preview-Controls (oben, via .ui) + eingebetteter QtInteractor (unten).
    Hält GENAU ZWEI Actor-Handles: _act_mount, _act_sub.
    Die gesamte Render-Logik (clear + set + render_from_model) lebt hier.
    """

    def __init__(self, *, ctx, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx

        # --- UI laden ---
        uic.loadUi(_ui_path("coating_preview_panel.ui"), self)

        # vRoot-Layout holen (nicht neu setzen!)
        vroot = self.findChild(QVBoxLayout, "vRoot")
        if vroot is None:
            raise RuntimeError("coating_preview_panel.ui benötigt ein QVBoxLayout namens 'vRoot'.")

        # --- Plotter einhängen ---
        pv.OFF_SCREEN = False
        pv.global_theme.depth_peeling.enabled = False
        pv.global_theme.smooth_shading = False

        self._p: QtInteractor = QtInteractor(self)  # Parent = Panel
        vroot.addWidget(self._p, 1)

        # exakt zwei Actor-Referenzen
        self._act_mount: Optional[Any] = None
        self._act_sub:   Optional[Any] = None

        # Startszene minimal initialisieren
        try:
            _ = self._p.renderer  # Renderer anlegen
            self._p.show_axes()
            self._p.view_isometric()
            self._p.reset_camera()
            self._p.render()
        except Exception:
            pass

        self._wire_camera_buttons()

    # -------------------------------------------------------------------------
    # Öffentliche API
    # -------------------------------------------------------------------------
    def clear_scene(self) -> None:
        """Nur unsere zwei Actors entfernen."""
        for attr in ("_act_mount", "_act_sub"):
            act = getattr(self, attr, None)
            if act is not None:
                try:
                    self._p.remove_actor(act)
                except Exception:
                    pass
                setattr(self, attr, None)
        try:
            self._p.render()
        except Exception:
            pass

    def set_mount_mesh(self, mesh, **kwargs) -> None:
        """Mount ersetzen/setzen (führt ggf. Triangulation aus)."""
        defaults = dict(
            color="lightgray",
            opacity=0.30,
            show_edges=False,
            smooth_shading=False,
            lighting=False,
            reset_camera=False,
            copy_mesh=True,
        )
        defaults.update(kwargs)
        self._act_mount = self._replace_actor(self._act_mount, mesh, **defaults)

    def set_substrate_mesh(self, mesh, **kwargs) -> None:
        """Substrate ersetzen/setzen (führt ggf. Triangulation aus)."""
        defaults = dict(
            color="#3498db",
            opacity=0.95,
            show_edges=False,
            smooth_shading=False,
            lighting=False,
            reset_camera=False,
            copy_mesh=True,
        )
        defaults.update(kwargs)
        self._act_sub = self._replace_actor(self._act_sub, mesh, **defaults)

    def render_from_model(self, model: object, sides: list[str] | None = None) -> None:
        """
        Rendert Mount + Substrat basierend auf dem übergebenen Model.
        'sides' ist optional; wenn nicht gegeben, wird es aus dem Model ermittelt.
        """
        # --- sides ggf. aus dem Model bestimmen ---
        if sides is None:
            pbs = getattr(model, "paths_by_side", None) or (
                model.get("paths_by_side") if isinstance(model, dict) else None
            )
            sides = list(pbs.keys()) if isinstance(pbs, dict) else []

        mount_key = self._get_required_str(model, "substrate_mount", "Recipe benötigt ein gültiges 'substrate_mount'.")
        substrate_key = self._get_required_str(model, "substrate", "Recipe benötigt ein gültiges 'substrate'.")
        _LOG.info("Preview: mount=%s, substrate=%s, sides=%s", mount_key, substrate_key, sides)

        from .mesh_utils import (
            load_mount_mesh_from_key,
            load_substrate_mesh_from_key,
            place_substrate_on_mount,
        )

        self.clear_scene()

        try:
            mount_mesh = load_mount_mesh_from_key(self.ctx, mount_key)
            self.set_mount_mesh(mount_mesh)
        except Exception as e:
            _LOG.error("Mount-Mesh Fehler: %s", e, exc_info=True)

        try:
            sub_mesh = load_substrate_mesh_from_key(self.ctx, substrate_key)
            sub_mesh = place_substrate_on_mount(self.ctx, sub_mesh, mount_key=mount_key)
            self.set_substrate_mesh(sub_mesh)
        except Exception as e:
            _LOG.error("Substrat-Mesh Fehler: %s", e, exc_info=True)

        self.view_iso()

    # -------------------------------------------------------------------------
    # Internes
    # -------------------------------------------------------------------------
    def _replace_actor(self, old_handle, mesh, **kwargs):
        # robust: falls Mesh nicht trianguliert ist
        try:
            is_all_tris = getattr(mesh, "is_all_triangles", None)
            if callable(is_all_tris) and not is_all_tris():
                mesh = mesh.triangulate()
        except Exception:
            pass

        if old_handle is not None:
            try:
                self._p.remove_actor(old_handle)
            except Exception:
                pass

        ret = self._p.add_mesh(mesh, **kwargs)
        new_handle = ret[0] if isinstance(ret, tuple) else ret
        try:
            self._p.render()
        except Exception:
            pass
        return new_handle

    @staticmethod
    def _get_required_str(model: object, key: str, err: str) -> str:
        val = getattr(model, key, None) if hasattr(model, key) else (model.get(key) if isinstance(model, dict) else None)
        if not isinstance(val, str) or not val.strip():
            raise ValueError(err)
        return val.strip()

    # -------------------------------------------------------------------------
    # Kamera-Buttons (optional)
    # -------------------------------------------------------------------------
    def _wire_camera_buttons(self) -> None:
        mapping = [
            ("btnCamIso",   self.view_iso),
            ("btnCamTop",   self.view_top),
            ("btnCamFront", self.view_front),
            ("btnCamLeft",  self.view_left),
            ("btnCamRight", self.view_right),
            ("btnCamBack",  self.view_back),
        ]
        for obj_name, handler in mapping:
            btn = self.findChild(QPushButton, obj_name)
            if btn is not None:
                btn.clicked.connect(handler)

    # -------------------------------------------------------------------------
    # Kamera
    # -------------------------------------------------------------------------
    def view_iso(self):   self._p.view_isometric(); self._p.reset_camera(); self._p.render()
    def view_top(self):   self._p.view_xy();        self._p.reset_camera(); self._p.render()
    def view_front(self): self._p.view_yz();        self._p.reset_camera(); self._p.render()
    def view_left(self):  self._p.view_xz();        self._p.reset_camera(); self._p.render()

    def view_right(self):
        self._p.view_xz()
        try:
            self._p.camera.azimuth(180)
        except Exception:
            pass
        self._p.reset_camera()
        self._p.render()

    def view_back(self):
        self._p.view_yz()
        try:
            self._p.camera.azimuth(180)
        except Exception:
            pass
        self._p.reset_camera()
        self._p.render()
