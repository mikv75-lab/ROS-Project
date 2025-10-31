# Spraycoater/src/app/tabs/recipe/coating_preview_panel/coating_preview_panel.py
from __future__ import annotations
import os, logging
from typing import Optional, Any

from PyQt6 import uic
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QPushButton, QFrame, QSizePolicy

import pyvista as pv
from pyvistaqt import QtInteractor

_LOG = logging.getLogger("app.tabs.recipe.preview")

def _project_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))
    return os.path.abspath(os.path.join(here, "..", "..", "..", "..", ".."))

def _ui_path(filename: str) -> str:
    return os.path.join(_project_root(), "resource", "ui", "tabs", "recipe", filename)

class CoatingPreviewPanel(QWidget):
    """Preview-Controls (oben) + stabil eingebetteter QtInteractor (unten)."""

    def __init__(self, *, ctx, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        uic.loadUi(_ui_path("coating_preview_panel.ui"), self)

        vroot = self.findChild(QVBoxLayout, "vRoot")
        if vroot is None:
            raise RuntimeError("coating_preview_panel.ui benötigt ein QVBoxLayout namens 'vRoot'.")

        # --- robuste PyVista Defaults ---
        pv.OFF_SCREEN = False
        pv.global_theme.depth_peeling.enabled = False
        pv.global_theme.smooth_shading = False

        # --- eigener Container als Host (wichtig!) ---
        self._host = QFrame(self)
        self._host.setFrameShape(QFrame.Shape.NoFrame)
        self._host.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        host_layout = QVBoxLayout(self._host)
        host_layout.setContentsMargins(0, 0, 0, 0)
        host_layout.setSpacing(0)
        vroot.addWidget(self._host, 1)

        # --- Interactor erzeugen und *dessen* QWidget einsetzen ---
        self._plotter = QtInteractor(self._host)           # Parent = Host-Frame
        host_layout.addWidget(self._plotter.interactor)    # <- wichtig: .interactor verwenden!

        # eigene Actor-Handles
        self._act_mount: Optional[Any] = None
        self._act_sub:   Optional[Any] = None

        # Kamera-Buttons verbinden
        self._wire_camera_buttons()

        # --- Deferred Init: erst nach 1 Tick Szene aufbauen ---
        self._inited = False
        QTimer.singleShot(0, self._deferred_init)

    # öffentlich: falls du von außen etwas brauchst
    def plotter(self) -> QtInteractor:
        return self._plotter

    def _deferred_init(self):
        if self._inited:
            return
        self._inited = True

        # Renderer sicher initialisieren
        try:
            _ = self._plotter.renderer
        except Exception:
            pass

        # kleine Test-Geom. (funktioniert auch im Tab):
        try:
            self._plotter.add_mesh(pv.Sphere(radius=5.0), opacity=0.9, lighting=False, show_edges=True)
            self._plotter.show_axes()
            self.view_iso()
        except Exception as e:
            _LOG.error("deferred init add_mesh failed: %s", e, exc_info=True)

    # --- Render-API wie gehabt (vereinfacht) ---
    def clear_scene(self) -> None:
        for attr in ("_act_mount", "_act_sub"):
            act = getattr(self, attr, None)
            if act is not None:
                try:
                    self._plotter.remove_actor(act)
                except Exception:
                    pass
                setattr(self, attr, None)
        self._plotter.render()

    def set_mount_mesh(self, mesh, **kwargs) -> None:
        self._act_mount = self._replace_actor(self._act_mount, mesh, color="lightgray", opacity=0.3, lighting=False, **kwargs)

    def set_substrate_mesh(self, mesh, **kwargs) -> None:
        self._act_sub = self._replace_actor(self._act_sub, mesh, color="#3498db", opacity=0.95, lighting=False, **kwargs)

    def render_from_model(self, model: object) -> None:
        """
        Leichtgewichtige Vorarbeit (Keys/Sides) jetzt — aber KEINE VTK-Calls.
        Das eigentliche Rendern passiert per QTimer im nächsten Tick.
        """
        # Sides extrahieren (optional im Modell)
        pbs = getattr(model, "paths_by_side", None)
        if pbs is None and isinstance(model, dict):
            pbs = model.get("paths_by_side")
        sides = list(pbs.keys()) if isinstance(pbs, dict) else []

        # Pflichtfelder robust ziehen
        mount_key = self._get_required_str(
            model, "substrate_mount", "Recipe benötigt ein gültiges 'substrate_mount'."
        )
        substrate_key = self._get_required_str(
            model, "substrate", "Recipe benötigt ein gültiges 'substrate'."
        )

        _LOG.info("Preview (queued): mount=%s, substrate=%s, sides=%s",
                mount_key, substrate_key, sides)

        # WICHTIG: Alles schwere/VTK im nächsten Event-Loop-Tick ausführen.
        # Capture per Default-Args, damit spätere Mutationen nichts beeinflussen.
        QTimer.singleShot(
            0,
            lambda mk=mount_key, sk=substrate_key, sd=tuple(sides): self._render_from_model_impl(mk, sk, sd)
        )


    def _render_from_model_impl(self, mount_key: str, substrate_key: str, sides: tuple[str, ...]) -> None:
        """
        Hier passieren alle VTK-Operationen. Wird von render_from_model() via QTimer aufgerufen.
        """
        from .mesh_utils import (
            load_mount_mesh_from_key,
            load_substrate_mesh_from_key,
            place_substrate_on_mount,
        )

        # Nur dynamische Szene leeren (behält Boden/Cage etc.)
        self.clear_scene()

        # Mount
        try:
            mount_mesh = load_mount_mesh_from_key(self.ctx, mount_key)
            self.set_mount_mesh(mount_mesh)
        except Exception as e:
            _LOG.error("Mount-Mesh Fehler: %s", e, exc_info=True)

        # Substrat
        try:
            sub_mesh = load_substrate_mesh_from_key(self.ctx, substrate_key)
            sub_mesh = place_substrate_on_mount(self.ctx, sub_mesh, mount_key=mount_key)
            self.set_substrate_mesh(sub_mesh)
        except Exception as e:
            _LOG.error("Substrat-Mesh Fehler: %s", e, exc_info=True)

        # Kamera zum Schluss
        self.view_iso()

    def _replace_actor(self, old_handle, mesh, **kwargs):
        try:
            fn = getattr(mesh, "is_all_triangles", None)
            if callable(fn) and not fn():
                mesh = mesh.triangulate()
        except Exception:
            pass
        if old_handle is not None:
            try:
                self._plotter.remove_actor(old_handle)
            except Exception:
                pass
        ret = self._plotter.add_mesh(mesh, reset_camera=False, **kwargs)
        actor = ret[0] if isinstance(ret, tuple) else ret
        self._plotter.render()
        return actor

    # --- Kamera ---
    def _wire_camera_buttons(self):
        mapping = [
            ("btnCamIso",   self.view_iso),
            ("btnCamTop",   self.view_top),
            ("btnCamFront", self.view_front),
            ("btnCamLeft",  self.view_left),
            ("btnCamRight", self.view_right),
            ("btnCamBack",  self.view_back),
        ]
        for name, slot in mapping:
            btn = self.findChild(QPushButton, name)
            if btn:
                btn.clicked.connect(slot)

    def view_iso(self):   self._plotter.view_isometric(); self._plotter.reset_camera(); self._plotter.render()
    def view_top(self):   self._plotter.view_xy();        self._plotter.reset_camera(); self._plotter.render()
    def view_front(self): self._plotter.view_yz();        self._plotter.reset_camera(); self._plotter.render()
    def view_left(self):  self._plotter.view_xz();        self._plotter.reset_camera(); self._plotter.render()
    def view_right(self):
        self._plotter.view_xz()
        try: self._plotter.camera.azimuth(180)
        except Exception: pass
        self._plotter.reset_camera(); self._plotter.render()
    def view_back(self):
        self._plotter.view_yz()
        try: self._plotter.camera.azimuth(180)
        except Exception: pass
        self._plotter.reset_camera(); self._plotter.render()
