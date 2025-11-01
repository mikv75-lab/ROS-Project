# -*- coding: utf-8 -*-
from __future__ import annotations
import os
import logging
from typing import Optional, Callable, Tuple, Any

from PyQt6 import uic
from PyQt6.QtWidgets import QWidget, QPushButton, QCheckBox, QFrame, QVBoxLayout

import numpy as np
import pyvista as pv

_LOG = logging.getLogger("app.tabs.recipe.preview")

# Optional: für instanceof-Check, wenn verfügbar
try:
    from pyvistaqt import QtInteractor as _QtInteractor
except Exception:
    _QtInteractor = None


def _project_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))
    # .../src/app/tabs/recipe/coating_preview_panel/ → bis Projektwurzel
    return os.path.abspath(os.path.join(here, "..", "..", "..", "..", ".."))


def _ui_path(filename: str) -> str:
    # erwartet: resource/ui/tabs/recipe/<filename>
    return os.path.join(_project_root(), "resource", "ui", "tabs", "recipe", filename)


class CoatingPreviewPanel(QWidget):
    """
    Lädt die UI aus .ui, stellt Controls + einen Host (previewHost) bereit.

    Preview-API:
      - automatische Interactor-Ermittlung via Elternkette (self._ensure_interactor)
      - build_init_scene(bounds, grid_step)
      - clear(), add_mesh(), view_*(), render()
      - set_grid_visible(), set_bounds(), set_grid()
      - clear_layer(name), add_path_polyline(...), add_path_markers(...)

    Optional:
      - attach_interactor(interactor) erlaubt explizites Setzen, ist aber nicht notwendig.
    """

    # ===== Standardfarben (kannst du oben zentral anpassen) =====
    DEFAULT_MOUNT_COLOR = "lightgray"
    DEFAULT_SUBSTRATE_COLOR = "#3498db"
    DEFAULT_PATH_COLOR = "#ff7f0e"
    DEFAULT_MARKER_COLOR = "#2ecc71"

    def __init__(self, *, ctx, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx

        uic.loadUi(_ui_path("coating_preview_panel.ui"), self)

        # Controls aus der UI
        self.chkShowMask: QCheckBox = self.findChild(QCheckBox, "chkShowMask")
        self.chkShowRays: QCheckBox = self.findChild(QCheckBox, "chkShowRays")
        self.chkShowLocalFrames: QCheckBox = self.findChild(QCheckBox, "chkShowLocalFrames")

        self.btnCamIso:   QPushButton = self.findChild(QPushButton, "btnCamIso")
        self.btnCamTop:   QPushButton = self.findChild(QPushButton, "btnCamTop")
        self.btnCamFront: QPushButton = self.findChild(QPushButton, "btnCamFront")
        self.btnCamBack:  QPushButton = self.findChild(QPushButton, "btnCamBack")
        self.btnCamLeft:  QPushButton = self.findChild(QPushButton, "btnCamLeft")
        self.btnCamRight: QPushButton = self.findChild(QPushButton, "btnCamRight")
        self.btnValidate: QPushButton = self.findChild(QPushButton, "btnValidate")

        # Host für QtInteractor
        self._host: QFrame = self.findChild(QFrame, "previewHost")
        if self._host is None:
            raise RuntimeError("coating_preview_panel.ui muss ein QFrame namens 'previewHost' enthalten.")

        # Externer QtInteractor (pyvistaqt.QtInteractor) – wird automatisch via Eltern gesucht
        self._ia: Optional[Any] = None

        # Zuletzt genutzte Grid-/Bounds-Configs
        self._bounds: Tuple[float, float, float, float, float, float] = (-120, 120, -120, 120, 0, 240)
        self._grid_step: float = 10.0
        self._grid_actor = None
        self._grid_visible: bool = True

        # Layer-Verwaltung (z. B. "path", "path_markers")
        self._layer_actors: dict[str, list] = {}

        # Kamera-Callbacks (intern, da Main unverändert bleiben soll)
        self._hooks: dict[str, Optional[Callable[[], None]]] = {
            "iso": self.view_isometric,
            "top": self.view_top,
            "front": self.view_front,
            "left": self.view_left,
            "right": self.view_right,
            "back": self.view_back,
            "after": lambda: self.render(reset_camera=False),
        }
        self._wire_buttons()

        # einmaliges Auto-Init der Szene beim ersten Anzeigen
        self._did_init_scene_once = False

    # -------------------------------------------------------------------------
    # Interactor-Auflösung
    # -------------------------------------------------------------------------
    def _resolve_interactor_via_parents(self) -> Optional[Any]:
        """
        Geht die Elternkette hoch und sucht nach einem Attribut,
        das wie ein QtInteractor aussieht.
        Bevorzugt 'previewPlot' (dein MainWindow), fällt aber auf
        generische Namen zurück.
        """
        cand_attr_names = ("previewPlot", "plotter", "interactor", "plot")
        p = self
        while p is not None:
            for name in cand_attr_names:
                if hasattr(p, name):
                    obj = getattr(p, name)
                    if obj is None:
                        continue
                    # Sanity: typische QtInteractor-Methoden vorhanden?
                    if all(hasattr(obj, m) for m in ("add_mesh", "view_xy", "render")):
                        # Optional genauer prüfen
                        if _QtInteractor is None or isinstance(obj, _QtInteractor) or hasattr(obj, "camera"):
                            return obj
            p = p.parent()
        return None

    def _ensure_interactor(self) -> bool:
        """Sorgt dafür, dass self._ia gesetzt ist (via Elternauflösung)."""
        if self._ia is not None:
            return True
        ia = self._resolve_interactor_via_parents()
        if ia is not None:
            self._ia = ia
            return True
        return False

    def _try_adopt_interactor_from_window(self) -> bool:
        """Sucht im Top-Level-Window nach 'previewPlot' (oder alternativen Namen) und dockt ihn an."""
        try:
            win = self.window()
            if not win:
                return False
            # bevorzugt 'previewPlot'
            for name in ("previewPlot", "plotter", "interactor", "plot"):
                if hasattr(win, name):
                    interactor = getattr(win, name)
                    if interactor is None:
                        continue
                    # nur wenn noch keiner gesetzt ist:
                    if self._ia is None:
                        self.attach_interactor(interactor)
                    return True
            return False
        except Exception:
            _LOG.exception("adopt_interactor_from_window failed")
            return False

    def showEvent(self, ev):
        """Beim ersten Anzeigen: Interactor adoptieren und Init-Szene bauen (Main-Style)."""
        super().showEvent(ev)
        if self._did_init_scene_once:
            return
        # Versuche Interactor aus dem Window zu adoptieren (falls attach schon passierte, ist _ia gesetzt)
        if self._ia is None:
            self._try_adopt_interactor_from_window()
        if self._ia is not None:
            try:
                self.build_init_scene_mainstyle(grid_step=10.0)
            except Exception:
                _LOG.exception("build_init_scene_mainstyle() in showEvent failed")
            self._did_init_scene_once = True

    # -------------------------------------------------------------------------
    # Public API – Hosting
    # -------------------------------------------------------------------------
    # --- 1:1-Übernahme der Main-Init-Szene -----------------------------------
    def build_init_scene_mainstyle(self, grid_step: float = 10.0):
        """Repliziert die bisherige _build_init_scene() aus MainWindow 1:1."""
        if self._ia is None:
            _LOG.warning("build_init_scene_mainstyle(): kein Interactor angehängt")
            return

        p = self._ia
        p.clear()

        bounds = (-120, 120, -120, 120, 0, 240)
        axes = p.show_grid(
            bounds=bounds,
            xtitle="X (mm)", ytitle="Y (mm)", ztitle="Z (mm)",
            show_xaxis=True, show_yaxis=True, show_zaxis=True,
            show_xlabels=True, show_ylabels=True, show_zlabels=True,
            n_xlabels=6, n_ylabels=6, n_zlabels=7,
            ticks='both', grid='back', render=False
        )
        try:
            axes.SetShowEdges(False)
            axes.SetDrawXGridlines(True); axes.SetDrawYGridlines(True); axes.SetDrawZGridlines(True)
            axes.SetDrawXInnerGridlines(True); axes.SetDrawYInnerGridlines(True); axes.SetDrawZInnerGridlines(True)
            axes.SetUseTextActor3D(1)
        except Exception:
            pass

        try:
            p.view_isometric()
            p.reset_camera(bounds=bounds)
        except Exception:
            pass

        p.render()

    def preview_host(self) -> QFrame:
        """Host-Widget, in das MainWindow den QtInteractor einhängt (per setParent/addWidget)."""
        return self._host

    def attach_interactor(self, interactor: Any) -> None:
        """
        Optional: explizit einen Interactor setzen und in den Host einhängen.
        Dein MainWindow kann unverändert bleiben; diese Methode wird nicht zwingend benötigt.
        """
        if interactor is None:
            raise ValueError("attach_interactor: interactor is None")

        try:
            self._ia = interactor
            ly = self._host.layout()
            if ly is None:
                ly = QVBoxLayout(self._host)
                ly.setContentsMargins(0, 0, 0, 0)
                ly.setSpacing(0)

            self._ia.setParent(self._host)
            try:
                ly.addWidget(self._ia)
            except Exception:
                pass  # Duplikat-Hinzufügen ignorieren

            self._ia.setEnabled(True)
            self._ia.show()
            self._ia.update()
            if hasattr(self._ia, "render"):
                self._ia.render()
        except Exception:
            _LOG.exception("attach_interactor() failed")

    # -------------------------------------------------------------------------
    # Public API – Grid/Bounds
    # -------------------------------------------------------------------------
    def build_init_scene(
        self,
        bounds: Tuple[float, float, float, float, float, float] = (-120, 120, -120, 120, 0, 240),
        grid_step: float = 10.0,
    ) -> None:
        """Leere Szene + (optional) Gitternetz + Achsenbeschriftung."""
        if not self._ensure_interactor():
            _LOG.warning("build_init_scene(): kein Interactor via Eltern gefunden")
            return

        self._bounds = bounds
        self._grid_step = grid_step

        p = self._ia
        p.clear()
        self._grid_actor = None

        if self._grid_visible:
            self._grid_actor = p.show_grid(
                bounds=bounds,
                xtitle="X (mm)", ytitle="Y (mm)", ztitle="Z (mm)",
                show_xaxis=True, show_yaxis=True, show_zaxis=True,
                show_xlabels=True, show_ylabels=True, show_zlabels=True,
                n_xlabels=max(2, int((bounds[1] - bounds[0]) // (2 * grid_step))),
                n_ylabels=max(2, int((bounds[3] - bounds[2]) // (2 * grid_step))),
                n_zlabels=max(2, int((bounds[5] - bounds[4]) // (grid_step * 4))),
                ticks='both',
                grid='back',
                render=False,
            )
            try:
                ax = self._grid_actor  # vtkCubeAxesActor
                ax.SetShowEdges(False)
                ax.SetDrawXGridlines(True); ax.SetDrawYGridlines(True); ax.SetDrawZGridlines(True)
                ax.SetDrawXInnerGridlines(True); ax.SetDrawYInnerGridlines(True); ax.SetDrawZInnerGridlines(True)
                ax.SetUseTextActor3D(1)
            except Exception:
                pass

        try:
            p.view_isometric()
            p.reset_camera(bounds=bounds)
        except Exception:
            pass

        p.render()

    def set_bounds(self, bounds: Tuple[float, float, float, float, float, float]) -> None:
        """Bounds aktualisieren (Grid neu zeichnen, Kamera bleibt)."""
        self._bounds = bounds
        if not self._ensure_interactor():
            return
        try:
            if self._grid_actor is not None:
                try:
                    self._ia.remove_actor(self._grid_actor, render=False)
                except Exception:
                    pass
                self._grid_actor = None
            if self._grid_visible:
                self._grid_actor = self._ia.show_grid(
                    bounds=bounds,
                    xtitle="X (mm)", ytitle="Y (mm)", ztitle="Z (mm)",
                    ticks='both', grid='back', render=False
                )
                try:
                    ax = self._grid_actor
                    ax.SetShowEdges(False)
                    ax.SetDrawXGridlines(True); ax.SetDrawYGridlines(True); ax.SetDrawZGridlines(True)
                    ax.SetDrawXInnerGridlines(True); ax.SetDrawYInnerGridlines(True); ax.SetDrawZInnerGridlines(True)
                    ax.SetUseTextActor3D(1)
                except Exception:
                    pass
            self._ia.render()
        except Exception:
            _LOG.exception("set_bounds() failed")

    def set_grid(self, *, bounds: Optional[Tuple[float, float, float, float, float, float]] = None,
                 step: Optional[float] = None) -> None:
        """Bounds/Step zusammen anpassen und Grid neu zeichnen."""
        if bounds is not None:
            self._bounds = bounds
        if step is not None:
            self._grid_step = float(step)
        self.set_bounds(self._bounds)

    def set_grid_visible(self, visible: bool) -> None:
        """Grid ein-/ausblenden, ohne Szene zu löschen."""
        self._grid_visible = bool(visible)
        if not self._ensure_interactor():
            return
        try:
            if not visible and self._grid_actor is not None:
                try:
                    self._ia.remove_actor(self._grid_actor, render=False)
                except Exception:
                    pass
                self._grid_actor = None
            elif visible and self._grid_actor is None:
                # neu zeichnen
                self.set_bounds(self._bounds)
            self._ia.render()
        except Exception:
            _LOG.exception("set_grid_visible() failed")

    # -------------------------------------------------------------------------
    # Public API – Szene/Meshes
    # -------------------------------------------------------------------------
    def clear(self) -> None:
        """Szene leeren und Grid neu aufbauen."""
        if not self._ensure_interactor():
            return
        try:
            self._ia.clear()
            self.build_init_scene(self._bounds, self._grid_step)
        except Exception:
            _LOG.exception("clear() failed")

    def add_mesh(self, mesh, **kwargs) -> None:
        """Mesh hinzufügen (robust triangulieren, nicht sofort rendern)."""
        if not self._ensure_interactor():
            _LOG.warning("add_mesh(): kein Interactor via Eltern gefunden")
            return
        try:
            fn = getattr(mesh, "is_all_triangles", None)
            if callable(fn) and not fn():
                mesh = mesh.triangulate()
        except Exception:
            pass
        try:
            actor = self._ia.add_mesh(mesh, reset_camera=False, render=False, **kwargs)
            if actor is None:
                # add_mesh kann je nach Backend None liefern; ignorieren
                pass
        except Exception:
            _LOG.exception("add_mesh() failed")

    # ---- Layer-Handling ------------------------------------------------------
    def _layer_add_actor(self, layer: str, actor) -> None:
        self._layer_actors.setdefault(layer, []).append(actor)

    def clear_layer(self, layer: str) -> None:
        """Entfernt alle Actoren eines Layers, ohne die komplette Szene zu löschen."""
        if not self._ensure_interactor():
            return
        actors = self._layer_actors.pop(layer, [])
        for a in actors:
            try:
                self._ia.remove_actor(a, render=False)
            except Exception:
                pass
        try:
            self._ia.render()
        except Exception:
            pass

    # ---- Pfad-Zeichner -------------------------------------------------------
    def add_path_polyline(
        self,
        points_mm: np.ndarray,
        *,
        color: str = None,
        line_width: float = 2.0,
        as_tube: bool = False,
        tube_radius_mm: float = 0.6,
        layer: str = "path",
        lighting: bool = False,
    ) -> None:
        """
        Zeichnet eine durchgehende Polyline durch Nx3-Punkte in mm.
        - as_tube=True: röhrenförmig mit tube_radius_mm
        """
        if not self._ensure_interactor():
            _LOG.warning("add_path_polyline(): kein Interactor via Eltern gefunden")
            return
        pts = np.asarray(points_mm, dtype=np.float64)
        if pts.ndim != 2 or pts.shape[1] != 3 or pts.shape[0] < 2:
            _LOG.error("add_path_polyline(): erwartet Nx3 Punkte, N>=2, got %r", pts.shape)
            return

        if color is None:
            color = self.DEFAULT_PATH_COLOR

        n = pts.shape[0]
        lines = np.hstack(([n], np.arange(n, dtype=np.int64)))  # ein Strang
        poly = pv.PolyData(pts, lines=lines)

        try:
            if as_tube:
                tube = poly.tube(radius=float(tube_radius_mm), n_sides=12, capping=True)
                actor = self._ia.add_mesh(tube, color=color, render=False, reset_camera=False, lighting=lighting)
            else:
                actor = self._ia.add_mesh(
                    poly, color=color, render=False, reset_camera=False,
                    line_width=float(line_width), lighting=lighting
                )
            self._layer_add_actor(layer, actor)
        except Exception:
            _LOG.exception("add_path_polyline() failed")

    def add_path_markers(
        self,
        points_mm: np.ndarray,
        *,
        radius_mm: float = 0.8,
        step: int = 10,
        color: str = None,
        layer: str = "path_markers",
        lighting: bool = False,
    ) -> None:
        """
        Markiert jeden 'step'-ten Wegpunkt mit einer kleinen Kugel.
        """
        if not self._ensure_interactor():
            return
        pts = np.asarray(points_mm, dtype=np.float64)
        if pts.ndim != 2 or pts.shape[1] != 3 or pts.shape[0] < 1:
            return

        if color is None:
            color = self.DEFAULT_MARKER_COLOR

        try:
            pick = pts[::max(1, int(step))]
            if pick.shape[0] == 0:
                pick = pts[:1]
            src = pv.Sphere(radius=float(radius_mm))
            cloud = pv.PolyData(pick)
            glyphs = cloud.glyph(scale=False, geom=src)
            actor = self._ia.add_mesh(glyphs, color=color, render=False, reset_camera=False, lighting=lighting)
            self._layer_add_actor(layer, actor)
        except Exception:
            _LOG.exception("add_path_markers() failed")

    # -------------------------------------------------------------------------
    # Public API – Views
    # -------------------------------------------------------------------------
    def view_isometric(self) -> None:
        if not self._ensure_interactor():
            return
        try:
            self._ia.view_isometric()
        except Exception:
            pass

    def view_top(self) -> None:
        if not self._ensure_interactor():
            return
        try:
            self._ia.view_xy()
        except Exception:
            pass

    def view_front(self) -> None:
        if not self._ensure_interactor():
            return
        try:
            self._ia.view_yz()
        except Exception:
            pass

    def view_left(self) -> None:
        if not self._ensure_interactor():
            return
        try:
            self._ia.view_xz()
        except Exception:
            pass

    def view_right(self) -> None:
        if not self._ensure_interactor():
            return
        try:
            self._ia.view_xz()
            try:
                self._ia.camera.azimuth(180)
            except Exception:
                pass
        except Exception:
            pass

    def view_back(self) -> None:
        if not self._ensure_interactor():
            return
        try:
            self._ia.view_yz()
            try:
                self._ia.camera.azimuth(180)
            except Exception:
                pass
        except Exception:
            pass

    # -------------------------------------------------------------------------
    # Public API – Render
    # -------------------------------------------------------------------------
    def render(self, *, reset_camera: bool = True) -> None:
        if not self._ensure_interactor():
            return
        try:
            if reset_camera:
                self._ia.reset_camera(bounds=self._bounds)
            self._ia.render()
        except Exception:
            _LOG.exception("render() failed")

    # -------------------------------------------------------------------------
    # intern
    # -------------------------------------------------------------------------
    def _wire_buttons(self):
        if self.btnCamIso:   self.btnCamIso.clicked.connect(lambda: self._call_cam("iso"))
        if self.btnCamTop:   self.btnCamTop.clicked.connect(lambda: self._call_cam("top"))
        if self.btnCamFront: self.btnCamFront.clicked.connect(lambda: self._call_cam("front"))
        if self.btnCamLeft:  self.btnCamLeft.clicked.connect(lambda: self._call_cam("left"))
        if self.btnCamRight: self.btnCamRight.clicked.connect(lambda: self._call_cam("right"))
        if self.btnCamBack:  self.btnCamBack.clicked.connect(lambda: self._call_cam("back"))

    def _call_cam(self, key: str):
        fn = self._hooks.get(key)
        if callable(fn):
            try:
                fn()
            except Exception:
                _LOG.exception("Kamera-Button '%s' fehlgeschlagen", key)
        after = self._hooks.get("after")
        if callable(after):
            try:
                after()
            except Exception:
                _LOG.exception("Kamera after-hook fehlgeschlagen")
