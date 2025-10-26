# -*- coding: utf-8 -*-
from __future__ import annotations
import os
import logging
from typing import Any, Dict, Optional, List, Tuple

from PyQt5 import uic
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QWidget, QVBoxLayout

from pyvistaqt import QtInteractor
import pyvista as pv
import numpy as np

from .preview import PreviewEngine, Trajectory, TrajPose
from .path_builder import PathBuilder
from .trajectory_builder import TrajectoryBuilder  # optional weiter nutzbar
from .mesh_utils import resolve_pkg_or_abs, apply_translation_deg
from .raycast_projector import (
    project_path_via_rays_reflected,
    raydir_topdown,
    raydir_radial_xy,
    quat_from_two_vectors,
)

_LOG = logging.getLogger("app.tabs.recipe.coating_preview_panel")

# ---------- Pfad-Helfer ----------
def _project_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))
    return os.path.abspath(os.path.join(here, "..", "..", "..", "..", ".."))

def _ui_path(filename: str) -> str:
    return os.path.join(_project_root(), "resource", "ui", "tabs", "recipe", filename)

class CoatingPreviewPanel(QWidget):
    """
    MITTLERES PANEL – Rendering & Preview-Controls:
      - hostet PyVista
      - lädt STL (nur .stl) für mount/substrate/tool aus Pfaden der startup.yaml
      - baut Path (2D) + projiziert via Raycasting auf Substrat, TCP = Hit + stand_off * reflektierter_Ray
      - hält last_traj_dict
      - meldet readiness via readyChanged(ok, msg)
    """
    readyChanged = pyqtSignal(bool, str)  # (ok, message)

    def __init__(self, *, ctx, parent=None):
        super().__init__(parent)
        self.ctx = ctx
        uic.loadUi(_ui_path("coating_preview_panel.ui"), self)  # <- richtiger UI-Dateiname

        # Plotter
        if self.pvContainer.layout() is None:
            self.pvContainer.setLayout(QVBoxLayout(self.pvContainer))
        self.plotter = QtInteractor(self.pvContainer)
        self.pvContainer.layout().addWidget(self.plotter)
        self.preview = PreviewEngine(self.plotter)

        # Builder (falls du später noch brauchst)
        self.traj_builder = TrajectoryBuilder()
        self._last_traj_dict: Optional[Dict[str, Any]] = None

        # Gehaltene Meshes
        self._substrate_mesh: Optional[pv.PolyData] = None
        self._mount_mesh: Optional[pv.PolyData] = None

        # Kamera-Buttons (optional in UI)
        for nm, fn in [
            ("btnViewIso", self.preview.view_iso),
            ("btnViewTop", self.preview.view_top),
            ("btnViewFront", self.preview.view_front),
            ("btnViewBack", self.preview.view_back),
            ("btnViewLeft", self.preview.view_left),
            ("btnViewRight", self.preview.view_right),
        ]:
            if hasattr(self, nm):
                getattr(self, nm).clicked.connect(fn)

        # Toggles (optional)
        if hasattr(self, "checkNormals"):
            self.checkNormals.toggled.connect(lambda b: self.preview.set_normals_visible(bool(b)))
        if hasattr(self, "checkRaycasts"):
            self.checkRaycasts.toggled.connect(lambda b: self.preview.set_rays_visible(bool(b)))

        # Clear (optional)
        if hasattr(self, "btnClear"):
            self.btnClear.clicked.connect(self.clear)

        # Anfangszustand
        self._emit_ready(False, "idle")

    # ---------- Public API ----------
    def clear(self) -> None:
        self.preview.clear()
        self._last_traj_dict = None
        self._substrate_mesh = None
        self._mount_mesh = None
        self._emit_ready(False, "cleared")

    def last_trajectory_dict(self) -> Optional[Dict[str, Any]]:
        return self._last_traj_dict

    def render_from_model(self, model, sides: List[str]) -> bool:
        """Baut Szene (Mount/Substrate) + Trajektorie und rendert. Liefert ok."""
        # 1) Model prüfen
        try:
            errs = model.validate_required()
        except Exception as e:
            self._emit_ready(False, f"model invalid: {e}")
            return False
        if errs:
            self._emit_ready(False, "; ".join(errs))
            return False

        # 2) Szene aufbauen
        self.preview.clear()
        self._substrate_mesh = None
        self._mount_mesh = None

        ok_scene = True
        try:
            self._add_mount_mesh(model)
        except Exception as e:
            _LOG.warning("Mount mesh not loaded: %s", e)
            ok_scene = False
        try:
            self._add_substrate_mesh(model)
        except Exception as e:
            _LOG.warning("Substrate mesh not loaded: %s", e)
            ok_scene = False

        # ohne Substrat keine Projektion
        if self._substrate_mesh is None:
            self._last_traj_dict = None
            self._emit_ready(False, "no substrate mesh")
            return False

        # 3) 2D-Pfad erzeugen
        try:
            path_data = PathBuilder.from_recipe(model)
        except Exception as e:
            self._last_traj_dict = None
            self._emit_ready(False, f"path build failed: {e}")
            _LOG.exception("PathBuilder failed")
            return False

        P2 = path_data.points_mm[:, :2]
        P3 = np.c_[P2, np.zeros((len(P2),))]

        # 4) Raycasting-Projektion (reflektierter Strahl)
        ptype = str((getattr(model, "path", {}) or {}).get("type", "")).lower()
        if ptype in ("spiral_cylinder", "helix"):
            ray_fn = lambda P: raydir_radial_xy(P, center_xy=(0.0, 0.0))
        else:
            ray_fn = raydir_topdown

        params = getattr(model, "parameters", {}) or {}
        stand_off = float(params.get("stand_off_mm", 10.0))

        rc = project_path_via_rays_reflected(
            P3, self._substrate_mesh,
            stand_off_mm=stand_off,
            ray_dir_fn=ray_fn,
        )

        valid = rc.valid
        if not np.any(valid):
            self._last_traj_dict = None
            self._emit_ready(False, "no ray hits on substrate")
            return False

        # 5) Trajectory für Preview (m, quat xyzw)
        P_tcp_m = (rc.tcp_mm[valid] / 1000.0).astype(float)  # mm → m
        tool_axis = np.array([0.0, 0.0, -1.0])
        Q = quat_from_two_vectors(
            np.tile(tool_axis, (np.count_nonzero(valid), 1)),
            rc.refl_dir[valid]
        )
        # simple timestamps
        Tt = np.linspace(0.0, float(len(P_tcp_m)) * 0.02, len(P_tcp_m))
        poses = [TrajPose(p=P_tcp_m[i], q=Q[i], t=Tt[i]) for i in range(len(P_tcp_m))]
        traj = Trajectory(poses, frame_id="scene", meta={"stand_off_mm": stand_off})

        self.preview.render_trajectory(
            traj,
            show_normals=bool(getattr(self, "checkNormals", None) and self.checkNormals.isChecked()),
            show_rays=bool(getattr(self, "checkRaycasts", None) and self.checkRaycasts.isChecked()),
            append=False,
        )
        self.preview.view_iso()

        self._last_traj_dict = {
            "frame_id": "scene",
            "points_m": P_tcp_m.tolist(),
            "quat_xyzw": Q.tolist(),
            "timestamps_s": Tt.tolist(),
            "stand_off_mm": stand_off,
        }

        msg = "scene ok" if ok_scene else "scene partial (some meshes missing)"
        self._emit_ready(True, msg)
        return True

    # ---------- Intern ----------
    def _emit_ready(self, ok: bool, msg: str):
        if hasattr(self, "lblReady"):
            self.lblReady.setText("OK" if ok else f"Fehler: {msg}")
            self.lblReady.setStyleSheet("color:#0a0;" if ok else "color:#a00;")
        self.readyChanged.emit(ok, msg)

    # --- Mesh Laden mit startup-Pfaden ---
    def _resolve_mesh_from_dirs(self, name_or_uri: str, subdir: str) -> Optional[str]:
        """
        name_or_uri: Basename ohne Endung ODER absolute/ROS-URI zu .stl
        subdir: einer von 'tools_dir' | 'substrates_dir' | 'substrate_mounts_dir'
        """
        if not name_or_uri:
            return None

        # Bereits absolut oder package:// ?
        p = resolve_pkg_or_abs(name_or_uri, base_dirs=[])
        if p and p.lower().endswith(".stl") and os.path.exists(p):
            return p

        # Sonst Basename -> .stl in den startup-Dirs
        base = os.path.splitext(os.path.basename(name_or_uri))[0] + ".stl"
        dirs = []
        paths = getattr(self.ctx, "paths", None)
        if paths is not None and hasattr(paths, subdir):
            d = getattr(paths, subdir)
            rd = resolve_pkg_or_abs(d, base_dirs=[])
            if rd and os.path.isdir(rd):
                dirs.append(rd)

        for d in dirs:
            cand = os.path.join(d, base)
            if os.path.exists(cand):
                return cand
        return None

    def _active_mount_scene_offset(self) -> Tuple[float, float, float, Tuple[float,float,float]]:
        """
        Aus substrate_mounts.yaml:
          version: 1
          active_mount: "<key>"
          mounts:
            <key>:
              scene_offset: { xyz: [..], rpy_deg: [..] }
        Fallback: (0,0,0), (0,0,0)
        """
        try:
            mounts_yaml = getattr(self.ctx, "mounts_yaml", None)
            if not mounts_yaml:
                return 0.0, 0.0, 0.0, (0.0, 0.0, 0.0)
            active = mounts_yaml.get("active_mount")
            m = (mounts_yaml.get("mounts") or {}).get(active or "")
            if not m:
                return 0.0, 0.0, 0.0, (0.0, 0.0, 0.0)
            off = (m.get("scene_offset") or {})
            xyz = off.get("xyz") or [0.0, 0.0, 0.0]
            rpy = off.get("rpy_deg") or [0.0, 0.0, 0.0]
            return float(xyz[0]), float(xyz[1]), float(xyz[2]), (float(rpy[0]), float(rpy[1]), float(rpy[2]))
        except Exception:
            return 0.0, 0.0, 0.0, (0.0, 0.0, 0.0)

    def _add_mount_mesh(self, model) -> None:
        # Quelle: Rezept-Feld 'mount' oder 'substrate_mount'
        name = getattr(model, "mount", None) or model.to_dict().get("mount") \
               or getattr(model, "substrate_mount", None) or model.to_dict().get("substrate_mount")

        p = resolve_pkg_or_abs(str(name) if name else "", base_dirs=[])
        if not p:
            p = self._resolve_mesh_from_dirs(str(name or ""), "substrate_mounts_dir")
        if not p:
            raise FileNotFoundError(f"Mount STL nicht gefunden für '{name}'")

        mesh = pv.read(p)
        self._mount_mesh = mesh
        # Mount bleibt im Ursprung
        self.preview.add_mesh(mesh, color="lightgray", opacity=0.25)

    def _add_substrate_mesh(self, model) -> None:
        name = getattr(model, "substrate", None) or model.to_dict().get("substrate")
        p = resolve_pkg_or_abs(str(name) if name else "", base_dirs=[])
        if not p:
            p = self._resolve_mesh_from_dirs(str(name or ""), "substrates_dir")
        if not p:
            raise FileNotFoundError(f"Substrate STL nicht gefunden für '{name}'")

        mesh = pv.read(p)

        # Substrat auf Mount-Top legen: scene_offset aus mounts_yaml
        dx, dy, dz, rpy = self._active_mount_scene_offset()
        if any(abs(v) > 1e-9 for v in (dx, dy, dz)) or any(abs(a) > 1e-9 for a in rpy):
            mesh = apply_translation_deg(mesh, translate_mm=(dx, dy, dz), rpy_deg=rpy)

        self._substrate_mesh = mesh
        self.preview.add_mesh(mesh, color="#4c6ef5", opacity=0.30)
