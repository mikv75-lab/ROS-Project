# -*- coding: utf-8 -*-
# Spraycoater/src/app/tabs/recipe/coating_preview_panel/coating_preview_panel.py
from __future__ import annotations
import os
import logging
from typing import Optional, Any, Iterable, Tuple, Dict

import numpy as np
import pyvista as pv
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget, QVBoxLayout
# Beide Plotter-Varianten:
from pyvistaqt import QtInteractor, BackgroundPlotter

from .preview import PreviewEngine
from .mesh_utils import (
    get_mount_scene_offset_from_key,
    load_substrate_mesh_from_key,
    place_substrate_on_mount,
    load_mount_mesh_from_key,
)
from .path_builder import PathBuilder, PathData
from .raycast_projector import cast_rays_for_side  # robust, mit Miss-Handling

# Pfad-Helfer
def _project_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))
    return os.path.abspath(os.path.join(here, "..", "..", "..", "..", ".."))

def _ui_path(filename: str) -> str:
    return os.path.join(_project_root(), "resource", "ui", "tabs", "recipe", filename)

_LOG = logging.getLogger("app.tabs.recipe.coating_preview_panel")


class CoatingPreviewPanel(QWidget):
    """
    3D-Preview für Mount + Substrat + Pfad.
    Entweder eingebettet (QtInteractor) oder externes Fenster (BackgroundPlotter).

    Args:
        ctx: Kontext/Config
        parent: Qt-Parent
        embedded: Wenn True, QtInteractor im UI verwenden; sonst externes Fenster.
    """

    def __init__(self, *, ctx, parent: Optional[QWidget] = None, embedded: bool = False):
        super().__init__(parent)
        self.ctx = ctx
        self._embedded = bool(embedded)

        # UI laden (Buttons/Checkboxen/Kamera-Controls etc.)
        uic.loadUi(_ui_path("coating_preview_panel.ui"), self)

        # --- Plotter/Viewer wählen ---
        if self._embedded:
            # Eingebettet: QtInteractor im vorhandenen Container/Layout
            self._pv: QtInteractor = self.findChild(QtInteractor, "pv")
            if self._pv is None:
                # Fallback: dynamisch einfügen
                self._pv = QtInteractor(self)
                cont = self.findChild(QWidget, "pvContainer")
                if cont is not None:
                    lay = cont.layout()
                    if lay is None:
                        lay = QVBoxLayout(cont)
                        cont.setLayout(lay)
                    lay.addWidget(self._pv)
                else:
                    # Ganz zur Not direkt in das Root-Layout
                    root_lay = self.layout() or QVBoxLayout(self)
                    if self.layout() is None:
                        self.setLayout(root_lay)
                    root_lay.addWidget(self._pv)
        else:
            # Extern: separates Fenster
            self._pv: BackgroundPlotter = BackgroundPlotter(
                show=True, title="Coating Preview", off_screen=False
            )
            # Den eingebetteten UI-Container ausblenden (optional)
            cont = self.findChild(QWidget, "pvContainer")
            if cont:
                cont.setVisible(False)

        # Engine auf gewählten Plotter setzen
        self._engine: PreviewEngine = PreviewEngine(self._pv)

        # Sichtbarkeits-Handles (Actors) für UI-Toggles
        self._actors: Dict[str, Any] = {
            "mask": None,
            "rays": None,
            "triads_x": None,
            "triads_y": None,
            "triads_z": None,
            "tcp_line": None,
        }

        # Export-Zwischenspeicher
        self._last_export_poses = None
        self._last_export_frame = "scene"

        # --- UI-Signale verbinden ---
        self._wire_ui_signals()

        # Externen Plotter beim Widget-Abbau sauber schließen
        self.destroyed.connect(self._shutdown_plotter)

    # ---------- UI WIRING ----------
    def _wire_ui_signals(self) -> None:
        # Checkboxes
        chk_mask = self.findChild(QWidget, "chkShowMask")
        chk_rays = self.findChild(QWidget, "chkShowRays")
        chk_frames = self.findChild(QWidget, "chkShowLocalFrames")
        if chk_mask:
            chk_mask.toggled.connect(lambda on: self._set_actor_visible("mask", on))
        if chk_rays:
            chk_rays.toggled.connect(lambda on: self._set_actor_visible("rays", on))
        if chk_frames:
            # Triads sind drei getrennte Actors
            chk_frames.toggled.connect(self._toggle_triads)

        # Kamera-Buttons
        btn_iso   = self.findChild(QWidget, "btnCamIso")
        btn_top   = self.findChild(QWidget, "btnCamTop")
        btn_front = self.findChild(QWidget, "btnCamFront")
        btn_back  = self.findChild(QWidget, "btnCamBack")
        btn_left  = self.findChild(QWidget, "btnCamLeft")
        btn_right = self.findChild(QWidget, "btnCamRight")
        if btn_iso:   btn_iso.clicked.connect(self._view_iso)
        if btn_top:   btn_top.clicked.connect(self._view_top)
        if btn_front: btn_front.clicked.connect(self._view_front)
        if btn_back:  btn_back.clicked.connect(self._view_back)
        if btn_left:  btn_left.clicked.connect(self._view_left)
        if btn_right: btn_right.clicked.connect(self._view_right)

        # Validate-Button (nur Hook – eigentliche Aktion bindest du dort ein)
        btn_validate = self.findChild(QWidget, "btnValidate")
        if btn_validate:
            btn_validate.clicked.connect(self._on_validate_clicked)

    # --- Clean-up für externen Plotter ---
    def _shutdown_plotter(self, *_args):
        if not self._embedded and self._pv is not None:
            try:
                if hasattr(self._pv, "close"):
                    self._pv.close()
            except Exception:
                pass

    def _set_actor_visible(self, key: str, visible: bool) -> None:
        actor = self._actors.get(key)
        if actor is None:
            return
        try:
            actor.SetVisibility(bool(visible))
            self._pv.render()
        except Exception:
            # PreviewEngine könnte eigenen Wrapper nutzen – notfalls ignorieren
            pass

    def _toggle_triads(self, on: bool) -> None:
        for k in ("triads_x", "triads_y", "triads_z"):
            self._set_actor_visible(k, on)

    # ---------- Public ----------
    def render_from_model(self, model: Any, sides: Iterable[str]) -> None:
        # --- Input prüfen ---
        mount_key = self._get_attr(model, "substrate_mount")
        if not isinstance(mount_key, str) or not mount_key.strip():
            raise ValueError("Recipe benötigt ein gültiges 'substrate_mount'.")
        mount_key = mount_key.strip()

        substrate_key = self._get_attr(model, "substrate")
        if not isinstance(substrate_key, str) or not substrate_key.strip():
            raise ValueError("Recipe benötigt ein gültiges 'substrate'.")
        substrate_key = substrate_key.strip()

        _LOG.info("Preview: mount=%s, substrate=%s", mount_key, substrate_key)

        # --- Szene vorbereiten ---
        self._engine.clear_scene()
        self._view_iso()  # stabile Startpose

        # Mount (solid, dunkelgrau)
        mount_mesh = load_mount_mesh_from_key(self.ctx, mount_key)
        self._engine.add_mesh(
            mount_mesh,
            color=(0.25, 0.25, 0.25),
            opacity=1.0,
            smooth_shading=True,
            lighting=False,
            show_edges=False,
        )

        # Substrat laden (lokal) + platziertes Substrat (Welt) für Anzeige
        sub_mesh_local = load_substrate_mesh_from_key(self.ctx, substrate_key)

        # Ground shift (lokal z_min -> 0)
        zmin = float(sub_mesh_local.bounds[4])
        ground_shift = np.array([0.0, 0.0, -zmin], dtype=float)

        # Mount scene_offset (xyz + rpy)
        xyz_mm, rpy_deg = get_mount_scene_offset_from_key(self.ctx, mount_key)
        R_sub = _rpy_deg_to_matrix_np(rpy_deg)  # 3x3
        t_sub = np.asarray(xyz_mm, dtype=float)  # 3,

        # Substrat positionieren und rendern (hellgrau)
        sub_scene = place_substrate_on_mount(self.ctx, sub_mesh_local, mount_key=mount_key)
        self._engine.add_mesh(
            sub_scene,
            color=(0.78, 0.78, 0.78),
            opacity=1.0,
            smooth_shading=True,
            lighting=False,
            show_edges=False,
        )

        # Zusätzlich: lokales Substrat -> Welt transformieren (für Rays/Bounds in Welt)
        T = np.eye(4, dtype=float)
        T[:3, :3] = R_sub
        T[:3, 3] = t_sub + ground_shift
        sub_mesh_world = sub_mesh_local.copy(deep=True)
        sub_mesh_world.transform(T, inplace=True)
        sub_mesh_world.compute_normals(cell_normals=True, point_normals=False, inplace=True)

        try:
            # ---------------------------
            # 3 STEPS: PATH → (WELT)RAYCAST (seitenabhängig) → DIREKTES RENDERING/EXPORT
            # ---------------------------
            side = next(iter(sides), "top") if sides else "top"

            # 1) PATH (lokal in Substrat-Koords, Z=0)
            params = getattr(model, "parameters", {}) if hasattr(model, "parameters") else {}
            sample_step = float(params.get("sample_step_mm", 1.0))
            max_points  = int(params.get("max_points", 200000))

            pd_local: PathData = PathBuilder.from_side(
                model,
                side=side,
                sample_step_mm=sample_step,
                max_points=max_points,
            )

            if pd_local.points_mm is None or len(pd_local.points_mm) < 2:
                _LOG.warning("Kein Pfad generiert.")
                self._focus_camera_isometric(sub_scene)
                return

            # --- 1b) „Blaue Maske“ anhand Bounds + Offsets ---
            path_offset_mm   = float(params.get("path_offset_mm", 0.0))
            ray_clearance_mm = float(params.get("ray_clearance_mm", 100.0))  # z.B. 100 mm Freiraum

            P_local0 = np.array(pd_local.points_mm, dtype=float, copy=True)
            P_local0[:, 2] = 0.0
            P_world_base = ((P_local0 + ground_shift) @ R_sub.T) + t_sub

            plane_axis, plane_value = _side_plane_from_bounds(
                sub_mesh_world.bounds, side, path_offset_mm, ray_clearance_mm
            )
            P_world_start = P_world_base.copy()
            P_world_start[:, plane_axis] = plane_value

            # Blaue Maskenlinie
            mask_actor = self._engine.add_mesh(
                pv.lines_from_points(P_world_start, close=False),
                color="royalblue",
                opacity=1.0,
                smooth_shading=False,
                lighting=False,
                line_width=2.0,
            )
            self._actors["mask"] = getattr(mask_actor, "GetActor", lambda: mask_actor)()

            # 2) RAYCAST in WELT
            stand_off_mm = float(params.get("stand_off_mm", 10.0))
            src = str(pd_local.meta.get("source", "")).lower()

            rc, rays_hit_poly, _ = cast_rays_for_side(
                P_world_start,
                sub_mesh_world=sub_mesh_world,
                side=side,
                source=src,
                stand_off_mm=stand_off_mm,
                ray_len_mm=1000.0,
                start_lift_mm=10.0,               # 1 cm über dem blauen Pfad
                flip_normals_to_face_rays=False,
                invert_dirs=False,
                lock_xy=True,
            )

            # Mess-Rays (hellblau)
            rays_actor = None
            if rays_hit_poly.n_points > 0:
                rays_actor = self._engine.add_mesh(
                    rays_hit_poly,
                    color="#87CEFA",
                    opacity=1.0,
                    smooth_shading=False,
                    lighting=False,
                    line_width=1.5,
                )
            self._actors["rays"] = getattr(rays_actor, "GetActor", lambda: rays_actor)() if rays_actor else None

            mask = rc.valid
            if not np.any(mask):
                _LOG.warning("Raycast ergab keine Trefferpunkte.")
                self._focus_camera_isometric(sub_scene)
                return

            # Treffer & Normalen in Welt
            P_world_hits = rc.hit_mm[mask]    # (N,3) mm
            N_world      = rc.normal[mask]    # (N,3)

            # 3) DIREKT: TCP-Pfad (ohne TrajectoryBuilder)
            P_tcp_mm, Q_xyzw = _poses_from_hits_and_normals(P_world_hits, N_world, stand_off_mm)

            # Gelbe TCP-Linie
            tcp_line_actor = None
            if len(P_tcp_mm) >= 2:
                tcp_line_actor = self._engine.add_mesh(
                    pv.lines_from_points(P_tcp_mm, close=False),
                    color="yellow",
                    opacity=1.0,
                    smooth_shading=False,
                    lighting=False,
                    line_width=2.0,
                )
            self._actors["tcp_line"] = getattr(tcp_line_actor, "GetActor", lambda: tcp_line_actor)() if tcp_line_actor else None

            # Mini-Triads mit zusätzlicher 180°-Y-Rotation NUR für Visualisierung
            q_y180 = np.array([0.0, 1.0, 0.0, 0.0], dtype=float)  # 180° um Y
            Q_triads = np.empty_like(Q_xyzw)
            for i in range(len(Q_xyzw)):
                Q_triads[i] = _quat_mul(Q_xyzw[i], q_y180)

            factor_old = max(5.0, 0.6 * stand_off_mm)
            triad_len  = factor_old / 10.0  # mm
            x_pd, y_pd, z_pd = _frames_triad_polydata(P_tcp_mm, Q_triads, triad_len)

            tri_x = tri_y = tri_z = None
            if x_pd.n_points > 0:
                tri_x = self._engine.add_mesh(x_pd, color="red",   opacity=1.0, lighting=False, smooth_shading=False, line_width=1.5)
            if y_pd.n_points > 0:
                tri_y = self._engine.add_mesh(y_pd, color="green", opacity=1.0, lighting=False, smooth_shading=False, line_width=1.5)
            if z_pd.n_points > 0:
                tri_z = self._engine.add_mesh(z_pd, color="blue",  opacity=1.0, lighting=False, smooth_shading=False, line_width=1.5)

            self._actors["triads_x"] = getattr(tri_x, "GetActor", lambda: tri_x)() if tri_x else None
            self._actors["triads_y"] = getattr(tri_y, "GetActor", lambda: tri_y)() if tri_y else None
            self._actors["triads_z"] = getattr(tri_z, "GetActor", lambda: tri_z)() if tri_z else None

            # Export-Daten für Validate (ohne Y-Flip der Triads!)
            self._last_export_poses = _to_ros_pose_list_mm(P_tcp_mm, Q_xyzw)
            self._last_export_frame = "scene"

            # Checkbox-Initialzustand respektieren
            self._apply_checkbox_states()

        except Exception as e:
            _LOG.error("Path/Raycast/Trajectory fehlgeschlagen: %s", e, exc_info=True)

        # Kamera sauber auf Substrat fokussieren
        self._focus_camera_isometric(sub_scene)

    # ---------- Camera helpers ----------
    def _focus_camera_isometric(self, mesh: pv.PolyData) -> None:
        try:
            center = _center_of_bounds(mesh.bounds)
            diag = _diag_of_bounds(mesh.bounds)
            dist = max(50.0, 2.0 * diag)
            dir_iso = np.array([1.0, 1.0, 1.0], dtype=float)
            dir_iso /= np.linalg.norm(dir_iso) + 1e-12
            pos = center + dir_iso * dist
            viewup = (0.0, 0.0, 1.0)
            self._pv.camera_position = (tuple(pos), tuple(center), viewup)
            self._pv.reset_camera_clipping_range()
            self._pv.render()
        except Exception:
            self._pv.view_isometric()
            self._pv.reset_camera()

    def _view_iso(self):   self._pv.view_isometric(); self._pv.reset_camera()
    def _view_top(self):   self._pv.view_xy();        self._pv.reset_camera()
    def _view_front(self): self._pv.view_yz();        self._pv.reset_camera()
    def _view_back(self):  self._pv.view_yz();        self._pv.camera.azimuth = 180; self._pv.reset_camera()
    def _view_left(self):  self._pv.view_xz();        self._pv.reset_camera()
    def _view_right(self): self._pv.view_xz();        self._pv.camera.azimuth = 180; self._pv.reset_camera()

    def _apply_checkbox_states(self):
        def is_checked(name: str, default=True):
            w = self.findChild(QWidget, name)
            return bool(w.isChecked()) if w else default

        self._set_actor_visible("mask",    is_checked("chkShowMask", True))
        self._set_actor_visible("rays",    is_checked("chkShowRays", True))
        tri_on = is_checked("chkShowLocalFrames", True)
        for k in ("triads_x", "triads_y", "triads_z"):
            self._set_actor_visible(k, tri_on)

    # Validate Button Hook – hier später Export / ROS-Aufruf
    def _on_validate_clicked(self):
        if not self._last_export_poses:
            _LOG.warning("Kein gültiger Pfad zum Export vorhanden.")
            return
        # Beispiel: in ctx/bridge publizieren oder speichern
        _LOG.info("Validate: %d Posen im Frame '%s' bereit.", len(self._last_export_poses), self._last_export_frame)
        # TODO: hier ROS Bridge/Service aufrufen

    # ---------- Intern ----------
    @staticmethod
    def _get_attr(model: Any, key: str):
        if hasattr(model, key):
            return getattr(model, key)
        if isinstance(model, dict):
            return model.get(key)
        return None


# ---------- Helpers ----------
def _center_of_bounds(bounds):
    return np.array([
        0.5 * (bounds[0] + bounds[1]),
        0.5 * (bounds[2] + bounds[3]),
        0.5 * (bounds[4] + bounds[5]),
    ], dtype=float)

def _diag_of_bounds(bounds) -> float:
    p0 = np.array([bounds[0], bounds[2], bounds[4]], dtype=float)
    p1 = np.array([bounds[1], bounds[3], bounds[5]], dtype=float)
    return float(np.linalg.norm(p1 - p0))

def _rpy_deg_to_matrix_np(rpy_deg):
    import math
    r, p, y = [math.radians(float(v)) for v in (rpy_deg or (0.0, 0.0, 0.0))]
    Rx = np.array([[1, 0, 0],
                   [0, math.cos(r), -math.sin(r)],
                   [0, math.sin(r),  math.cos(r)]])
    Ry = np.array([[ math.cos(p), 0, math.sin(p)],
                   [0,            1, 0           ],
                   [-math.sin(p), 0, math.cos(p)]])
    Rz = np.array([[math.cos(y), -math.sin(y), 0],
                   [math.sin(y),  math.cos(y), 0],
                   [0,            0,           1]])
    return Rz @ Ry @ Rx

def _side_plane_from_bounds(bounds, side: str, path_offset_mm: float, clearance_mm: float) -> Tuple[int, float]:
    """
    Bestimmt (axis, value) für die Start-Ebene der blauen Maske abhängig von der Side.
    axis: 0=x, 1=y, 2=z. value ist der Ebenenwert in Weltkoordinaten (mm).
    """
    x_min, x_max, y_min, y_max, z_min, z_max = bounds
    s = (side or "").lower()

    if s == "top":
        return 2, float(z_max + path_offset_mm + clearance_mm)
    if s == "front":
        return 1, float(y_max + path_offset_mm + clearance_mm)
    if s == "back":
        return 1, float(y_min - path_offset_mm - clearance_mm)
    if s == "left":
        return 0, float(x_min - path_offset_mm - clearance_mm)
    if s == "right":
        return 0, float(x_max + path_offset_mm + clearance_mm)
    return 2, float(z_max + path_offset_mm + clearance_mm)

def _segments_polydata(P0: np.ndarray, P1: np.ndarray) -> pv.PolyData:
    assert P0.shape == P1.shape and P0.shape[1] == 3
    N = P0.shape[0]
    if N == 0:
        return pv.PolyData()
    all_pts = np.empty((2 * N, 3), dtype=float)
    all_pts[0::2] = P0
    all_pts[1::2] = P1
    lines = np.empty((N, 3), dtype=np.int64)
    lines[:, 0] = 2
    lines[:, 1] = np.arange(0, 2 * N, 2, dtype=np.int64)
    lines[:, 2] = np.arange(1, 2 * N, 2, dtype=np.int64)
    poly = pv.PolyData(all_pts)
    poly.lines = lines.reshape(-1)
    return poly

def _frames_triad_polydata(P_mm: np.ndarray, Q_xyzw: np.ndarray, scale_mm: float) -> Tuple[pv.PolyData, pv.PolyData, pv.PolyData]:
    """
    Baut drei PolyData-Objekte (X/Y/Z-Achsen) als kleine Triads an jeder Pose.
    P_mm:   (N,3) Punkte in mm
    Q_xyzw: (N,4) Quaternionen (x,y,z,w)
    scale_mm: Achsenlänge in mm
    """
    if P_mm.size == 0 or Q_xyzw.size == 0:
        return pv.PolyData(), pv.PolyData(), pv.PolyData()

    N = P_mm.shape[0]
    ex = np.array([1.0, 0.0, 0.0])
    ey = np.array([0.0, 1.0, 0.0])
    ez = np.array([0.0, 0.0, 1.0])

    ends_x = np.empty_like(P_mm)
    ends_y = np.empty_like(P_mm)
    ends_z = np.empty_like(P_mm)

    for i in range(N):
        x, y, z, w = Q_xyzw[i]
        Rm = np.array([
            [1 - 2 * (y*y + z*z), 2 * (x*y - z*w),     2 * (x*z + y*w)],
            [2 * (x*y + z*w),     1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
            [2 * (x*z - y*w),     2 * (y*z + x*w),     1 - 2 * (x*x + y*y)],
        ], dtype=float)
        ax = Rm @ ex
        ay = Rm @ ey
        az = Rm @ ez
        ends_x[i] = P_mm[i] + ax * scale_mm
        ends_y[i] = P_mm[i] + ay * scale_mm
        ends_z[i] = P_mm[i] + az * scale_mm

    x_pd = _segments_polydata(P_mm, ends_x)
    y_pd = _segments_polydata(P_mm, ends_y)
    z_pd = _segments_polydata(P_mm, ends_z)
    return x_pd, y_pd, z_pd

# ---------- Orientation / Export helpers ----------
def _quat_from_two_vectors(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    a = a / (np.linalg.norm(a) + 1e-12)
    b = b / (np.linalg.norm(b) + 1e-12)
    v = np.cross(a, b)
    w = 1.0 + float(np.dot(a, b))
    if w < 1e-8:
        axis = np.array([1.0, 0.0, 0.0])
        if abs(a[0]) > 0.9:
            axis = np.array([0.0, 1.0, 0.0])
        v = np.cross(a, axis)
        v = v / (np.linalg.norm(v) + 1e-12)
        return np.array([v[0], v[1], v[2], 0.0], dtype=float)
    q = np.array([v[0], v[1], v[2], w], dtype=float)
    q /= (np.linalg.norm(q) + 1e-12)
    return q  # xyzw

def _quat_mul(q1_xyzw: np.ndarray, q2_xyzw: np.ndarray) -> np.ndarray:
    x1, y1, z1, w1 = q1_xyzw
    x2, y2, z2, w2 = q2_xyzw
    x = w1*x2 + w2*x1 + (y1*z2 - z1*y2)
    y = w1*y2 + w2*y1 + (z1*x2 - x1*z2)
    z = w1*z2 + w2*z1 + (x1*y2 - y1*x2)
    w = w1*w2 - (x1*x2 + y1*y2 + z1*z2)
    out = np.array([x, y, z, w], dtype=float)
    out /= (np.linalg.norm(out) + 1e-12)
    return out

def _poses_from_hits_and_normals(P_hit_mm: np.ndarray, N_world: np.ndarray, stand_off_mm: float):
    N = len(P_hit_mm)
    P_tcp_mm = np.empty_like(P_hit_mm)
    Q_xyzw   = np.empty((N, 4), dtype=float)
    zminus = np.array([0.0, 0.0, -1.0], dtype=float)
    for i in range(N):
        n = N_world[i] / (np.linalg.norm(N_world[i]) + 1e-12)
        P_tcp_mm[i] = P_hit_mm[i] + n * float(stand_off_mm)
        Q_xyzw[i]   = _quat_from_two_vectors(zminus, n)
    return P_tcp_mm, Q_xyzw

def _to_ros_pose_list_mm(P_mm: np.ndarray, Q_xyzw: np.ndarray):
    P_m = P_mm / 1000.0
    out = []
    for p, q in zip(P_m, Q_xyzw):
        out.append({
            "position": {"x": float(p[0]), "y": float(p[1]), "z": float(p[2])},
            "orientation": {"x": float(q[0]), "y": float(q[1]), "z": float(q[2]), "w": float(q[3])},
        })
    return out
