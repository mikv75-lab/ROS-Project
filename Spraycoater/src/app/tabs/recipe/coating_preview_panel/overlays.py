# -*- coding: utf-8 -*-
# app/tabs/recipe/coating_preview_panel/overlays.py
from __future__ import annotations
import logging
from typing import Any, Callable, Dict, Optional, List, Tuple

import numpy as np
import pyvista as pv

# PathBuilder wird für den Legacy-Aufbau genutzt (strict).
try:
    from app.model.recipe.path_builder import PathBuilder
except Exception:  # pragma: no cover
    PathBuilder = None  # type: ignore

from .raycast_projector import cast_rays_for_side

_LOG = logging.getLogger("app.tabs.recipe.preview.overlays")

# -------- Helpers & Mathe ----------------------------------------------------
def _safe_norm(v: np.ndarray, eps: float = 1e-9) -> np.ndarray:
    v = np.asarray(v, dtype=float).reshape(3)
    n = float(np.linalg.norm(v))
    if n < eps:
        return np.array([0.0, 0.0, 1.0], dtype=float)
    return v / n

def _orthonormal_basis_from_z(z: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    z = _safe_norm(z)
    h = np.array([1.0, 0.0, 0.0], dtype=float) if abs(z[0]) < 0.9 else np.array([0.0, 1.0, 0.0], dtype=float)
    x = _safe_norm(np.cross(h, z))
    y = _safe_norm(np.cross(z, x))
    return x, y, z

def _orthonormal_basis_from_zx(z: np.ndarray, x_hint: np.ndarray | None) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    z = _safe_norm(z)
    if x_hint is None:
        return _orthonormal_basis_from_z(z)
    x = _safe_norm(x_hint)
    x = _safe_norm(np.cross(np.cross(x, z), z))
    y = _safe_norm(np.cross(z, x))
    return x, y, z

def _rotmat_to_rpy_xyz(R: np.ndarray) -> tuple[float, float, float]:
    R = np.asarray(R, dtype=float).reshape(3, 3)
    eps = 1e-9
    sy = -R[2, 0]
    sy = np.clip(sy, -1.0, 1.0)
    ry = np.arcsin(sy)
    cy = np.cos(ry)
    if abs(cy) > eps:
        rx = np.arctan2(R[2, 1] / cy, R[2, 2] / cy)
        rz = np.arctan2(R[1, 0] / cy, R[0, 0] / cy)
    else:
        rx = 0.0
        rz = np.arctan2(-R[0, 1], R[1, 1])
    return float(np.degrees(rx)), float(np.degrees(ry)), float(np.degrees(rz))

def _tangents_from_path(P: np.ndarray) -> np.ndarray:
    P = np.asarray(P, dtype=float).reshape(-1, 3)
    n = P.shape[0]
    if n == 0:
        return np.zeros((0, 3), dtype=float)
    if n == 1:
        return np.array([[1.0, 0.0, 0.0]], dtype=float)
    T = np.empty((n, 3), dtype=float)
    T[0]  = P[1]  - P[0]
    T[-1] = P[-1] - P[-2]
    if n > 2:
        T[1:-1] = P[2:] - P[:-2]
    for i in range(n):
        nrm = np.linalg.norm(T[i])
        T[i] = (T[i] / nrm) if nrm > 1e-12 else (T[i-1] if i else np.array([1.0, 0.0, 0.0]))
    for i in range(1, n):
        if np.dot(T[i-1], T[i]) < 0.0:
            T[i] = -T[i]
    return T

def _make_yaml_hits_with_defaults(
    points_hits: Optional[np.ndarray],
    z_dirs_hits: Optional[np.ndarray],
    x_dirs_hits: Optional[np.ndarray],
    *,
    default_rpy: tuple[float, float, float] = (0.0, 0.0, -90.0)
) -> Optional[str]:
    if points_hits is None or z_dirs_hits is None:
        return None
    O = np.asarray(points_hits, float).reshape(-1, 3)
    Z = np.asarray(z_dirs_hits, float).reshape(-1, 3)
    if O.shape != Z.shape or O.shape[0] == 0:
        return None

    X = None
    if x_dirs_hits is not None:
        X = np.asarray(x_dirs_hits, float).reshape(-1, 3)
        if X.shape != O.shape:
            X = None

    lines = ["points:"]
    for i in range(O.shape[0]):
        z = Z[i]
        if not np.all(np.isfinite(z)) or np.linalg.norm(z) < 1e-9:
            rx, ry, rz = default_rpy
        else:
            x_hint = X[i] if X is not None else None
            x, y, z = _orthonormal_basis_from_zx(z, x_hint)
            R = np.column_stack([x, y, z])
            rx, ry, rz = _rotmat_to_rpy_xyz(R)
        lines.append(
            "  - {x: %.6f, y: %.6f, z: %.6f, rx: %.6f, ry: %.6f, rz: %.6f}" % (
                float(O[i, 0]), float(O[i, 1]), float(O[i, 2]),
                float(rx), float(ry), float(rz)
            )
        )
    return "\n".join(lines) + "\n"

# -------- Strict-Helper für Globals / SampleStep / MaxPoints -----------------
def _strict_globals_from_model(model) -> dict:
    g = dict(getattr(model, "parameters", {}) or {})
    required = [
        "stand_off_mm",
        "max_angle_deg",
        "predispense.angle_deg",
        "predispense.distance_mm",
        "retreat.angle_deg",
        "retreat.distance_mm",
    ]
    missing = [k for k in required if k not in g]
    if missing:
        raise ValueError(f"Preview: Missing required globals in model.parameters: {missing} (kein Fallback).")
    return {k: g[k] for k in required}

def _strict_sample_step_from_model(model) -> float:
    g = dict(getattr(model, "parameters", {}) or {})
    if "sample_step_mm" not in g:
        raise ValueError("Preview: parameters.sample_step_mm fehlt (kein Fallback).")
    return float(g["sample_step_mm"])

def _strict_max_points_from_model(model) -> int:
    g = dict(getattr(model, "parameters", {}) or {})
    if "max_points" not in g:
        raise ValueError("Preview: parameters.max_points fehlt (kein Fallback).")
    return int(g["max_points"])

# -------- Neu: Side-Frame & Einbettung ---------------------------------------
def _side_frame(bounds: Tuple[float, float, float, float, float, float]):
    xmin, xmax, ymin, ymax, zmin, zmax = bounds
    cx = 0.5 * (xmin + xmax)
    cy = 0.5 * (ymin + ymax)
    cz = 0.5 * (zmin + zmax)
    cfgs = {
        "top":   {"normal": np.array([0,  0, -1.0]), "anchor": ("z", "max"), "axes": ("x", "y")},
        "front": {"normal": np.array([0, +1,  0.0]), "anchor": ("y", "min"), "axes": ("x", "z")},
        "back":  {"normal": np.array([0, -1,  0.0]), "anchor": ("y", "max"), "axes": ("x", "z")},
        "left":  {"normal": np.array([+1, 0,  0.0]), "anchor": ("x", "min"), "axes": ("y", "z")},
        "right": {"normal": np.array([-1, 0,  0.0]), "anchor": ("x", "max"), "axes": ("y", "z")},
    }
    return cfgs, (cx, cy, cz)

def _embed_path_on_face(P0_local: np.ndarray, side: str, bounds) -> np.ndarray:
    (cfgs, (cx, cy, cz)) = _side_frame(bounds)
    cfg = cfgs.get(side, cfgs["top"])
    ax0, ax1 = cfg["axes"]
    anchor_axis, anchor_side = cfg["anchor"]
    xmin, xmax, ymin, ymax, zmin, zmax = bounds
    anchor_val = {"x": (xmin if anchor_side == "min" else xmax),
                  "y": (ymin if anchor_side == "min" else ymax),
                  "z": (zmin if anchor_side == "min" else zmax)}[anchor_axis]

    P0 = np.asarray(P0_local, float).reshape(-1, 3)
    out = np.empty_like(P0)

    def place(axis_name: str, arr: np.ndarray):
        if axis_name == "x":
            return cx + arr
        if axis_name == "y":
            return cy + arr
        if axis_name == "z":
            return cz + arr
        return arr

    a0 = P0[:, 0]
    a1 = P0[:, 1]

    axes = {"x": None, "y": None, "z": None}
    axes[ax0] = place(ax0, a0)
    axes[ax1] = place(ax1, a1)
    axes[anchor_axis] = np.full(P0.shape[0], anchor_val, dtype=float)

    out[:, 0] = axes["x"]
    out[:, 1] = axes["y"]
    out[:, 2] = axes["z"]
    return out

# -------- OverlayRenderer -----------------------------------------------------
class OverlayRenderer:
    def __init__(
        self,
        *,
        add_mesh_fn: Callable[..., Any],
        clear_layer_fn: Callable[[str], None],
        add_path_polyline_fn: Callable[..., Any],
        show_poly_fn: Callable[..., Any],
        show_frames_at_fn: Callable[..., None],
        set_layer_visible_fn: Callable[[str, bool, bool], None] | Callable[[str, bool], None],
        update_2d_scene_fn: Callable[[Optional[pv.PolyData], Optional[np.ndarray], Optional[pv.PolyData]], None],
        layers: Dict[str, str],
        get_bounds: Callable[[], Any],
        yaml_out_fn: Optional[Callable[[str], None]] = None,
    ):
        self._add_mesh = add_mesh_fn
        self._clear_layer = clear_layer_fn
        self._add_path_polyline = add_path_polyline_fn
        self._show_poly = show_poly_fn
        self._show_frames_at = show_frames_at_fn
        self._set_layer_visible = set_layer_visible_fn
        self._update_2d = update_2d_scene_fn
        self._layers = dict(layers or {})
        self._get_bounds = get_bounds
        self._yaml_out = yaml_out_fn

        self._out: Dict[str, Any] = {}
        self._vis = {"mask": False, "path": True, "hits": False, "misses": False, "normals": False, "frames": False}

    # ---- Sichtbarkeit ----
    def _L(self, name: str) -> str:
        return self._layers.get(name, name)

    def _set_vis_layer(self, layer: str, vis: bool, *, render: bool):
        try:
            self._set_layer_visible(layer, vis, render)  # type: ignore[misc]
        except TypeError:
            self._set_layer_visible(layer, vis)          # type: ignore[misc]

    def set_mask_visible(self, vis: bool, *, render: bool = True):
        self._vis["mask"] = bool(vis)
        self._set_vis_layer(self._L("mask"), vis, render=render)
        self._set_vis_layer(self._L("mask_mrk"), vis, render=render)

    def set_path_visible(self, vis: bool, *, render: bool = True):
        self._vis["path"] = bool(vis)
        self._set_vis_layer(self._L("path"), vis, render=render)
        self._set_vis_layer(self._L("path_mrk"), vis, render=render)

    def set_hits_visible(self, vis: bool, *, render: bool = True):
        self._vis["hits"] = bool(vis)
        self._set_vis_layer(self._L("rays_hit"), vis, render=render)

    def set_misses_visible(self, vis: bool, *, render: bool = True):
        self._vis["misses"] = bool(vis)
        self._set_vis_layer(self._L("rays_miss"), vis, render=render)

    def set_normals_visible(self, vis: bool, *, render: bool = True):
        self._vis["normals"] = bool(vis)
        self._set_vis_layer(self._L("normals"), vis, render=render)

    def set_frames_visible(self, vis: bool, *, render: bool = True):
        self._vis["frames"] = bool(vis)
        for lyr in ("frames_x", "frames_y", "frames_z", "frames_labels"):
            self._set_vis_layer(self._L(lyr), vis, render=render)

    def apply_visibility(self, vis: dict | None):
        if vis is not None:
            self._vis.update({
                "mask":    bool(vis.get("mask",    self._vis["mask"])),
                "path":    bool(vis.get("path",    self._vis["path"])),
                "hits":    bool(vis.get("hits",    self._vis["hits"])),
                "misses":  bool(vis.get("misses",  self._vis["misses"])),
                "normals": bool(vis.get("normals", self._vis["normals"])),
                "frames":  bool(vis.get("frames",  self._vis["frames"])),
            })
        self.set_path_visible(self._vis["path"], render=False)
        self.set_mask_visible(self._vis["mask"], render=False)
        self.set_hits_visible(self._vis["hits"], render=False)
        self.set_misses_visible(self._vis["misses"], render=False)
        self.set_normals_visible(self._vis["normals"], render=False)
        self.set_frames_visible(self._vis["frames"], render=True)

    # ---- Neuer Weg: vor-kompilierte Pfade zeichnen ----------------------
    def render_compiled(
        self,
        *,
        substrate_mesh: pv.PolyData,
        compiled: dict,                 # output von Recipe.compile_paths()
        visibility: Optional[Dict[str, bool]] = None,
        mask_lift_mm: float = 50.0,
        default_stand_off_mm: float = 10.0,
        ray_len_mm: float = 1000.0,
        sides: Optional[List[str]] = None,   # Seitenfilter
    ) -> Dict[str, Dict[str, Any]]:
        pbs_all = dict(compiled.get("paths_by_side") or {})
        # Seitenfilter anwenden (nur gecheckte)
        if sides:
            side_set = {str(s) for s in sides}
            pbs = {s: rec for s, rec in pbs_all.items() if s in side_set}
        else:
            pbs = pbs_all

        for ly in ("mask", "mask_markers", "rays_hit", "rays_miss", "normals", "path", "path_markers",
                   "frames_x", "frames_y", "frames_z", "frames_labels"):
            self._clear_layer(self._L(ly))

        bounds = substrate_mesh.bounds
        (cfgs, _) = _side_frame(bounds)

        outputs_by_side: Dict[str, Dict[str, Any]] = {}

        for side, rec in pbs.items():
            pts = np.asarray(rec.get("points_mm", []), dtype=float).reshape(-1, 3)
            if pts.size == 0:
                outputs_by_side[side] = {}
                continue

            cfg = cfgs.get(side, cfgs["top"])
            normal = cfg["normal"]

            P_face = _embed_path_on_face(pts, side, bounds)

            try:
                nrm = normal / (np.linalg.norm(normal) + 1e-12)
                P_mask = P_face - float(mask_lift_mm) * nrm
                mask_poly = pv.lines_from_points(P_mask, close=False)
            except Exception:
                P_mask = None
                mask_poly = None

            tcp_points = None
            tcp_poly = None
            rays_hit_poly = None
            valid_mask = None
            z_dirs = None
            x_dirs = None
            try:
                stand_off = float(default_stand_off_mm)
                rc, rays_hit_poly, tcp_poly = cast_rays_for_side(
                    P_world_start=P_mask,
                    sub_mesh_world=substrate_mesh,
                    side=side,
                    source=str(rec.get("type") or "points"),
                    stand_off_mm=stand_off,
                    ray_len_mm=float(ray_len_mm),
                    start_lift_mm=0.0,
                    flip_normals_to_face_rays=True,
                    invert_dirs=False,
                    lock_xy=True,
                )
                valid_mask = getattr(rc, "valid", None)

                if rc is not None and getattr(rc, "tcp_mm", None) is not None:
                    tcp_all = rc.tcp_mm
                    tcp_points = tcp_all[valid_mask] if (valid_mask is not None and np.any(valid_mask)) else tcp_all

                z_full = getattr(rc, "refl_dir", None)
                x_full = getattr(rc, "x_dir", None) or getattr(rc, "frame_x", None)

                if z_full is not None:
                    ZF = np.asarray(z_full, dtype=float).reshape(-1, 3)
                    z_dirs = ZF[valid_mask] if (valid_mask is not None and len(ZF) == len(valid_mask)) else ZF

                if x_full is not None:
                    XF = np.asarray(x_full, dtype=float).reshape(-1, 3)
                    x_dirs = XF[valid_mask] if (valid_mask is not None and len(XF) == len(valid_mask)) else XF

            except Exception:
                _LOG.exception("cast_rays_for_side failed (%s)", side)

            if tcp_points is not None and (x_dirs is None or len(x_dirs) != len(tcp_points)):
                try:
                    T = _tangents_from_path(tcp_points)
                    if z_dirs is not None and len(z_dirs) == len(T):
                        Xo = np.cross(np.cross(T, z_dirs), z_dirs)
                        for i in range(Xo.shape[0]):
                            nrm = np.linalg.norm(Xo[i])
                            Xo[i] = (Xo[i] / nrm) if nrm > 1e-12 else _orthonormal_basis_from_z(z_dirs[i])[0]
                        for i in range(1, Xo.shape[0]):
                            if np.dot(Xo[i-1], Xo[i]) < 0.0:
                                Xo[i] = -Xo[i]
                        x_dirs = Xo
                except Exception:
                    _LOG.exception("Fallback X via tangents failed (%s)", side)

            rays_miss = None
            try:
                xmin, xmax, ymin, ymax, zmin, zmax = bounds
                cfg = cfgs.get(side, cfgs["top"])
                anchor_axis, anchor_side = cfg["anchor"]
                anchor_val = {"x": (xmin if anchor_side == "min" else xmax),
                              "y": (ymin if anchor_side == "min" else ymax),
                              "z": (zmin if anchor_side == "min" else zmax)}[anchor_axis]
                axis_idx = {"x": 0, "y": 1, "z": 2}[anchor_axis]
                if valid_mask is not None and P_mask is not None and len(P_mask) == len(valid_mask) and np.any(~valid_mask):
                    base_dir = cfg["normal"]
                    Pmiss_start = P_mask[~valid_mask]
                    D = np.tile(base_dir / (np.linalg.norm(base_dir) + 1e-12), (len(Pmiss_start), 1))
                    have_comp = np.abs(D[:, axis_idx]) > 1e-9
                    if np.any(have_comp):
                        S = Pmiss_start[have_comp]; Dn = D[have_comp]
                        t = (anchor_val - S[:, axis_idx]) / Dn[:, axis_idx]
                        pos = t > 0
                        S = S[pos]; Dn = Dn[pos]; t = t[pos]
                        if len(S):
                            Pend = S + Dn * t.reshape(-1, 1)
                            pts = np.vstack([S, Pend])
                            nseg = len(S)
                            lines = np.empty((nseg, 3), dtype=np.int64)
                            lines[:, 0] = 2
                            lines[:, 1] = np.arange(nseg, dtype=np.int64)
                            lines[:, 2] = lines[:, 1] + nseg
                            rays_miss = pv.PolyData(pts)
                            rays_miss.lines = lines.reshape(-1)
            except Exception:
                _LOG.exception("miss rays build failed (%s)", side)

            if mask_poly is not None:
                self._add_mesh(mask_poly, color="royalblue", layer=self._L("mask"),
                               line_width=2.0, lighting=False, render=False, reset_camera=False)

            if getattr(rays_hit_poly, "n_lines", 0):
                self._add_mesh(rays_hit_poly, color="#85C1E9", layer=self._L("rays_hit"),
                               line_width=1.5, lighting=False, render=False, reset_camera=False)

            if rays_miss is not None:
                self._add_mesh(rays_miss, color="#e74c3c", layer=self._L("rays_miss"),
                               line_width=1.2, lighting=False, render=False, reset_camera=False)

            if getattr(tcp_poly, "n_lines", 0):
                self._add_mesh(tcp_poly, color="#f1c40f", layer=self._L("normals"),
                               line_width=1.3, lighting=False, render=False, reset_camera=False)

            if tcp_points is not None and len(tcp_points):
                self._add_path_polyline(tcp_points, layer=self._L("path"), color="#2ecc71",
                                        line_width=2.0, lighting=False, render=False, reset_camera=False)
                try:
                    pts_s = tcp_points[::max(1, int(round(max(1, tcp_points.shape[0] // 200))))]
                    self._add_mesh(pv.PolyData(pts_s), color="#2ecc71", layer=self._L("path_mrk"),
                                   point_size=8.0, lighting=False, render=False, reset_camera=False)
                except Exception:
                    _LOG.exception("path markers failed (%s)", side)

            outputs_by_side[side] = {
                "tcp_points": tcp_points,
                "mask_poly": mask_poly,
                "z_dirs": z_dirs,
                "x_dirs": x_dirs,
                "valid_mask": valid_mask,
            }

        # Sichtbarkeit anwenden
        self.apply_visibility(visibility)

        # ---- 2D-Ansicht versorgen (erste Side mit Pfad nehmen) ----
        try:
            tcp_for_2d: Optional[np.ndarray] = None
            mask_for_2d: Optional[pv.PolyData] = None
            for s in pbs.keys():
                out = outputs_by_side.get(s) or {}
                t = out.get("tcp_points")
                if t is not None and len(t):
                    tcp_for_2d = t
                    mask_for_2d = out.get("mask_poly")
                    break
            # substrate_mesh, tcp_points, mask_poly
            self._update_2d(substrate_mesh, tcp_for_2d, mask_for_2d)
        except Exception:
            _LOG.exception("update_2d (compiled) failed")

        return outputs_by_side

    # ---- Strict Legacy-Aufbau (wenn keine Punkte im Modell) --------------
    def render_from_model(
        self,
        *,
        model: object,
        substrate_mesh: pv.PolyData,
        side: Optional[str] = None,
        sides: Optional[List[str]] = None,
        default_stand_off_mm: float = 10.0,
        mask_lift_mm: float = 50.0,
        ray_len_mm: float = 1000.0,
        visibility: Optional[Dict[str, bool]] = None,
    ) -> Dict[str, Dict[str, Any]]:
        # 1) Side-Liste
        if sides and len(sides) > 0:
            side_list = list(dict.fromkeys([str(s) for s in sides if str(s).strip()]))
        elif side and str(side).strip():
            side_list = [str(side).strip()]
        else:
            if isinstance(model, dict):
                pbs = dict(model.get("paths_by_side") or {})
            else:
                pbs = dict(getattr(model, "paths_by_side", {}) or {})
            side_list = list(pbs.keys()) if pbs else ["top"]

        # 2) Wenn bereits Punkte vorhanden → direkt rendern
        def _has_points(m) -> bool:
            if not m:
                return False
            pbs = dict(m.get("paths_by_side") or {})
            for _, rec in pbs.items():
                if isinstance(rec, dict) and rec.get("points_mm") is not None:
                    return True
            return False

        if isinstance(model, dict) and _has_points(model):
            return self.render_compiled(
                substrate_mesh=substrate_mesh,
                compiled=model,
                visibility=visibility,
                mask_lift_mm=mask_lift_mm,
                default_stand_off_mm=default_stand_off_mm,
                ray_len_mm=ray_len_mm,
                sides=side_list,  # Seitenfilter durchreichen
            )

        # 3) Strikter Pfadaufbau via PathBuilder (kein Fallback)
        if PathBuilder is None:
            raise RuntimeError("PathBuilder unavailable and no points_mm in model (strict mode).")

        if isinstance(model, dict):
            params = dict(model.get("parameters") or {})
        else:
            params = dict(getattr(model, "parameters", {}) or {})

        globals_params = _strict_globals_from_model(model)
        sample_step = _strict_sample_step_from_model(model)  # STRICT global
        max_points = _strict_max_points_from_model(model)

        compiled = {"paths_by_side": {}}
        for s in side_list:
            # Typ-Info als rein kosmetische Meta (falls vorhanden)
            if isinstance(model, dict):
                pdef = dict((model.get("paths_by_side") or {}).get(s) or {})
            else:
                pdef = dict((getattr(model, "paths_by_side", {}) or {}).get(s) or {})

            try:
                pd = PathBuilder.from_side(
                    model,
                    side=s,
                    globals_params=globals_params,
                    sample_step_mm=sample_step,  # global
                    max_points=max_points,
                )
                compiled["paths_by_side"][s] = {
                    "type": (pdef.get("type") or pd.meta.get("source") or "points"),
                    "points_mm": np.asarray(pd.points_mm, float).reshape(-1, 3).tolist(),
                    "meta": dict(pd.meta or {}),
                }
            except Exception:
                _LOG.exception("PathBuilder.from_side failed (%s)", s)
                compiled["paths_by_side"][s] = {"type": pdef.get("type") or "points", "points_mm": [], "meta": {}}

        out = self.render_compiled(
            substrate_mesh=substrate_mesh,
            compiled=compiled,
            visibility=visibility,
            mask_lift_mm=mask_lift_mm,
            default_stand_off_mm=default_stand_off_mm,
            ray_len_mm=ray_len_mm,
            sides=side_list,  # Seitenfilter durchreichen
        )
        return out
