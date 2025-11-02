# -*- coding: utf-8 -*-
from __future__ import annotations
import logging
from typing import Any, Callable, Dict, Optional, Tuple

import numpy as np
import pyvista as pv

from .path_builder import PathBuilder
from .raycast_projector import cast_rays_for_side

_LOG = logging.getLogger("app.tabs.recipe.preview.overlays")


class OverlayRenderer:
    """
    Rechnet immer alle Overlays (Maske, Rays, Misses, Normals, TCP/Path, Frames),
    zeigt aber nur die Overlays an, deren Checkbox aktiv ist.
    Ergebnisse werden im Puffer gehalten und sind via get_outputs() abrufbar.
    Beim Einschalten per Checkbox wird – falls noch kein Actor existiert – aus dem Cache nachgezeichnet.
    """

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

        # Zwischenspeicher (Outputs für Downstream)
        self._out: Dict[str, Any] = {
            "substrate_mesh": None,
            "mask_poly": None,
            "rays_hit_poly": None,
            "rays_miss_poly": None,
            "tcp_poly": None,
            "tcp_points": None,     # (N,3)
            "valid_mask": None,     # bool mask
            "rc": None,             # RaycastResult
            "side": None,
            "stand_off_mm": None,
        }

        # Sichtbarkeit (letzter Zustand)
        self._vis = {
            "mask": False,
            "path": True,
            "hits": False,
            "misses": False,
            "normals": False,
            "frames": False,
        }

    # -------- util --------
    @staticmethod
    def _on(vis: Optional[Dict[str, bool]] | None, key: str, default: bool = False) -> bool:
        if not vis:
            return default
        try:
            return bool(vis.get(key, default))
        except Exception:
            return default

    def _L(self, name: str) -> str:
        return self._layers.get(name, name)

    def _set_vis_layer(self, layer: str, vis: bool, *, render: bool):
        try:
            self._set_layer_visible(layer, vis, render)  # type: ignore[misc]
        except TypeError:
            self._set_layer_visible(layer, vis)          # type: ignore[misc]

    # -------- public outputs --------
    def get_outputs(self) -> Dict[str, Any]:
        return self._out

    # ===== visibility toggles (zeichnen beim Einschalten nach) =====
    def set_mask_visible(self, vis: bool, *, render: bool = True):
        self._vis["mask"] = bool(vis)
        layer = self._L("mask")
        if vis:
            poly = self._out.get("mask_poly")
            if poly is not None:
                # neu zeichnen (idempotent: clear_layer vor add)
                self._clear_layer(layer)
                self._add_mesh(poly, color="royalblue", layer=layer,
                               line_width=2.0, lighting=False, render=False, reset_camera=False)
        else:
            self._clear_layer(layer)
        self._set_vis_layer(layer, vis, render=render)

    def set_hits_visible(self, vis: bool, *, render: bool = True):
        self._vis["hits"] = bool(vis)
        layer = self._L("rays_hit")
        if vis:
            poly = self._out.get("rays_hit_poly")
            if getattr(poly, "n_lines", 0):
                self._clear_layer(layer)
                self._add_mesh(poly, color="#85C1E9", layer=layer,
                               line_width=1.5, lighting=False, render=False, reset_camera=False)
        else:
            self._clear_layer(layer)
        self._set_vis_layer(layer, vis, render=render)

    def set_misses_visible(self, vis: bool, *, render: bool = True):
        self._vis["misses"] = bool(vis)
        layer = self._L("rays_miss")
        if vis:
            poly = self._out.get("rays_miss_poly")
            if poly is not None:
                self._clear_layer(layer)
                self._add_mesh(poly, color="#e74c3c", layer=layer,
                               line_width=1.2, lighting=False, render=False, reset_camera=False)
        else:
            self._clear_layer(layer)
        self._set_vis_layer(layer, vis, render=render)

    def set_normals_visible(self, vis: bool, *, render: bool = True):
        self._vis["normals"] = bool(vis)
        layer = self._L("normals")
        if vis:
            poly = self._out.get("tcp_poly")
            if getattr(poly, "n_lines", 0):
                self._clear_layer(layer)
                self._add_mesh(poly, color="#f1c40f", layer=layer,
                               line_width=1.3, lighting=False, render=False, reset_camera=False)
        else:
            self._clear_layer(layer)
        self._set_vis_layer(layer, vis, render=render)

    def set_path_visible(self, vis: bool, *, render: bool = True):
        self._vis["path"] = bool(vis)
        layer = self._L("path")
        if vis:
            pts = self._out.get("tcp_points")
            if pts is not None and len(pts):
                self._clear_layer(layer)
                # Nur Linie – keine Marker mehr
                self._add_path_polyline(pts, layer=layer, color="#2ecc71",
                                        line_width=2.0, lighting=False, render=False, reset_camera=False)
        else:
            self._clear_layer(layer)
            self._clear_layer(self._L("path_mrk"))  # sicherheitshalber, auch wenn wir keine Marker mehr setzen
        self._set_vis_layer(layer, vis, render=render)

    def set_frames_visible(self, vis: bool, *, render: bool = True):
        self._vis["frames"] = bool(vis)
        if vis:
            P = self._out.get("tcp_points")
            rc = self._out.get("rc")
            Z = self._extract_z_dirs(rc)
            P_aligned, Z_aligned = self._align_tcp_and_dirs(rc, P, Z)
            if P_aligned is not None and Z_aligned is not None and len(P_aligned) and len(Z_aligned):
                try:
                    self._show_frames_at(
                        origins=P_aligned,
                        z_dirs=Z_aligned,
                        scale_mm=None,      # Auto-Scale im Panel
                        line_width=1.0,
                        clear_old=True,
                        add_labels=False,
                    )
                except Exception:
                    _LOG.exception("show_frames_at (toggle) failed")
            # Sichtbar schalten (falls eben gezeichnet)
            for lyr in ("frames_x", "frames_y", "frames_z", "frames_labels"):
                self._set_vis_layer(self._L(lyr), True, render=False)
        else:
            for lyr in ("frames_x", "frames_y", "frames_z", "frames_labels"):
                self._clear_layer(self._L(lyr))
        if render:
            pass

    def apply_visibility(self, vis: Dict[str, bool] | None):
        if vis:
            self._vis.update({
                "mask":    bool(vis.get("mask",    self._vis["mask"])),
                "path":    bool(vis.get("path",    self._vis["path"])),
                "hits":    bool(vis.get("hits",    self._vis["hits"])),
                "misses":  bool(vis.get("misses",  self._vis["misses"])),
                "normals": bool(vis.get("normals", self._vis["normals"])),
                "frames":  bool(vis.get("frames",  self._vis["frames"])),
            })
        # Reihenfolge: zusammengehörige Layer ohne Zwischenrender setzen und am Ende rendern
        self.set_mask_visible(self._vis["mask"], render=False)
        self.set_path_visible(self._vis["path"], render=False)
        self.set_hits_visible(self._vis["hits"], render=False)
        self.set_misses_visible(self._vis["misses"], render=False)
        self.set_normals_visible(self._vis["normals"], render=False)
        self.set_frames_visible(self._vis["frames"], render=True)

    # -------- helper: z_dirs robust extrahieren & alignen --------
    def _extract_z_dirs(self, rc) -> Optional[np.ndarray]:
        if rc is None:
            return None
        for name in ("refl_dir", "hit_normals", "normals_world", "n_hat", "normals"):
            z = getattr(rc, name, None)
            if z is not None:
                z = np.asarray(z, dtype=float)
                if z.ndim == 2 and z.shape[1] == 3 and len(z):
                    return z
        return None

    def _align_tcp_and_dirs(
        self, rc, tcp_points: Optional[np.ndarray], z_dirs: Optional[np.ndarray]
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        if tcp_points is None or z_dirs is None:
            return None, None
        P = np.asarray(tcp_points, dtype=float).reshape(-1, 3)
        Z = np.asarray(z_dirs, dtype=float).reshape(-1, 3)

        if rc is not None:
            valid = getattr(rc, "valid", None)
            tcp_all = getattr(rc, "tcp_mm", None)
            if valid is not None and tcp_all is not None:
                valid = np.asarray(valid, dtype=bool).reshape(-1)
                tcp_all = np.asarray(tcp_all, dtype=float).reshape(-1, 3)
                if len(Z) == len(tcp_all):
                    Z = Z[valid] if len(valid) == len(tcp_all) else Z
                if len(P) != len(Z):
                    if len(valid) == len(tcp_all):
                        Z = Z[: np.count_nonzero(valid)]
                    else:
                        n = min(len(P), len(Z))
                        if n == 0:
                            return None, None
                        P, Z = P[:n], Z[:n]
            else:
                n = min(len(P), len(Z))
                if n == 0:
                    return None, None
                P, Z = P[:n], Z[:n]
        else:
            n = min(len(P), len(Z))
            if n == 0:
                return None, None
            P, Z = P[:n], Z[:n]

        Z = Z / (np.linalg.norm(Z, axis=1, keepdims=True) + 1e-12)
        return P, Z

    # -------- render orchestration (immer berechnen, selektiv anzeigen) --------
    def render_from_model(
        self,
        *,
        model: object,
        substrate_mesh: pv.PolyData,
        side: Optional[str] = None,
        default_stand_off_mm: float = 10.0,
        mask_lift_mm: float = 50.0,
        ray_len_mm: float = 1000.0,
        visibility: Optional[Dict[str, bool]] = None,
    ):
        self._out.update({
            "substrate_mesh": substrate_mesh,
            "mask_poly": None,
            "rays_hit_poly": None,
            "rays_miss_poly": None,
            "tcp_poly": None,
            "tcp_points": None,
            "valid_mask": None,
            "rc": None,
            "side": side,
            "stand_off_mm": None,
        })

        # 1) Pfad sammeln
        pbs = dict(model.get("paths_by_side") or {}) if isinstance(model, dict) \
              else dict(getattr(model, "paths_by_side", {}) or {})
        if not pbs:
            for ly in ("mask", "rays_hit", "rays_miss", "normals", "path", "path_mrk"):
                self._clear_layer(self._L(ly))
            try:
                self._update_2d(substrate_mesh, None, None)
            except Exception:
                _LOG.exception("update_2d (no paths) failed")
            return

        if side is None:
            side = "top" if "top" in pbs else next(iter(pbs.keys()))
        self._out["side"] = side

        sample_step = float(pbs.get(side, {}).get("sample_step_mm", 1.0))
        pd = PathBuilder.from_side(model, side=side, sample_step_mm=sample_step)
        P0 = np.asarray(pd.points_mm, dtype=float).reshape(-1, 3)
        if len(P0) == 0:
            for ly in ("mask", "rays_hit", "rays_miss", "normals", "path", "path_mrk"):
                self._clear_layer(self._L(ly))
            try:
                self._update_2d(substrate_mesh, None, None)
            except Exception:
                _LOG.exception("update_2d (empty path) failed")
            return

        # 2) Welt-Offset
        xmin, xmax, ymin, ymax, zmin, zmax = substrate_mesh.bounds
        cx, cy = 0.5 * (xmin + xmax), 0.5 * (ymin + ymax)
        z_plane = float(zmin)
        P0_world = P0 + np.array([cx, cy, z_plane], dtype=float)

        # 3) Maske (immer bauen)
        P_mask = None
        mask_poly = None
        try:
            z_mask = float(zmax + mask_lift_mm)
            P_mask = P0_world.copy()
            P_mask[:, 2] = z_mask
            mask_poly = pv.lines_from_points(P_mask, close=False)
        except Exception:
            mask_poly = None
        self._out["mask_poly"] = mask_poly

        # 4) Rays & TCP (immer bauen)
        rc = None
        rays_hit_poly = None
        tcp_poly = None
        tcp_points = None
        valid_mask = None
        try:
            stand_off = float(
                getattr(model, "stand_off_mm", None)
                or (isinstance(model, dict) and model.get("stand_off_mm"))
                or default_stand_off_mm
            )
            self._out["stand_off_mm"] = stand_off

            rc, rays_hit_poly, tcp_poly = cast_rays_for_side(
                P_world_start=P_mask,
                sub_mesh_world=substrate_mesh,
                side=side,
                source=pd.meta.get("source", "points"),
                stand_off_mm=stand_off,
                ray_len_mm=float(ray_len_mm),
                start_lift_mm=0.0,
                flip_normals_to_face_rays=True,
                invert_dirs=False,
                lock_xy=True,
            )
            valid_mask = getattr(rc, "valid", None)
            if rc is not None and getattr(rc, "tcp_mm", None) is not None:
                tcp_points = rc.tcp_mm[valid_mask] if (valid_mask is not None and np.any(valid_mask)) else rc.tcp_mm
        except Exception:
            _LOG.exception("cast_rays_for_side failed")

        self._out.update({
            "rc": rc,
            "rays_hit_poly": rays_hit_poly,
            "tcp_poly": tcp_poly,
            "tcp_points": tcp_points,
            "valid_mask": valid_mask,
        })

        # 5) Miss-Rays (immer bauen)
        rays_miss = None
        try:
            if valid_mask is not None and P_mask is not None and len(P_mask) == len(valid_mask) and np.any(~valid_mask):
                base_dir = {
                    "top":   np.array([0.0,  0.0, -1.0], dtype=float),
                    "front": np.array([0.0,  1.0,  0.0], dtype=float),
                    "back":  np.array([0.0, -1.0,  0.0], dtype=float),
                    "left":  np.array([1.0,  0.0,  0.0], dtype=float),
                    "right": np.array([-1.0, 0.0,  0.0], dtype=float),
                }.get(side, np.array([0.0, 0.0, -1.0], dtype=float))

                Pmiss_start = P_mask[~valid_mask]
                D = np.tile(base_dir / (np.linalg.norm(base_dir) + 1e-12), (len(Pmiss_start), 1))
                have_z = np.abs(D[:, 2]) > 1e-9
                if np.any(have_z):
                    S = Pmiss_start[have_z]
                    Dz = D[have_z]
                    t = (z_plane - S[:, 2]) / Dz[:, 2]
                    pos = t > 0
                    S = S[pos]; Dz = Dz[pos]; t = t[pos]
                    if len(S):
                        Pend = S + Dz * t.reshape(-1, 1)
                        pts = np.vstack([S, Pend])
                        nseg = len(S)
                        lines = np.empty((nseg, 3), dtype=np.int64)
                        lines[:, 0] = 2
                        lines[:, 1] = np.arange(nseg, dtype=np.int64)
                        lines[:, 2] = lines[:, 1] + nseg
                        rays_miss = pv.PolyData(pts)
                        rays_miss.lines = lines.reshape(-1)
        except Exception:
            _LOG.exception("miss rays build failed")
        self._out["rays_miss_poly"] = rays_miss

        # 6) Sichtbarkeiten anwenden (zeichnet bei Bedarf nach)
        # (Die eigentliche add_mesh-Logik steckt in den set_*_visible-Methoden)
        if visibility:
            self._vis.update({k: bool(visibility.get(k, self._vis[k])) for k in self._vis.keys()})

        self.set_mask_visible(self._vis["mask"], render=False)
        self.set_hits_visible(self._vis["hits"], render=False)
        self.set_misses_visible(self._vis["misses"], render=False)
        self.set_normals_visible(self._vis["normals"], render=False)
        self.set_path_visible(self._vis["path"], render=False)

        # 2D-Szene (gerichtet nach aktueller Sichtbarkeit)
        try:
            self._update_2d(
                substrate_mesh,
                self._out["tcp_points"] if self._vis["path"] else None,
                self._out["mask_poly"]  if self._vis["mask"] else None
            )
        except Exception:
            _LOG.exception("update_2d_scene failed")

        # Frames (lokale KS) – jetzt mit Alignment
        try:
            if self._vis["frames"] and self._out.get("tcp_points") is not None and self._out.get("rc") is not None:
                Z = self._extract_z_dirs(self._out["rc"])
                P_aligned, Z_aligned = self._align_tcp_and_dirs(self._out["rc"], self._out["tcp_points"], Z)
                if P_aligned is not None and Z_aligned is not None and len(P_aligned) and len(Z_aligned):
                    self._show_frames_at(
                        origins=P_aligned,
                        z_dirs=Z_aligned,
                        scale_mm=None,
                        line_width=1.0,
                        clear_old=True,
                        add_labels=False,
                    )
            else:
                for lyr in ("frames_x", "frames_y", "frames_z", "frames_labels"):
                    self._clear_layer(self._L(lyr))
        except Exception:
            _LOG.exception("render frames failed")
