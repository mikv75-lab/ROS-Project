# -*- coding: utf-8 -*-
from __future__ import annotations
import logging
from typing import Any, Callable, Dict, Optional

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
        # akzeptiert evtl. (layer, vis) oder (layer, vis, render)
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
            "tcp_points": None,     # (N,3) numpy
            "valid_mask": None,     # bool mask für tcp_points (falls gefiltert)
            "rc": None,             # RaycastResult o.ä. (wenn vorhanden)
            "side": None,
            "stand_off_mm": None,
        }

        # Sichtbarkeit (letzter Zustand)
        self._vis: Dict[str, bool] = {
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
        """Verträgt beide Signaturen (2 oder 3 Parameter) der set_layer_visible-Funktion."""
        try:
            # (layer, vis, render) bevorzugen
            self._set_layer_visible(layer, vis, render)  # type: ignore[misc]
        except TypeError:
            # Fallback: (layer, vis)
            self._set_layer_visible(layer, vis)          # type: ignore[misc]

    # -------- public outputs --------
    def get_outputs(self) -> Dict[str, Any]:
        return self._out

    # -------- visibility toggles (für Checkbox-Wiring) --------
    # Beim Einschalten wird aus dem Cache (_out) neu gezeichnet,
    # beim Ausschalten nur Sichtbarkeit/Layer geleert.
    def set_mask_visible(self, vis: bool, *, render: bool = True):
        self._vis["mask"] = bool(vis)
        layer = self._L("mask")
        if vis:
            poly = self._out.get("mask_poly")
            self._clear_layer(layer)
            if poly is not None:
                self._add_mesh(poly, color="royalblue", layer=layer,
                               line_width=2.0, lighting=False, render=False, reset_camera=False)
        else:
            # Optional: nur unsichtbar schalten, aber wir räumen sauber
            self._clear_layer(layer)
        self._set_vis_layer(layer, vis, render=render)

    def set_path_visible(self, vis: bool, *, render: bool = True):
        self._vis["path"] = bool(vis)
        layer = self._L("path")
        layer_m = self._L("path_mrk")
        if vis:
            P = self._out.get("tcp_points")
            self._clear_layer(layer); self._clear_layer(layer_m)
            if P is not None and len(P):
                self._add_path_polyline(P, layer=layer, color="#2ecc71",
                                        line_width=2.0, lighting=False, render=False, reset_camera=False)
                # optionale Marker dezent
                try:
                    pts = np.asarray(P, float)
                    step = max(1, int(round(max(1, pts.shape[0] // 200))))
                    pts_s = pts[::step]
                    self._add_mesh(pv.PolyData(pts_s), color="#2ecc71", layer=layer_m,
                                   point_size=8.0, lighting=False, render=False, reset_camera=False)
                except Exception:
                    _LOG.exception("path markers (toggle) failed")
        else:
            self._clear_layer(layer); self._clear_layer(layer_m)
        self._set_vis_layer(layer, vis, render=False)
        self._set_vis_layer(layer_m, vis, render=render)

    def set_hits_visible(self, vis: bool, *, render: bool = True):
        self._vis["hits"] = bool(vis)
        layer = self._L("rays_hit")
        if vis:
            poly = self._out.get("rays_hit_poly")
            self._clear_layer(layer)
            if getattr(poly, "n_lines", 0):
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
            self._clear_layer(layer)
            if poly is not None:
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
            self._clear_layer(layer)
            if getattr(poly, "n_lines", 0):
                self._add_mesh(poly, color="#f1c40f", layer=layer,
                               line_width=1.3, lighting=False, render=False, reset_camera=False)
        else:
            self._clear_layer(layer)
        self._set_vis_layer(layer, vis, render=render)

    def set_frames_visible(self, vis: bool, *, render: bool = True):
        self._vis["frames"] = bool(vis)
        if vis:
            P = self._out.get("tcp_points")
            rc = self._out.get("rc")
            z_dirs = getattr(rc, "refl_dir", None) if rc is not None else None
            if P is not None and z_dirs is not None and len(P) and len(z_dirs):
                self._show_frames_at(
                    origins=P, z_dirs=z_dirs, scale_mm=None,
                    line_width=1.0, clear_old=True, add_labels=False
                )
            # Sichtbarkeit der vier Frame-Layer einschalten
            for lyr in ("frames_x", "frames_y", "frames_z", "frames_labels"):
                self._set_vis_layer(self._L(lyr), True, render=False)
        else:
            for lyr in ("frames_x", "frames_y", "frames_z", "frames_labels"):
                self._clear_layer(self._L(lyr))
        # Ein einziges optionales Render am Ende
        if render:
            try:
                # Wir kennen hier keinen direkten Render-Handle; SceneManager übernimmt es
                # via set_layer_visible(..., render=True) bei obigen Aufrufen.
                pass
            except Exception:
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
        # Reihenfolge: zusammengehörige Layer ohne Zwischenrender setzen, letztes mit Render
        self.set_mask_visible(self._vis["mask"], render=False)
        self.set_path_visible(self._vis["path"], render=False)
        self.set_hits_visible(self._vis["hits"], render=False)
        self.set_misses_visible(self._vis["misses"], render=False)
        self.set_normals_visible(self._vis["normals"], render=True)
        # Frames separat (kann viel Geometrie sein)
        self.set_frames_visible(self._vis["frames"], render=False)

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

        # 6) Selektiv anzeigen (add/clear pro Overlay)
        # Hinweis: Wir ADDEN nur, wenn sichtbar. Beim späteren Toggle übernehmen die set_* Methoden.

        # Mask
        self._clear_layer(self._L("mask"))
        if self._on(visibility, "mask", self._vis["mask"]) and mask_poly is not None:
            self._add_mesh(mask_poly, color="royalblue", layer=self._L("mask"),
                           line_width=2.0, lighting=False, render=False, reset_camera=False)

        # Rays (Hits)
        self._clear_layer(self._L("rays_hit"))
        if self._on(visibility, "hits", self._vis["hits"]) and getattr(rays_hit_poly, "n_lines", 0):
            self._add_mesh(rays_hit_poly, color="#85C1E9", layer=self._L("rays_hit"),
                           line_width=1.5, lighting=False, render=False, reset_camera=False)

        # Misses
        self._clear_layer(self._L("rays_miss"))
        if self._on(visibility, "misses", self._vis["misses"]) and rays_miss is not None:
            self._add_mesh(rays_miss, color="#e74c3c", layer=self._L("rays_miss"),
                           line_width=1.2, lighting=False, render=False, reset_camera=False)

        # Normals (gelbe TCP-Poly)
        self._clear_layer(self._L("normals"))
        if self._on(visibility, "normals", self._vis["normals"]) and getattr(tcp_poly, "n_lines", 0):
            self._add_mesh(tcp_poly, color="#f1c40f", layer=self._L("normals"),
                           line_width=1.3, lighting=False, render=False, reset_camera=False)

        # TCP-Polyline (grün)
        self._clear_layer(self._L("path"))
        self._clear_layer(self._L("path_mrk"))
        if self._on(visibility, "path", self._vis["path"]) and tcp_points is not None and len(tcp_points):
            self._add_path_polyline(tcp_points, layer=self._L("path"), color="#2ecc71",
                                    line_width=2.0, lighting=False, render=False, reset_camera=False)
            # optionale Marker
            try:
                pts = np.asarray(tcp_points, float)
                step = max(1, int(round(max(1, pts.shape[0] // 200))))
                pts_s = pts[::step]
                self._add_mesh(pv.PolyData(pts_s), color="#2ecc71", layer=self._L("path_mrk"),
                               point_size=8.0, lighting=False, render=False, reset_camera=False)
            except Exception:
                _LOG.exception("path markers failed")

        # 2D-Szene (nur sichtbare Elemente einspeisen)
        try:
            self._update_2d(
                substrate_mesh,
                tcp_points if self._on(visibility, "path", self._vis["path"]) else None,
                mask_poly  if self._on(visibility, "mask", self._vis["mask"]) else None
            )
        except Exception:
            _LOG.exception("update_2d_scene failed")

        # Frames (lokale KS) – nur zeichnen, wenn explizit sichtbar
        try:
            # Übernahme/Update der Sichtbarkeit
            if visibility:
                for k in self._vis.keys():
                    if k in visibility:
                        self._vis[k] = bool(visibility[k])

            if self._vis["frames"] and tcp_points is not None and rc is not None:
                z_dirs = getattr(rc, "refl_dir", None)
                if z_dirs is not None and len(z_dirs):
                    self._show_frames_at(
                        origins=tcp_points,
                        z_dirs=z_dirs,
                        scale_mm=None,      # Auto-Scale im Panel
                        line_width=1.0,
                        clear_old=True,
                        add_labels=False,
                    )
            else:
                for lyr in ("frames_x", "frames_y", "frames_z", "frames_labels"):
                    self._clear_layer(self._L(lyr))
        except Exception:
            _LOG.exception("render frames failed")

        # Sichtbarkeit konsistent anwenden (ruft die Toggle-Methoden, die im Zweifel neu aufbauen)
        self.apply_visibility(self._vis)
