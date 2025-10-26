# Spraycoater/src/app/tabs/recipe/trajectory_builder.py
# -*- coding: utf-8 -*-
from __future__ import annotations
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple
import math
import numpy as np
import yaml

from .recipe_model import Recipe
from .path_builder import PathBuilder, PathData
from .surface_projector import SurfaceProjector, ProjectionResult

import pyvista as pv

@dataclass
class TrajPose:
    p: np.ndarray  # [m]
    q: np.ndarray  # [x,y,z,w]
    t: float       # [s]


@dataclass
class Trajectory:
    poses: List[TrajPose]
    frame_id: str = "scene"
    meta: Dict[str, Any] | None = None

    def as_arrays(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        if not self.poses:
            return (np.zeros((0, 3)), np.zeros((0, 4)), np.zeros((0,)))
        P = np.vstack([tp.p for tp in self.poses])
        Q = np.vstack([tp.q for tp in self.poses])
        Tt = np.array([tp.t for tp in self.poses], dtype=float)
        return P, Q, Tt


class TrajectoryBuilder:
    """
    Pipeline:
      PathBuilder (mm, Oberfläche flach/parametrisch) -> [optional] SurfaceProjector (Raycast auf echtes Mesh, mm)
      -> TCP aus Oberfläche (mm->m) -> optional Welt-Transform.
    """

    def __init__(self):
        self._path_builder = PathBuilder()
        self._projector = SurfaceProjector()

    # -------- Orchestrator --------
    def build(
        self,
        recipe: Recipe,
        *,
        mesh_mm: Optional["pv.PolyData"] = None,  # echtes Substrat-Mesh in mm (falls vorhanden)
        side: str = "top",                        # Projektrichtung (scene)
        T_world_scene: Optional[np.ndarray] = None,
        force_project: bool = False,              # auch Helix usw. projizieren
    ) -> Trajectory:
        p = recipe.path or {}
        ptype = (p.get("type") or "").strip().lower()

        # 1) explicit TCP (as-is) bleibt unangetastet
        if ptype == "explicit" and ("tcp_points_mm" in p or "tcp_points_m" in p):
            traj_scene = self._explicit_tcp_as_is(recipe)
            return self._to_world(traj_scene, T_world_scene)

        # 2) Oberflächenpfad (mm) von PathBuilder
        base: PathData = self._path_builder.build(recipe)
        surf_pts_mm = base.points_mm
        normals = base.normals

        # 3) Optionale Projektion auf echtes Mesh per Raycast
        #    – standardmäßig nur für plane/explicit-surface; mit force_project=True für alles
        needs_projection = mesh_mm is not None and (
            force_project or ptype in {"meander", "meander_plane", "spiral", "spiral_plane", "explicit"}
        )
        if needs_projection:
            proj: ProjectionResult = self._projector.project_on_side(surf_pts_mm, mesh_mm, side)
            surf_pts_mm = proj.points_mm
            normals = proj.normals  # Normale vom Mesh

        # 4) TCP aus Oberfläche und Normale
        traj_scene = self._surface_to_tcp(
            surf_pts_mm,
            normals if normals is not None else self._default_normals(len(surf_pts_mm)),
            speed_mm_s=float(self._param(recipe, "speed_mm_s", 200.0)),
            stand_off_mm=float(self._param(recipe, "stand_off_mm", 10.0)),
            spray_angle_deg=float(self._param(recipe, "spray_angle_deg", 0.0)),
            frame_id="scene",
        )
        return self._to_world(traj_scene, T_world_scene)

    def build_bundle(
        self,
        recipe: Recipe,
        *,
        mesh_mm: Optional["pv.PolyData"] = None,
        side: str = "top",
        tools_yaml: Dict[str, Any] | str | None = None,
        T_world_scene: Optional[np.ndarray] = None,
        force_project: bool = False,
    ) -> Dict[str, Any]:
        # Pflichtfelder
        missing = []
        if not recipe.tool:      missing.append("tool")
        if not recipe.mount:     missing.append("mount")
        if not recipe.substrate: missing.append("substrate")
        if missing:
            raise ValueError("Recipe unvollständig: " + ", ".join(missing))

        tcp_scene = self.build(
            recipe, mesh_mm=mesh_mm, side=side,
            T_world_scene=None, force_project=force_project
        )
        out: Dict[str, Any] = {"tcp_scene": tcp_scene}

        if T_world_scene is not None:
            out["tcp_world"] = self._to_world(tcp_scene, T_world_scene)

        if tools_yaml is not None:
            T_mount_tcp = self._load_T_mount_tcp(tools_yaml, recipe.tool)
            out["T_mount_tcp"] = T_mount_tcp
            out["mount_scene"] = self._traj_tcp_to_mount(tcp_scene, T_mount_tcp)
            if "tcp_world" in out:
                out["mount_world"] = self._traj_tcp_to_mount(out["tcp_world"], T_mount_tcp)
        return out

    # -------- explicit TCP (as-is) --------
    def _explicit_tcp_as_is(self, recipe: Recipe) -> Trajectory:
        p = recipe.path or {}
        P_mm = np.asarray(p.get("tcp_points_mm") or [], dtype=float).reshape(-1, 3)
        if P_mm.size == 0 and "tcp_points_m" in p:
            P_mm = 1000.0 * np.asarray(p["tcp_points_m"], dtype=float).reshape(-1, 3)
        if P_mm.size == 0:
            return Trajectory(poses=[], frame_id="scene", meta={"empty": True, "explicit_tcp": True})

        Q = None
        if "tcp_quat_xyzw" in p:
            Q = np.asarray(p["tcp_quat_xyzw"], dtype=float).reshape(-1, 4)

        if "timestamps_s" in p:
            t_arr = np.asarray(p["timestamps_s"], dtype=float).reshape(-1)
            if len(t_arr) != len(P_mm):
                raise ValueError("timestamps_s Länge muss mit tcp_points_* übereinstimmen.")
        else:
            speed = float(self._param(recipe, "speed_mm_s", 200.0))
            s = self._arclen_mm(P_mm); t_arr = s / max(speed, 1e-9)

        if Q is None or len(Q) != len(P_mm):
            T = self._tangents(P_mm)
            z_axis = np.tile(np.array([0.0, 0.0, -1.0]), (len(P_mm), 1))
            x_axis = T
            y_axis = self._safe_cross(z_axis, x_axis); y_axis /= (np.linalg.norm(y_axis, axis=1, keepdims=True) + 1e-12)
            x_axis = self._safe_cross(y_axis, z_axis); x_axis /= (np.linalg.norm(x_axis, axis=1, keepdims=True) + 1e-12)
            Q = np.vstack([self._axes_to_quat(x_axis[i], y_axis[i], z_axis[i]) for i in range(len(P_mm))])

        P_m = P_mm * 0.001
        poses = [TrajPose(p=P_m[i], q=Q[i], t=float(t_arr[i])) for i in range(len(P_m))]
        return Trajectory(poses=poses, frame_id="scene", meta={"explicit_tcp": True})

    # -------- Oberfläche -> TCP (mm -> m) --------
    def _surface_to_tcp(self, surf_pts_mm, normals, *, speed_mm_s, stand_off_mm, spray_angle_deg, frame_id: str) -> Trajectory:
        if len(surf_pts_mm) == 0:
            return Trajectory(poses=[], frame_id=frame_id, meta={"empty": True})

        T = self._tangents(surf_pts_mm)
        N = normals / (np.linalg.norm(normals, axis=1, keepdims=True) + 1e-12)
        ang = math.radians(spray_angle_deg)

        B = self._safe_cross(T, N); B /= (np.linalg.norm(B, axis=1, keepdims=True) + 1e-12)
        A = self._rotate_about_axis(N, B, ang)
        z_axis = -A; z_axis /= (np.linalg.norm(z_axis, axis=1, keepdims=True) + 1e-12)

        T_proj = T - (np.sum(T * z_axis, axis=1, keepdims=True)) * z_axis
        x_axis = T_proj / (np.linalg.norm(T_proj, axis=1, keepdims=True) + 1e-12)
        y_axis = self._safe_cross(z_axis, x_axis); y_axis /= (np.linalg.norm(y_axis, axis=1, keepdims=True) + 1e-12)

        tcp_mm = surf_pts_mm - A * float(stand_off_mm)
        Q = np.vstack([self._axes_to_quat(x_axis[i], y_axis[i], z_axis[i]) for i in range(len(x_axis))])

        s = self._arclen_mm(surf_pts_mm)
        t = s / max(speed_mm_s, 1e-9)

        tcp_m = tcp_mm * 0.001
        poses = [TrajPose(p=tcp_m[i], q=Q[i], t=float(t[i])) for i in range(len(tcp_m))]
        return Trajectory(poses=poses, frame_id=frame_id, meta={
            "stand_off_mm": stand_off_mm, "spray_angle_deg": spray_angle_deg, "speed_mm_s": speed_mm_s
        })

    # -------- Welt-Transform --------
    def _to_world(self, traj_scene: Trajectory, T_world_scene: Optional[np.ndarray]) -> Trajectory:
        if T_world_scene is None:
            return traj_scene
        if not traj_scene.poses:
            return Trajectory(poses=[], frame_id="world", meta=dict(traj_scene.meta or {}))
        Rws = T_world_scene[:3, :3]; tws = T_world_scene[:3, 3]
        P, Q, Tt = traj_scene.as_arrays()
        Pw = (P @ Rws.T) + tws
        Rws_q = self._rot_to_quat(Rws)
        Qw = np.vstack([self._quat_mul(Rws_q, q) for q in Q])
        poses = [TrajPose(p=Pw[i], q=Qw[i], t=float(Tt[i])) for i in range(len(Pw))]
        return Trajectory(poses=poses, frame_id="world", meta=dict(traj_scene.meta or {}))

    # -------- (optional) Tools.yaml -> T_mount_tcp --------
    @staticmethod
    def _load_yaml_or_dict(cfg_or_path: Dict[str, Any] | str) -> Dict[str, Any]:
        if isinstance(cfg_or_path, str):
            with open(cfg_or_path, "r", encoding="utf-8") as f:
                return yaml.safe_load(f) or {}
        return dict(cfg_or_path or {})

    def _load_T_mount_tcp(self, tools_yaml: Dict[str, Any] | str, tool_name: str) -> np.ndarray:
        cfg = self._load_yaml_or_dict(tools_yaml)
        tools = cfg.get("tools", {}) or {}
        if tool_name not in tools:
            raise KeyError(f"Tool '{tool_name}' nicht in tools.yaml gefunden.")
        tc = dict(tools[tool_name] or {})
        off = tc.get("tcp_offset", [0.0, 0.0, 0.0])  # Meter
        rpy = tc.get("tcp_rpy",    [0.0, 0.0, 0.0])  # Grad
        R = self._rpy_deg(*map(float, rpy))
        t = np.array(list(map(float, off)), dtype=float).reshape(3)
        T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t
        return T

    def _traj_tcp_to_mount(self, traj_tcp: Optional[Trajectory], T_mount_tcp: np.ndarray) -> Optional[Trajectory]:
        if traj_tcp is None:
            return None
        if not traj_tcp.poses:
            return Trajectory(poses=[], frame_id=traj_tcp.frame_id, meta=dict(traj_tcp.meta or {}))

        R_mt = T_mount_tcp[:3, :3]; t_mt = T_mount_tcp[:3, 3]
        R_tm = R_mt.T; t_tm = -R_mt.T @ t_mt

        P, Q, Tt = traj_tcp.as_arrays()
        Pm = np.empty_like(P)
        for i in range(len(P)):
            R_wt = self._quat_to_rot(Q[i])
            Pm[i] = P[i] + (R_wt @ t_tm)

        q_mt = self._rot_to_quat(R_mt)
        q_tm = self._quat_conj(q_mt)
        Qm = np.vstack([self._quat_mul(Q[i], q_tm) for i in range(len(Q))])

        poses = [TrajPose(p=Pm[i], q=Qm[i], t=float(Tt[i])) for i in range(len(Pm))]
        meta = dict(traj_tcp.meta or {}); meta.update({"tool_mount_from": True})
        return Trajectory(poses=poses, frame_id=traj_tcp.frame_id, meta=meta)

    # -------- Math --------
    @staticmethod
    def _default_normals(n: int) -> np.ndarray:
        return np.tile(np.array([0.0, 0.0, 1.0]), (n, 1))

    @staticmethod
    def _param(recipe: Recipe, key: str, default: float) -> float:
        try:
            return float(recipe.parameters.get(key, default))
        except Exception:
            return float(default)

    @staticmethod
    def _tangents(P: np.ndarray) -> np.ndarray:
        if len(P) < 2:
            return np.tile(np.array([1.0, 0.0, 0.0]), (len(P), 1))
        T = np.zeros_like(P)
        T[1:-1] = P[2:] - P[:-2]
        T[0] = P[1] - P[0]
        T[-1] = P[-1] - P[-2]
        T /= (np.linalg.norm(T, axis=1, keepdims=True) + 1e-12)
        return T

    @staticmethod
    def _safe_cross(a: np.ndarray, b: np.ndarray) -> np.ndarray:
        c = np.cross(a, b)
        mask = np.linalg.norm(c, axis=1) < 1e-9
        if np.any(mask):
            c[mask] = np.cross(a[mask], np.array([[0, 0, 1.0]]))
        return c

    @staticmethod
    def _rotate_about_axis(v: np.ndarray, axis: np.ndarray, ang: float) -> np.ndarray:
        c, s = math.cos(ang), math.sin(ang)
        k = axis / (np.linalg.norm(axis, axis=1, keepdims=True) + 1e-12)
        kv = np.sum(k * v, axis=1, keepdims=True)
        cross = np.cross(k, v)
        return v * c + cross * s + k * kv * (1.0 - c)

    @staticmethod
    def _arclen_mm(P: np.ndarray) -> np.ndarray:
        if len(P) == 0:
            return np.zeros((0,))
        d = np.linalg.norm(P[1:] - P[:-1], axis=1)
        return np.concatenate([[0.0], np.cumsum(d)])

    @staticmethod
    def _axes_to_quat(x: np.ndarray, y: np.ndarray, z: np.ndarray) -> np.ndarray:
        R = np.array([[x[0], y[0], z[0]],
                      [x[1], y[1], z[1]],
                      [x[2], y[2], z[2]]], dtype=float)
        U, _, Vt = np.linalg.svd(R)
        R = U @ Vt
        tr = np.trace(R)
        if tr > 0.0:
            S = math.sqrt(tr + 1.0) * 2.0; qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S; qy = (R[0, 2] - R[2, 0]) / S; qz = (R[1, 0] - R[0, 1]) / S
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
                qw = (R[2, 1] - R[1, 2]) / S; qx = 0.25 * S; qy = (R[0, 1] + R[1, 0]) / S; qz = (R[0, 2] + R[2, 0]) / S
            elif R[1, 1] > R[2, 2]:
                S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
                qw = (R[0, 2] - R[2, 0]) / S; qx = (R[0, 1] + R[1, 0]) / S; qy = 0.25 * S; qz = (R[1, 2] + R[2, 1]) / S
            else:
                S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
                qw = (R[1, 0] - R[0, 1]) / S; qx = (R[0, 2] + R[2, 0]) / S; qy = (R[1, 2] + R[2, 1]) / S; qz = 0.25 * S
        return np.array([qx, qy, qz, qw], dtype=float)

    @staticmethod
    def _rot_to_quat(R: np.ndarray) -> np.ndarray:
        tr = np.trace(R)
        if tr > 0:
            S = math.sqrt(tr + 1.0) * 2.0; qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S; qy = (R[0, 2] - R[2, 0]) / S; qz = (R[1, 0] - R[0, 1]) / S
        else:
            if (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
                S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
                qw = (R[2, 1] - R[1, 2]) / S; qx = 0.25 * S; qy = (R[0, 1] + R[1, 0]) / S; qz = (R[0, 2] + R[2, 0]) / S
            elif R[1, 1] > R[2, 2]:
                S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
                qw = (R[0, 2] - R[2, 0]) / S; qx = (R[0, 1] + R[1, 0]) / S; qy = 0.25 * S; qz = (R[1, 2] + R[2, 1]) / S
            else:
                S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
                qw = (R[1, 0] - R[0, 1]) / S; qx = (R[0, 2] + R[2, 0]) / S; qy = (R[1, 2] + R[2, 1]) / S; qz = 0.25 * S
        return np.array([qx, qy, qz, qw], dtype=float)

    @staticmethod
    def _quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        x1, y1, z1, w1 = q1; x2, y2, z2, w2 = q2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        return np.array([x, y, z, w], dtype=float)
