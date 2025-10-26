# -*- coding: utf-8 -*-
from __future__ import annotations
from dataclasses import dataclass
from typing import Any, Dict, List, Optional
import math
import numpy as np

# Keine Imports von Recipe/Preview hier, um Zirkularimporte zu vermeiden.

@dataclass
class TrajPose:
    p: np.ndarray  # [m]
    q: np.ndarray  # [x,y,z,w]
    t: float       # [s]

@dataclass
class Trajectory:
    poses: List[TrajPose]
    frame_id: str = "scene"
    meta: Dict[str, Any] = None

    def as_arrays(self):
        if not self.poses:
            return (np.zeros((0,3)), np.zeros((0,4)), np.zeros((0,)))
        P = np.vstack([tp.p for tp in self.poses])
        Q = np.vstack([tp.q for tp in self.poses])
        Tt = np.array([tp.t for tp in self.poses], dtype=float)
        return P, Q, Tt


class TrajectoryBuilder:
    """
    Baut eine TCP-Trajektorie (m) aus Pfaddaten (mm).
    - Erwartet PathData (points_mm, optional normals, optional tangents)
    - Orientiert TCP mit -Z entlang der Flächennormale (falls vorhanden),
      andernfalls -Z = [0,0,-1].
    - X-Achse entlang der projizierten Tangente (falls vorhanden), sonst [1,0,0] fallback.
    """

    def build_from_pathdata(self, recipe: Any, path_data: Any, *, frame_id: str = "scene") -> Trajectory:
        # Parameter
        params = dict(getattr(recipe, "parameters", {}) or {})
        speed_mm_s = float(params.get("speed_mm_s", 200.0))
        stand_off_mm = float(params.get("stand_off_mm", 10.0))
        spray_angle_deg = float(params.get("spray_angle_deg", 0.0))

        Pmm = np.asarray(path_data.points_mm, dtype=float).reshape(-1, 3)
        if len(Pmm) == 0:
            return Trajectory(poses=[], frame_id=frame_id, meta={"empty": True})

        N = None
        if getattr(path_data, "normals", None) is not None:
            N = np.asarray(path_data.normals, dtype=float).reshape(-1, 3)
        T = None
        if getattr(path_data, "tangents", None) is not None:
            T = np.asarray(path_data.tangents, dtype=float).reshape(-1, 3)
        if T is None:
            T = self._tangents(Pmm)

        # Normale: vorhanden -> nutzen, sonst global +Z
        if N is None or len(N) != len(Pmm):
            N = np.tile(np.array([0.0, 0.0, 1.0], dtype=float), (len(Pmm), 1))
        N = N / (np.linalg.norm(N, axis=1, keepdims=True) + 1e-12)

        # Spray-Achse: -Z entlang (evtl. geneigter) Normale
        # Optional Spraywinkel um Tangenten-rechte-Normalenachse drehen
        B = self._safe_cross(T, N); B /= (np.linalg.norm(B, axis=1, keepdims=True) + 1e-12)
        A = self._rotate_about_axis(N, B, math.radians(float(spray_angle_deg)))
        z_axis = -A
        z_axis /= (np.linalg.norm(z_axis, axis=1, keepdims=True) + 1e-12)

        # X-Achse: Tangente in Ebene senkrecht zu z
        T_proj = T - (np.sum(T * z_axis, axis=1, keepdims=True)) * z_axis
        x_axis = T_proj / (np.linalg.norm(T_proj, axis=1, keepdims=True) + 1e-12)

        # Y-Achse: z x x
        y_axis = self._safe_cross(z_axis, x_axis)
        y_axis /= (np.linalg.norm(y_axis, axis=1, keepdims=True) + 1e-12)
        # X nochmal orthogonalisieren
        x_axis = self._safe_cross(y_axis, z_axis)
        x_axis /= (np.linalg.norm(x_axis, axis=1, keepdims=True) + 1e-12)

        # TCP-Position: Stand-off entlang +A (also entgegen z_axis)
        tcp_mm = Pmm - A * float(stand_off_mm)

        # Quaternions
        Q = np.vstack([self._axes_to_quat(x_axis[i], y_axis[i], z_axis[i]) for i in range(len(x_axis))])

        # Timestamps aus Bogenlänge
        s = self._arclen_mm(Pmm)
        t_arr = s / max(speed_mm_s, 1e-9)

        # mm -> m
        Pm = tcp_mm * 0.001

        poses = [TrajPose(p=Pm[i], q=Q[i], t=float(t_arr[i])) for i in range(len(Pm))]
        meta = {
            "stand_off_mm": stand_off_mm,
            "spray_angle_deg": spray_angle_deg,
            "speed_mm_s": speed_mm_s
        }
        return Trajectory(poses=poses, frame_id=frame_id, meta=meta)

    # -------------- helpers --------------
    @staticmethod
    def _tangents(P: np.ndarray) -> np.ndarray:
        if len(P) < 2:
            return np.tile(np.array([1.0, 0.0, 0.0]), (len(P), 1))
        T = np.zeros_like(P)
        T[1:-1] = P[2:] - P[:-2]
        T[0]    = P[1] - P[0]
        T[-1]   = P[-1] - P[-2]
        n = np.linalg.norm(T, axis=1, keepdims=True) + 1e-12
        return T / n

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
    def _axes_to_quat(x: np.ndarray, y: np.ndarray, z: np.ndarray) -> np.ndarray:
        R = np.array([[x[0], y[0], z[0]],
                      [x[1], y[1], z[1]],
                      [x[2], y[2], z[2]]], dtype=float)
        U, _, Vt = np.linalg.svd(R)
        R = U @ Vt
        tr = np.trace(R)
        if tr > 0.0:
            S = math.sqrt(tr + 1.0) * 2.0; qw = 0.25 * S
            qx = (R[2,1]-R[1,2]) / S; qy = (R[0,2]-R[2,0]) / S; qz = (R[1,0]-R[0,1]) / S
        else:
            if R[0,0] > R[1,1] and R[0,0] > R[2,2]:
                S = math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2.0
                qw = (R[2,1]-R[1,2]) / S; qx = 0.25*S; qy = (R[0,1]+R[1,0]) / S; qz = (R[0,2]+R[2,0]) / S
            elif R[1,1] > R[2,2]:
                S = math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2.0
                qw = (R[0,2]-R[2,0]) / S; qx = (R[0,1]+R[1,0]) / S; qy = 0.25*S; qz = (R[1,2]+R[2,1]) / S
            else:
                S = math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2.0
                qw = (R[1,0]-R[0,1]) / S; qx = (R[0,2]+R[2,0]) / S; qy = (R[1,2]+R[2,1]) / S; qz = 0.25*S
        return np.array([qx, qy, qz, qw], dtype=float)

    @staticmethod
    def _arclen_mm(P: np.ndarray) -> np.ndarray:
        if len(P) == 0:
            return np.zeros((0,))
        d = np.linalg.norm(P[1:] - P[:-1], axis=1)
        return np.concatenate([[0.0], np.cumsum(d)])
