# -*- coding: utf-8 -*-
# app/model/recipe/recipe.py
from __future__ import annotations
import os, math, yaml
import numpy as np
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Iterable, Tuple

@dataclass
class Recipe:
    """
    Schlanke Rezeptstruktur:
      - parameters:     globale Instanzwerte (aus recipe_params.globals)
      - planner:        Planner-/Pipeline-Instanzwerte (aus recipe_params.planner)
      - paths_by_side:  pro Side genau EINE Path-Definition (dict)
      - _paths_cache:   rohe Geometriepunkte (N,3) je Side (nur RAM)
      - paths_compiled: gespeicherte Posen (Quaternionen) + Meta (für Export/Save)
    """
    # Meta / Auswahl
    id: str
    description: str = ""
    tool: Optional[str] = None
    substrate: Optional[str] = None
    substrates: List[str] = field(default_factory=list)
    substrate_mount: Optional[str] = None

    # Instanzparameter
    parameters: Dict[str, Any] = field(default_factory=dict)  # globals
    planner:    Dict[str, Any] = field(default_factory=dict)  # planner

    # Pfad-Definitionen pro Side
    paths_by_side: Dict[str, Dict[str, Any]] = field(default_factory=dict)

    # Laufzeit
    _paths_cache: Dict[str, np.ndarray] = field(default_factory=dict, repr=False, compare=False)

    # Ergebnis (wird gespeichert)
    paths_compiled: Dict[str, Any] = field(default_factory=dict)

    # ---------- YAML ----------
    @staticmethod
    def from_dict(d: Dict[str, Any]) -> "Recipe":
        return Recipe(
            id=str(d.get("id") or "recipe"),
            description=str(d.get("description") or ""),
            tool=d.get("tool"),
            substrate=d.get("substrate"),
            substrates=list(d.get("substrates") or ([] if not d.get("substrate") else [d.get("substrate")])),
            substrate_mount=d.get("substrate_mount") or d.get("mount"),
            parameters=dict(d.get("parameters") or {}),
            planner=dict(d.get("planner") or {}),
            paths_by_side=dict(d.get("paths_by_side") or d.get("paths") or {}),
            paths_compiled=dict(d.get("paths_compiled") or {}),
        )

    def to_dict(self) -> Dict[str, Any]:
        out = {
            "id": self.id,
            "description": self.description,
            "tool": self.tool,
            "substrate": self.substrate,
            "substrates": self.substrates if self.substrates else ([self.substrate] if self.substrate else []),
            "substrate_mount": self.substrate_mount,
            "parameters": dict(self.parameters or {}),
            "planner":    dict(self.planner or {}),
            "paths_by_side": {s: dict(p or {}) for s, p in (self.paths_by_side or {}).items()},
        }
        if self.paths_compiled:
            out["paths_compiled"] = self.paths_compiled
        return out

    @staticmethod
    def load_yaml(path: str) -> "Recipe":
        with open(path, "r", encoding="utf-8") as f:
            return Recipe.from_dict(yaml.safe_load(f) | {})  # type: ignore

    def save_yaml(self, path: str) -> None:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(self.to_dict(), f, allow_unicode=True, sort_keys=False)

    # ------- Pflicht-Globals für PathBuilder (strict, kein Fallback) -------
    def _globals_params_strict(self) -> Dict[str, Any]:
        g = dict(self.parameters or {})
        required = [
            "stand_off_mm",
            "max_angle_deg",
            "sample_step_mm",   # strikt global
            "max_points",       # strikt global
            "predispense.angle_deg",
            "predispense.distance_mm",
            "retreat.angle_deg",
            "retreat.distance_mm",
        ]
        missing = [k for k in required if k not in g]
        if missing:
            raise ValueError(f"Recipe: Missing required globals in parameters: {missing} (kein Fallback).")
        return {k: g[k] for k in required}

    # ------- Pfad-Cache -------
    def _side_list(self, sides: Optional[Iterable[str]]) -> List[str]:
        return [str(s) for s in sides] if sides else list(self.paths_by_side.keys())

    def invalidate_paths(self) -> None:
        self._paths_cache.clear()

    def rebuild_paths(
        self,
        *,
        sides: Optional[Iterable[str]] = None,
        max_points: Optional[int] = None
    ) -> Dict[str, np.ndarray]:
        """
        Erzeugt rohe Geometriepfade (N,3) je Side (nur RAM) – strikt, ohne Fallbacks.
        - sample_step_mm ausschließlich global (parameters.sample_step_mm)
        - max_points strikt (parameters.max_points oder explizit)
        """
        from app.model.recipe.path_builder import PathBuilder

        globals_params = self._globals_params_strict()  # prüft sample_step_mm + max_points etc.

        # max_points strikt
        if max_points is None:
            max_points = int(self.parameters["max_points"])

        # Schritt strikt global:
        step = float(self.parameters["sample_step_mm"])

        if not sides:
            self._paths_cache.clear()

        for side in self._side_list(sides):
            pd = PathBuilder.from_side(
                self,
                side=side,
                globals_params=globals_params,
                sample_step_mm=step,     # global
                max_points=max_points,
            )
            self._paths_cache[side] = np.asarray(pd.points_mm, float).reshape(-1, 3)

        return self._paths_cache

    @property
    def paths(self) -> Dict[str, np.ndarray]:
        return self._paths_cache

    # ------- Quaternion-Posen + Entry/Exit -------
    def compile_poses(
        self,
        *,
        bounds: Tuple[float, float, float, float, float, float],
        sides: Optional[Iterable[str]] = None,
        stand_off_mm: Optional[float] = None,
        tool_frame: Optional[str] = None,
        predispense: Optional[Dict[str, float]] = None,  # {"distance_mm":..,"angle_deg":..}
        retreat: Optional[Dict[str, float]] = None,      # {"distance_mm":..,"angle_deg":..}
    ) -> Dict[str, Any]:
        """
        Ergebnisformat:
        paths_compiled = {
          "frame": "scene",
          "tool_frame": "tool_mount",
          "sides": {
            "<side>": {
              "meta": { path_type, path_params, globals, sample_step_mm, stand_off_mm, tool_frame, ... },
              "predispense": {x,y,z,qx,qy,qz,qw}?,   # optional
              "retreat":     {x,y,z,qx,qy,qz,qw}?,   # optional
              "poses_quat": [ {x,y,z,qx,qy,qz,qw}, ... ]
            }
          }
        }
        """
        xmin, xmax, ymin, ymax, zmin, zmax = bounds
        cx, cy, cz = (0.5 * (xmin + xmax), 0.5 * (ymin + ymax), 0.5 * (zmin + zmax))

        # Pflicht-Globals prüfen
        _ = self._globals_params_strict()

        # Compile-Parameter (Default aus parameters / planner)
        if stand_off_mm is None:
            stand_off_mm = float(self.parameters.get("stand_off_mm", 0.0))
        if tool_frame is None:
            tool_frame = str((self.planner or {}).get("tool_frame", "tool_mount"))
        if predispense is None:
            predispense = {
                "distance_mm": float(self.parameters.get("predispense.distance_mm", 30.0)),
                "angle_deg":   float(self.parameters.get("predispense.angle_deg",   15.0)),
            }
        if retreat is None:
            retreat = {
                "distance_mm": float(self.parameters.get("retreat.distance_mm", 30.0)),
                "angle_deg":   float(self.parameters.get("retreat.angle_deg",   15.0)),
            }

        face = {
            "top":   {"n": np.array([0, 0, -1.0]), "anchor": ("z", zmax), "axes": ("x", "y")},
            "front": {"n": np.array([0, 1,  0.0]), "anchor": ("y", ymin), "axes": ("x", "z")},
            "back":  {"n": np.array([0,-1,  0.0]), "anchor": ("y", ymax), "axes": ("x", "z")},
            "left":  {"n": np.array([1, 0,  0.0]), "anchor": ("x", xmin), "axes": ("y", "z")},
            "right": {"n": np.array([-1,0,  0.0]), "anchor": ("x", xmax), "axes": ("y", "z")},
            "helix": {"n": np.array([0, 0,  1.0]), "anchor": ("z", cz),   "axes": ("x", "y")},
        }

        def _safe_norm(v: np.ndarray) -> np.ndarray:
            v = np.asarray(v, float).reshape(3)
            n = float(np.linalg.norm(v))
            return v / (n if n > 1e-9 else 1.0)

        def _embed_local(P: np.ndarray, side: str) -> np.ndarray:
            cfg = face.get(side, face["top"])
            ax0, ax1 = cfg["axes"]
            anch_ax, anch_val = cfg["anchor"]
            P = np.asarray(P, float).reshape(-1, 3)
            out = np.empty_like(P)
            a0, a1 = P[:, 0], P[:, 1]
            mp = {"x": cx, "y": cy, "z": cz}
            def place(axis, arr): return (mp[axis] + arr)
            axes = {"x": None, "y": None, "z": None}
            axes[ax0] = place(ax0, a0)
            axes[ax1] = place(ax1, a1)
            axes[anch_ax] = np.full(P.shape[0], anch_val, float)
            out[:, 0], out[:, 1], out[:, 2] = axes["x"], axes["y"], axes["z"]
            return out

        def _quat_from_axes(x, y, z) -> Tuple[float, float, float, float]:
            x = _safe_norm(x); y = _safe_norm(y); z = _safe_norm(z)
            R = np.array([[x[0], y[0], z[0]],
                          [x[1], y[1], z[1]],
                          [x[2], y[2], z[2]]], float)
            t = float(np.trace(R))
            if t > 0:
                s = math.sqrt(t + 1.0) * 2.0
                qw = 0.25 * s
                qx = (R[2,1] - R[1,2]) / s
                qy = (R[0,2] - R[2,0]) / s
                qz = (R[1,0] - R[0,1]) / s
            else:
                i = int(np.argmax([R[0,0], R[1,1], R[2,2]]))
                if i == 0:
                    s = math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2.0
                    qw = (R[2,1] - R[1,2]) / s; qx = 0.25 * s
                    qy = (R[0,1] + R[1,0]) / s; qz = (R[0,2] + R[2,0]) / s
                elif i == 1:
                    s = math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2.0
                    qw = (R[0,2] - R[2,0]) / s
                    qx = (R[0,1] + R[1,0]) / s; qy = 0.25 * s
                    qz = (R[1,2] + R[2,1]) / s
                else:
                    s = math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2.0
                    qw = (R[1,0] - R[0,1]) / s
                    qx = (R[0,2] + R[2,0]) / s
                    qy = (R[1,2] + R[2,1]) / s; qz = 0.25 * s
            return float(qx), float(qy), float(qz), float(qw)

        sides_in = self._side_list(sides)
        sides_out: Dict[str, Any] = {}

        for side in sides_in:
            pdef = dict(self.paths_by_side.get(side, {}))

            # Schritt STRICT global
            step = float(self.parameters["sample_step_mm"])

            # Geometrie aus Cache (wenn vorhanden), sonst frisch generieren
            P_local = self._paths_cache.get(side)
            if P_local is None:
                from app.model.recipe.path_builder import PathBuilder
                pd = PathBuilder.from_side(
                    self,
                    side=side,
                    globals_params=self._globals_params_strict(),
                    sample_step_mm=step,
                    max_points=int(self.parameters["max_points"]),
                )
                P_local = np.asarray(pd.points_mm, float).reshape(-1, 3)
                source = pd.meta.get("source", "points")
            else:
                source = pdef.get("type", "points")

            cfg = face.get(side, face["top"])
            n = _safe_norm(cfg["n"])

            if P_local.shape[0] == 0:
                sides_out[side] = {
                    "meta": {
                        "side": side, "source": source, "path_type": pdef.get("type"),
                        "path_params": pdef, "globals": dict(self.parameters or {}),
                        "sample_step_mm": step, "stand_off_mm": float(stand_off_mm),
                        "tool_frame": tool_frame
                    },
                    "poses_quat": []
                }
                continue

            Pw = _embed_local(P_local, side)
            if abs(stand_off_mm) > 1e-9:
                Pw = Pw + n.reshape(1, 3) * stand_off_mm

            T = np.empty_like(Pw)
            T[0] = Pw[1] - Pw[0]
            T[-1] = Pw[-1] - Pw[-2] if len(Pw) >= 2 else np.array([1.0, 0.0, 0.0])
            if len(Pw) > 2:
                T[1:-1] = Pw[2:] - Pw[:-2]
            for i in range(len(T)):
                t = T[i] - n * float(np.dot(T[i], n))
                T[i] = (t / (np.linalg.norm(t) + 1e-12))
            for i in range(1, len(T)):
                if float(np.dot(T[i-1], T[i])) < 0.0:
                    T[i] = -T[i]

            poses: List[Dict[str, float]] = []
            for i in range(len(Pw)):
                x = T[i]
                y = np.cross(n, x)
                qx, qy, qz, qw = _quat_from_axes(x, y, n)
                poses.append({"x": float(Pw[i,0]), "y": float(Pw[i,1]), "z": float(Pw[i,2]),
                              "qx": qx, "qy": qy, "qz": qz, "qw": qw})

            pre_pose = None
            if predispense:
                dist = float(predispose.get("distance_mm", predispense.get("distance_mm", 30.0))) if "predispose" in locals() else float(predispose.get("distance_mm", 30.0))  # type: ignore
                ang  = math.radians(float(predispose.get("angle_deg", predispense.get("angle_deg", 0.0)))) if "predispose" in locals() else math.radians(float(predispose.get("angle_deg", 0.0)))  # type: ignore
                dir_out = ( (-math.cos(ang))*n + (-math.sin(ang))*T[0] )
                dir_out = dir_out / (np.linalg.norm(dir_out) + 1e-12)
                P0 = Pw[0] + dist * dir_out
                x0 = T[0]; y0 = np.cross(n, x0)
                qx, qy, qz, qw = _quat_from_axes(x0, y0, n)
                pre_pose = {"x": float(P0[0]), "y": float(P0[1]), "z": float(P0[2]),
                            "qx": qx, "qy": qy, "qz": qz, "qw": qw}

            ret_pose = None
            if retreat:
                dist = float(retreat.get("distance_mm", 30.0))
                ang  = math.radians(float(retreat.get("angle_deg", 0.0)))
                tend = T[-1]
                dir_out = ( (-math.cos(ang))*n + ( math.sin(ang))*tend )
                dir_out = dir_out / (np.linalg.norm(dir_out) + 1e-12)
                Pe = Pw[-1] + dist * dir_out
                xE = tend; yE = np.cross(n, xE)
                qx, qy, qz, qw = _quat_from_axes(xE, yE, n)
                ret_pose = {"x": float(Pe[0]), "y": float(Pe[1]), "z": float(Pe[2]),
                            "qx": qx, "qy": qy, "qz": qz, "qw": qw}

            meta = {
                "side": side,
                "source": source,
                "path_type": pdef.get("type"),
                "path_params": pdef,
                "globals": dict(self.parameters or {}),
                "sample_step_mm": step,
                "stand_off_mm": float(stand_off_mm),
                "tool_frame": tool_frame,
            }
            if predispense:
                meta["predispense_cfg"] = {
                    "distance_mm": float(predispose.get("distance_mm", predispense.get("distance_mm", 30.0))) if "predispose" in locals() else float(predispose.get("distance_mm", 30.0)),  # type: ignore
                    "angle_deg":   float(predispose.get("angle_deg",   predispense.get("angle_deg",   15.0))) if "predispose" in locals() else float(predispose.get("angle_deg", 15.0)),  # type: ignore
                }
            if retreat:
                meta["retreat_cfg"] = {
                    "distance_mm": float(retreat.get("distance_mm", 30.0)),
                    "angle_deg":   float(retreat.get("angle_deg",   15.0)),
                }

            item = {"meta": meta, "poses_quat": poses}
            if pre_pose: item["predispense"] = pre_pose
            if ret_pose: item["retreat"] = ret_pose
            sides_out[side] = item

        self.paths_compiled = {"frame": "scene", "tool_frame": tool_frame, "sides": sides_out}
        return self.paths_compiled

    # ------- Setter -------
    def set_parameters(self, params: Dict[str, Any]) -> None:
        self.parameters = dict(params or {})
        self.invalidate_paths()

    def set_planner(self, planner: Dict[str, Any]) -> None:
        self.planner = dict(planner or {})

    def set_paths_by_side(self, pbs: Dict[str, Dict[str, Any]]) -> None:
        self.paths_by_side = dict(pbs or {})
        self.invalidate_paths()

    def set_selection(self, *, tool: Optional[str], substrate: Optional[str], substrate_mount: Optional[str]) -> None:
        self.tool = tool; self.substrate = substrate; self.substrate_mount = substrate_mount
        self.invalidate_paths()
