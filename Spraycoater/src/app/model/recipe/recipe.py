# -*- coding: utf-8 -*-
# app/model/recipe/recipe.py
from __future__ import annotations

import os
import math
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Optional, Tuple

import yaml
import numpy as np


@dataclass
class Recipe:
    """
    Rezeptdaten + Laufzeit-Caches + optionale, gespeicherte Kompilate.

    Felder:
      - parameters:     globale Instanzwerte
      - planner:        planer-/pipeline-nahe Instanzwerte (frei)
      - paths_by_side:  pro Side eine Path-Definition (dict)
      - _paths_cache:   rohe Pfadpunkte (N,3) je Side (nur RAM)
      - paths_compiled: gespeicherte Posen (Quaternionen) + Meta
      - info:           abgeleitete Kennzahlen (z.B. Längen, Punktanzahl)
    """

    # Meta / Auswahl
    id: str
    description: str = ""
    tool: Optional[str] = None
    substrate: Optional[str] = None
    substrate_mount: Optional[str] = None

    # Instanzparameter
    parameters: Dict[str, Any] = field(default_factory=dict)
    planner: Dict[str, Any] = field(default_factory=dict)

    # Pfad-Definitionen pro Side
    paths_by_side: Dict[str, Dict[str, Any]] = field(default_factory=dict)

    # Laufzeit (nicht speichern)
    _paths_cache: Dict[str, np.ndarray] = field(default_factory=dict, repr=False, compare=False)

    # Ergebnis (wird gespeichert)
    paths_compiled: Dict[str, Any] = field(default_factory=dict)

    # Abgeleitete Info (wird gespeichert)
    info: Dict[str, Any] = field(default_factory=dict)

    # ---------- YAML ----------
    @staticmethod
    def from_dict(d: Dict[str, Any]) -> "Recipe":
        return Recipe(
            id=str(d.get("id") or "recipe"),
            description=str(d.get("description") or ""),
            tool=d.get("tool"),
            substrate=d.get("substrate"),
            substrate_mount=d.get("substrate_mount"),
            parameters=dict(d.get("parameters") or {}),
            planner=dict(d.get("planner") or {}),
            paths_by_side=dict(d.get("paths_by_side") or {}),
            paths_compiled=dict(d.get("paths_compiled") or {}),
            info=dict(d.get("info") or {}),
        )

    @staticmethod
    def _to_plain(obj: Any) -> Any:
        """
        Rekursive Normalisierung auf YAML-freundliche Python-Typen:
        - numpy scalars -> Python scalars
        - dict/list/tuple/set -> rekursiv
        - ndarray -> List
        - sonst -> str(obj)
        """
        if isinstance(obj, np.generic):
            return obj.item()

        if isinstance(obj, (str, bool, int, float)) or obj is None:
            return obj

        if isinstance(obj, dict):
            return {str(k): Recipe._to_plain(v) for k, v in obj.items()}

        if isinstance(obj, (list, tuple, set)):
            return [Recipe._to_plain(x) for x in obj]

        if isinstance(obj, np.ndarray):
            return [Recipe._to_plain(x) for x in obj.tolist()]

        return str(obj)

    def to_dict(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {
            "id": self.id,
            "description": self.description,
            "tool": self.tool,
            "substrate": self.substrate,
            "substrate_mount": self.substrate_mount,
            "parameters": Recipe._to_plain(self.parameters or {}),
            "planner": Recipe._to_plain(self.planner or {}),
            "paths_by_side": Recipe._to_plain({s: dict(p or {}) for s, p in (self.paths_by_side or {}).items()}),
        }
        if self.paths_compiled:
            out["paths_compiled"] = Recipe._to_plain(self.paths_compiled)
        if self.info:
            out["info"] = Recipe._to_plain(self.info)
        return out

    @staticmethod
    def load_yaml(path: str) -> "Recipe":
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return Recipe.from_dict(dict(data))

    def save_yaml(self, path: str) -> None:
        """
        Speichert das Rezept als YAML.

        Wenn paths_compiled vorhanden ist, wird info vorher aktualisiert,
        damit die gespeicherten Kennzahlen konsistent bleiben.
        """
        if self.paths_compiled:
            self._recompute_info_from_compiled()

        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        data = self.to_dict()
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)

    # ------- benötigte Globals -------
    def _globals_params_strict(self) -> Dict[str, Any]:
        """
        Extrahiert die Globals, die für Pfaderzeugung/Kompilierung benötigt werden.
        """
        g = dict(self.parameters or {})
        required = ["stand_off_mm", "max_angle_deg", "sample_step_mm", "max_points"]
        missing = [k for k in required if k not in g]
        if missing:
            raise ValueError(f"Recipe: fehlende Parameter: {missing}")
        return {k: g[k] for k in required}

    # ------- PLC: Payload aus aktuellen parameters bauen -------
    def plc_payload_from_schema(self, plc_globals_schema: Dict[str, Any]) -> Dict[str, Any]:
        """
        Gibt {name: value} für alle Parameter zurück, die im übergebenen Schema
        enthalten sind und in self.parameters existieren.
        """
        out: Dict[str, Any] = {}
        params = self.parameters or {}
        for name in plc_globals_schema.keys():
            if name in params:
                out[name] = params[name]
        return out

    # ------- Pfad-Cache -------
    def _side_list(self, sides: Optional[Iterable[str]]) -> List[str]:
        return [str(s) for s in sides] if sides else list(self.paths_by_side.keys())

    def invalidate_paths(self) -> None:
        self._paths_cache.clear()

    def rebuild_paths(
        self,
        *,
        sides: Optional[Iterable[str]] = None,
        max_points: Optional[int] = None,
    ) -> Dict[str, np.ndarray]:
        """
        Erzeugt rohe Geometriepfade (N,3) je Side (nur RAM).
        - sample_step_mm kommt aus parameters
        - max_points kommt aus parameters oder wird explizit übergeben
        """
        from app.model.recipe.path_builder import PathBuilder

        _ = self._globals_params_strict()

        if max_points is None:
            max_points = int(self.parameters["max_points"])

        step = float(self.parameters["sample_step_mm"])

        if not sides:
            self._paths_cache.clear()

        for side in self._side_list(sides):
            pd = PathBuilder.from_side(
                self,
                side=side,
                globals_params=self._globals_params_strict(),
                sample_step_mm=step,
                max_points=max_points,
            )
            self._paths_cache[side] = np.asarray(pd.points_mm, float).reshape(-1, 3)

        return self._paths_cache

    @property
    def paths(self) -> Dict[str, np.ndarray]:
        return self._paths_cache

    # ------- kompilierten Pfad als Punktwolke (mm) holen -------
    def compiled_points_mm_for_side(self, side: str) -> Optional[np.ndarray]:
        """Gibt kompilierten (x,y,z)-Punktpfad (mm) für eine Side zurück, falls vorhanden."""
        pc = self.paths_compiled or {}
        sides = pc.get("sides") or {}
        sdata = sides.get(side)
        if not isinstance(sdata, dict):
            return None

        poses = sdata.get("poses_quat") or []
        if not poses:
            return None

        P = np.array(
            [[float(p.get("x", 0.0)), float(p.get("y", 0.0)), float(p.get("z", 0.0))] for p in poses],
            dtype=float,
        ).reshape(-1, 3)
        return P

    # ------- interne Info-Berechnung aus paths_compiled -------
    def _recompute_info_from_compiled(self) -> None:
        """
        Aktualisiert self.info auf Basis von paths_compiled.

        Neu berechnet:
          - total_points
          - total_length_mm
          - sides[side].num_points / length_mm

        Alle anderen Keys in self.info bleiben erhalten.
        """
        pc = self.paths_compiled or {}
        sides = pc.get("sides") or {}
        old = dict(self.info or {})

        if not isinstance(sides, dict) or not sides:
            info_new: Dict[str, Any] = {"total_points": 0, "total_length_mm": 0.0, "sides": {}}
            for k, v in old.items():
                if k not in ("total_points", "total_length_mm", "sides"):
                    info_new[k] = v
            self.info = info_new
            return

        total_points = 0
        total_length = 0.0
        sides_info: Dict[str, Any] = {}

        for side, sdata in sides.items():
            poses = (sdata or {}).get("poses_quat") or []
            if not poses:
                sides_info[str(side)] = {"num_points": 0, "length_mm": 0.0}
                continue

            P = np.array(
                [[float(p.get("x", 0.0)), float(p.get("y", 0.0)), float(p.get("z", 0.0))] for p in poses],
                dtype=float,
            ).reshape(-1, 3)

            num = int(P.shape[0])
            length = 0.0
            if num >= 2:
                d = np.linalg.norm(P[1:] - P[:-1], axis=1)
                length = float(d.sum())

            sides_info[str(side)] = {"num_points": num, "length_mm": length}
            total_points += num
            total_length += length

        info_new: Dict[str, Any] = {"total_points": total_points, "total_length_mm": total_length, "sides": sides_info}
        for k, v in old.items():
            if k not in ("total_points", "total_length_mm", "sides"):
                info_new[k] = v

        self.info = info_new

    # ------- Quaternion-Posen erzeugen -------
    def compile_poses(
        self,
        *,
        bounds: Tuple[float, float, float, float, float, float],
        sides: Optional[Iterable[str]] = None,
        stand_off_mm: Optional[float] = None,
        tool_frame: Optional[str] = None,
    ) -> Dict[str, Any]:
        from app.model.recipe.path_builder import PathBuilder

        xmin, xmax, ymin, ymax, zmin, zmax = bounds
        cx, cy, cz = (0.5 * (xmin + xmax), 0.5 * (ymin + ymax), 0.5 * (zmin + zmax))

        _ = self._globals_params_strict()

        if stand_off_mm is None:
            stand_off_mm = float(self.parameters.get("stand_off_mm", 0.0))
        if tool_frame is None:
            tool_frame = str((self.planner or {}).get("tool_frame", "tool_mount"))

        explicit_scene_z_off = self.parameters.get("scene_z_offset_mm", None)
        auto_z_off = float(zmin)
        z_offset_top_default = float(explicit_scene_z_off) if explicit_scene_z_off is not None else auto_z_off

        face = {
            "top":   {"surface_n": np.array([0, 0,  1.0]), "anchor": ("z", zmax), "axes": ("x", "y")},
            "front": {"surface_n": np.array([0, -1, 0.0]), "anchor": ("y", ymin), "axes": ("x", "z")},
            "back":  {"surface_n": np.array([0,  1, 0.0]), "anchor": ("y", ymax), "axes": ("x", "z")},
            "left":  {"surface_n": np.array([-1, 0, 0.0]), "anchor": ("x", xmin), "axes": ("y", "z")},
            "right": {"surface_n": np.array([ 1, 0, 0.0]), "anchor": ("x", xmax), "axes": ("y", "z")},
            "helix": {"surface_n": np.array([0, 0,  1.0]), "anchor": ("z", cz),   "axes": ("x", "y")},
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

            def place(axis: str, arr: np.ndarray) -> np.ndarray:
                return mp[axis] + arr

            axes: Dict[str, Optional[np.ndarray]] = {"x": None, "y": None, "z": None}
            axes[ax0] = place(ax0, a0)
            axes[ax1] = place(ax1, a1)
            axes[anch_ax] = np.full(P.shape[0], anch_val, float)

            out[:, 0], out[:, 1], out[:, 2] = axes["x"], axes["y"], axes["z"]
            return out

        def _quat_from_axes(x: np.ndarray, y: np.ndarray, z: np.ndarray) -> Tuple[float, float, float, float]:
            x = _safe_norm(x)
            y = _safe_norm(y)
            z = _safe_norm(z)

            R = np.array(
                [[x[0], y[0], z[0]],
                 [x[1], y[1], z[1]],
                 [x[2], y[2], z[2]]],
                float,
            )
            t = float(np.trace(R))

            if t > 0:
                s = math.sqrt(t + 1.0) * 2.0
                qw = 0.25 * s
                qx = (R[2, 1] - R[1, 2]) / s
                qy = (R[0, 2] - R[2, 0]) / s
                qz = (R[1, 0] - R[0, 1]) / s
            else:
                i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
                if i == 0:
                    s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
                    qw = (R[2, 1] - R[1, 2]) / s
                    qx = 0.25 * s
                    qy = (R[0, 1] + R[1, 0]) / s
                    qz = (R[0, 2] + R[2, 0]) / s
                elif i == 1:
                    s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
                    qw = (R[0, 2] - R[2, 0]) / s
                    qx = (R[0, 1] + R[1, 0]) / s
                    qy = 0.25 * s
                    qz = (R[1, 2] + R[2, 1]) / s
                else:
                    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
                    qw = (R[1, 0] - R[0, 1]) / s
                    qx = (R[0, 2] + R[2, 0]) / s
                    qy = (R[1, 2] + R[2, 1]) / s
                    qz = 0.25 * s

            return float(qx), float(qy), float(qz), float(qw)

        sides_in = self._side_list(sides)
        sides_out: Dict[str, Any] = {}

        for side in sides_in:
            pdef = dict(self.paths_by_side.get(side, {}))
            step = float(self.parameters["sample_step_mm"])

            P_local = self._paths_cache.get(side)
            if P_local is None:
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
            surface_n = _safe_norm(cfg["surface_n"])
            tool_z = -surface_n  # Tool zeigt zur Fläche

            if P_local.shape[0] == 0:
                sides_out[side] = {
                    "meta": {
                        "side": side,
                        "source": source,
                        "path_type": pdef.get("type"),
                        "path_params": pdef,
                        "globals": dict(self.parameters or {}),
                        "sample_step_mm": step,
                        "stand_off_mm": float(stand_off_mm),
                        "tool_frame": tool_frame,
                        "angle_hints": _extract_angle_hints(pdef),
                        "z_offset_top_mm": z_offset_top_default if side == "top" else 0.0,
                    },
                    "poses_quat": [],
                }
                continue

            Pw = _embed_local(P_local, side)

            if abs(float(stand_off_mm)) > 1e-9:
                Pw = Pw + surface_n.reshape(1, 3) * float(stand_off_mm)

            if side == "top":
                Pw = Pw.copy()
                Pw[:, 2] -= z_offset_top_default

            # Tangenten (projiziert in Ebene senkrecht zu tool_z)
            T = np.empty_like(Pw)
            T[0] = Pw[1] - Pw[0]
            T[-1] = Pw[-1] - Pw[-2] if len(Pw) >= 2 else np.array([1.0, 0.0, 0.0])
            if len(Pw) > 2:
                T[1:-1] = Pw[2:] - Pw[:-2]

            for i in range(len(T)):
                t = T[i] - tool_z * float(np.dot(T[i], tool_z))
                nrm = float(np.linalg.norm(t))
                T[i] = (t / nrm) if nrm > 1e-12 else np.array([1.0, 0.0, 0.0])

            for i in range(1, len(T)):
                if float(np.dot(T[i - 1], T[i])) < 0.0:
                    T[i] = -T[i]

            poses: List[Dict[str, float]] = []
            for i in range(len(Pw)):
                x = T[i]
                y = np.cross(tool_z, x)
                qx, qy, qz, qw = _quat_from_axes(x, y, tool_z)
                poses.append(
                    {
                        "x": float(Pw[i, 0]),
                        "y": float(Pw[i, 1]),
                        "z": float(Pw[i, 2]),
                        "qx": qx,
                        "qy": qy,
                        "qz": qz,
                        "qw": qw,
                    }
                )

            meta: Dict[str, Any] = {
                "side": side,
                "source": source,
                "path_type": pdef.get("type"),
                "path_params": pdef,
                "globals": dict(self.parameters or {}),
                "sample_step_mm": step,
                "stand_off_mm": float(stand_off_mm),
                "tool_frame": tool_frame,
                "angle_hints": _extract_angle_hints(pdef),
            }
            if side == "top":
                meta["z_offset_top_mm"] = z_offset_top_default

            sides_out[side] = {"meta": meta, "poses_quat": poses}

        self.paths_compiled = {"frame": "scene", "tool_frame": tool_frame, "sides": sides_out}
        self._recompute_info_from_compiled()
        return self.paths_compiled

    # ---------- Text-Repräsentation ----------
    def __str__(self) -> str:
        lines: List[str] = []

        lines.append(f"id: {self.id or ''}")
        if self.description:
            lines.append(f"description: {self.description}")
        if self.tool:
            lines.append(f"tool: {self.tool}")
        if self.substrate:
            lines.append(f"substrate: {self.substrate}")
        if self.substrate_mount:
            lines.append(f"substrate_mount: {self.substrate_mount}")

        if self.parameters:
            lines.append("")
            lines.append("parameters:")
            for k in sorted(self.parameters.keys()):
                lines.append(f"  {k}: {self.parameters[k]}")

        if self.planner:
            lines.append("")
            lines.append("planner:")
            for k in sorted(self.planner.keys()):
                lines.append(f"  {k}: {self.planner[k]}")

        if self.info:
            lines.append("")
            lines.append("info:")
            total_pts = self.info.get("total_points")
            total_len = self.info.get("total_length_mm")
            if total_pts is not None:
                lines.append(f"  total_points: {total_pts}")
            if total_len is not None:
                lines.append(f"  total_length_mm: {total_len}")

            sides = self.info.get("sides") or {}
            if isinstance(sides, dict) and sides:
                lines.append("  sides:")
                for s, d in sides.items():
                    lines.append(f"    {s}:")
                    if d.get("num_points") is not None:
                        lines.append(f"      num_points: {d.get('num_points')}")
                    if d.get("length_mm") is not None:
                        lines.append(f"      length_mm: {d.get('length_mm')}")

        if self.paths_by_side:
            lines.append("")
            lines.append("paths_by_side:")
            for side, p in self.paths_by_side.items():
                lines.append(f"  {side}:")
                for pk, pv in (p or {}).items():
                    if pk in ("points_mm", "polyline_mm"):
                        continue
                    lines.append(f"    {pk}: {pv}")

        pc = self.paths_compiled or {}
        sides = pc.get("sides") or {}
        if isinstance(sides, dict) and sides:
            frame = pc.get("frame")
            tool_frame = pc.get("tool_frame")

            lines.append("")
            lines.append("# compiled poses")
            if frame:
                lines.append(f"frame: {frame}")
            if tool_frame:
                lines.append(f"tool_frame: {tool_frame}")

            for side, sdata in sides.items():
                lines.append(f"[side: {side}]")
                poses = (sdata or {}).get("poses_quat") or []
                if not poses:
                    lines.append("  (no poses)")
                    continue

                for i, p in enumerate(poses):
                    x = float(p.get("x", 0.0))
                    y = float(p.get("y", 0.0))
                    z = float(p.get("z", 0.0))
                    qx = float(p.get("qx", 0.0))
                    qy = float(p.get("qy", 0.0))
                    qz = float(p.get("qz", 0.0))
                    qw = float(p.get("qw", 1.0))
                    lines.append(
                        f"  {i:04d}: "
                        f"x={x:.3f}, y={y:.3f}, z={z:.3f}, "
                        f"qx={qx:.4f}, qy={qy:.4f}, qz={qz:.4f}, qw={qw:.4f}"
                    )

        return "\n".join(lines)


def _extract_angle_hints(path_params: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    def pick(prefix: str) -> Dict[str, Any]:
        out: Dict[str, Any] = {}
        am = path_params.get(f"{prefix}.angle_mode")
        ad = path_params.get(f"{prefix}.angle_deg")
        if am is not None:
            out["angle_mode"] = am
        if ad is not None:
            out["angle_deg"] = ad
        return out

    return {
        "predispense": pick("predispense"),
        "retreat": pick("retreat"),
    }
