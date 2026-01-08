# -*- coding: utf-8 -*-
# File: src/model/recipe/recipe.py
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, Optional, Tuple

# ============================================================
# YAML Schemas (v1, strict; no legacy)
# ============================================================
#
# params.yaml  (authoritative parameters/meta; stored separately)
#   id: str
#   description: str
#   tool: str | null
#   substrate: str | null
#   substrate_mount: str | null
#   parameters: dict
#   planner: dict
#   paths_by_side: dict
#   meta: dict (optional)
#   info: dict (optional)
#
# draft.yaml  (coordinates only; no meta)
#   version: 1
#   sides:
#     <side>:
#       poses_quat:
#         - {x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float}
#       normals_xyz:            # optional (same length as poses_quat)
#         - {x: float, y: float, z: float}
#
# planned_traj.yaml / executed_traj.yaml  (replay-able JointTrajectory by segment)
#   version: 1
#   segments:
#     <SEGMENT_ID>:
#       joint_names: [str, ...]
#       points:
#         - positions: [float, ...]
#           time_from_start: [sec:int, nanosec:int]
#         - ...


def _as_dict(x: Any, *, name: str) -> Dict[str, Any]:
    if not isinstance(x, dict):
        raise ValueError(f"{name} muss dict sein, ist {type(x).__name__}")
    return x


def _as_list(x: Any, *, name: str) -> List[Any]:
    if not isinstance(x, list):
        raise ValueError(f"{name} muss list sein, ist {type(x).__name__}")
    return x


def _require_int(x: Any, *, name: str) -> int:
    try:
        return int(x)
    except Exception:
        raise ValueError(f"{name} muss int sein, ist {x!r}")


def _require_float(x: Any, *, name: str) -> float:
    try:
        return float(x)
    except Exception:
        raise ValueError(f"{name} muss float sein, ist {x!r}")


@dataclass(frozen=True)
class PoseQuat:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float

    @staticmethod
    def from_dict(d: Mapping[str, Any]) -> "PoseQuat":
        return PoseQuat(
            x=_require_float(d.get("x"), name="pose.x"),
            y=_require_float(d.get("y"), name="pose.y"),
            z=_require_float(d.get("z"), name="pose.z"),
            qx=_require_float(d.get("qx"), name="pose.qx"),
            qy=_require_float(d.get("qy"), name="pose.qy"),
            qz=_require_float(d.get("qz"), name="pose.qz"),
            qw=_require_float(d.get("qw"), name="pose.qw"),
        )

    def to_dict(self) -> Dict[str, float]:
        return {
            "x": float(self.x),
            "y": float(self.y),
            "z": float(self.z),
            "qx": float(self.qx),
            "qy": float(self.qy),
            "qz": float(self.qz),
            "qw": float(self.qw),
        }


@dataclass(frozen=True)
class Vec3:
    x: float
    y: float
    z: float

    @staticmethod
    def from_dict(d: Mapping[str, Any]) -> "Vec3":
        return Vec3(
            x=_require_float(d.get("x"), name="vec.x"),
            y=_require_float(d.get("y"), name="vec.y"),
            z=_require_float(d.get("z"), name="vec.z"),
        )

    def to_dict(self) -> Dict[str, float]:
        return {"x": float(self.x), "y": float(self.y), "z": float(self.z)}


@dataclass(frozen=True)
class DraftSide:
    poses_quat: List[PoseQuat] = field(default_factory=list)
    normals_xyz: Optional[List[Vec3]] = None

    @staticmethod
    def from_dict(d: Mapping[str, Any]) -> "DraftSide":
        poses = [_as_dict(p, name="draft.pose") for p in _as_list(d.get("poses_quat", []), name="poses_quat")]
        poses_q = [PoseQuat.from_dict(p) for p in poses]

        normals_raw = d.get("normals_xyz", None)
        normals: Optional[List[Vec3]] = None
        if normals_raw is not None:
            normals_list = _as_list(normals_raw, name="normals_xyz")
            normals = [Vec3.from_dict(_as_dict(n, name="draft.normal")) for n in normals_list]
            if len(normals) != len(poses_q):
                raise ValueError(
                    f"draft.normals_xyz muss gleiche Länge wie poses_quat haben "
                    f"({len(normals)} != {len(poses_q)})"
                )

        return DraftSide(poses_quat=poses_q, normals_xyz=normals)

    def to_dict(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {"poses_quat": [p.to_dict() for p in self.poses_quat]}
        if self.normals_xyz is not None:
            out["normals_xyz"] = [n.to_dict() for n in self.normals_xyz]
        return out


@dataclass(frozen=True)
class Draft:
    version: int = 1
    sides: Dict[str, DraftSide] = field(default_factory=dict)

    @staticmethod
    def from_yaml_dict(d: Mapping[str, Any]) -> "Draft":
        root = _as_dict(dict(d), name="draft.yaml")
        version = _require_int(root.get("version", 1), name="draft.version")
        if version != 1:
            raise ValueError(f"draft.yaml: version muss 1 sein, ist {version}")

        sides_raw = _as_dict(root.get("sides", {}), name="draft.sides")
        sides: Dict[str, DraftSide] = {}
        for side, v in sides_raw.items():
            sides[str(side)] = DraftSide.from_dict(_as_dict(v, name=f"draft.sides[{side}]"))
        if not sides:
            raise ValueError("draft.yaml: sides ist leer.")
        return Draft(version=version, sides=sides)

    def to_yaml_dict(self) -> Dict[str, Any]:
        return {
            "version": 1,
            "sides": {k: v.to_dict() for k, v in self.sides.items()},
        }


@dataclass(frozen=True)
class JTPoint:
    positions: List[float]
    time_from_start: Tuple[int, int]  # (sec, nanosec)

    @staticmethod
    def from_dict(d: Mapping[str, Any]) -> "JTPoint":
        dd = _as_dict(d, name="jt.point")
        pos = _as_list(dd.get("positions", None), name="positions")
        positions = [_require_float(x, name="positions[]") for x in pos]
        tfs = _as_list(dd.get("time_from_start", None), name="time_from_start")
        if len(tfs) != 2:
            raise ValueError("time_from_start muss [sec, nanosec] sein.")
        sec = _require_int(tfs[0], name="time_from_start[0]")
        nsec = _require_int(tfs[1], name="time_from_start[1]")
        if sec < 0 or nsec < 0:
            raise ValueError("time_from_start darf nicht negativ sein.")
        return JTPoint(positions=positions, time_from_start=(sec, nsec))

    def to_dict(self) -> Dict[str, Any]:
        return {
            "positions": [float(x) for x in self.positions],
            "time_from_start": [int(self.time_from_start[0]), int(self.time_from_start[1])],
        }


@dataclass(frozen=True)
class JTSegment:
    joint_names: List[str]
    points: List[JTPoint]

    @staticmethod
    def from_dict(d: Mapping[str, Any]) -> "JTSegment":
        dd = _as_dict(d, name="jt.segment")
        jn = _as_list(dd.get("joint_names", None), name="joint_names")
        joint_names = [str(x) for x in jn]
        pts = _as_list(dd.get("points", None), name="points")
        points = [JTPoint.from_dict(_as_dict(p, name="jt.point")) for p in pts]
        if not joint_names:
            raise ValueError("jt.segment.joint_names ist leer.")
        if not points:
            raise ValueError("jt.segment.points ist leer.")
        for p in points:
            if len(p.positions) != len(joint_names):
                raise ValueError(
                    f"jt.point.positions Länge passt nicht zu joint_names "
                    f"({len(p.positions)} != {len(joint_names)})"
                )
        return JTSegment(joint_names=joint_names, points=points)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "joint_names": list(self.joint_names),
            "points": [p.to_dict() for p in self.points],
        }


@dataclass(frozen=True)
class JTBySegment:
    version: int = 1
    segments: Dict[str, JTSegment] = field(default_factory=dict)

    @staticmethod
    def from_yaml_dict(d: Mapping[str, Any]) -> "JTBySegment":
        root = _as_dict(dict(d), name="traj.yaml")
        version = _require_int(root.get("version", 1), name="traj.version")
        if version != 1:
            raise ValueError(f"traj.yaml: version muss 1 sein, ist {version}")

        segs_raw = _as_dict(root.get("segments", {}), name="traj.segments")
        segs: Dict[str, JTSegment] = {}
        for seg_id, seg in segs_raw.items():
            segs[str(seg_id)] = JTSegment.from_dict(_as_dict(seg, name=f"traj.segments[{seg_id}]"))
        if not segs:
            raise ValueError("traj.yaml: segments ist leer.")
        return JTBySegment(version=version, segments=segs)

    def to_yaml_dict(self) -> Dict[str, Any]:
        return {
            "version": 1,
            "segments": {k: v.to_dict() for k, v in self.segments.items()},
        }


@dataclass
class Recipe:
    """
    Recipe = params + optional attachments (draft/planned/executed).

    Persistence lives in RecipeRepo/RecipeBundle.
    """

    # params (authoritative)
    id: str
    description: str = ""
    tool: Optional[str] = None
    substrate: Optional[str] = None
    substrate_mount: Optional[str] = None
    parameters: Dict[str, Any] = field(default_factory=dict)
    planner: Dict[str, Any] = field(default_factory=dict)
    paths_by_side: Dict[str, Any] = field(default_factory=dict)
    meta: Dict[str, Any] = field(default_factory=dict)
    info: Dict[str, Any] = field(default_factory=dict)

    # UI/runtime convenience (not part of strict YAML schemas)
    trajectories: Dict[str, Any] = field(default_factory=dict)          # Editor setzt das zurück
    tool_frame: Optional[str] = None                                   # optional
    paths_compiled: Optional[Dict[str, Any]] = None                     # optional (Preview/TCP compiled)

    # attachments (optional; strict schemas)
    draft: Optional[Draft] = None
    planned_traj: Optional[JTBySegment] = None
    executed_traj: Optional[JTBySegment] = None

    # ----------------------------
    # Constructors / converters
    # ----------------------------

    @staticmethod
    def from_params_dict(d: Mapping[str, Any]) -> "Recipe":
        dd = _as_dict(d, name="params.yaml")
        rid = str(dd.get("id", "")).strip()
        if not rid:
            raise ValueError("params.yaml: id fehlt/leer.")
        return Recipe(
            id=rid,
            description=str(dd.get("description", "") or ""),
            tool=dd.get("tool", None),
            substrate=dd.get("substrate", None),
            substrate_mount=dd.get("substrate_mount", None),
            parameters=_as_dict(dd.get("parameters", {}), name="parameters"),
            planner=_as_dict(dd.get("planner", {}), name="planner"),
            paths_by_side=_as_dict(dd.get("paths_by_side", {}), name="paths_by_side"),
            meta=_as_dict(dd.get("meta", {}), name="meta"),
            info=_as_dict(dd.get("info", {}), name="info"),
        )

    @staticmethod
    def from_dict(d: Mapping[str, Any]) -> "Recipe":
        """
        UI-compat constructor.

        RecipeEditorPanel baut ein Dict mit Keys:
          id, description, tool, substrate, substrate_mount,
          parameters, planner, paths_by_side,
          trajectories (optional)
        Zusätzlich tolerieren wir:
          meta, info, tool_frame, paths_compiled
        """
        dd = _as_dict(dict(d), name="recipe")
        rid = str(dd.get("id", "")).strip()
        if not rid:
            raise ValueError("recipe: id fehlt/leer.")

        params = dd.get("parameters", {})
        planner = dd.get("planner", {})
        pbs = dd.get("paths_by_side", {})

        # trajectories kann beliebig sein (Editor nutzt es als Scratch)
        traj = dd.get("trajectories", {})
        if traj is None:
            traj = {}
        if not isinstance(traj, dict):
            raise ValueError("recipe.trajectories muss dict sein (oder fehlen).")

        meta = dd.get("meta", {}) or {}
        info = dd.get("info", {}) or {}

        tf = dd.get("tool_frame", None)
        pc = dd.get("paths_compiled", None)

        return Recipe(
            id=rid,
            description=str(dd.get("description", "") or ""),
            tool=dd.get("tool", None),
            substrate=dd.get("substrate", None),
            substrate_mount=dd.get("substrate_mount", None),
            parameters=_as_dict(params, name="recipe.parameters"),
            planner=_as_dict(planner, name="recipe.planner"),
            paths_by_side=_as_dict(pbs, name="recipe.paths_by_side"),
            meta=_as_dict(meta, name="recipe.meta"),
            info=_as_dict(info, name="recipe.info"),
            trajectories=dict(traj),
            tool_frame=(str(tf) if tf is not None else None),
            paths_compiled=(dict(pc) if isinstance(pc, dict) else None),
        )

    def to_params_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "description": self.description or "",
            "tool": self.tool,
            "substrate": self.substrate,
            "substrate_mount": self.substrate_mount,
            "parameters": dict(self.parameters or {}),
            "planner": dict(self.planner or {}),
            "paths_by_side": dict(self.paths_by_side or {}),
            "meta": dict(self.meta or {}),
            "info": dict(self.info or {}),
        }

    def to_dict(self) -> Dict[str, Any]:
        """
        UI-friendly export (mirrors from_dict inputs).
        Not intended as strict persistence format.
        """
        out: Dict[str, Any] = {
            "id": self.id,
            "description": self.description or "",
            "tool": self.tool,
            "substrate": self.substrate,
            "substrate_mount": self.substrate_mount,
            "parameters": dict(self.parameters or {}),
            "planner": dict(self.planner or {}),
            "paths_by_side": dict(self.paths_by_side or {}),
            "trajectories": dict(self.trajectories or {}),
        }
        if self.tool_frame is not None:
            out["tool_frame"] = self.tool_frame
        if self.paths_compiled is not None:
            out["paths_compiled"] = dict(self.paths_compiled)
        if self.meta:
            out["meta"] = dict(self.meta)
        if self.info:
            out["info"] = dict(self.info)
        return out

    # ----------------------------
    # convenience used by planning/process code
    # ----------------------------

    def draft_poses_quat(self, side: str) -> List[PoseQuat]:
        if not self.draft:
            raise KeyError("Recipe hat kein draft geladen.")
        if side not in self.draft.sides:
            raise KeyError(f"draft.sides hat keinen Eintrag für side={side!r}")
        return self.draft.sides[side].poses_quat

    def compiled_points_mm_for_side(self, side: str) -> List[List[float]]:
        # Backwards-friendly accessor: returns [[x,y,z], ...] in mm.
        poses = self.draft_poses_quat(side)
        return [[p.x, p.y, p.z] for p in poses]
