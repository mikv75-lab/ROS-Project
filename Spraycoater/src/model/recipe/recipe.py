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
# draft.yaml  (coordinates only; no meta; no normals)
# planned_tcp.yaml / executed_tcp.yaml  (coordinates only; same schema as draft)
#   version: 1
#   sides:
#     <side>:
#       poses_quat:
#         - {x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float}
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


# ============================================================
# Path (draft/planned_tcp/executed_tcp)
# ============================================================

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
class PathSide:
    poses_quat: List[PoseQuat] = field(default_factory=list)

    @staticmethod
    def from_dict(d: Mapping[str, Any], *, name: str = "side") -> "PathSide":
        dd = _as_dict(d, name=name)
        poses = _as_list(dd.get("poses_quat", []), name=f"{name}.poses_quat")
        poses_q = [PoseQuat.from_dict(_as_dict(p, name=f"{name}.pose")) for p in poses]
        if not poses_q:
            raise ValueError(f"{name}.poses_quat ist leer.")
        return PathSide(poses_quat=poses_q)

    def to_dict(self) -> Dict[str, Any]:
        return {"poses_quat": [p.to_dict() for p in self.poses_quat]}


@dataclass(frozen=True)
class Path:
    """
    Path v1 = pure geometry.
    Used for:
      - draft.yaml
      - planned_tcp.yaml
      - executed_tcp.yaml
    """
    version: int = 1
    sides: Dict[str, PathSide] = field(default_factory=dict)

    @staticmethod
    def from_yaml_dict(d: Mapping[str, Any], *, name: str = "path.yaml") -> "Path":
        root = _as_dict(dict(d), name=name)
        version = _require_int(root.get("version", 1), name=f"{name}.version")
        if version != 1:
            raise ValueError(f"{name}: version muss 1 sein, ist {version}")

        sides_raw = _as_dict(root.get("sides", {}), name=f"{name}.sides")
        sides: Dict[str, PathSide] = {}
        for side, v in sides_raw.items():
            s = str(side)
            sides[s] = PathSide.from_dict(_as_dict(v, name=f"{name}.sides[{s}]"), name=f"{name}.sides[{s}]")
        if not sides:
            raise ValueError(f"{name}: sides ist leer.")
        return Path(version=version, sides=sides)

    def to_yaml_dict(self) -> Dict[str, Any]:
        return {"version": 1, "sides": {k: v.to_dict() for k, v in self.sides.items()}}

    def poses_quat(self, side: str) -> List[PoseQuat]:
        if side not in self.sides:
            raise KeyError(f"sides hat keinen Eintrag für side={side!r}")
        return self.sides[side].poses_quat


# Backwards naming: keep Draft as alias to Path (so other modules can still import Draft)
Draft = Path


# ============================================================
# planned_traj.yaml / executed_traj.yaml
# ============================================================

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
        return {"joint_names": list(self.joint_names), "points": [p.to_dict() for p in self.points]}


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
        return {"version": 1, "segments": {k: v.to_dict() for k, v in self.segments.items()}}

    # ------------------------------------------------------------
    # Convert to trajectory_msgs/JointTrajectory (concat)
    # ------------------------------------------------------------

    def to_joint_trajectory(self, *, segment_order: Optional[List[str]] = None):
        """
        Convert this JTBySegment into a single trajectory_msgs/JointTrajectory.

        - Concatenates segments in given segment_order (if provided), else in dict insertion order.
        - Ensures time_from_start is monotonically non-decreasing across concatenation by applying
          an offset at each segment boundary.
        - Returns None if ROS messages are unavailable (import error).
        """
        try:
            from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  # type: ignore
            from builtin_interfaces.msg import Duration as DurationMsg  # type: ignore
        except Exception:
            return None

        if not self.segments:
            return None

        order: List[str]
        if segment_order:
            order = [s for s in segment_order if s in self.segments]
            for s in self.segments.keys():
                if s not in order:
                    order.append(s)
        else:
            order = list(self.segments.keys())

        first = self.segments[order[0]]
        joint_names = list(first.joint_names)

        for sid in order[1:]:
            seg = self.segments[sid]
            if list(seg.joint_names) != joint_names:
                raise ValueError(
                    f"JTBySegment.to_joint_trajectory: joint_names mismatch in segment '{sid}'. "
                    f"expected={joint_names}, got={seg.joint_names}"
                )

        jt = JointTrajectory()
        jt.joint_names = joint_names

        off_sec = 0
        off_nsec = 0

        def _add_time(a_sec: int, a_nsec: int, b_sec: int, b_nsec: int) -> Tuple[int, int]:
            n = a_nsec + b_nsec
            s = a_sec + b_sec + (n // 1_000_000_000)
            n = n % 1_000_000_000
            return s, n

        def _max_time_in_segment(seg: JTSegment) -> Tuple[int, int]:
            ms, mn = 0, 0
            for p in seg.points:
                s, n = int(p.time_from_start[0]), int(p.time_from_start[1])
                if (s > ms) or (s == ms and n > mn):
                    ms, mn = s, n
            return ms, mn

        for sid in order:
            seg = self.segments[sid]

            for p in seg.points:
                pt = JointTrajectoryPoint()
                pt.positions = [float(x) for x in p.positions]

                s, n = _add_time(off_sec, off_nsec, int(p.time_from_start[0]), int(p.time_from_start[1]))
                d = DurationMsg()
                d.sec = int(s)
                d.nanosec = int(n)
                pt.time_from_start = d

                jt.points.append(pt)

            ms, mn = _max_time_in_segment(seg)
            off_sec, off_nsec = _add_time(off_sec, off_nsec, ms, mn)

        return jt


# ============================================================
# Recipe (params + attachments)
# ============================================================

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
    trajectories: Dict[str, Any] = field(default_factory=dict)          # Editor scratch
    tool_frame: Optional[str] = None                                   # optional
    paths_compiled: Optional[Dict[str, Any]] = None                     # optional (Preview compiled)

    # attachments (optional; strict schemas)
    draft: Optional[Draft] = None
    planned_traj: Optional[JTBySegment] = None
    executed_traj: Optional[JTBySegment] = None
    planned_tcp: Optional[Path] = None
    executed_tcp: Optional[Path] = None

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
            out["meta"] = dict(self.meta or {})
        if self.info:
            out["info"] = dict(self.info or {})
        return out

    # ----------------------------
    # canonical sampling parameters (from params.yaml)
    # ----------------------------

    def sample_step_mm(self, default: float = 1.0) -> float:
        try:
            v = float(self.parameters.get("sample_step_mm", default))
            return v if v > 0 else float(default)
        except Exception:
            return float(default)

    def max_points(self, default: int = 500) -> int:
        try:
            v = int(self.parameters.get("max_points", default))
            return v if v > 0 else int(default)
        except Exception:
            return int(default)

    # ----------------------------
    # convenience used by planning/process code
    # ----------------------------

    def draft_poses_quat(self, side: str) -> List[PoseQuat]:
        if not self.draft:
            raise KeyError("Recipe hat kein draft geladen.")
        return self.draft.poses_quat(side)

    def compiled_points_mm_for_side(self, side: str) -> List[List[float]]:
        poses = self.draft_poses_quat(side)
        return [[p.x, p.y, p.z] for p in poses]

    def planned_tcp_poses_quat(self, side: str) -> List[PoseQuat]:
        if self.planned_tcp is None:
            raise KeyError("Recipe hat kein planned_tcp geladen.")
        return self.planned_tcp.poses_quat(side)

    def executed_tcp_poses_quat(self, side: str) -> List[PoseQuat]:
        if self.executed_tcp is None:
            raise KeyError("Recipe hat kein executed_tcp geladen.")
        return self.executed_tcp.poses_quat(side)

    # ------------------------------------------------------------
    # traj convenience used by markers/FK tooling
    # ------------------------------------------------------------

    def planned_joint_trajectory(self, *, segment_order: Optional[List[str]] = None):
        """
        Returns a concatenated trajectory_msgs/JointTrajectory for planned_traj (or None if missing).
        """
        if self.planned_traj is None:
            return None
        return self.planned_traj.to_joint_trajectory(segment_order=segment_order)

    def executed_joint_trajectory(self, *, segment_order: Optional[List[str]] = None):
        """
        Returns a concatenated trajectory_msgs/JointTrajectory for executed_traj (or None if missing).
        """
        if self.executed_traj is None:
            return None
        return self.executed_traj.to_joint_trajectory(segment_order=segment_order)
