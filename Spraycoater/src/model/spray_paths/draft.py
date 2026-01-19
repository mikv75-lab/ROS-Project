# -*- coding: utf-8 -*-
# File: src/model/spray_paths/draft.py
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping


# ============================================================
# Helpers (tolerant parsing; schema v1)
# ============================================================

def _float(v: Any, default: float = 0.0) -> float:
    try:
        return float(v)
    except Exception:
        return float(default)


def _as_dict(v: Any) -> Dict[str, Any]:
    return v if isinstance(v, dict) else {}


def _as_list(v: Any) -> List[Any]:
    return v if isinstance(v, list) else []


def _parse_pose_list(v: Any, *, name: str) -> List["PoseQuat"]:
    """
    Accepts:
      - list[dict]
      - dict (single pose -> wrapped into list)
      - None/other -> []
    """
    if isinstance(v, dict):
        v = [v]
    raw = _as_list(v)
    out: List[PoseQuat] = []
    for i, p in enumerate(raw):
        if isinstance(p, dict):
            out.append(PoseQuat.from_dict(p, name=f"{name}[{i}]"))
    return out


# ============================================================
# Data model
# ============================================================

@dataclass(frozen=True)
class PoseQuat:
    """Pose in millimeters plus unit quaternion."""

    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float

    @staticmethod
    def from_dict(d: Mapping[str, Any], *, name: str = "pose") -> "PoseQuat":
        d = dict(d or {})
        return PoseQuat(
            x=_float(d.get("x")),
            y=_float(d.get("y")),
            z=_float(d.get("z")),
            qx=_float(d.get("qx")),
            qy=_float(d.get("qy")),
            qz=_float(d.get("qz")),
            qw=_float(d.get("qw")),
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
    """
    Per-side TCP geometry.

    YAML v1 (extended):
      sides:
        top:
          predispense: [ {pose}, ... ]   # usually 1 pose (derived from first)
          retreat:     [ {pose}, ... ]   # usually 1 pose (derived from last)
          poses_quat:  [ {pose}, ... ]   # full path
    """

    predispense: List[PoseQuat] = field(default_factory=list)
    retreat: List[PoseQuat] = field(default_factory=list)
    poses_quat: List[PoseQuat] = field(default_factory=list)

    @staticmethod
    def from_dict(d: Mapping[str, Any], *, name: str = "side") -> "PathSide":
        dct = dict(d or {})

        pre = _parse_pose_list(dct.get("predispense", []), name=f"{name}.predispense")
        ret = _parse_pose_list(dct.get("retreat", []), name=f"{name}.retreat")
        poses = _parse_pose_list(dct.get("poses_quat", []), name=f"{name}.poses_quat")

        return PathSide(predispense=pre, retreat=ret, poses_quat=poses)

    def to_dict(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {}

        # Keep YAML tidy: only emit if present
        if self.predispense:
            out["predispense"] = [p.to_dict() for p in self.predispense]
        if self.retreat:
            out["retreat"] = [p.to_dict() for p in self.retreat]

        out["poses_quat"] = [p.to_dict() for p in self.poses_quat]
        return out


@dataclass(frozen=True)
class Draft:
    """
    Draft/TCP geometry document (v1)

    YAML shape:
      version: 1
      sides:
        top:
          predispense: [...]
          retreat: [...]
          poses_quat: [...]
        front: ...
    """

    version: int = 1
    sides: Dict[str, PathSide] = field(default_factory=dict)

    @staticmethod
    def from_yaml_dict(d: Mapping[str, Any], *, name: str = "draft") -> "Draft":
        dct = _as_dict(d)
        ver = int(dct.get("version", 1) or 1)

        sides_raw = dct.get("sides", {})
        if not isinstance(sides_raw, dict):
            sides_raw = {}

        sides: Dict[str, PathSide] = {}
        for k, v in sides_raw.items():
            if isinstance(v, dict):
                sides[str(k)] = PathSide.from_dict(v, name=f"{name}.sides[{k}]")

        return Draft(version=ver, sides=sides)

    def to_yaml_dict(self) -> Dict[str, Any]:
        return {
            "version": int(self.version or 1),
            "sides": {k: v.to_dict() for k, v in (self.sides or {}).items()},
        }

    # Convenience getters
    def poses_quat(self, side: str) -> List[PoseQuat]:
        s = str(side)
        ps = (self.sides or {}).get(s)
        return list(ps.poses_quat) if ps else []

    def predispense(self, side: str) -> List[PoseQuat]:
        s = str(side)
        ps = (self.sides or {}).get(s)
        return list(ps.predispense) if ps else []

    def retreat(self, side: str) -> List[PoseQuat]:
        s = str(side)
        ps = (self.sides or {}).get(s)
        return list(ps.retreat) if ps else []
