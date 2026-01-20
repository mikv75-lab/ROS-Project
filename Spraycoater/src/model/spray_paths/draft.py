# -*- coding: utf-8 -*-
# File: src/model/spray_paths/draft.py
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, Sequence, Optional


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


def _as_str(v: Any) -> str:
    return str(v or "").strip()


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


def _pose_close(a: "PoseQuat", b: "PoseQuat", *, pos_eps: float = 1e-6, quat_eps: float = 1e-6) -> bool:
    return (
        abs(a.x - b.x) <= pos_eps
        and abs(a.y - b.y) <= pos_eps
        and abs(a.z - b.z) <= pos_eps
        and abs(a.qx - b.qx) <= quat_eps
        and abs(a.qy - b.qy) <= quat_eps
        and abs(a.qz - b.qz) <= quat_eps
        and abs(a.qw - b.qw) <= quat_eps
    )


def _strip_prefix(full: Sequence["PoseQuat"], prefix: Sequence["PoseQuat"]) -> List["PoseQuat"]:
    if not prefix:
        return list(full)
    if len(full) < len(prefix):
        return list(full)
    for i in range(len(prefix)):
        if not _pose_close(full[i], prefix[i]):
            return list(full)
    return list(full[len(prefix) :])


def _strip_suffix(full: Sequence["PoseQuat"], suffix: Sequence["PoseQuat"]) -> List["PoseQuat"]:
    if not suffix:
        return list(full)
    if len(full) < len(suffix):
        return list(full)
    off = len(full) - len(suffix)
    for i in range(len(suffix)):
        if not _pose_close(full[off + i], suffix[i]):
            return list(full)
    return list(full[:off])


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
          predispense: [ {pose}, ... ]   # may be 1 pose or a short path
          retreat:     [ {pose}, ... ]   # may be 1 pose or a short path
          poses_quat:  [ {pose}, ... ]   # full path (may include pre/ret depending on producer)
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

    Supports BOTH:
      - recipe drafts (predispense/retreat in PathSide)
      - tcp docs produced by TrajFkBuilder (frame/segments/meta/segments_included[/eval])

    YAML shape (recipe):
      version: 1
      sides:
        top:
          predispense: [...]
          retreat: [...]
          poses_quat: [...]

    YAML shape (tcp from FK):
      version: 1
      frame: substrate
      sides: { ... }                       # optional (global)
      segments:
        MOVE_RECIPE:
          sides:
            top: { poses_quat: [...] }
      meta: { segment_slices: ... }
      segments_included: [MOVE_RECIPE, ...]
      eval: {...}                          # optional embedded results
    """

    version: int = 1

    # Optional but important for strict validation and round-trip
    frame: str = ""

    # Global sides (recipe drafts OR concatenated TCP sides)
    sides: Dict[str, PathSide] = field(default_factory=dict)

    # Per-segment sides (TCP docs from FK)
    # segments[seg_id][side] -> PathSide
    segments: Dict[str, Dict[str, PathSide]] = field(default_factory=dict)

    # Misc metadata from FK builder (segment_slices etc.)
    meta: Dict[str, Any] = field(default_factory=dict)

    # FK builder included segments
    segments_included: List[str] = field(default_factory=list)

    # Embedded evaluation (optional)
    eval: Dict[str, Any] = field(default_factory=dict)

    @staticmethod
    def from_yaml_dict(d: Mapping[str, Any], *, name: str = "draft") -> "Draft":
        dct = _as_dict(d)
        ver = int(dct.get("version", 1) or 1)
        frame = _as_str(dct.get("frame") or "")

        # global sides
        sides_raw = dct.get("sides", {})
        if not isinstance(sides_raw, dict):
            sides_raw = {}
        sides: Dict[str, PathSide] = {}
        for k, v in sides_raw.items():
            if isinstance(v, dict):
                sides[str(k)] = PathSide.from_dict(v, name=f"{name}.sides[{k}]")

        # segments -> per-seg "sides"
        segs_raw = dct.get("segments", {})
        segments: Dict[str, Dict[str, PathSide]] = {}
        if isinstance(segs_raw, dict):
            for seg_id, seg_obj in segs_raw.items():
                if not isinstance(seg_obj, dict):
                    continue
                seg_sides = seg_obj.get("sides", {})
                if not isinstance(seg_sides, dict):
                    continue
                parsed: Dict[str, PathSide] = {}
                for side, side_obj in seg_sides.items():
                    if isinstance(side_obj, dict):
                        parsed[str(side)] = PathSide.from_dict(
                            side_obj, name=f"{name}.segments[{seg_id}].sides[{side}]"
                        )
                if parsed:
                    segments[str(seg_id)] = parsed

        meta = dct.get("meta", {})
        if not isinstance(meta, dict):
            meta = {}

        seg_inc = dct.get("segments_included", [])
        if not isinstance(seg_inc, list):
            seg_inc = []
        segments_included = [_as_str(x) for x in seg_inc if _as_str(x)]

        ev = dct.get("eval", {})
        if not isinstance(ev, dict):
            ev = {}

        return Draft(
            version=ver,
            frame=frame,
            sides=sides,
            segments=segments,
            meta=dict(meta),
            segments_included=list(segments_included),
            eval=dict(ev),
        )

    def to_yaml_dict(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {
            "version": int(self.version or 1),
            "sides": {k: v.to_dict() for k, v in (self.sides or {}).items()},
        }

        fr = _as_str(self.frame)
        if fr:
            out["frame"] = fr

        if self.segments:
            out["segments"] = {
                seg_id: {"sides": {side: ps.to_dict() for side, ps in seg_sides.items()}}
                for seg_id, seg_sides in self.segments.items()
                if seg_sides
            }

        if self.meta:
            out["meta"] = dict(self.meta)

        if self.segments_included:
            out["segments_included"] = list(self.segments_included)

        if self.eval:
            out["eval"] = dict(self.eval)

        return out

    # ------------------------------------------------------------
    # Convenience getters
    # ------------------------------------------------------------

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

    # ------------------------------------------------------------
    # STRICT: recipe-only extraction + segment API
    # ------------------------------------------------------------

    def recipe_poses_quat(self, side: str) -> List[PoseQuat]:
        """
        Returns ONLY the recipe portion of the path.

        If producers accidentally concatenate pre/ret into poses_quat, we remove:
          - an exact prefix match of side.predispense
          - an exact suffix match of side.retreat

        Tolerant compare is used; if no match, nothing is stripped.
        """
        s = str(side)
        ps = (self.sides or {}).get(s)
        if not ps:
            return []

        full = list(ps.poses_quat)
        if not full:
            return []

        pre = list(ps.predispense)
        ret = list(ps.retreat)

        out = full
        if pre and len(out) > len(pre) + 1:
            out = _strip_prefix(out, pre)
        if ret and len(out) > len(ret) + 1:
            out = _strip_suffix(out, ret)

        return list(out)

    def poses_quat_segment(self, side: str, seg_id: str) -> List[PoseQuat]:
        """
        Segment accessor for evaluators.

        Priority:
          1) If this Draft contains FK-style segments, use them (segments[seg_id][side].poses_quat).
          2) Else: recipe mapping:
               MOVE_PREDISPENSE -> predispense(side)
               MOVE_RETREAT     -> retreat(side)
               MOVE_RECIPE      -> recipe_poses_quat(side)
               default          -> recipe_poses_quat(side)
        """
        s = str(side)
        sid = _as_str(seg_id)

        # 1) FK-style segment data
        segmap = (self.segments or {}).get(sid)
        if segmap:
            ps = segmap.get(s)
            if ps:
                return list(ps.poses_quat)

        # 2) Recipe-style fallback
        if sid == "MOVE_PREDISPENSE":
            return self.predispense(s)
        if sid == "MOVE_RETREAT":
            return self.retreat(s)
        return self.recipe_poses_quat(s)
