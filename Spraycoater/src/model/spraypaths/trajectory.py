# -*- coding: utf-8 -*-
# File: src/model/recipe/trajectory.py
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, Tuple, Optional

@dataclass(frozen=True)
class JTPoint:
    positions: List[float]
    time_from_start: Tuple[int, int]  # (sec, nanosec)

    @staticmethod
    def from_dict(d: Mapping[str, Any]) -> JTPoint:
        pos = d.get("positions", [])
        tfs = d.get("time_from_start", [0, 0])
        return JTPoint(
            positions=[float(x) for x in (pos if isinstance(pos, list) else [])],
            time_from_start=(int(tfs[0]), int(tfs[1])) if isinstance(tfs, list) and len(tfs)==2 else (0,0)
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "positions": self.positions,
            "time_from_start": [self.time_from_start[0], self.time_from_start[1]]
        }

@dataclass(frozen=True)
class JTSegment:
    joint_names: List[str]
    points: List[JTPoint]

    @staticmethod
    def from_dict(d: Mapping[str, Any]) -> JTSegment:
        jn = d.get("joint_names", [])
        pts = d.get("points", [])
        return JTSegment(
            joint_names=[str(x) for x in (jn if isinstance(jn, list) else [])],
            points=[JTPoint.from_dict(p) for p in (pts if isinstance(pts, list) else []) if isinstance(p, dict)]
        )

    def to_dict(self) -> Dict[str, Any]:
        return {"joint_names": self.joint_names, "points": [p.to_dict() for p in self.points]}

@dataclass(frozen=True)
class JTBySegment:
    """
    Repräsentiert eine JointTrajectory, aufgeteilt nach Segmenten (planned_traj, executed_traj).
    """
    version: int = 1
    segments: Dict[str, JTSegment] = field(default_factory=dict)

    @staticmethod
    def from_yaml_dict(d: Mapping[str, Any]) -> JTBySegment:
        d = dict(d or {})
        ver = int(d.get("version", 1) or 1)
        segs_raw = d.get("segments") or {}
        if not isinstance(segs_raw, dict): segs_raw = {}
        
        segments = {k: JTSegment.from_dict(v) for k, v in segs_raw.items() if isinstance(v, dict)}
        return JTBySegment(version=ver, segments=segments)

    def to_yaml_dict(self) -> Dict[str, Any]:
        return {"version": self.version, "segments": {k: v.to_dict() for k, v in self.segments.items()}}

    def to_joint_trajectory(self, *, segment_order: Optional[List[str]] = None) -> Any:
        """
        Konvertiert zu trajectory_msgs/JointTrajectory (ROS Message).
        Gibt None zurück, wenn trajectory_msgs nicht importierbar ist (z.B. in reiner UI Umgebung).
        """
        try:
            from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint # type: ignore
            from builtin_interfaces.msg import Duration # type: ignore
        except ImportError:
            return None

        if not self.segments: return JointTrajectory()

        msg = JointTrajectory()
        # Nehme Joint Names vom ersten Segment (Annahme: konstant über Trajektorie)
        first_seg = next(iter(self.segments.values()))
        msg.joint_names = list(first_seg.joint_names)

        order = segment_order or list(self.segments.keys())
        current_time_ns = 0

        for seg_id in order:
            if seg_id not in self.segments: continue
            seg = self.segments[seg_id]
            
            # Konsistenzcheck
            if seg.joint_names != msg.joint_names:
                continue # oder log warn

            for pt in seg.points:
                ros_pt = JointTrajectoryPoint()
                ros_pt.positions = pt.positions
                
                # Zeit akkumulieren
                pt_ns = pt.time_from_start[0] * 1_000_000_000 + pt.time_from_start[1]
                total_ns = current_time_ns + pt_ns
                
                sec = total_ns // 1_000_000_000
                nanosec = total_ns % 1_000_000_000
                ros_pt.time_from_start = Duration(sec=int(sec), nanosec=int(nanosec))
                
                msg.points.append(ros_pt)
            
            # Offset für nächstes Segment setzen (basierend auf letztem Punkt dieses Segments)
            if seg.points:
                last_pt = seg.points[-1]
                last_ns = last_pt.time_from_start[0] * 1_000_000_000 + last_pt.time_from_start[1]
                current_time_ns += last_ns

        return msg