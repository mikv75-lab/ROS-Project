# -*- coding: utf-8 -*-
# File: src/tabs/process/process_serialization.py
import copy
from typing import Any, Dict, List, Optional
from builtin_interfaces.msg import Duration as RosDuration
from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def is_empty_robot_trajectory(obj: Any) -> bool:
    try:
        jt = getattr(obj, "joint_trajectory", None)
        if jt is None: return True
        pts = list(getattr(jt, "points", []) or [])
        return len(pts) == 0
    except Exception:
        return False

def jt_msg_to_dict_normalized(traj_msg: Any) -> Optional[Dict[str, Any]]:
    """RobotTrajectoryMsg -> Dict {joint_names, points: [{positions, time_from_start:[s,ns]}]}."""
    if traj_msg is None: return None
    try:
        jt = getattr(traj_msg, "joint_trajectory", None)
        if jt is None: return None
        jn = [str(x) for x in list(getattr(jt, "joint_names", []) or [])]
        pts_msg = list(getattr(jt, "points", []) or [])
        if not jn or not pts_msg: return None
        
        pts = []
        for p in pts_msg:
            tfs = getattr(p, "time_from_start", None)
            sec = int(getattr(tfs, "sec", 0)) if tfs is not None else 0
            nsec = int(getattr(tfs, "nanosec", 0)) if tfs is not None else 0
            pts.append({
                "positions": [float(x) for x in list(getattr(p, "positions", []) or [])],
                "time_from_start": [sec, nsec],
            })
        return {"joint_names": jn, "points": pts}
    except Exception:
        return None

def concat_jt_dicts_sequential(dicts: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    """Klebt mehrere JT-Dicts hintereinander (Merge) und korrigiert Zeiten."""
    if not dicts: return None
    base_names = list(dicts[0].get("joint_names") or [])
    merged_pts = []
    t_offset_ns = 0
    last_global_ns = -1

    for d in dicts:
        pts = d.get("points") or []
        for p in pts:
            tfs = p.get("time_from_start") or [0, 0]
            t_local_ns = int(tfs[0]) * 1_000_000_000 + int(tfs[1])
            t_global_ns = t_offset_ns + t_local_ns
            if t_global_ns <= last_global_ns:
                t_global_ns = last_global_ns + 1_000_000 # +1ms safety
            
            q = copy.deepcopy(p)
            q["time_from_start"] = [t_global_ns // 1_000_000_000, t_global_ns % 1_000_000_000]
            merged_pts.append(q)
            last_global_ns = t_global_ns
        t_offset_ns = last_global_ns

    return {"joint_names": base_names, "points": merged_pts}

def jt_dict_to_robottrajectory_msg(jt_dict: Dict[str, Any], header_frame_id: str) -> RobotTrajectoryMsg:
    """Dict -> RobotTrajectoryMsg."""
    jn = jt_dict.get("joint_names") or []
    pts = jt_dict.get("points") or []
    
    jt = JointTrajectory()
    jt.joint_names = [str(x) for x in jn]
    jt.header.frame_id = header_frame_id

    out_pts = []
    for p in pts:
        if not isinstance(p, dict): continue
        pos = p.get("positions")
        tfs = p.get("time_from_start")
        
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in pos]
        d = RosDuration()
        if isinstance(tfs, (list, tuple)) and len(tfs) >= 2:
            d.sec, d.nanosec = int(tfs[0]), int(tfs[1])
        elif isinstance(tfs, dict):
            d.sec, d.nanosec = int(tfs.get("sec", 0)), int(tfs.get("nanosec", 0))
        pt.time_from_start = d
        out_pts.append(pt)

    jt.points = out_pts
    rt = RobotTrajectoryMsg()
    rt.joint_trajectory = jt
    return rt