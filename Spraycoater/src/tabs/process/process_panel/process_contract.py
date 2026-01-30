# -*- coding: utf-8 -*-
# File: src/tabs/process/process_contract.py
import json
from dataclasses import dataclass
from typing import Any, Dict, Tuple

def _norm(s: Any) -> str:
    return str(s or "").strip()

def _as_dict(x: Any) -> Dict[str, Any]:
    return x if isinstance(x, dict) else {}

@dataclass(frozen=True)
class ReqKey:
    """Identifiziert einen Request (Variant-A)."""
    run: str
    id: int
    seg: str
    op: str

    def as_tuple(self) -> Tuple[str, int, str, str]:
        return (self.run, int(self.id), self.seg, self.op)

    def token_base(self) -> str:
        return f"run={self.run}|id={int(self.id)}|seg={self.seg}|op={self.op}"

def create_header_frame_id(run: str, req_id: int, seg: str, op: str) -> str:
    """Erstellt den JSON-String für header.frame_id."""
    return json.dumps(
        {"key": {"run": str(run), "id": int(req_id), "seg": str(seg), "op": str(op)}},
        ensure_ascii=False,
    )

def parse_motion_result(text: str) -> Tuple[ReqKey, str, Dict[str, Any]]:
    """Parst den String vom motionResultChanged Signal."""
    raw = _norm(text)
    if not raw:
        raise ValueError("motion_result empty")
    try:
        d = json.loads(raw)
    except Exception as e:
        raise ValueError(f"motion_result not valid JSON: {e!r}")

    if not isinstance(d, dict):
        raise ValueError("motion_result JSON is not an object")

    key = _as_dict(d.get("key"))
    status = _norm(d.get("status"))
    if not key or not status:
        raise ValueError("motion_result missing key/status")
    
    return _dict_to_reqkey(key), status, d

def parse_key_from_traj_header(obj: Any) -> ReqKey:
    """Extrahiert Key aus RobotTrajectoryMsg."""
    jt = getattr(obj, "joint_trajectory", None)
    if jt is None:
        raise ValueError("trajectory has no joint_trajectory")
    hdr = getattr(jt, "header", None)
    if hdr is None:
        raise ValueError("trajectory.joint_trajectory has no header")
    frame_id = _norm(getattr(hdr, "frame_id", ""))
    if not frame_id:
        raise ValueError("trajectory header.frame_id empty")

    try:
        d = json.loads(frame_id)
    except Exception as e:
        raise ValueError(f"trajectory key JSON invalid: {e!r}")
    
    k = _as_dict(d.get("key")) if "key" in d and isinstance(d.get("key"), dict) else d
    return _dict_to_reqkey(k)

def _dict_to_reqkey(k: Dict[str, Any]) -> ReqKey:
    run = _norm(k.get("run"))
    seg = _norm(k.get("seg"))
    op = _norm(k.get("op"))
    rid = k.get("id")
    if not run: raise ValueError("key.run empty")
    if not isinstance(rid, int): raise ValueError(f"key.id must be int, got {type(rid)}")
    if not seg: raise ValueError("key.seg empty")
    if not op: raise ValueError("key.op empty")
    return ReqKey(run=run, id=int(rid), seg=seg, op=op)