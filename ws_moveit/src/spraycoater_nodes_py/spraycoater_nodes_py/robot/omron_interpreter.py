# spraycoater_nodes_py/robot/omron_interpreter.py
# -*- coding: utf-8 -*-
from __future__ import annotations

from typing import Any, Dict, List, Optional


class OmronInterpreter:
    """
    Schlanker Interpreter für Textantworten des Omron-TCP-Servers.

    Unterstützte Beispieleingaben (erweiterbar):
      - "STATUS J 0 10 20 30 40 50"
      - "STATUS P x y z qx qy qz qw"
      - "STATUS M <mode>"
      - "ACK <cmd> <optional...>"
      - "NACK <cmd> <optional...>"
      - "ERR <text...>" / "ERROR <text...>"

    Rückgabe von parse_line():
      - None → Zeile ignorieren
      - {"type": "...", ...}
    """

    def __init__(self) -> None:
        self._last_joints: Optional[List[float]] = None
        self._last_pose: Optional[Dict[str, Any]] = None
        self._last_mode: Optional[str] = None
        self._last_error: Optional[str] = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def parse_line(self, line: str) -> Optional[Dict[str, Any]]:
        if not line:
            return None

        line = line.strip()
        if not line:
            return None

        if line.startswith("STATUS "):
            return self._parse_status(line)

        if line.startswith("ACK"):
            return self._parse_ack(line)

        if line.startswith("NACK"):
            return self._parse_nack(line)

        if line.startswith("ERR ") or line.startswith("ERROR "):
            return self._parse_err(line)

        return {"type": "other", "raw": line}

    # ------------------------------------------------------------------
    # STATUS
    # ------------------------------------------------------------------
    def _parse_status(self, line: str) -> Optional[Dict[str, Any]]:
        """
        Erwartete Formen (aktuell):
          - STATUS J j1 j2 j3 j4 j5 j6
          - STATUS P x y z qx qy qz qw
          - STATUS M <mode...>
        """
        parts = line.split()
        if len(parts) < 2:
            return None

        kind = parts[1].upper()

        # ---------------- Joints ----------------
        if kind in ("J", "JOINTS"):
            if len(parts) < 8:
                return None
            try:
                joints = [float(p) for p in parts[2:8]]
            except ValueError:
                return None

            self._last_joints = joints
            return {"type": "status_joints", "joints": joints, "raw": line}

        # ---------------- Pose ----------------
        if kind in ("P", "POSE", "TCP"):
            # STATUS P x y z qx qy qz qw
            if len(parts) < 10:
                return None
            try:
                x, y, z = float(parts[2]), float(parts[3]), float(parts[4])
                qx, qy, qz, qw = float(parts[5]), float(parts[6]), float(parts[7]), float(parts[8])
            except ValueError:
                return None

            pose = {
                "xyz": [x, y, z],
                "quat_xyzw": [qx, qy, qz, qw],
            }
            self._last_pose = pose
            return {"type": "status_pose", "pose": pose, "raw": line}

        # ---------------- Mode ----------------
        if kind in ("M", "MODE"):
            mode = " ".join(parts[2:]).strip()
            self._last_mode = mode
            return {"type": "status_mode", "mode": mode, "raw": line}

        # Fallback
        return {"type": "status_unknown", "raw": line}

    # ------------------------------------------------------------------
    # ACK / NACK / ERR
    # ------------------------------------------------------------------
    def _parse_ack(self, line: str) -> Dict[str, Any]:
        parts = line.split(maxsplit=2)
        cmd = parts[1] if len(parts) > 1 else ""
        rest = parts[2] if len(parts) > 2 else ""
        return {"type": "ack", "cmd": cmd, "rest": rest, "raw": line}

    def _parse_nack(self, line: str) -> Dict[str, Any]:
        parts = line.split(maxsplit=2)
        cmd = parts[1] if len(parts) > 1 else ""
        rest = parts[2] if len(parts) > 2 else ""
        return {"type": "nack", "cmd": cmd, "rest": rest, "raw": line}

    def _parse_err(self, line: str) -> Dict[str, Any]:
        # "ERR ..." oder "ERROR ..."
        parts = line.split(maxsplit=1)
        msg = parts[1] if len(parts) > 1 else ""
        self._last_error = msg
        return {"type": "error", "message": msg, "raw": line}

    # ------------------------------------------------------------------
    # Optional: getters (falls du sie irgendwo nutzen willst)
    # ------------------------------------------------------------------
    @property
    def last_joints(self) -> Optional[List[float]]:
        return self._last_joints

    @property
    def last_pose(self) -> Optional[Dict[str, Any]]:
        return self._last_pose

    @property
    def last_mode(self) -> Optional[str]:
        return self._last_mode

    @property
    def last_error(self) -> Optional[str]:
        return self._last_error
