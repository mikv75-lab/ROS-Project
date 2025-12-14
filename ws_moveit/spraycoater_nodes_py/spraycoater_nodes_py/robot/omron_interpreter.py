# spraycoater_nodes_py/robot/omron_interpreter.py
# -*- coding: utf-8 -*-
from __future__ import annotations

from typing import Any, Dict, List, Optional


class OmronInterpreter:
    """
    Schlanker Interpreter für die Textantworten des Omron-TCP-Servers.

    Beispieleingaben:
      - "STATUS J 0 10 20 30 40 50"
      - "ACK PING"
      - "NACK TRAJSTART"

    Rückgabe von parse_line():
      - None                        → Zeile ignorieren
      - {"type": "status_joints", ...}
      - {"type": "ack", ...}
      - {"type": "nack", ...}
      - {"type": "other", ...}
    """

    def __init__(self) -> None:
        self._last_joints: Optional[List[float]] = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def parse_line(self, line: str) -> Optional[Dict[str, Any]]:
        """Analysiert eine Antwortzeile vom Server und gibt ein Dict zurück."""
        if not line:
            return None

        line = line.strip()
        if not line:
            return None

        if line.startswith("STATUS "):
            return self._parse_status(line)

        if line.startswith("ACK "):
            return self._parse_ack(line)

        if line.startswith("NACK "):
            return self._parse_nack(line)

        return {
            "type": "other",
            "raw": line,
        }

    # ------------------------------------------------------------------
    # STATUS
    # ------------------------------------------------------------------
    def _parse_status(self, line: str) -> Optional[Dict[str, Any]]:
        """
        Erwartet aktuell:
            STATUS J j1 j2 j3 j4 j5 j6
        """
        parts = line.split()
        if len(parts) < 2:
            return None

        kind = parts[1]

        if kind == "J":
            # STATUS J + 6 Werte
            if len(parts) < 8:
                return None
            try:
                joints = [float(p) for p in parts[2:8]]
            except ValueError:
                return None

            self._last_joints = joints
            return {
                "type": "status_joints",
                "joints": joints,
                "raw": line,
            }

        # für spätere Erweiterungen (STATUS T, STATUS S, ...)
        return {
            "type": "status_unknown",
            "raw": line,
        }

    # ------------------------------------------------------------------
    # ACK / NACK
    # ------------------------------------------------------------------
    def _parse_ack(self, line: str) -> Dict[str, Any]:
        parts = line.split(maxsplit=2)
        cmd = parts[1] if len(parts) > 1 else ""
        rest = parts[2] if len(parts) > 2 else ""
        return {
            "type": "ack",
            "cmd": cmd,
            "rest": rest,
            "raw": line,
        }

    def _parse_nack(self, line: str) -> Dict[str, Any]:
        parts = line.split(maxsplit=2)
        cmd = parts[1] if len(parts) > 1 else ""
        rest = parts[2] if len(parts) > 2 else ""
        return {
            "type": "nack",
            "cmd": cmd,
            "rest": rest,
            "raw": line,
        }
