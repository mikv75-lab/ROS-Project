# -*- coding: utf-8 -*-
# spraycoater_nodes_py/utils/omron_tcp_client.py

from __future__ import annotations

import socket
import time


class OmronProtocolError(Exception):
    """Fehler im OMRON-Protokoll (NACK, NDONE, TIMEOUT, Ungültige Antwort)."""
    pass


class OmronTcpClient:
    """
    TCP-Client für den Omron V+ TCP-Server.

    - sendet 1:1 ASCII-Befehle ohne CRLF
    - liest Zeilen solange bis ACK + DONE/NDONE ankommen
    - thread-safe? Nein, aber sequentiell sicher im Node genutzt
    """

    def __init__(self, host: str, port: int, timeout: float = 3.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock: socket.socket | None = None

    # ----------------------------------------------------------------------
    # Verbindung aufbauen
    # ----------------------------------------------------------------------
    def connect(self) -> None:
        if self.sock:
            return

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(self.timeout)

        s.connect((self.host, self.port))
        self.sock = s

    # ----------------------------------------------------------------------
    # Verbindung schließen
    # ----------------------------------------------------------------------
    def close(self) -> None:
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
        self.sock = None

    # ----------------------------------------------------------------------
    # Low-Level senden (keine CRLF!)
    # ----------------------------------------------------------------------
    def _send_line(self, line: str) -> None:
        if not self.sock:
            raise OmronProtocolError("Not connected")

        # V+ erwartet ASCII exakt so, ohne \n \r
        data = line.encode("ascii", errors="ignore")
        self.sock.sendall(data)

    # ----------------------------------------------------------------------
    # Low-Level lesen (liest bis CR/LF oder Timeout)
    # ----------------------------------------------------------------------
    def _read_line(self) -> str:
        """
        Liest eine Zeile vom Omron-Server.
        Der V+ Server beendet Zeilen mit CR (ASCII 13).
        Wir akzeptieren CR oder LF oder EOF.
        """
        if not self.sock:
            raise OmronProtocolError("Not connected")

        buf = bytearray()
        end_time = time.time() + self.timeout

        while time.time() < end_time:
            try:
                b = self.sock.recv(1)
            except socket.timeout:
                raise OmronProtocolError("Timeout while reading")

            if not b:
                # Verbindung beendet
                raise OmronProtocolError("Socket closed by server")

            if b == b'\r' or b == b'\n':
                break

            buf.extend(b)

            # Sicherheit
            if len(buf) > 1024:
                raise OmronProtocolError("Line too long")

        return buf.decode("ascii", errors="ignore")

    # ----------------------------------------------------------------------
    # High-Level API: EIN einzelnes Kommando senden
    # ----------------------------------------------------------------------
    def send_command(self, cmd: str) -> bool:
        """
        sendet ein Kommando (z.B. "PING") und
        wartet auf ACK + DONE/NDONE.

        Rückgabe:
          True  = DONE
          False = NDONE
        """

        if not self.sock:
            raise OmronProtocolError("Not connected")

        # -----------------------------------------------------
        # 1. Senden
        # -----------------------------------------------------
        self._send_line(cmd)

        # -----------------------------------------------------
        # 2. ACK abwarten
        # -----------------------------------------------------
        line = self._read_line()
        if not line.startswith("ACK"):
            raise OmronProtocolError(f"Expected ACK, got '{line}'")

        # -----------------------------------------------------
        # 3. DONE oder NDONE abwarten
        # -----------------------------------------------------
        line = self._read_line()
        if line.startswith("DONE"):
            return True
        if line.startswith("NDONE"):
            return False

        raise OmronProtocolError(f"Expected DONE/NDONE, got '{line}'")

    # ----------------------------------------------------------------------
    # Trajektorie Kommandos
    # ----------------------------------------------------------------------
    def traj_start(self) -> None:
        ok = self.send_command("TRAJSTART")
        if not ok:
            raise OmronProtocolError("TRAJSTART NDONE")

    def traj_point(self, t: float, joint_values: list[float]) -> None:
        """
        joint_values = [j1,j2,j3,j4,j5,j6] (radians)
        V+ erwartet GRAD!
        """
        if len(joint_values) != 6:
            raise ValueError("traj_point requires 6 joint values")

        # ros → v+ conversion: rad → deg
        jdeg = [v * 180.0 / 3.141592653589793 for v in joint_values]

        # Format: TRAJPOINT t j1 j2 j3 j4 j5 j6
        cmd = (
            f"TRAJPOINT {t:.6f} "
            f"{jdeg[0]:.6f} {jdeg[1]:.6f} {jdeg[2]:.6f} "
            f"{jdeg[3]:.6f} {jdeg[4]:.6f} {jdeg[5]:.6f}"
        )

        ok = self.send_command(cmd)
        if not ok:
            raise OmronProtocolError("TRAJPOINT NDONE")

    def traj_end(self) -> bool:
        return self.send_command("TRAJEND")

    def traj_abort(self) -> None:
        self.send_command("TRAJABORT")

