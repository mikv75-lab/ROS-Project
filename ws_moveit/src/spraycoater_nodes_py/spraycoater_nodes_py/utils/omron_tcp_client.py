# spraycoater_nodes_py/utils/omron_tcp_client.py
# -*- coding: utf-8 -*-
from __future__ import annotations

import socket
import threading
from typing import Optional


class OmronTcpClient:
    """
    Sehr schlanker TCP-Client für den Omron ACE/eV+ Server.

    Features:
      - connect() / close()
      - _send_line(text): schickt "text" mit CRLF als Zeilenende
      - recv_line(): liest blockierend bis '\n' oder Timeout / Verbindungsabbruch

    Der Client ist thread-sicher genug für:
      - 1 Sender-Thread (Bridge _on_command)
      - 1 Reader-Thread  (Bridge _reader_loop)
    """

    def __init__(self, host: str, port: int, timeout: float = 3.0) -> None:
        self._host = host
        self._port = int(port)
        self._timeout = float(timeout)

        self._sock: Optional[socket.socket] = None
        self._sock_lock = threading.RLock()
        self._rx_lock = threading.RLock()
        self._rx_buffer = b""

    # ------------------------------------------------------------------
    # Verbindung
    # ------------------------------------------------------------------
    def connect(self) -> None:
        """
        Baut eine neue TCP-Verbindung auf. Schließt ggf. vorherige.
        """
        with self._sock_lock:
            if self._sock is not None:
                try:
                    self._sock.close()
                except Exception:
                    pass
                self._sock = None

            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(self._timeout)
            s.connect((self._host, self._port))
            self._sock = s

            # RX-Puffer leeren
            with self._rx_lock:
                self._rx_buffer = b""

    def close(self) -> None:
        """
        Verbindung sauber schließen.
        """
        with self._sock_lock:
            if self._sock is not None:
                try:
                    self._sock.close()
                except Exception:
                    pass
                self._sock = None
        with self._rx_lock:
            self._rx_buffer = b""

    def _ensure_socket(self) -> socket.socket:
        """
        Interner Helper: wirft Exception, wenn keine Verbindung besteht.
        """
        with self._sock_lock:
            if self._sock is None:
                raise ConnectionError("OmronTcpClient: socket is not connected")
            return self._sock

    # ------------------------------------------------------------------
    # Senden
    # ------------------------------------------------------------------
    def _send_line(self, text: str) -> None:
        """
        Schickt eine Zeile mit CRLF als Terminator.

        Hinweis:
          - Bridge ruft diese Methode direkt auf.
          - text wird am Ende von CR/LF befreit, dann wird '\r\n' angehängt.
        """
        if text is None:
            text = ""
        # vorhandene Zeilenenden entfernen und CRLF anhängen
        line = text.rstrip("\r\n") + "\r\n"
        data = line.encode("ascii", errors="ignore")

        sock = self._ensure_socket()
        with self._sock_lock:
            sock.sendall(data)

    # ------------------------------------------------------------------
    # Empfangen
    # ------------------------------------------------------------------
    def recv_line(self) -> str:
        """
        Liest eine Zeile vom Socket, endet bei '\n'.

        Rückgabe:
          - String inklusive evtl. '\r\n' am Ende (Bridge macht .strip()).
        Exceptions:
          - ConnectionError bei Verbindungsabbruch (recv() == b"")
          - socket.timeout kann durchgereicht werden (Bridge fängt Exception).
        """
        sock = self._ensure_socket()

        with self._rx_lock:
            while True:
                # Haben wir schon ein '\n' im Puffer?
                idx = self._rx_buffer.find(b"\n")
                if idx != -1:
                    chunk = self._rx_buffer[: idx + 1]
                    self._rx_buffer = self._rx_buffer[idx + 1 :]
                    return chunk.decode("ascii", errors="ignore")

                # Sonst mehr Daten nachladen
                try:
                    data = sock.recv(4096)
                except socket.timeout:
                    # Timeout -> keine Zeile komplett, Bridge behandelt das als "leer"
                    return ""
                if not data:
                    # 0 Bytes -> Verbindung vom Server geschlossen
                    raise ConnectionError("OmronTcpClient: peer closed connection")

                self._rx_buffer += data
