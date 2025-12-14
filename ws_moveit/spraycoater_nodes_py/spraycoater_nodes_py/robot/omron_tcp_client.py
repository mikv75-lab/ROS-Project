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
      - recv_line(): liest blockierend bis '\\n' oder Timeout / Verbindungsabbruch
      - recv_line_nowait(): kurz-blockierendes Polling, gibt "" zurück, wenn keine
                            vollständige Zeile verfügbar ist

    Der Client ist thread-sicher genug für:
      - 1 Sender-Thread (Bridge _on_command)
      - 1 Reader-Thread  (Bridge _reader_loop / Poll-Loop)
    """

    def __init__(self, host: str, port: int, timeout: float = 3.0, poll_timeout: float = 0.01) -> None:
        """
        :param host: ACE-Host
        :param port: ACE-Port (z.B. 5000)
        :param timeout: Standard-Timeout für blockierende Operationen (recv_line)
        :param poll_timeout: kurzer Timeout für recv_line_nowait() (Polling)
        """
        self._host = host
        self._port = int(port)
        self._timeout = float(timeout)
        self._poll_timeout = float(poll_timeout)

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
          - text wird am Ende von CR/LF befreit, dann wird '\\r\\n' angehängt.
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
    # Empfangen (blockierend)
    # ------------------------------------------------------------------
    def recv_line(self) -> str:
        """
        Liest eine Zeile vom Socket, endet bei '\\n'.

        Rückgabe:
          - String inklusive evtl. '\\r\\n' am Ende (Bridge macht .strip()).
          - "" bei socket.timeout (keine komplette Zeile im Timeout)
        Exceptions:
          - ConnectionError bei Verbindungsabbruch (recv() == b"")
          - andere socket-Fehler werden weitergereicht.
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
                    # Timeout -> keine Zeile komplett, Bridge kann "" als "nichts" behandeln
                    return ""
                if not data:
                    # 0 Bytes -> Verbindung vom Server geschlossen
                    raise ConnectionError("OmronTcpClient: peer closed connection")

                self._rx_buffer += data

    # ------------------------------------------------------------------
    # Empfangen (kurz blockierend / Polling)
    # ------------------------------------------------------------------
    def recv_line_nowait(self) -> str:
        """
        Kurzes Poll-Read:
          - prüft zuerst, ob bereits eine komplette Zeile im internen Puffer liegt
          - falls nicht, versucht mit kurzem Timeout (poll_timeout) nachzuladen
          - gibt eine komplette Zeile zurück, falls vorhanden
          - gibt "" zurück, wenn keine vollständige Zeile verfügbar ist

        Exceptions:
          - ConnectionError bei Verbindungsabbruch
        """
        sock = self._ensure_socket()

        with self._rx_lock:
            # 1) Erst schauen, ob schon ein '\n' im Puffer liegt
            idx = self._rx_buffer.find(b"\n")
            if idx != -1:
                chunk = self._rx_buffer[: idx + 1]
                self._rx_buffer = self._rx_buffer[idx + 1 :]
                return chunk.decode("ascii", errors="ignore")

            # 2) Kurz versuchen, neue Daten zu holen
            #    -> poll_timeout, dann wieder auf ursprüngliches Timeout zurück
            with self._sock_lock:
                try:
                    old_timeout = sock.gettimeout()
                except Exception:
                    old_timeout = self._timeout

                try:
                    sock.settimeout(self._poll_timeout)
                    try:
                        data = sock.recv(4096)
                    except socket.timeout:
                        # nichts Neues im Poll-Fenster
                        return ""
                    if not data:
                        # Verbindung geschlossen
                        raise ConnectionError("OmronTcpClient: peer closed connection")
                    self._rx_buffer += data
                finally:
                    try:
                        sock.settimeout(old_timeout)
                    except Exception:
                        pass

            # 3) Nochmal nach '\n' suchen
            idx = self._rx_buffer.find(b"\n")
            if idx != -1:
                chunk = self._rx_buffer[: idx + 1]
                self._rx_buffer = self._rx_buffer[idx + 1 :]
                return chunk.decode("ascii", errors="ignore")

            # Noch immer keine komplette Zeile
            return ""
