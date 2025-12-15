# spraycoater_nodes_py/robot/omron_tcp_client.py
# -*- coding: utf-8 -*-
from __future__ import annotations

import socket
import threading
import time
from typing import Callable, Optional


class OmronTcpClient:
    """
    Reiner TCP-Client für den Omron V+-Controller.

    - KEINE ROS-Abhängigkeit
    - KEINE Topics
    - KEIN Namespace
    - Callback-basiert (RX)
    """

    def __init__(
        self,
        host: str,
        port: int,
        *,
        timeout_s: float = 2.0,
        on_rx: Optional[Callable[[str], None]] = None,
        on_disconnect: Optional[Callable[[], None]] = None,
    ) -> None:
        self.host = host
        self.port = port
        self.timeout_s = float(timeout_s)

        self._on_rx = on_rx
        self._on_disconnect = on_disconnect

        self._sock: Optional[socket.socket] = None
        self._rx_thread: Optional[threading.Thread] = None
        self._alive = threading.Event()
        self._lock = threading.Lock()

    # ------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------

    def connect(self) -> None:
        with self._lock:
            if self._sock is not None:
                return

            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(self.timeout_s)
            sock.connect((self.host, self.port))
            sock.settimeout(None)  # danach blocking

            self._sock = sock
            self._alive.set()

            self._rx_thread = threading.Thread(
                target=self._rx_loop,
                name="OmronTcpClientRx",
                daemon=True,
            )
            self._rx_thread.start()

    def disconnect(self) -> None:
        with self._lock:
            self._alive.clear()
            if self._sock is not None:
                try:
                    self._sock.shutdown(socket.SHUT_RDWR)
                except Exception:
                    pass
                try:
                    self._sock.close()
                except Exception:
                    pass
                self._sock = None

        if self._on_disconnect:
            try:
                self._on_disconnect()
            except Exception:
                pass

    def is_connected(self) -> bool:
        return self._sock is not None

    # ------------------------------------------------------------
    # TX / RX
    # ------------------------------------------------------------

    def send_line(self, line: str) -> None:
        """
        Sendet genau eine Textzeile (LF wird automatisch angehängt).
        """
        data = (line.rstrip() + "\n").encode("utf-8")
        with self._lock:
            if self._sock is None:
                raise RuntimeError("TCP socket not connected")
            self._sock.sendall(data)

    def _rx_loop(self) -> None:
        buf = b""
        try:
            while self._alive.is_set():
                if self._sock is None:
                    break

                data = self._sock.recv(4096)
                if not data:
                    break

                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    text = line.decode("utf-8", errors="ignore").strip()
                    if not text:
                        continue
                    if self._on_rx:
                        try:
                            self._on_rx(text)
                        except Exception:
                            pass
        except Exception:
            pass
        finally:
            self.disconnect()

    # ------------------------------------------------------------
    # Context-Manager (optional)
    # ------------------------------------------------------------

    def __enter__(self) -> "OmronTcpClient":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.disconnect()
