# source/ros/clients/mount_client.py
from __future__ import annotations
import os
from typing import Iterable, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty

from ..common.qos import qos_default  # gleiche QoS wie bei dir

class MountClient:
    """
    Dünner Client zum Verwalten des Substratemounts (STL) via Manager-Node.

    Publish:
      - meca/mount/load   (std_msgs/String)  -> Pfad zu .stl senden
      - meca/mount/remove (std_msgs/Empty)   -> Mount entfernen
    """

    def __init__(
        self,
        node: Node,
        *,
        topic_load: str = "meca/mount/load",
        topic_remove: str = "meca/mount/remove",
        search_dirs: Optional[Iterable[str]] = None,
        default_exts: Iterable[str] = ("stl",),
    ) -> None:
        self._node = node
        self._log = node.get_logger()
        self._topic_load = topic_load
        self._topic_remove = topic_remove
        self._search_dirs: List[str] = list(search_dirs or [])
        self._default_exts: List[str] = [e.lstrip(".") for e in default_exts]

        self._pub_load = node.create_publisher(String, topic_load, qos_default())
        self._pub_remove = node.create_publisher(Empty, topic_remove, qos_default())

    # ---------------- API ----------------

    def add_search_dir(self, path: str) -> None:
        p = os.path.abspath(os.path.expanduser(path))
        if p not in self._search_dirs:
            self._search_dirs.append(p)
            self._log.info(f"MountClient: Suchpfad hinzugefügt: {p}")

    def load(self, name_or_path: str, *, must_exist: bool = True) -> bool:
        resolved = self._resolve_path(name_or_path)
        if resolved is None:
            msg = f"MountClient: Datei nicht gefunden: {name_or_path}"
            if must_exist:
                self._log.error(msg)
                return False
            else:
                self._log.warning(msg + " (sende trotzdem)")
        payload = resolved or name_or_path
        self._pub_load.publish(String(data=payload))
        self._log.info(f"🧱 Mount-Ladebefehl gesendet: {payload}")
        return True

    def remove(self) -> None:
        self._pub_remove.publish(Empty())
        self._log.info("🧹 Mount-Entfernen gesendet")

    # -------------- intern --------------

    def _resolve_path(self, name_or_path: str) -> Optional[str]:
        s = (name_or_path or "").strip()
        if not s:
            return None

        p = os.path.abspath(os.path.expanduser(os.path.expandvars(s)))
        if os.path.isfile(p):
            return p

        # Falls keine Extension mitgegeben wurde
        candidates = [s] if os.path.splitext(s)[1] else [f"{s}.{ext}" for ext in self._default_exts]

        for base in self._search_dirs:
            for nm in candidates:
                cand = os.path.join(base, nm)
                if os.path.isfile(cand):
                    return os.path.abspath(cand)
        return None
