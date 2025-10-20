# ros/clients/substrate_client.py
from __future__ import annotations

import os
import time
from typing import Iterable, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty

from ..common.topics import Topics
from ..common.qos import qos_default


class SubstrateClient:
    """
    Dünner Client zum Verwalten des Substrats (Workpiece) via Manager-Node.

    Topics:
      - publish: Topics.workspace_load   (std_msgs/String) -> Pfad zu .stl senden
      - publish: Topics.workspace_remove (std_msgs/Empty)  -> Substrat entfernen

    Features:
      - Pfad-Resolver: akzeptiert Dateinamen oder relative Pfade und sucht in gegebenen Suchpfaden.
      - Optionales Must-Exist: bricht ab, wenn Datei nicht existiert (Default: True).
    """

    def __init__(
        self,
        node: Node,
        topics: Optional[Topics] = None,
        *,
        search_dirs: Optional[Iterable[str]] = None,
        default_exts: Iterable[str] = ("stl",),
    ) -> None:
        self._node = node
        self._topics = topics or Topics()
        self._log = node.get_logger()

        # Wo nach Dateien gesucht wird, wenn nur ein Name übergeben wurde
        self._search_dirs: List[str] = list(search_dirs or [])
        self._default_exts: List[str] = [e.lstrip(".") for e in default_exts]

        # Publisher
        self._pub_load = node.create_publisher(String, self._topics.workspace_load, qos_default())
        self._pub_remove = node.create_publisher(Empty, self._topics.workspace_remove, qos_default())

    # --------------------- Public API ---------------------

    def add_search_dir(self, path: str) -> None:
        """Fügt einen Suchpfad hinzu (z. B. 'app/resource/stl/workpieces')."""
        p = os.path.abspath(os.path.expanduser(path))
        if p not in self._search_dirs:
            self._search_dirs.append(p)
            self._log.info(f"SubstrateClient: Suchpfad hinzugefügt: {p}")

    def load(self, name_or_path: str, *, must_exist: bool = True) -> bool:
        """
        Substrat laden:
        - akzeptiert absolute/relative Pfade ODER nur Dateinamen (wird in search_dirs gesucht)
        - hängend .stl automatisch an, falls keine Extension übergeben wurde (configurierbar via default_exts)
        """
        resolved = self._resolve_path(name_or_path)
        if resolved is None:
            msg = f"SubstrateClient: Datei nicht gefunden: {name_or_path}"
            if must_exist:
                self._log.error(msg)
                return False
            else:
                self._log.warning(msg + " (sende trotzdem)")

        payload = resolved or name_or_path  # falls must_exist=False und wir nichts fanden
        self._pub_load.publish(String(data=payload))
        self._log.info(f"📦 Substrat-Ladebefehl gesendet: {payload}")
        return True

    def remove(self) -> None:
        """Aktuelles Substrat entfernen (Marker & CollisionObject werden vom Manager gecleart)."""
        self._pub_remove.publish(Empty())
        self._log.info("🗑️ Substrat-Entfernen gesendet")

    def destroy(self) -> None:
        """Publisher sauber freigeben (optional)."""
        try:
            self._node.destroy_publisher(self._pub_load)
            self._node.destroy_publisher(self._pub_remove)
        except Exception:
            pass

    # --------------------- Helper ---------------------

    def _resolve_path(self, name_or_path: str) -> Optional[str]:
        """
        Versucht, einen lesbaren Pfad zu ermitteln:
        - Expand ~, env vars
        - wenn absolut/relativ existiert -> return
        - sonst in search_dirs + default_exts suchen
        """
        s = (name_or_path or "").strip()
        if not s:
            return None

        # Direkt-Path prüfen
        p = os.path.abspath(os.path.expanduser(os.path.expandvars(s)))
        if os.path.isfile(p):
            return p

        # Falls keine Extension angegeben wurde, Kandidaten generieren
        cand_names: List[str]
        if os.path.splitext(s)[1]:
            cand_names = [s]  # Extension vorhanden
        else:
            cand_names = [f"{s}.{ext}" for ext in self._default_exts]

        # In Suchpfaden suchen
        for base in self._search_dirs:
            for nm in cand_names:
                candidate = os.path.join(base, nm)
                if os.path.isfile(candidate):
                    return os.path.abspath(candidate)

        return None
