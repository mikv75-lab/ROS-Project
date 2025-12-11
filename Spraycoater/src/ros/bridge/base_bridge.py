# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Callable, Dict, Tuple, Optional

import rclpy
from rclpy.node import Node

from config.startup import AppContent, TopicSpec

# (group, topic_id) -> method name  (Handler für eingehende Nachrichten)
_SUB_HANDLERS: Dict[Tuple[str, str], str] = {}


def sub_handler(group: str, topic_id: str):
    """
    Dekorator für Callback-Handler eingehender Nachrichten.
    Wird von BaseBridge genutzt, um automatisch den passenden
    Callback für (group, topic_id) zu finden.
    """
    def _wrap(fn: Callable):
        _SUB_HANDLERS[(group, topic_id)] = fn.__name__
        return fn
    return _wrap


class BaseBridge(Node):
    """
    UI-Bridge-Basis:

    - Invertiert die Rollen aus topics.yaml (Node-Perspektive):
        * topics[group]["publish"]  -> UI-Bridge erstellt **Subscriptions**
        * topics[group]["subscribe"]-> UI-Bridge erstellt **Publishers**

    - Subklassen setzen GROUP und definieren @sub_handler-Methoden für
      eingehende Nachrichten (aus den "publish"-Topics des ROS-Nodes).

    - Namespace:
        Alle Topics dieser Bridge laufen unter demselben ROS-Namespace, z. B.
        'shadow' oder 'live'. In Kombination mit root_ns aus topics.yaml
        ergibt das z. B. /shadow/spraycoater/...
    """
    GROUP: str = ""  # in Subklassen setzen

    def __init__(self, node_name: str, content: AppContent, *, namespace: str = ""):
        # Namespace sauber normalisieren: "" oder "shadow"/"live"
        ns = (namespace or "").strip().strip("/")
        # rclpy.Node akzeptiert namespace="", das ist ok
        super().__init__(node_name, namespace=ns or None)

        if not self.GROUP:
            raise RuntimeError("GROUP muss in der Subklasse gesetzt sein.")

        self._namespace = ns
        self._content = content
        self._pubs: Dict[str, any] = {}
        self._subs: Dict[str, any] = {}

        # Wichtig: Perspektive invertieren (UI-Bridge)
        self._init_publishers_from_node_subscribe()
        self._init_subscriptions_from_node_publish()

        full_name = self.get_fully_qualified_name()
        self.get_logger().info(
            f"✅ {node_name} ready (group={self.GROUP}, ns='{self._namespace}', fqdn='{full_name}')"
        )

    # --- Properties ---

    @property
    def namespace(self) -> str:
        """Roh-Namespace der Bridge (ohne führenden '/')."""
        return self._namespace

    @property
    def content(self) -> AppContent:
        return self._content

    # --- Init ---

    def _init_publishers_from_node_subscribe(self) -> None:
        """
        Erzeuge Publisher für alle Topics, die der ROS-Node laut topics.yaml 'subscribe't.
        (UI schickt dort ihre Set-Kommandos hin.)
        """
        try:
            for spec in self._content.topics(self.GROUP, "subscribe"):
                msg_type = spec.resolve_type()
                pub = self.create_publisher(
                    msg_type,
                    spec.name,
                    self._content.qos(spec.qos_key),
                )
                self._pubs[spec.id] = pub
                self.get_logger().debug(
                    f"[PUB] {spec.id} -> {spec.name} ({spec.type_str}) [ns={self._namespace}]"
                )
        except KeyError:
            # Gruppe ohne 'subscribe' Block -> ok
            pass

    def _init_subscriptions_from_node_publish(self) -> None:
        """
        Erzeuge Subscriptions für alle Topics, die der ROS-Node laut topics.yaml 'publish't.
        (UI liest dort Current-States / Listen etc.)
        """
        try:
            for spec in self._content.topics(self.GROUP, "publish"):
                cb = self._resolve_handler(self.GROUP, spec.id) or (lambda _msg: None)
                msg_type = spec.resolve_type()
                sub = self.create_subscription(
                    msg_type,
                    spec.name,
                    cb,
                    self._content.qos(spec.qos_key),
                )
                self._subs[spec.id] = sub
                self.get_logger().debug(
                    f"[SUB] {spec.id} <- {spec.name} ({spec.type_str}) [ns={self._namespace}]"
                )
        except KeyError:
            # Gruppe ohne 'publish' Block -> ok
            pass

    # --- Helpers ---

    def _resolve_handler(self, group: str, topic_id: str) -> Optional[Callable]:
        name = _SUB_HANDLERS.get((group, topic_id))
        return getattr(self, name, None) if name else None

    def pub(self, topic_id: str):
        """
        Publisher-Handle für ein Topic, das der ROS-Node laut topics.yaml 'subscribe't.
        (UI-Bridge sendet dort hin.)
        """
        if topic_id not in self._pubs:
            raise KeyError(f"Publisher '{topic_id}' unbekannt in {self.GROUP}")
        return self._pubs[topic_id]

    def spec(self, direction: str, topic_id: str) -> TopicSpec:
        """
        Liefert die TopicSpec aus AppContent.
        direction: 'publish' oder 'subscribe' (Node-Perspektive).
        """
        return self._content.topic_by_id(self.GROUP, direction, topic_id)
