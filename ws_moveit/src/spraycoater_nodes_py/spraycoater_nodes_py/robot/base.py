# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Callable, Dict, Tuple, Optional

import logging
from rclpy.node import Node

from config.startup import AppContent, TopicSpec
from app.utils.logging_setup import add_file_logger

# (group, topic_id) -> method name
_SUB_HANDLERS: Dict[Tuple[str, str], str] = {}


def sub_handler(group: str, topic_id: str):
    """
    Decorator für Callback-Handler eingehender Nachrichten.
    """
    def _wrap(fn: Callable):
        _SUB_HANDLERS[(group, topic_id)] = fn.__name__
        return fn
    return _wrap


class BaseBridge(Node):
    """
    UI-Bridge-Basis.

    - Namespace: "shadow" | "live" | ""
    - Topics kommen RELATIV aus topics.yaml (z.B. spraycoater/robot/...)
    - Effektive Topics: /<namespace>/spraycoater/...

    Logging:
      - pro Namespace genau eine Datei:
          shadow.bridge.log
          live.bridge.log
    """

    GROUP: str = ""   # MUSS in Subklassen gesetzt sein

    def __init__(self, node_name: str, content: AppContent, *, namespace: str = ""):
        # Namespace normalisieren
        ns = (namespace or "").strip().strip("/")
        super().__init__(node_name, namespace=ns or None)

        if not self.GROUP:
            raise RuntimeError("GROUP muss in der Subklasse gesetzt sein.")

        self._namespace = ns
        self._content = content
        self._pubs: Dict[str, any] = {}
        self._subs: Dict[str, any] = {}

        # ============================================================
        # Logging: eine Datei pro Namespace
        # ============================================================
        ns_key = self._namespace if self._namespace else "root"
        self._logger_name = f"ros.{ns_key}.bridge"

        try:
            self._pylog = add_file_logger(
                self._logger_name,
                file_name=f"{ns_key}.bridge.log",
                level=logging.DEBUG,
            )
        except Exception:
            logging.getLogger().exception(
                "add_file_logger failed for namespace=%r", ns_key
            )
            self._pylog = logging.getLogger(self._logger_name)

        # ============================================================
        # Topics initialisieren
        # ============================================================
        self._init_publishers_from_node_subscribe()
        self._init_subscriptions_from_node_publish()

        fqdn = self.get_fully_qualified_name()
        self.get_logger().info(
            "✅ %s ready (group=%s, ns=%r, fqdn=%s)",
            node_name, self.GROUP, self._namespace, fqdn
        )
        self._pylog.info(
            "Bridge ready: node=%s group=%s ns=%r pubs=%d subs=%d fqdn=%s",
            node_name, self.GROUP, self._namespace,
            len(self._pubs), len(self._subs), fqdn
        )

    # ============================================================
    # Helpers
    # ============================================================

    def _full_topic(self, name: str) -> str:
        """
        Effektiver Topic-Name inkl. Namespace.
        """
        node_ns = (self.get_namespace() or "/").rstrip("/")
        if node_ns == "" or node_ns == "/":
            return f"/{name.lstrip('/')}"
        return f"{node_ns}/{name.lstrip('/')}"

    def _resolve_handler(self, group: str, topic_id: str) -> Optional[Callable]:
        name = _SUB_HANDLERS.get((group, topic_id))
        return getattr(self, name, None) if name else None

    # ============================================================
    # Init: Publisher (UI → ROS)
    # ============================================================

    def _init_publishers_from_node_subscribe(self) -> None:
        try:
            specs = self._content.topics(self.GROUP, "subscribe")
        except KeyError:
            specs = []

        for spec in specs:
            pub = self.create_publisher(
                spec.resolve_type(),
                spec.name,
                self._content.qos(spec.qos_key),
            )
            self._pubs[spec.id] = pub

            self._pylog.info(
                "[PUB] %-16s -> %s (%s) qos=%s",
                spec.id,
                self._full_topic(spec.name),
                spec.type_str,
                spec.qos_key,
            )
            self.get_logger().info(
                "[PUB] %s -> %s",
                spec.id,
                self._full_topic(spec.name),
            )

    # ============================================================
    # Init: Subscriptions (ROS → UI)
    # ============================================================

    def _init_subscriptions_from_node_publish(self) -> None:
        try:
            specs = self._content.topics(self.GROUP, "publish")
        except KeyError:
            specs = []

        for spec in specs:
            cb = self._resolve_handler(self.GROUP, spec.id) or (lambda _msg: None)

            sub = self.create_subscription(
                spec.resolve_type(),
                spec.name,
                cb,
                self._content.qos(spec.qos_key),
            )
            self._subs[spec.id] = sub

            cb_name = getattr(cb, "__name__", "lambda")

            self._pylog.info(
                "[SUB] %-16s <- %s (%s) qos=%s cb=%s",
                spec.id,
                self._full_topic(spec.name),
                spec.type_str,
                spec.qos_key,
                cb_name,
            )
            self.get_logger().info(
                "[SUB] %s <- %s",
                spec.id,
                self._full_topic(spec.name),
            )

    # ============================================================
    # Public API
    # ============================================================

    def pub(self, topic_id: str):
        if topic_id not in self._pubs:
            raise KeyError(f"Publisher '{topic_id}' unbekannt in {self.GROUP}")
        return self._pubs[topic_id]

    def spec(self, direction: str, topic_id: str) -> TopicSpec:
        return self._content.topic_by_id(self.GROUP, direction, topic_id)
