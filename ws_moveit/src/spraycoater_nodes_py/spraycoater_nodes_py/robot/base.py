# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Callable, Dict, Tuple, Optional

import logging
from rclpy.node import Node

from config.startup import AppContent, TopicSpec
from app.utils.logging_setup import add_file_logger  # ✅ DIREKT HIERHER

_SUB_HANDLERS: Dict[Tuple[str, str], str] = {}


def sub_handler(group: str, topic_id: str):
    def _wrap(fn: Callable):
        _SUB_HANDLERS[(group, topic_id)] = fn.__name__
        return fn
    return _wrap


class BaseBridge(Node):
    """
    UI-Bridge-Basis.
    """
    GROUP: str = ""

    def __init__(self, node_name: str, content: AppContent, *, namespace: str = ""):
        ns = (namespace or "").strip().strip("/")
        super().__init__(node_name, namespace=ns or None)

        if not self.GROUP:
            raise RuntimeError("GROUP muss in der Subklasse gesetzt sein.")

        self._namespace = ns
        self._content = content
        self._pubs: Dict[str, any] = {}
        self._subs: Dict[str, any] = {}

        # ------------------------------------------------------------
        # ✅ File-Logger für diese Bridge aktivieren (aus logging_setup.py)
        # ------------------------------------------------------------
        # Logger-Name so wählen, dass er sinnvoll pro GROUP geloggt wird
        self._logger_name = f"ros.bridge.{self.GROUP}"
        try:
            add_file_logger(self._logger_name)  # erstellt RotatingFileHandler
        except Exception as e:
            # nicht hart failen – Node soll trotzdem laufen
            logging.getLogger("stderr").error("add_file_logger(%s) failed: %s", self._logger_name, e)

        self._pylog = logging.getLogger(self._logger_name)

        # ------------------------------------------------------------
        # Init pubs/subs
        # ------------------------------------------------------------
        self._init_publishers_from_node_subscribe()
        self._init_subscriptions_from_node_publish()

        full_name = self.get_fully_qualified_name()
        self.get_logger().info(
            f"✅ {node_name} ready (group={self.GROUP}, ns='{self._namespace}', fqdn='{full_name}')"
        )
        self._pylog.info(
            "Bridge ready: node=%s group=%s ns=%r pubs=%d subs=%d fqdn=%s",
            node_name, self.GROUP, self._namespace, len(self._pubs), len(self._subs), full_name
        )

    # --- Helpers ---

    def _full_topic(self, name: str) -> str:
        """
        Effektiver Topic inkl. Node-Namespace (shadow/live).
        spec.name ist relativ: 'spraycoater/...'
        """
        node_ns = (self.get_namespace() or "/").rstrip("/")
        if node_ns == "" or node_ns == "/":
            return f"/{name.lstrip('/')}"
        return f"{node_ns}/{name.lstrip('/')}"

    def _resolve_handler(self, group: str, topic_id: str) -> Optional[Callable]:
        name = _SUB_HANDLERS.get((group, topic_id))
        return getattr(self, name, None) if name else None

    # --- Init ---

    def _init_publishers_from_node_subscribe(self) -> None:
        try:
            specs = self._content.topics(self.GROUP, "subscribe")
        except KeyError:
            specs = []

        for spec in specs:
            msg_type = spec.resolve_type()
            pub = self.create_publisher(msg_type, spec.name, self._content.qos(spec.qos_key))
            self._pubs[spec.id] = pub

            # ✅ Start-Log: sichtbar + in Datei
            self._pylog.info(
                "[PUB] id=%s -> %s (%s) qos=%s",
                spec.id, self._full_topic(spec.name), spec.type_str, spec.qos_key
            )
            self.get_logger().info(
                f"[PUB] {spec.id} -> {self._full_topic(spec.name)} ({spec.type_str}) qos={spec.qos_key}"
            )

    def _init_subscriptions_from_node_publish(self) -> None:
        try:
            specs = self._content.topics(self.GROUP, "publish")
        except KeyError:
            specs = []

        for spec in specs:
            cb = self._resolve_handler(self.GROUP, spec.id) or (lambda _msg: None)
            msg_type = spec.resolve_type()
            sub = self.create_subscription(msg_type, spec.name, cb, self._content.qos(spec.qos_key))
            self._subs[spec.id] = sub

            cb_name = getattr(cb, "__name__", "lambda")

            # ✅ Start-Log: sichtbar + in Datei
            self._pylog.info(
                "[SUB] id=%s <- %s (%s) qos=%s cb=%s",
                spec.id, self._full_topic(spec.name), spec.type_str, spec.qos_key, cb_name
            )
            self.get_logger().info(
                f"[SUB] {spec.id} <- {self._full_topic(spec.name)} ({spec.type_str}) qos={spec.qos_key} cb={cb_name}"
            )

    # --- Public API ---

    def pub(self, topic_id: str):
        if topic_id not in self._pubs:
            raise KeyError(f"Publisher '{topic_id}' unbekannt in {self.GROUP}")
        return self._pubs[topic_id]

    def spec(self, direction: str, topic_id: str) -> TopicSpec:
        return self._content.topic_by_id(self.GROUP, direction, topic_id)
