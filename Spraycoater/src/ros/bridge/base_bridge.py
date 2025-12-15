# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Callable, Dict, Tuple, Optional, List

import logging

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

        if not self.GROUP:
            raise RuntimeError("GROUP muss in der Subklasse gesetzt sein.")

        # rclpy.Node akzeptiert namespace=None oder "" -> wir nutzen None, wenn leer
        super().__init__(node_name, namespace=ns or None)

        self._namespace = ns
        self._content = content
        self._pubs: Dict[str, any] = {}
        self._subs: Dict[str, any] = {}

        # ------------------------------------------------------------
        # Python File-Logger für diese Bridge (data/logs/...)
        # ------------------------------------------------------------
        self._pylog = self._init_bridge_file_logger(node_name=node_name, namespace=ns)

        # ------------------------------------------------------------
        # Wichtig: Perspektive invertieren (UI-Bridge)
        # ------------------------------------------------------------
        self._init_publishers_from_node_subscribe()
        self._init_subscriptions_from_node_publish()

        full_name = self.get_fully_qualified_name()
        self.get_logger().info(
            f"✅ {node_name} ready (group={self.GROUP}, ns='{self._namespace}', fqdn='{full_name}')"
        )

        # Zusätzlich: Alles in eine Bridge-spezifische Datei loggen
        self._pylog.info(
            "Bridge started: node=%s group=%s ns=%s fqdn=%s",
            node_name, self.GROUP, self._namespace or "(root)", full_name
        )
        self._log_topic_summary()

    # --- Properties ---

    @property
    def namespace(self) -> str:
        """Roh-Namespace der Bridge (ohne führenden '/')."""
        return self._namespace

    @property
    def content(self) -> AppContent:
        return self._content

    # --- Logging helpers ---

    def _init_bridge_file_logger(self, *, node_name: str, namespace: str) -> logging.Logger:
        """
        Erzeugt pro Bridge einen Python-Logger mit FileHandler unter data/logs.

        Erwartet, dass init_logging(LOG_DIR) bereits gelaufen ist, damit
        logging.add_file_logger(...) existiert.
        """
        log_name = f"ros.bridge.{namespace or 'root'}.{self.GROUP}.{node_name}"
        lg = logging.getLogger(log_name)

        # Ziel-Dateiname: bridge_<namespace>_<group>.log
        safe_ns = (namespace or "root").replace("/", "_")
        file_name = f"bridge_{safe_ns}_{self.GROUP}.log"

        try:
            add = getattr(logging, "add_file_logger", None)
            if callable(add):
                add(log_name, file_name=file_name, level=logging.DEBUG)
            else:
                lg.warning("logging.add_file_logger is not available (init_logging not called?)")
        except Exception as e:
            lg.exception("Failed to init bridge file logger: %s", e)

        return lg

    def _resolved_topic(self, name: str) -> str:
        """
        Liefert den final aufgelösten Topic-Namen, so wie er im ROS-Graph erscheint.
        Berücksichtigt Namespace des Nodes und Remappings.
        """
        try:
            return self.resolve_topic_name(name)
        except Exception:
            ns = (self._namespace or "").strip("/")
            if not name.startswith("/"):
                name = "/" + name
            return f"/{ns}{name}" if ns else name

    def _log_topic_summary(self) -> None:
        """
        Schreibt eine Übersicht aller Topics (finale Namen) dieser Bridge ins Logfile.
        """
        pub_specs: List[TopicSpec] = []
        sub_specs: List[TopicSpec] = []

        # Node-Perspektive:
        # - topics[group]["subscribe"] => UI-Bridge publishers (UI->ROS)
        # - topics[group]["publish"]   => UI-Bridge subscriptions (ROS->UI)
        try:
            pub_specs = list(self._content.topics(self.GROUP, "subscribe"))
        except KeyError:
            pub_specs = []
        except Exception as e:
            self._pylog.exception("Topic summary: failed reading subscribe-specs: %s", e)
            pub_specs = []

        try:
            sub_specs = list(self._content.topics(self.GROUP, "publish"))
        except KeyError:
            sub_specs = []
        except Exception as e:
            self._pylog.exception("Topic summary: failed reading publish-specs: %s", e)
            sub_specs = []

        self._pylog.info(
            "=== Topic summary (group=%s, ns=%s, fqdn=%s) ===",
            self.GROUP,
            self._namespace or "(root)",
            self.get_fully_qualified_name(),
        )

        # Publisher (UI -> ROS Node subscribe)
        if pub_specs:
            self._pylog.info("Publishers (UI -> ROS node subscribes): %d", len(pub_specs))
            for s in pub_specs:
                resolved = self._resolved_topic(s.name)
                self._pylog.info(
                    "  PUB  id=%s name=%s resolved=%s type=%s qos=%s",
                    s.id, s.name, resolved, s.type_str, getattr(s, "qos_key", None)
                )
        else:
            self._pylog.info("Publishers (UI -> ROS node subscribes): none")

        # Subscriptions (ROS Node publishes -> UI)
        if sub_specs:
            self._pylog.info("Subscriptions (ROS node publishes -> UI): %d", len(sub_specs))
            for s in sub_specs:
                resolved = self._resolved_topic(s.name)
                self._pylog.info(
                    "  SUB  id=%s name=%s resolved=%s type=%s qos=%s",
                    s.id, s.name, resolved, s.type_str, getattr(s, "qos_key", None)
                )
        else:
            self._pylog.info("Subscriptions (ROS node publishes -> UI): none")

        self._pylog.info("=== End topic summary ===")

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

                # ROS2 logger: nur EINEN String (keine printf-Args)
                self.get_logger().debug(
                    f"[PUB] id={spec.id} name={spec.name} resolved={self._resolved_topic(spec.name)} "
                    f"type={spec.type_str} qos={getattr(spec, 'qos_key', None)} ns={self._namespace}"
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

                # ROS2 logger: nur EINEN String (keine printf-Args)
                self.get_logger().debug(
                    f"[SUB] id={spec.id} name={spec.name} resolved={self._resolved_topic(spec.name)} "
                    f"type={spec.type_str} qos={getattr(spec, 'qos_key', None)} ns={self._namespace}"
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
