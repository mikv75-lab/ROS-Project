# -*- coding: utf-8 -*-
from __future__ import annotations

from typing import Callable, Dict, Tuple, Optional, List, Any, Mapping
import logging
import hashlib

from rclpy.node import Node
from config.startup import AppContent, TopicSpec

# (group, topic_id) -> method name
_SUB_HANDLERS: Dict[Tuple[str, str], str] = {}


def sub_handler(group: str, topic_id: str):
    """
    Decorator for inbound subscription callbacks (ROS -> UI).
    BaseBridge uses this to auto-wire topic_id handlers from topics.yaml publish-specs.
    """
    def _wrap(fn: Callable):
        _SUB_HANDLERS[(group, topic_id)] = fn.__name__
        return fn
    return _wrap


class BaseBridge(Node):
    """
    UI-Bridge base class.

    Important inversion (Node perspective vs UI perspective):
      - topics[group]["publish"]   -> UI bridge creates SUBSCRIPTIONS (Node publishes -> UI reads)
      - topics[group]["subscribe"] -> UI bridge creates PUBLISHERS   (UI writes -> Node subscribes)

    Subclasses:
      - MUST set GROUP
      - MAY implement @sub_handler(GROUP, "<topic_id>") handlers for inbound topics
    """

    GROUP: str = ""

    # Startup topic summary is always enabled.
    # "SUB update ..." logging is default OFF.
    ENABLE_SUB_CHANGED_LOGGING: bool = False
    SUB_CHANGED_ONLY_ON_CHANGE: bool = False
    SUB_CHANGED_LEVEL: int = logging.INFO
    SUB_CHANGED_SKIP_IDS: set[str] = {"joints"}  # avoid spam

    def __init__(
        self,
        node_name: str,
        content: AppContent,
        *,
        namespace: str = "",
        config: Optional[Mapping[str, Any]] = None,
        **_: Any,
    ):
        # config is intentionally ignored (legacy compatibility)
        _ = config

        ns = (namespace or "").strip().strip("/")
        if not self.GROUP:
            raise RuntimeError("GROUP muss in der Subklasse gesetzt sein.")

        super().__init__(node_name, namespace=ns or None)

        self._namespace = ns
        self._content = content
        self._pubs: Dict[str, Any] = {}
        self._subs: Dict[str, Any] = {}

        self._last_msg_fingerprint: Dict[str, str] = {}

        # bridge file logger (optional; depends on your logging bootstrap)
        self._pylog = self._init_bridge_file_logger(node_name=node_name, namespace=ns)

        # auto-wire from topics.yaml
        self._init_publishers_from_node_subscribe()
        self._init_subscriptions_from_node_publish()

        fqdn = self.get_fully_qualified_name()
        self.get_logger().info(
            f"✅ {node_name} ready (group={self.GROUP}, ns='{self._namespace}', fqdn='{fqdn}')"
        )

        self._pylog.info(
            "Bridge started: node=%s group=%s ns=%s fqdn=%s",
            node_name, self.GROUP, self._namespace or "(root)", fqdn
        )

        self._log_topic_summary()

        self._pylog.info(
            "SUB changed logging: enabled=%s only_on_change=%s level=%s skip_ids=%s",
            self.ENABLE_SUB_CHANGED_LOGGING,
            self.SUB_CHANGED_ONLY_ON_CHANGE,
            logging.getLevelName(self.SUB_CHANGED_LEVEL),
            sorted(self.SUB_CHANGED_SKIP_IDS),
        )

    # ---------------- Properties ----------------

    @property
    def namespace(self) -> str:
        return self._namespace

    @property
    def content(self) -> AppContent:
        return self._content

    # ---------------- Public helpers ----------------

    def spec(self, direction: str, topic_id: str) -> TopicSpec:
        """Return TopicSpec by id for this bridge GROUP."""
        return self._content.topic_by_id(self.GROUP, direction, topic_id)

    def pub(self, topic_id: str):
        """Return publisher created from topics.yaml subscribe-spec (UI->Node)."""
        if topic_id not in self._pubs:
            raise KeyError(f"[{self.GROUP}] publisher id '{topic_id}' not initialized")
        return self._pubs[topic_id]

    # ---------------- Logging helpers ----------------

    def _init_bridge_file_logger(self, *, node_name: str, namespace: str) -> logging.Logger:
        log_name = f"ros.bridge.{namespace or 'root'}.{self.GROUP}.{node_name}"
        lg = logging.getLogger(log_name)

        # optional integration with your logging bootstrap
        try:
            add = getattr(logging, "add_file_logger", None)
            if callable(add):
                # uses default file_name policy from your init_logging()
                add(log_name, file_name=None, level=logging.DEBUG)
            else:
                lg.debug("logging.add_file_logger not available (init_logging not called?)")
        except Exception:
            lg.exception("Failed to init bridge file logger")

        return lg

    def _resolved_topic(self, name: str) -> str:
        try:
            return self.resolve_topic_name(name)
        except Exception:
            ns = (self._namespace or "").strip("/")
            if not name.startswith("/"):
                name = "/" + name
            return f"/{ns}{name}" if ns else name

    def _log_topic_summary(self) -> None:
        pub_specs: List[TopicSpec] = []
        sub_specs: List[TopicSpec] = []

        try:
            pub_specs = list(self._content.topics(self.GROUP, "subscribe"))
        except Exception:
            pub_specs = []

        try:
            sub_specs = list(self._content.topics(self.GROUP, "publish"))
        except Exception:
            sub_specs = []

        self._pylog.info(
            "=== Topic summary (group=%s, ns=%s, fqdn=%s) ===",
            self.GROUP,
            self._namespace or "(root)",
            self.get_fully_qualified_name(),
        )

        if pub_specs:
            self._pylog.info("Publishers (UI -> ROS node subscribes): %d", len(pub_specs))
            for s in pub_specs:
                self._pylog.info(
                    "  PUB  id=%s name=%s resolved=%s type=%s qos=%s",
                    s.id, s.name, self._resolved_topic(s.name), s.type_str, getattr(s, "qos_key", None),
                )
        else:
            self._pylog.info("Publishers (UI -> ROS node subscribes): none")

        if sub_specs:
            self._pylog.info("Subscriptions (ROS node publishes -> UI): %d", len(sub_specs))
            for s in sub_specs:
                self._pylog.info(
                    "  SUB  id=%s name=%s resolved=%s type=%s qos=%s",
                    s.id, s.name, self._resolved_topic(s.name), s.type_str, getattr(s, "qos_key", None),
                )
        else:
            self._pylog.info("Subscriptions (ROS node publishes -> UI): none")

        self._pylog.info("=== End topic summary ===")

    # ---------------- SUB changed logging ----------------

    def _msg_fingerprint(self, msg: Any) -> str:
        try:
            raw = repr(msg).encode("utf-8", errors="replace")
        except Exception:
            raw = str(type(msg)).encode("utf-8", errors="replace")
        return hashlib.sha1(raw).hexdigest()

    def _msg_brief(self, msg: Any) -> str:
        try:
            if hasattr(msg, "data"):
                return f"data={getattr(msg, 'data')!r}"

            if hasattr(msg, "pose") and hasattr(msg, "header"):
                p = msg.pose.position
                q = msg.pose.orientation
                return (
                    f"pose=({p.x:.3f},{p.y:.3f},{p.z:.3f}) "
                    f"quat=({q.x:.3f},{q.y:.3f},{q.z:.3f},{q.w:.3f}) "
                    f"frame={getattr(msg.header, 'frame_id', '')!r}"
                )

            if hasattr(msg, "name") and hasattr(msg, "position"):
                n = len(getattr(msg, "name", []) or [])
                pos = getattr(msg, "position", []) or []
                head = ", ".join([f"{v:.3f}" for v in list(pos)[:6]])
                tail = "..." if len(pos) > 6 else ""
                return f"joints={n} pos=[{head}{tail}]"

            s = repr(msg)
            return (s[:300] + "…") if len(s) > 300 else s
        except Exception:
            return "<unprintable msg>"

    def _make_logged_callback(self, *, topic_id: str, spec: TopicSpec, user_cb: Optional[Callable]) -> Callable:
        resolved = self._resolved_topic(spec.name)

        def _cb(msg: Any):
            if self.ENABLE_SUB_CHANGED_LOGGING and (topic_id not in self.SUB_CHANGED_SKIP_IDS):
                try:
                    fp = self._msg_fingerprint(msg)
                    changed = (self._last_msg_fingerprint.get(topic_id) != fp)

                    if (not self.SUB_CHANGED_ONLY_ON_CHANGE) or changed:
                        self._last_msg_fingerprint[topic_id] = fp
                        self._pylog.log(
                            self.SUB_CHANGED_LEVEL,
                            "SUB update id=%s name=%s resolved=%s -> %s",
                            topic_id, spec.name, resolved, self._msg_brief(msg),
                        )
                except Exception:
                    self._pylog.exception("SUB logging failed (id=%s, topic=%s)", topic_id, spec.name)

            if user_cb is not None:
                try:
                    user_cb(msg)
                except Exception:
                    self._pylog.exception("User callback failed (id=%s, topic=%s)", topic_id, spec.name)

        return _cb

    # ---------------- Auto wiring ----------------

    def _init_publishers_from_node_subscribe(self) -> None:
        """Create publishers for topics.yaml subscribe-specs (UI -> Node)."""
        try:
            for spec in self._content.topics(self.GROUP, "subscribe"):
                msg_type = spec.resolve_type()
                self._pubs[spec.id] = self.create_publisher(
                    msg_type,
                    spec.name,
                    self._content.qos(spec.qos_key),
                )
        except KeyError:
            pass

    def _init_subscriptions_from_node_publish(self) -> None:
        """Create subscriptions for topics.yaml publish-specs (Node -> UI)."""
        try:
            for spec in self._content.topics(self.GROUP, "publish"):
                user_cb = self._resolve_handler(self.GROUP, spec.id)
                cb = self._make_logged_callback(topic_id=spec.id, spec=spec, user_cb=user_cb)

                msg_type = spec.resolve_type()
                self._subs[spec.id] = self.create_subscription(
                    msg_type,
                    spec.name,
                    cb,
                    self._content.qos(spec.qos_key),
                )
        except KeyError:
            pass

    def _resolve_handler(self, group: str, topic_id: str) -> Optional[Callable]:
        name = _SUB_HANDLERS.get((group, topic_id))
        return getattr(self, name, None) if name else None
