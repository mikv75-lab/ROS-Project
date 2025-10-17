# ros/clients/poses_client.py
from __future__ import annotations

import time
from typing import Callable, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty
from geometry_msgs.msg import PoseStamped

from ..common.topics import Topics
from ..common.qos import qos_default, qos_latched


class PosesClient:
    """
    Dünner Client für Basis-Posen:
      - latched beziehen (home/workspace_center/predispense/service …)
      - republish triggern
      - Pose aus TCP speichern
      - optional: Pose direkt setzen (PoseStamped) – falls vom PosesManager unterstützt

    Topics:
      publish:
        - poses_republish    (std_msgs/Empty)
        - poses_set_from_tcp (std_msgs/String, payload=name)
        - poses/set/<name>   (geometry_msgs/PoseStamped) [optional, wenn vorhanden]
      subscribe (latched):
        - poses/<name>       (geometry_msgs/PoseStamped)

    Nutzung:
        topics = Topics()
        poses  = PosesClient(node, topics)

        # sicherstellen, dass wir einen ersten Stand haben
        poses.wait_for("home", timeout=1.5)

        # Pose abfragen (aus Cache)
        ps = poses.get("workspace_center")

        # Pose aus TCP speichern lassen
        poses.set_from_tcp("predispense", wait_for_update=True)

        # (falls unterstützt) Pose direkt setzen
        poses.set_pose("service", pose_stamped_obj)
    """

    DEFAULT_POSE_NAMES: Tuple[str, ...] = ("home", "workspace_center", "predispense", "service")

    def __init__(
        self,
        node: Node,
        topics: Optional[Topics] = None,
        *,
        known_pose_names: Optional[Tuple[str, ...]] = None,
    ) -> None:
        self._node = node
        self._topics = topics or Topics()
        self._logger = node.get_logger()

        self._pose_names: Tuple[str, ...] = known_pose_names or self.DEFAULT_POSE_NAMES

        # Cache & Plumbing
        self._pose_cache: Dict[str, PoseStamped] = {}
        self._subs: Dict[str, any] = {}
        self._on_change: Dict[str, List[Callable[[PoseStamped], None]]] = {}
        self._set_pubs: Dict[str, any] = {}

        # Publishers
        self._pub_republish = node.create_publisher(Empty, self._topics.poses_republish, qos_default())
        self._pub_set_from_tcp = node.create_publisher(String, self._topics.poses_set_from_tcp, qos_default())

        # Lazy-Subscribe: wir abonnieren erst bei Bedarf (get()/wait_for()/on_change()).
        # Wer eager will, kann einmalig ensure_subscription_all() aufrufen.

    # ------------------------- Public API -------------------------

    def republish(self) -> None:
        """Bitten, alle Posen erneut zu publizieren (latched)."""
        try:
            self._pub_republish.publish(Empty())
            self._logger.info("♻️ Pose-Republish angefordert")
        except Exception as e:
            self._logger.warning(f"PosesClient: Republish fehlgeschlagen: {e}")

    def set_from_tcp(self, name: str, *, wait_for_update: bool = False, timeout: float = 2.0) -> bool:
        """
        Weist den PosesManager an, eine Pose mit 'name' aus dem aktuellen TCP zu speichern.
        Optional warten, bis das latched Topic 'poses/<name>' aktualisiert wurde.
        """
        name = name.strip()
        if not name:
            self._logger.error("PosesClient: Leerer Pose-Name für set_from_tcp()")
            return False

        # Vorherigen Zeitstempel merken, damit wir erkennen, ob die Pose erneuert wurde
        prev_stamp = self._pose_cache.get(name).header.stamp if name in self._pose_cache else None

        self._pub_set_from_tcp.publish(String(data=name))
        self._logger.info(f"📌 Pose aus TCP angefordert: {name}")

        if not wait_for_update:
            return True

        # sicherstellen, dass wir auf das Pose-Topic hören
        self._ensure_subscription(name)

        def _updated() -> bool:
            ps = self._pose_cache.get(name)
            if not ps:
                return False
            if prev_stamp is None:
                return True  # erste Pose für diesen Namen angekommen
            # Vergleich über sec/nanosec (ROS 2 builtin_interfaces/Time)
            return (ps.header.stamp.sec, ps.header.stamp.nanosec) != (prev_stamp.sec, prev_stamp.nanosec)

        ok = self._wait_until(_updated, timeout=timeout)
        if not ok:
            self._logger.warning(f"PosesClient: Keine Aktualisierung für '{name}' innerhalb {timeout:.1f}s.")
        return ok

    def set_pose(self, name: str, pose: PoseStamped) -> bool:
        """
        (Optional) Setzt eine Pose direkt via 'poses/set/<name>'.
        Funktioniert nur, wenn dein PosesManager dieses Topic unterstützt.
        """
        name = name.strip()
        if not name:
            self._logger.error("PosesClient: Leerer Pose-Name für set_pose()")
            return False

        pub = self._get_set_pub(name)
        try:
            pub.publish(pose)
            self._logger.info(f"📝 Pose gesetzt via Topic: {name}")
            return True
        except Exception as e:
            self._logger.error(f"PosesClient: Publish set_pose({name}) fehlgeschlagen: {e}")
            return False

    def get(self, name: str) -> Optional[PoseStamped]:
        """Gibt die letzte empfangene Pose (latched) zurück oder None."""
        return self._pose_cache.get(name.strip())

    def wait_for(self, name: str, *, timeout: float = 2.0, trigger_republish: bool = True) -> Optional[PoseStamped]:
        """
        Wartet auf eine Pose (latched). Optional triggert vorher ein Republish.
        """
        name = name.strip()
        self._ensure_subscription(name)
        if trigger_republish:
            self.republish()

        ok = self._wait_until(lambda: name in self._pose_cache, timeout=timeout)
        if not ok:
            self._logger.warning(f"PosesClient: Keine Pose '{name}' innerhalb {timeout:.1f}s empfangen.")
            return None
        return self._pose_cache[name]

    def ensure_subscription_all(self) -> None:
        """Abonniert alle bekannten Posen-Namen (DEFAULT_POSE_NAMES)."""
        for n in self._pose_names:
            self._ensure_subscription(n)

    def on_change(self, name: str, cb: Callable[[PoseStamped], None]) -> None:
        """Callback registrieren, der bei Änderung der benannten Pose aufgerufen wird."""
        name = name.strip()
        self._ensure_subscription(name)
        if callable(cb):
            self._on_change.setdefault(name, []).append(cb)

    def destroy(self) -> None:
        """Publisher/Subscriber sauber freigeben (optional)."""
        try:
            self._node.destroy_publisher(self._pub_republish)
            self._node.destroy_publisher(self._pub_set_from_tcp)
            for pub in self._set_pubs.values():
                self._node.destroy_publisher(pub)
            for sub in self._subs.values():
                self._node.destroy_subscription(sub)
        except Exception:
            pass
        finally:
            self._set_pubs.clear()
            self._subs.clear()
            self._on_change.clear()
            self._pose_cache.clear()

    # ------------------------- Internals -------------------------

    def _ensure_subscription(self, name: str) -> None:
        if name in self._subs:
            return
        topic = self._topics.pose(name)
        sub = self._node.create_subscription(PoseStamped, topic, lambda msg, n=name: self._on_pose(n, msg), qos_latched())
        self._subs[name] = sub
        self._logger.debug(f"PosesClient: subscribed to '{topic}'")

    def _get_set_pub(self, name: str):
        if name not in self._set_pubs:
            topic = self._topics.pose_set(name)
            self._set_pubs[name] = self._node.create_publisher(PoseStamped, topic, qos_default())
            self._logger.debug(f"PosesClient: set-publisher für '{topic}' erstellt")
        return self._set_pubs[name]

    def _on_pose(self, name: str, msg: PoseStamped) -> None:
        prev = self._pose_cache.get(name)
        self._pose_cache[name] = msg
        # Änderung erkennen (Zeitstempel vergleichen; bei None => immer Änderung)
        changed = (
            prev is None
            or (prev.header.stamp.sec, prev.header.stamp.nanosec)
            != (msg.header.stamp.sec, msg.header.stamp.nanosec)
        )
        if changed:
            self._logger.info(f"📍 Pose aktualisiert: {name} @ {msg.header.frame_id}")
            for cb in self._on_change.get(name, []):
                try:
                    cb(msg)
                except Exception as e:
                    self._logger.warning(f"PosesClient: Callback-Fehler für '{name}': {e}")

    def _wait_until(self, predicate, *, timeout: float) -> bool:
        deadline = time.monotonic() + max(0.0, timeout)
        while time.monotonic() < deadline:
            try:
                rclpy.spin_once(self._node, timeout_sec=0.05)
            except Exception:
                time.sleep(0.05)
            if predicate():
                return True
        return predicate()
