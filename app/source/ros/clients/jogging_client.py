# ros/clients/jogging_client.py
from __future__ import annotations

import time
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from moveit_msgs.msg import ServoStatus

from ..common.topics import Topics
from ..common.qos import qos_sensor_data, qos_default


class JoggingClient:
    """
    Leichter Client für MoveIt Servo Jogging (Cartesian & Joint).

    Topics (RELATIV, Remaps via Namespace/Launch):
      - publish:   Topics.servo_twist   (geometry_msgs/TwistStamped)
      - publish:   Topics.servo_joint   (control_msgs/JointJog)
      - subscribe: Topics.servo_status  (moveit_msgs/ServoStatus)

    Features
    - Deadman-Enable: Nur senden, wenn enabled (enable()/disable()).
    - Frame-Handling: frame_id für Twist-/Joint-Kommandos konfigurierbar.
    - Skalen: lineare & angulare Skalierung für normierte Eingaben (-1..+1).
    - Status: letzte Servo-Statusmeldung abrufen + Callback bei Änderung.
    - Stop: schnelles Nullsetzen der Kommandos (stop()).

    Beispiel
        topics = Topics(servo_ns="moveit_servo")
        jog = JoggingClient(node, topics, default_frame="meca_base")
        jog.enable()
        # normierte Werte (z. B. von Joystick) -> skaliert zu Twist
        jog.twist_norm(lx=0.3, ly=0.0, lz=0.0, ax=0.0, ay=0.0, az=0.1)
        # joints als velocities (rad/s)
        jog.joint_vel({"meca_axis_1_joint": 0.2, "meca_axis_2_joint": -0.1})
        jog.disable(send_stop=True)
    """

    STATUS_TEXT = {
        0: "OK ✅",
        1: "⚠️ Warning",
        2: "⛔ Stopped",
        3: "💥 ERROR",
        4: "⏸️ Paused",
    }

    def __init__(
        self,
        node: Node,
        topics: Optional[Topics] = None,
        *,
        default_frame: str = "meca_base",
        linear_scale: float = 0.5,     # m/s bei normierten Eingaben
        angular_scale: float = 0.5,    # rad/s bei normierten Eingaben
    ) -> None:
        self._node = node
        self._topics = topics or Topics()
        self._log = node.get_logger()

        # Config
        self._frame: str = default_frame
        self._lin_scale: float = float(linear_scale)
        self._ang_scale: float = float(angular_scale)

        # State
        self._enabled: bool = False
        self._last_status: Optional[ServoStatus] = None
        self._status_callbacks: List[Callable[[ServoStatus], None]] = []

        # Pubs/Subs
        self._pub_twist = node.create_publisher(TwistStamped, self._topics.servo_twist, qos_sensor_data())
        self._pub_joint = node.create_publisher(JointJog, self._topics.servo_joint, qos_sensor_data())
        self._sub_status = node.create_subscription(
            ServoStatus, self._topics.servo_status, self._on_status, qos_default()
        )

    # ------------- lifecycle -------------

    def enable(self) -> None:
        """Aktiviert den Deadman – danach werden Commands gesendet."""
        if not self._enabled:
            self._enabled = True
            self._log.info("🟢 Jogging ENABLED")

    def disable(self, *, send_stop: bool = True) -> None:
        """Deaktiviert den Deadman – optional sofort Stopp-Kommandos senden."""
        if self._enabled and send_stop:
            self.stop(repeat=3)
        self._enabled = False
        self._log.info("🔴 Jogging DISABLED")

    def destroy(self) -> None:
        """Publisher/Subscriber sauber freigeben (optional)."""
        try:
            self._node.destroy_publisher(self._pub_twist)
            self._node.destroy_publisher(self._pub_joint)
            self._node.destroy_subscription(self._sub_status)
        except Exception:
            pass

    # ------------- config -------------

    def set_frame(self, frame_id: str) -> None:
        """Setzt das frame_id für kommende Kommandos (z. B. 'meca_base' oder 'tcp')."""
        self._frame = (frame_id or "").strip() or self._frame
        self._log.info(f"🎯 Jog-Frame gesetzt: {self._frame}")

    def set_scales(self, *, linear: Optional[float] = None, angular: Optional[float] = None) -> None:
        """Aktualisiert Skalen für normierte Kommandos (-1..+1)."""
        if linear is not None and linear > 0:
            self._lin_scale = float(linear)
        if angular is not None and angular > 0:
            self._ang_scale = float(angular)
        self._log.info(f"📏 Scales: linear={self._lin_scale:.3f} m/s, angular={self._ang_scale:.3f} rad/s")

    # ------------- status -------------

    def on_status(self, cb: Callable[[ServoStatus], None]) -> None:
        """Registriert Callback für ServoStatus-Updates."""
        if callable(cb):
            self._status_callbacks.append(cb)

    def get_status(self) -> Optional[ServoStatus]:
        return self._last_status

    def get_status_text(self) -> str:
        code = int(self._last_status.code) if self._last_status is not None else None
        return self.STATUS_TEXT.get(code, "—")

    def wait_for_status(self, timeout: float = 2.0) -> bool:
        """Wartet, bis mindestens eine Statusmeldung empfangen wurde."""
        return self._wait_until(lambda: self._last_status is not None, timeout=timeout)

    # ------------- cartesian jog -------------

    def twist(
        self,
        *,
        vx: float = 0.0, vy: float = 0.0, vz: float = 0.0,
        wx: float = 0.0, wy: float = 0.0, wz: float = 0.0,
        frame: Optional[str] = None,
    ) -> bool:
        """
        Sendet absolute Twist-Werte in m/s und rad/s.
        Nutze `twist_norm()` für normierte Eingaben (-1..+1) mit Skalen.
        """
        if not self._enabled:
            return False

        msg = TwistStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = (frame or self._frame)

        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)
        msg.twist.angular.x = float(wx)
        msg.twist.angular.y = float(wy)
        msg.twist.angular.z = float(wz)

        self._pub_twist.publish(msg)
        return True

    def twist_norm(
        self,
        *,
        lx: float = 0.0, ly: float = 0.0, lz: float = 0.0,
        ax: float = 0.0, ay: float = 0.0, az: float = 0.0,
        frame: Optional[str] = None,
    ) -> bool:
        """
        Normierte Eingaben (-1..+1) werden mit den Skalen multipliziert.
        """
        return self.twist(
            vx=self._lin_scale * float(lx),
            vy=self._lin_scale * float(ly),
            vz=self._lin_scale * float(lz),
            wx=self._ang_scale * float(ax),
            wy=self._ang_scale * float(ay),
            wz=self._ang_scale * float(az),
            frame=frame,
        )

    # ------------- joint jog -------------

    def joint_vel(
        self,
        velocities: Dict[str, float],
        *,
        frame: Optional[str] = None,
    ) -> bool:
        """
        Sendet Joint-Velocities (rad/s) für beliebige Gelenke.
        Das Nachrichtenlayout von control_msgs/JointJog variiert historisch leicht;
        wir füllen vorhandene Felder per hasattr()-Check.
        """
        if not self._enabled or not velocities:
            return False

        msg = JointJog()
        # Header
        if hasattr(msg, "header"):
            msg.header.stamp = self._node.get_clock().now().to_msg()
            # Einige Servo-Configs nutzen frame_id aus JointJog; wir tragen dasselbe Frame wie beim Twist ein.
            try:
                msg.header.frame_id = (frame or self._frame)
            except Exception:
                pass

        # Pflichtfelder
        if hasattr(msg, "joint_names"):
            msg.joint_names = list(velocities.keys())

        # Manche Varianten haben 'velocities', manche zusätzlich/alternativ 'displacements'
        if hasattr(msg, "velocities"):
            msg.velocities = [float(v) for v in velocities.values()]
        if hasattr(msg, "displacements") and (not getattr(msg, "velocities", None)):
            # Falls es nur displacements gäbe, interpretieren wir sie als kleine Inkremente (Fallback)
            msg.displacements = [float(v) * 0.02 for v in velocities.values()]  # 20 ms Schritt

        # duration kann existieren; wenn ja, setzen wir eine kurze Dauer
        if hasattr(msg, "duration"):
            try:
                from builtin_interfaces.msg import Duration as RosDuration
                msg.duration = RosDuration(sec=0, nanosec=int(0.02 * 1e9))  # 20 ms
            except Exception:
                pass

        self._pub_joint.publish(msg)
        return True

    # ------------- emergency / stop -------------

    def stop(self, *, repeat: int = 2, frame: Optional[str] = None) -> None:
        """
        Sende ein paar Null-Kommandos, um Restbewegung zu dämpfen/abzubrechen.
        """
        f = (frame or self._frame)
        for _ in range(max(1, int(repeat))):
            try:
                self.twist(vx=0, vy=0, vz=0, wx=0, wy=0, wz=0, frame=f)
                # Joint-Nullen nur, wenn sinnvoll (keine Gelenke angegeben → überspringen)
            except Exception:
                pass
            # kleinen Spin, damit publish rausgeht
            try:
                rclpy.spin_once(self._node, timeout_sec=0.01)
            except Exception:
                time.sleep(0.01)

    # ------------- internals -------------

    def _on_status(self, msg: ServoStatus) -> None:
        prev = self._last_status.code if self._last_status is not None else None
        self._last_status = msg
        if prev != msg.code:
            # kleine, kompakte Log-Zeile
            txt = self.STATUS_TEXT.get(int(msg.code), f"code={int(msg.code)}")
            self._log.info(f"🛰️ ServoStatus: {txt}")
            for cb in list(self._status_callbacks):
                try:
                    cb(msg)
                except Exception as e:
                    self._log.warning(f"JoggingClient: Status-Callback-Fehler: {e}")

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
