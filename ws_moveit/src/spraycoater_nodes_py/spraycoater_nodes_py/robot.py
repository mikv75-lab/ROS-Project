# -*- coding: utf-8 -*-
# spraycoater_nodes_py/robot.py
#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from std_msgs.msg import Empty, Bool, String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from tf2_ros import (
    Buffer,
    TransformListener,
    LookupException,
    ConnectivityException,
    ExtrapolationException,
)

from spraycoater_nodes_py.utils.config_hub import (
    topics,
    frames,
)


class Robot(Node):
    """
    Robot-Node:

    - Bedient alle Topics aus topics.yaml für 'robot'.
    - Holt zyklisch die Pose von 'tool_mount' im Frame 'world' via TF und
      publiziert sie auf dem Topic 'tcp_pose' (PoseStamped).
    - Spiegelt /joint_states auf das 'joints'-Topic der GUI.
    """

    def __init__(self) -> None:
        super().__init__("robot")

        # Loader / Frames
        self.loader = topics()
        self.frames = frames()
        self._F = self.frames.resolve
        self.node_key = "robot"

        # TF-Buffer / Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Frames für TCP-Pose
        self.world_frame = self._detect_world_frame()
        self.tool_frame = self._detect_tool_frame()

        # Interner State
        self._connected: bool = False
        self._initialized: bool = False
        self._moving: bool = False
        self._servo_enabled: bool = False
        self._power: bool = False
        self._estop: bool = False
        self._mode: str = "DISCONNECTED"
        self._last_error: str = ""

        # TCP-/Joint-State Container
        self._tcp_pose = PoseStamped()
        self._tcp_pose.header.frame_id = self.world_frame

        # letzter JointState (aus /joint_states)
        self._joints = JointState()

        # ------------------------
        # Publisher
        # ------------------------
        self.pub_connection = self._make_pub(Bool, "connection")
        self.pub_mode = self._make_pub(String, "mode")
        self.pub_initialized = self._make_pub(Bool, "initialized")
        self.pub_moving = self._make_pub(Bool, "moving")
        self.pub_servo_enabled = self._make_pub(Bool, "servo_enabled")
        self.pub_power = self._make_pub(Bool, "power")
        self.pub_estop = self._make_pub(Bool, "estop")
        self.pub_errors = self._make_pub(String, "errors")

        self.pub_tcp_pose = self._make_pub(PoseStamped, "tcp_pose")
        self.pub_joints = self._make_pub(JointState, "joints")

        # ------------------------
        # Subscriber (Commands)
        # ------------------------
        self.sub_init = self._make_sub(Empty, "init", self._on_init)
        self.sub_stop = self._make_sub(Empty, "stop", self._on_stop)
        self.sub_clear_error = self._make_sub(Empty, "clear_error", self._on_clear_error)
        self.sub_power_on = self._make_sub(Empty, "power_on", self._on_power_on)
        self.sub_power_off = self._make_sub(Empty, "power_off", self._on_power_off)
        self.sub_servo_on = self._make_sub(Empty, "servo_on", self._on_servo_on)
        self.sub_servo_off = self._make_sub(Empty, "servo_off", self._on_servo_off)

        # ------------------------
        # Joint-State Quelle: /joint_states vom joint_state_broadcaster
        # ------------------------
        self.sub_joint_states = self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_states,
            10,  # QoS ok, wir decimaten später auf 10 Hz
        )

        # ------------------------
        # Timer für State + TCP-Pose
        # ------------------------
        # 10 Hz für GUI reicht völlig
        self.state_timer = self.create_timer(0.1, self._publish_state)

        # TF-Warnung nur EINMAL und erst nach einer kleinen "Grace Time"
        self._tf_warned = False
        self._startup_time = self.get_clock().now()
        self._tf_grace_duration = Duration(seconds=5.0)  # 5 s: währenddessen keine Warnung

        self.get_logger().info(
            f"🤖 Robot-Node gestartet: tcp_pose = Pose({self.tool_frame} in {self.world_frame}), "
            "Joints aus /joint_states"
        )

    # ------------------------------------------------------------------
    # Helpers: Topics / Frames
    # ------------------------------------------------------------------
    def _detect_world_frame(self) -> str:
        """Bestimmt den World-Frame aus frames.yaml, fallback 'world'."""
        for cand in ("world", "base_world", "map"):
            try:
                fid = self._F(cand)
                if fid:
                    return fid
            except Exception:
                continue
        return "world"

    def _detect_tool_frame(self) -> str:
        """Bestimmt den Tool-Frame aus frames.yaml, bevorzugt 'tool_mount'."""
        for cand in ("tool_mount", "tcp", "tool0", "ee_link"):
            try:
                fid = self._F(cand)
                if fid:
                    return fid
            except Exception:
                continue
        # Im Worst Case zeigen wir einfach 'world' auf sich selbst
        return self.world_frame

    def _make_pub(self, msg_type, topic_id: str):
        return self.create_publisher(
            msg_type,
            self.loader.publish_topic(self.node_key, topic_id),
            self.loader.qos_by_id("publish", self.node_key, topic_id),
        )

    def _make_sub(self, msg_type, topic_id: str, cb):
        return self.create_subscription(
            msg_type,
            self.loader.subscribe_topic(self.node_key, topic_id),
            cb,
            self.loader.qos_by_id("subscribe", self.node_key, topic_id),
        )

    # ------------------------------------------------------------------
    # State-Setter
    # ------------------------------------------------------------------
    def _set_mode(self, mode: str):
        if mode != self._mode:
            self._mode = mode
            self.get_logger().info(f"🔁 Robot-Mode -> {mode}")

    def _set_error(self, msg: str):
        msg = (msg or "").strip()
        if not msg:
            return
        self._last_error = msg
        self.get_logger().error(f"❌ Robot-Error: {msg}")
        self.pub_errors.publish(String(data=msg))

    # ------------------------------------------------------------------
    # Joint-State Callback
    # ------------------------------------------------------------------
    def _on_joint_states(self, msg: JointState):
        """
        /joint_states vom joint_state_broadcaster.
        Wird 1:1 gespeichert und später mit 10 Hz an die GUI gespiegelt.
        """
        self._joints = msg

    # ------------------------------------------------------------------
    # Command-Handler
    # ------------------------------------------------------------------
    def _on_init(self, _msg: Empty):
        self.get_logger().info("📥 Command: INIT")

        if self._estop:
            self._set_error("INIT verweigert – E-Stop aktiv.")
            return

        self._connected = True
        self._power = True
        self._servo_enabled = True
        self._initialized = True
        self._moving = False
        self._set_mode("IDLE")

    def _on_stop(self, _msg: Empty):
        self.get_logger().info("📥 Command: STOP")
        self._moving = False
        if self._power:
            self._set_mode("STOPPED")
        else:
            self._set_mode("POWERED_OFF")

    def _on_clear_error(self, _msg: Empty):
        self.get_logger().info("📥 Command: CLEAR_ERROR")
        self._last_error = ""
        self.pub_errors.publish(String(data=""))

    def _on_power_on(self, _msg: Empty):
        self.get_logger().info("📥 Command: POWER_ON")
        self._power = True
        self._connected = True
        if not self._initialized:
            self._set_mode("POWERED_ON")
        else:
            self._set_mode("IDLE")

    def _on_power_off(self, _msg: Empty):
        self.get_logger().info("📥 Command: POWER_OFF")
        self._power = False
        self._servo_enabled = False
        self._moving = False
        self._set_mode("POWERED_OFF")

    def _on_servo_on(self, _msg: Empty):
        self.get_logger().info("📥 Command: SERVO_ON")
        if not self._power:
            self._set_error("Servo ON nur möglich, wenn Power an ist.")
            return
        self._servo_enabled = True
        if self._initialized:
            self._set_mode("READY")
        else:
            self._set_mode("POWERED_ON")

    def _on_servo_off(self, _msg: Empty):
        self.get_logger().info("📥 Command: SERVO_OFF")
        self._servo_enabled = False
        self._moving = False
        if self._power:
            self._set_mode("POWERED_ON")
        else:
            self._set_mode("POWERED_OFF")

    # ------------------------------------------------------------------
    # TF → PoseStamped (tool_mount in world)
    # ------------------------------------------------------------------
    def _update_tcp_pose_from_tf(self):
        """
        Holt Transform world -> tool_mount aus TF und schreibt ihn in self._tcp_pose.

        - Nutzt Time() == "latest" Transform.
        - Wartet kurz (Timeout), bevor es aufgibt.
        - Warnung erst NACH einer Grace-Time, damit kein falscher Alarm
          beim Startup kommt.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.tool_frame,
                Time(),  # "latest" Transform
                timeout=Duration(seconds=0.2),
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as ex:
            # Während der Startup-Grace kein Warn-Spam, weil TF-Tree sich erst aufbaut
            now = self.get_clock().now()
            if (
                not self._tf_warned
                and (now - self._startup_time) > self._tf_grace_duration
            ):
                self.get_logger().warning(
                    f"⚠️ Kein TF {self.world_frame} -> {self.tool_frame} verfügbar – "
                    f"tcp_pose bleibt 0. ({ex})"
                )
                self._tf_warned = True
            return

        # Ab hier ist TF stabil verfügbar
        if self._tf_warned:
            self.get_logger().info(
                f"✅ TF {self.world_frame} -> {self.tool_frame} jetzt verfügbar."
            )
            self._tf_warned = False

        self._tcp_pose.header.stamp = tf.header.stamp
        self._tcp_pose.header.frame_id = self.world_frame

        self._tcp_pose.pose.position.x = tf.transform.translation.x
        self._tcp_pose.pose.position.y = tf.transform.translation.y
        self._tcp_pose.pose.position.z = tf.transform.translation.z

        self._tcp_pose.pose.orientation = tf.transform.rotation

    # ------------------------------------------------------------------
    # Zyklische Publikation
    # ------------------------------------------------------------------
    def _publish_state(self):
        """
        Publiziert States + TCP-Pose + JointState zyklisch.
        TCP-Pose: aus TF (world -> tool_mount)
        JointState: letztes /joint_states (falls vorhanden)
        """
        now_msg = self.get_clock().now().to_msg()

        # States
        self.pub_connection.publish(Bool(data=self._connected))
        self.pub_initialized.publish(Bool(data=self._initialized))
        self.pub_moving.publish(Bool(data=self._moving))
        self.pub_servo_enabled.publish(Bool(data=self._servo_enabled))
        self.pub_power.publish(Bool(data=self._power))
        self.pub_estop.publish(Bool(data=self._estop))
        self.pub_mode.publish(String(data=self._mode))

        # TCP-Pose aus TF aktualisieren und publizieren
        self._update_tcp_pose_from_tf()
        if self._tcp_pose.header.stamp.sec != 0 or self._tcp_pose.header.stamp.nanosec != 0:
            self.pub_tcp_pose.publish(self._tcp_pose)

        # JointState: nur senden, wenn wir sinnvolle Daten haben
        if self._joints.name and self._joints.position:
            self._joints.header.stamp = now_msg
            self.pub_joints.publish(self._joints)


def main():
    rclpy.init()
    node = Robot()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
