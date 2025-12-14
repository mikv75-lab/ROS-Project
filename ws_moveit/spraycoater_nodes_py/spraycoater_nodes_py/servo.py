# -*- coding: utf-8 -*-
# spraycoater_nodes_py/servo.py
#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from std_msgs.msg import String
from moveit_msgs.srv import ServoCommandType

from spraycoater_nodes_py.utils.config_hub import topics, frames

NODE_KEY = "servo"


class ServoBridge(Node):
    """
    Bridge zwischen UI-ServiceTabs und moveit_servo (Fake-/SIM-Roboter)
    ODER Omron-Real (via TCP-Bridge).

    SIM-Backends  (z.B. 'omron_sim', 'meca_sim', 'default'):
      - publizieren Twist/JointJog an moveit_servo:
          /servo/cartesian_mm (TwistStamped)
          /servo/joint_jog    (JointJog)
      - benutzen /omron_servo_node/switch_command_type

    REAL-Backends (z.B. 'omron_real'):
      - wandeln Twist/JointJog in ASCII-Kommandos:
          JOGJ axis step_deg speed_pct
          JOGC speed_mmps dx dy dz drx dry drz
        und publishen die Strings auf:
          /omron/command
      - kein moveit_servo Service, keine Publishes an cartesian_mm/joint_jog
    """

    # Mapping von String -> ServoCommandType
    MODE_MAP = {
        "JOINT_JOG": ServoCommandType.Request.JOINT_JOG,
        "JOINT":     ServoCommandType.Request.JOINT_JOG,
        "J":         ServoCommandType.Request.JOINT_JOG,

        "TWIST":     ServoCommandType.Request.TWIST,
        "CARTESIAN": ServoCommandType.Request.TWIST,
        "T":         ServoCommandType.Request.TWIST,

        "POSE":      ServoCommandType.Request.POSE,
        "P":         ServoCommandType.Request.POSE,
    }

    # simple Reihenfolge der Achsen
    AXIS_ORDER = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

    def __init__(self) -> None:
        super().__init__("servo_bridge")

        # ---------------- Backend-Parameter ----------------
        self.declare_parameter("backend", "default")
        self.backend: str = (
            self.get_parameter("backend").get_parameter_value().string_value or "default"
        )
        backend_lower = self.backend.lower()

        self.is_real_backend = "real" in backend_lower

        # moveit_servo-Service nur in SIM / default
        self.use_servo_service = (
            (not self.is_real_backend)
            and ("sim" in backend_lower or self.backend == "default")
        )

        # Defaults für REAL-Backend (kannst du aus Bringup setzen)
        # → landen im Command (JOGJ/JOGC), Omron setzt SPEED nur aus diesen Werten.
        self.declare_parameter("joint_speed_pct_default", 30.0)
        self.declare_parameter("cart_speed_mmps_default", 50.0)
        self.joint_speed_pct_default = (
            self.get_parameter("joint_speed_pct_default")
            .get_parameter_value()
            .double_value
        )
        self.cart_speed_mmps_default = (
            self.get_parameter("cart_speed_mmps_default")
            .get_parameter_value()
            .double_value
        )

        # Config-Loader / Frames (nur für SIM relevant)
        self.loader = topics()
        self.frames_cfg = frames()
        self._F = self.frames_cfg.resolve

        # Default-Frame (tcp aus frames.yaml, sonst "world")
        self.current_frame = self._F(self.frames_cfg.get("tcp", "world"))

        # ---- Topics aus config_hub ----
        topic_twist_in = self.loader.subscribe_topic(NODE_KEY, "twist_out")
        qos_twist_in = self.loader.qos_by_id("subscribe", NODE_KEY, "twist_out")

        topic_joint_in = self.loader.subscribe_topic(NODE_KEY, "joint_out")
        qos_joint_in = self.loader.qos_by_id("subscribe", NODE_KEY, "joint_out")

        topic_set_mode = self.loader.subscribe_topic(NODE_KEY, "set_mode")
        qos_set_mode = self.loader.qos_by_id("subscribe", NODE_KEY, "set_mode")

        topic_set_frame = self.loader.subscribe_topic(NODE_KEY, "set_frame")
        qos_set_frame = self.loader.qos_by_id("subscribe", NODE_KEY, "set_frame")

        topic_cartesian_cmd = self.loader.publish_topic(NODE_KEY, "cartesian_mm")
        qos_cartesian_cmd = self.loader.qos_by_id("publish", NODE_KEY, "cartesian_mm")

        topic_joint_cmd = self.loader.publish_topic(NODE_KEY, "joint_jog")
        qos_joint_cmd = self.loader.qos_by_id("publish", NODE_KEY, "joint_jog")

        # ---- Publisher für SIM (moveit_servo) ----
        self.pub_cartesian = self.create_publisher(
            TwistStamped, topic_cartesian_cmd, qos_cartesian_cmd
        )
        self.pub_joint = self.create_publisher(
            JointJog, topic_joint_cmd, qos_joint_cmd
        )

        # ---- Publisher für REAL (OmronTcpBridge) ----
        self.pub_omron_cmd = self.create_publisher(String, "/omron/command", 10)

        # ---- Subscriber (von UI-Tabs) ----
        self.create_subscription(
            TwistStamped, topic_twist_in, self._on_twist_cmd, qos_twist_in
        )
        self.create_subscription(
            JointJog, topic_joint_in, self._on_joint_cmd, qos_joint_in
        )
        self.create_subscription(
            String, topic_set_mode, self._on_set_mode, qos_set_mode
        )
        self.create_subscription(
            String, topic_set_frame, self._on_set_frame, qos_set_frame
        )

        # ---- Service-Client für CommandType (nur SIM/Fake) ----
        if self.use_servo_service:
            self.servo_client = self.create_client(
                ServoCommandType, "/omron_servo_node/switch_command_type"
            )
            self.get_logger().info(
                f"ServoBridge: benutze moveit_servo CommandType-Service (backend='{self.backend}')"
            )
        else:
            self.servo_client = None
            self.get_logger().info(
                f"ServoBridge: KEIN moveit_servo CommandType-Service aktiv (backend='{self.backend}')"
            )

        self.current_mode: int | None = None

        self.get_logger().info(
            f"✅ ServoBridge aktiv (backend='{self.backend}') – "
            f"Frame='{self.current_frame}', "
            f"SIM publish: '{topic_cartesian_cmd}', '{topic_joint_cmd}', "
            f"REAL publish: '/omron/command'"
        )

    # ------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------

    def _on_twist_cmd(self, msg: TwistStamped) -> None:
        """
        Cartesian Jog:
          SIM  -> TwistStamped an moveit_servo (/servo/cartesian_mm)
          REAL -> JOGC-String an OmronTcpBridge (/omron/command)
        """
        if self.is_real_backend:
            cmd = self._make_jogc_cmd_from_twist(msg)
            if cmd is None:
                return
            out = String()
            out.data = cmd
            self.pub_omron_cmd.publish(out)
            self.get_logger().debug(f"[{self.backend}] JOGC → '{cmd}'")
            return

        # --- SIM / Fake-Roboter ---
        out = TwistStamped()
        out.twist = msg.twist  # nur Inhalt übernehmen
        out.header.frame_id = self.current_frame
        out.header.stamp = self.get_clock().now().to_msg()
        self.pub_cartesian.publish(out)
        self.get_logger().debug(
            f"[{self.backend}] Twist → cartesian_mm (frame={out.header.frame_id}, "
            f"lin=({out.twist.linear.x:.4f},"
            f"{out.twist.linear.y:.4f},"
            f"{out.twist.linear.z:.4f}), "
            f"ang=({out.twist.angular.x:.4f},"
            f"{out.twist.angular.y:.4f},"
            f"{out.twist.angular.z:.4f}))"
        )

    def _on_joint_cmd(self, msg: JointJog) -> None:
        """
        Joint Jog:
          SIM  -> JointJog an moveit_servo (/servo/joint_jog)
          REAL -> JOGJ-String an OmronTcpBridge (/omron/command)
        """
        if self.is_real_backend:
            cmd = self._make_jogj_cmd_from_joint(msg)
            if cmd is None:
                return
            out = String()
            out.data = cmd
            self.pub_omron_cmd.publish(out)
            self.get_logger().debug(f"[{self.backend}] JOGJ → '{cmd}'")
            return

        # --- SIM / Fake-Roboter ---
        out = JointJog()
        out.header.frame_id = msg.header.frame_id or self.current_frame
        out.header.stamp = self.get_clock().now().to_msg()
        out.joint_names = list(msg.joint_names)
        out.velocities = list(msg.velocities)
        out.displacements = list(msg.displacements)
        out.duration = msg.duration

        self.pub_joint.publish(out)
        self.get_logger().debug(
            f"[{self.backend}] JointJog → joint_jog (n={len(out.joint_names)}, duration={out.duration:.3f})"
        )

    # ------------------------------------------------------------
    # Hilfsfunktionen REAL-Backend
    # ------------------------------------------------------------

    def _make_jogj_cmd_from_joint(self, msg: JointJog) -> str | None:
        """
        Erwartung:
          - genau ein Joint im Array
          - displacements[0] = Schritt in Grad
          - velocities[0]   = Speed in % (optional -> Default)
        """
        if not msg.joint_names:
            self.get_logger().warn(f"[{self.backend}] JOGJ: joint_names leer.")
            return None

        jname = msg.joint_names[0].lower()
        # Achsnummer bestimmen
        try:
            axis = self.AXIS_ORDER.index(jname) + 1
        except ValueError:
            # Fallback: z.B. 'Viper_s650_Link1' → letzte Ziffer
            try:
                axis = int(jname[-1])
            except Exception:
                self.get_logger().warn(
                    f"[{self.backend}] JOGJ: kann Achse aus '{msg.joint_names[0]}' nicht bestimmen."
                )
                return None

        if axis < 1 or axis > 6:
            self.get_logger().warn(
                f"[{self.backend}] JOGJ: ungültige Achse {axis} aus '{msg.joint_names[0]}'"
            )
            return None

        step_deg = 0.0
        if msg.displacements:
            step_deg = msg.displacements[0]
        else:
            self.get_logger().warn(
                f"[{self.backend}] JOGJ: displacements leer – Schritt=0."
            )

        if abs(step_deg) < 1e-6:
            return None

        if msg.velocities:
            speed_pct = msg.velocities[0]
        else:
            speed_pct = self.joint_speed_pct_default

        # $args: "axis step_deg [speed_pct]"
        cmd = f"JOGJ {axis} {step_deg:.3f} {speed_pct:.1f}"
        return cmd

    def _make_jogc_cmd_from_twist(self, msg: TwistStamped) -> str | None:
        """
        Erwartung (REAL-Backend):
          - twist.linear.{x,y,z}  = Schritt in mm
          - twist.angular.{x,y,z} = Schritt in Grad
          - Speed (mm/s) aus cart_speed_mmps_default

        Frame (WORLD/TOOL) ist bereits in Servo/MoveIt berücksichtigt
        und wird NICHT mehr an Omron übergeben.
        """
        dx = msg.twist.linear.x
        dy = msg.twist.linear.y
        dz = msg.twist.linear.z
        drx = msg.twist.angular.x
        dry = msg.twist.angular.y
        drz = msg.twist.angular.z

        # wenn alles 0 → nichts senden
        if (abs(dx) < 1e-6 and abs(dy) < 1e-6 and abs(dz) < 1e-6 and
                abs(drx) < 1e-6 and abs(dry) < 1e-6 and abs(drz) < 1e-6):
            return None

        speed_mmps = self.cart_speed_mmps_default

        # Neues Format: "JOGC <Speed> dx dy dz drx dry drz"
        cmd = (
            f"JOGC {speed_mmps:.1f} "
            f"{dx:.3f} {dy:.3f} {dz:.3f} "
            f"{drx:.3f} {dry:.3f} {drz:.3f}"
        )
        return cmd

    # ------------------------------------------------------------
    # Mode / Frame
    # ------------------------------------------------------------

    def _on_set_mode(self, msg: String) -> None:
        raw = (msg.data or "").strip().upper()
        if raw not in self.MODE_MAP:
            self.get_logger().warning(
                f"[{self.backend}] Unbekannter Servo-Mode '{msg.data}' – erwartet z.B. JOINT_JOG, TWIST oder POSE."
            )
            return

        cmd_type = self.MODE_MAP[raw]
        self._switch_mode(cmd_type, raw)

    def _switch_mode(self, cmd_type: int, label: str = "") -> None:
        """Interne Hilfsfunktion: CommandType-Service synchron aufrufen (nur SIM/Fake)."""
        if not self.use_servo_service:
            self.current_mode = cmd_type
            self.get_logger().info(
                f"[{self.backend}] (real) Servo-Mode lokal auf '{label}' gesetzt – "
                f"kein CommandType-Service aktiv."
            )
            return

        if self.current_mode == cmd_type:
            self.get_logger().info(
                f"[{self.backend}] Servo-Mode bereits '{label}' – kein Servicecall nötig."
            )
            return

        if self.servo_client is None:
            self.get_logger().warning(
                f"[{self.backend}] Servo-Client nicht initialisiert – kann Mode '{label}' nicht setzen."
            )
            return

        if not self.servo_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning(
                f"[{self.backend}] Service /omron_servo_node/switch_command_type nicht verfügbar."
            )
            return

        req = ServoCommandType.Request()
        req.command_type = cmd_type

        self.get_logger().info(
            f"[{self.backend}] ↪ switch_command_type: '{label}' → command_type={cmd_type}"
        )
        future = self.servo_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if not future.result():
            self.get_logger().error(
                f"[{self.backend}] switch_command_type: Keine Antwort vom Service."
            )
            return

        resp = future.result()
        if resp.success:
            self.current_mode = cmd_type
            self.get_logger().info(
                f"[{self.backend}] ✅ Servo-Mode geändert auf '{label}' (command_type={cmd_type})"
            )
        else:
            self.get_logger().warning(
                f"[{self.backend}] ❌ switch_command_type fehlgeschlagen (command_type={cmd_type})"
            )

    def _on_set_frame(self, msg: String) -> None:
        raw = (msg.data or "").strip().lower()
        if not raw:
            self.get_logger().warning(f"[{self.backend}] set_frame: Leerstring ignoriert.")
            return

        allowed = ("world", "tcp")
        if raw not in allowed:
            self.get_logger().warning(
                f"[{self.backend}] set_frame: Frame '{raw}' nicht erlaubt. "
                f"Erlaubt: 'world', 'tcp'. "
                f"Frames.yaml enthält, wird aber ignoriert: "
                f"{list(self.frames_cfg.frames.keys())}"
            )
            return

        resolved = self._F(self.frames_cfg.get(raw, raw))

        old = self.current_frame
        self.current_frame = resolved

        self.get_logger().info(
            f"[{self.backend}] Servo-Frame geändert: {old} → {self.current_frame} (gewählt: {raw})"
        )


# ------------------------------------------------------------------
# main: normaler Node-Start
# ------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ServoBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
