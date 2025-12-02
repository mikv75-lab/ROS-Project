# spraycoater_nodes_py/robot/sim.py
from __future__ import annotations

from rclpy.time import Time
from rclpy.duration import Duration

from std_msgs.msg import Empty, String
from sensor_msgs.msg import JointState

from tf2_ros import (
    Buffer,
    TransformListener,
    LookupException,
    ConnectivityException,
    ExtrapolationException,
)

from spraycoater_nodes_py.utils.config_hub import frames
from .base import BaseRobot


class SimRobot(BaseRobot):
    """
    Simulation:

    - Kommandos: /spraycoater/robot/init|stop|power_on|...
    - TCP-Pose aus TF (world → tool_mount/tcp)
    - Joints aus /joint_states
    """

    def __init__(self) -> None:
        super().__init__(node_name="robot")

        # Frames
        self.frames = frames()
        self._F = self.frames.resolve

        self.world_frame = self._detect_world_frame()
        self.tool_frame = self._detect_tool_frame()

        self._tcp_pose.header.frame_id = self.world_frame
        self._joints = JointState()

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._tf_warned = False
        self._startup_time = self.get_clock().now()
        self._tf_grace_duration = Duration(seconds=5.0)

        # Robot-Kommandos (topics.yaml.robot.subscribe)
        self.sub_init = self._make_sub(Empty, "init", self._on_init)
        self.sub_stop = self._make_sub(Empty, "stop", self._on_stop)
        self.sub_clear_error = self._make_sub(Empty, "clear_error", self._on_clear_error)
        self.sub_power_on = self._make_sub(Empty, "power_on", self._on_power_on)
        self.sub_power_off = self._make_sub(Empty, "power_off", self._on_power_off)
        self.sub_servo_on = self._make_sub(Empty, "servo_on", self._on_servo_on)
        self.sub_servo_off = self._make_sub(Empty, "servo_off", self._on_servo_off)

        # /joint_states vom joint_state_broadcaster
        self.sub_joint_states = self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_states,
            10,
        )

        self.get_logger().info(
            f"🤖 SimRobot gestartet: tcp_pose = Pose({self.tool_frame} in {self.world_frame}), "
            "Joints aus /joint_states"
        )

    # ---------------------------------------------------------
    # Frame-Detection
    # ---------------------------------------------------------
    def _detect_world_frame(self) -> str:
        for cand in ("world", "base_world", "map"):
            try:
                fid = self._F(cand)
                if fid:
                    return fid
            except Exception:
                continue
        return "world"

    def _detect_tool_frame(self) -> str:
        for cand in ("tool_mount", "tcp", "tool0", "ee_link"):
            try:
                fid = self._F(cand)
                if fid:
                    return fid
            except Exception:
                continue
        return "tcp"

    # ---------------------------------------------------------
    # Joint-State Callback
    # ---------------------------------------------------------
    def _on_joint_states(self, msg: JointState):
        self._joints = msg

    # ---------------------------------------------------------
    # Command-Handler (State-Maschine SIM)
    # ---------------------------------------------------------
    def _on_init(self, _msg: Empty):
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
        self._moving = False
        if self._power:
            self._set_mode("STOPPED")
        else:
            self._set_mode("POWERED_OFF")

    def _on_clear_error(self, _msg: Empty):
        self._last_error = ""
        self.pub_errors.publish(String(data=""))

    def _on_power_on(self, _msg: Empty):
        self._power = True
        self._connected = True
        if not self._initialized:
            self._set_mode("POWERED_ON")
        else:
            self._set_mode("IDLE")

    def _on_power_off(self, _msg: Empty):
        self._power = False
        self._servo_enabled = False
        self._moving = False
        self._set_mode("POWERED_OFF")

    def _on_servo_on(self, _msg: Empty):
        if not self._power:
            self._set_error("Servo ON nur möglich, wenn Power an ist.")
            return
        self._servo_enabled = True
        if self._initialized:
            self._set_mode("READY")
        else:
            self._set_mode("POWERED_ON")

    def _on_servo_off(self, _msg: Empty):
        self._servo_enabled = False
        self._moving = False
        if self._power:
            self._set_mode("POWERED_ON")
        else:
            self._set_mode("POWERED_OFF")

    # ---------------------------------------------------------
    # Hooks aus BaseRobot
    # ---------------------------------------------------------
    def _update_tcp_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.tool_frame,
                Time(),  # latest
                timeout=Duration(seconds=0.2),
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as ex:
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


def main():
    import rclpy
    rclpy.init()
    node = SimRobot()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
