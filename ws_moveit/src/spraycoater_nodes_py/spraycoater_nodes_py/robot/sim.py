#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# spraycoater_nodes_py/robot/sim.py

from __future__ import annotations

from rclpy.time import Time
from rclpy.duration import Duration

from std_msgs.msg import Empty
from sensor_msgs.msg import JointState

from tf2_ros import (
    Buffer,
    TransformListener,
    LookupException,
    ConnectivityException,
    ExtrapolationException,
)

from spraycoater_nodes_py.utils.config_hub import frames
from geometry_msgs.msg import PoseStamped
from .base import BaseRobot


class SimRobot(BaseRobot):
    """
    Simulation (event-driven, kein Polling):

    - Kommandos: robot.init|stop|power_on|... (topics.yaml.robot.subscribe)
    - TCP-Pose aus TF (world → tcp) wird bei Events aktualisiert (JointState/Commands)
    - Joints aus JointState-Topic (joint_state_broadcaster / ros2_control)
    """

    def __init__(self) -> None:
        super().__init__(node_name="robot")

        # Frames
        self.frames = frames()
        self._F = self.frames.resolve
        self.world_frame = self._F("world")
        self.tool_frame = self._F("tcp")

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._tf_warned = False
        self._startup_time = self.get_clock().now()
        self._tf_grace_duration = Duration(seconds=2.0)

        # Kommandos
        self.sub_init = self._make_sub(Empty, "init", self._on_init)
        self.sub_stop = self._make_sub(Empty, "stop", self._on_stop)
        self.sub_clear_error = self._make_sub(Empty, "clear_error", self._on_clear_error)
        self.sub_power_on = self._make_sub(Empty, "power_on", self._on_power_on)
        self.sub_power_off = self._make_sub(Empty, "power_off", self._on_power_off)
        self.sub_servo_on = self._make_sub(Empty, "servo_on", self._on_servo_on)
        self.sub_servo_off = self._make_sub(Empty, "servo_off", self._on_servo_off)

        # joint_states input
        self.sub_joint_states = self._make_sub(JointState, "joint_states_in", self._on_joint_states)

        js_topic = self.cfg.subscribe_topic("robot", "joint_states_in")
        self.get_logger().info(
            f"🤖 SimRobot gestartet: tcp_pose = TF({self.world_frame}->{self.tool_frame}), "
            f"JointStates subscribe='{js_topic}' (via config_hub, onChange)"
        )

        # Startzustand einmalig publishen
        self.set_mode("POWERED_OFF")
        self.publish_all_once()

    # ----------------------------------------------------------
    # joint_states event -> publish joints + update tcp once
    # ----------------------------------------------------------

    def _on_joint_states(self, msg: JointState) -> None:
        # Joints: publish nur bei Änderung
        self.publish_joints(msg)

        # Wenn JointStates kommen, ist das System "connected"
        self.set_connected(True)

        # TCP-Pose: event-driven update (und publish nur bei Änderung)
        self._update_tcp_pose_event()

    # ----------------------------------------------------------
    # Commands (event-driven) -> nur onChange publishen
    # ----------------------------------------------------------

    def _on_init(self, _msg: Empty) -> None:
        if self._estop:
            self.set_error("INIT verweigert – E-Stop aktiv.")
            return

        self.set_error("")  # cleared (nur bei Änderung)
        self.set_connected(True)
        self.set_power(True)
        self.set_servo_enabled(True)
        self.set_initialized(True)
        self.set_moving(False)
        self.set_mode("IDLE")
        self._update_tcp_pose_event()

    def _on_stop(self, _msg: Empty) -> None:
        self.set_moving(False)
        self.set_mode("STOPPED" if self._power else "POWERED_OFF")
        self._update_tcp_pose_event()

    def _on_clear_error(self, _msg: Empty) -> None:
        self.set_error("")

    def _on_power_on(self, _msg: Empty) -> None:
        self.set_connected(True)
        self.set_power(True)
        self.set_mode("IDLE" if self._initialized else "POWERED_ON")
        self._update_tcp_pose_event()

    def _on_power_off(self, _msg: Empty) -> None:
        self.set_power(False)
        self.set_servo_enabled(False)
        self.set_moving(False)
        self.set_mode("POWERED_OFF")
        self._update_tcp_pose_event()

    def _on_servo_on(self, _msg: Empty) -> None:
        if not self._power:
            self.set_error("SERVO_ON verweigert – Power OFF.")
            return
        self.set_error("")
        self.set_servo_enabled(True)
        self.set_mode("IDLE")
        self._update_tcp_pose_event()

    def _on_servo_off(self, _msg: Empty) -> None:
        self.set_servo_enabled(False)
        self.set_moving(False)
        self.set_mode("POWERED_ON" if self._power else "POWERED_OFF")
        self._update_tcp_pose_event()

    # ----------------------------------------------------------
    # TF -> tcp_pose (event-driven)
    # ----------------------------------------------------------

    def _update_tcp_pose_event(self) -> None:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.tool_frame,
                Time(),  # latest
                timeout=Duration(seconds=0.05),
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as ex:
            now = self.get_clock().now()
            if (not self._tf_warned) and ((now - self._startup_time) > self._tf_grace_duration):
                self.get_logger().warning(
                    f"⚠️ Kein TF {self.world_frame} -> {self.tool_frame} verfügbar ({ex})"
                )
                self._tf_warned = True
            return

        if self._tf_warned:
            self.get_logger().info(f"✅ TF {self.world_frame} -> {self.tool_frame} jetzt verfügbar.")
            self._tf_warned = False

        pose = PoseStamped()
        pose.header.frame_id = self.world_frame
        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.position.z = tf.transform.translation.z
        pose.pose.orientation = tf.transform.rotation

        self.publish_tcp_pose(pose)


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = SimRobot()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
