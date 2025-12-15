#!/usr/bin/env python3
# -*- coding: utf-8 -*-
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

    - Kommandos: robot.init|stop|power_on|... (topics.yaml.robot.subscribe)
    - TCP-Pose aus TF (world → tcp)
    - Joints aus JointState-Topic (joint_state_broadcaster / ros2_control)

    topics.yaml muss enthalten:
      topics:
        robot:
          subscribe:
            - id: joint_states_in
              name: joint_states
              type: sensor_msgs/msg/JointState
              qos: sensor_data
    """

    def __init__(self) -> None:
        super().__init__(node_name="robot")

        # Frames
        self.frames = frames()
        self._F = self.frames.resolve

        self.world_frame = self._F("world")
        self.tool_frame = self._F("tcp")

        self._tcp_pose.header.frame_id = self.world_frame
        self._joints = JointState()

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._tf_warned = False
        self._startup_time = self.get_clock().now()
        self._tf_grace_duration = Duration(seconds=2.0)

        # Kommandos (aus topics.yaml)
        self.sub_init = self._make_sub(Empty, "init", self._on_init)
        self.sub_stop = self._make_sub(Empty, "stop", self._on_stop)
        self.sub_clear_error = self._make_sub(Empty, "clear_error", self._on_clear_error)
        self.sub_power_on = self._make_sub(Empty, "power_on", self._on_power_on)
        self.sub_power_off = self._make_sub(Empty, "power_off", self._on_power_off)
        self.sub_servo_on = self._make_sub(Empty, "servo_on", self._on_servo_on)
        self.sub_servo_off = self._make_sub(Empty, "servo_off", self._on_servo_off)

        # joint_states (aus topics.yaml via config_hub)
        self.sub_joint_states = self._make_sub(
            JointState, "joint_states_in", self._on_joint_states
        )

        js_topic = self.cfg.subscribe_topic("robot", "joint_states_in")
        self.get_logger().info(
            f"🤖 SimRobot gestartet: tcp_pose = TF({self.world_frame}->{self.tool_frame}), "
            f"JointStates subscribe='{js_topic}' (via config_hub)"
        )

    # ----------------------------------------------------------
    # Hook (BaseRobot)
    # ----------------------------------------------------------

    def on_tick(self, now: Time) -> None:
        # TF updaten (Pose), Joints kommen asynchron über JointState-Callback
        self._update_tcp_pose()

    # ----------------------------------------------------------
    # Subscriptions
    # ----------------------------------------------------------

    def _on_joint_states(self, msg: JointState):
        self._joints = msg

        # optional: wenn JointStates kommen, sind wir "connected"
        if not self._connected:
            self._connected = True

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
        self._set_mode("STOPPED" if self._power else "POWERED_OFF")

    def _on_clear_error(self, _msg: Empty):
        self._last_error = ""
        self.pub_errors.publish(String(data=""))

    def _on_power_on(self, _msg: Empty):
        self._power = True
        self._connected = True
        self._set_mode("IDLE" if self._initialized else "POWERED_ON")

    def _on_power_off(self, _msg: Empty):
        self._power = False
        self._servo_enabled = False
        self._moving = False
        self._set_mode("POWERED_OFF")

    def _on_servo_on(self, _msg: Empty):
        if not self._power:
            self._set_error("SERVO_ON verweigert – Power OFF.")
            return
        self._servo_enabled = True
        self._set_mode("IDLE")

    def _on_servo_off(self, _msg: Empty):
        self._servo_enabled = False
        self._moving = False
        self._set_mode("POWERED_ON" if self._power else "POWERED_OFF")

    # ----------------------------------------------------------
    # TF -> tcp_pose
    # ----------------------------------------------------------

    def _update_tcp_pose(self) -> None:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.tool_frame,
                Time(),  # latest
                timeout=Duration(seconds=0.2),
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

        # Pose füllen (Stamp setzt BaseRobot im Publish!)
        self._tcp_pose.header.frame_id = self.world_frame
        self._tcp_pose.pose.position.x = tf.transform.translation.x
        self._tcp_pose.pose.position.y = tf.transform.translation.y
        self._tcp_pose.pose.position.z = tf.transform.translation.z
        self._tcp_pose.pose.orientation = tf.transform.rotation


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
