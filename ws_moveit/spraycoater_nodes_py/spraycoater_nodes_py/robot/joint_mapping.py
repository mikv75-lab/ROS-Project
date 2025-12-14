# -*- coding: utf-8 -*-
# spraycoater_nodes_py/utils/joint_mapping.py

from __future__ import annotations

from typing import List


def ros_to_omron_positions(
    ros_joint_names: List[str],
    ros_positions: List[float],
    joint_order_ros_names: List[str],
) -> List[float]:
    """
    Mappt Joint-Werte aus der ROS-Reihenfolge (z.B. JointTrajectory.joint_names)
    in die Omron-Reihenfolge (J1..J6).

    Parameter:
      ros_joint_names:
        Reihenfolge der Joints in der eingehenden ROS-Nachricht
        (z.B. JointTrajectory.joint_names).

      ros_positions:
        Positionswerte (rad) in GENAU derselben Reihenfolge wie ros_joint_names.

      joint_order_ros_names:
        Liste von ROS-Jointnamen in Omron-Reihenfolge.
        Beispiel:
          [
            "Adept_Viper_s650_Link1",  # J1
            "Adept_Viper_s650_Link2",  # J2
            "Adept_Viper_s650_Link3",  # J3
            "Adept_Viper_s650_Link4",  # J4
            "Adept_Viper_s650_Link5",  # J5
            "Adept_Viper_s650_Link6",  # J6
          ]

    Rückgabe:
      Liste von Positionswerten (rad) in Omron-Reihenfolge (J1..J6).
    """
    if len(ros_joint_names) != len(ros_positions):
        raise ValueError(
            f"ros_to_omron_positions: ros_joint_names ({len(ros_joint_names)}) "
            f"und ros_positions ({len(ros_positions)}) müssen gleich lang sein."
        )

    out: List[float] = []
    for name in joint_order_ros_names:
        try:
            idx = ros_joint_names.index(name)
        except ValueError as exc:
            raise ValueError(
                f"ros_to_omron_positions: ROS-Jointname '{name}' nicht in ros_joint_names vorhanden."
            ) from exc
        out.append(ros_positions[idx])
    return out


def omron_to_ros_positions(
    joint_order_ros_names: List[str],
    ros_joint_names: List[str],
    omron_positions: List[float],
) -> List[float]:
    """
    Mappt Joint-Werte aus Omron-Reihenfolge (J1..J6) zurück in eine beliebige
    ROS-Reihenfolge (z.B. sensor_msgs/JointState.name).

    Parameter:
      joint_order_ros_names:
        ROS-Jointnamen in Omron-Reihenfolge (wie oben beschrieben).

      ros_joint_names:
        Ziel-Reihenfolge, in der die Werte ausgegeben werden sollen
        (z.B. JointState.name / URDF-Reihenfolge).

      omron_positions:
        Joint-Werte (rad) in Omron-Reihenfolge (entspricht joint_order_ros_names).

    Rückgabe:
      Liste von Positionswerten (rad) in derselben Reihenfolge wie ros_joint_names.
    """
    if len(joint_order_ros_names) != len(omron_positions):
        raise ValueError(
            f"omron_to_ros_positions: joint_order_ros_names ({len(joint_order_ros_names)}) "
            f"und omron_positions ({len(omron_positions)}) müssen gleich lang sein."
        )

    positions_ros = [0.0] * len(ros_joint_names)

    for om_idx, name in enumerate(joint_order_ros_names):
        try:
            ros_idx = ros_joint_names.index(name)
        except ValueError as exc:
            raise ValueError(
                f"omron_to_ros_positions: ROS-Jointname '{name}' nicht in ros_joint_names vorhanden."
            ) from exc
        positions_ros[ros_idx] = omron_positions[om_idx]

    return positions_ros
