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

    Args:
        ros_joint_names: Joint-Namen in der Reihenfolge der ros_positions.
        ros_positions:  Positionswerte (selbe Länge wie ros_joint_names).
        joint_order_ros_names: Zielreihenfolge als ROS-Namen, z.B.
            ["joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"]

    Returns:
        Liste der Positionen in Omron-Reihenfolge (entspricht joint_order_ros_names).

    Raises:
        ValueError: Bei Längenfehlern oder fehlenden Joint-Namen.
    """
    if len(ros_joint_names) != len(ros_positions):
        raise ValueError(
            f"ros_to_omron_positions: len(ros_joint_names)={len(ros_joint_names)} "
            f"!= len(ros_positions)={len(ros_positions)}"
        )

    if not joint_order_ros_names:
        raise ValueError("ros_to_omron_positions: joint_order_ros_names ist leer.")

    out: List[float] = [0.0] * len(joint_order_ros_names)

    for om_idx, name in enumerate(joint_order_ros_names):
        try:
            ros_idx = ros_joint_names.index(name)
        except ValueError as exc:
            raise ValueError(
                f"ros_to_omron_positions: ROS-Jointname '{name}' nicht in ros_joint_names vorhanden."
            ) from exc
        out[om_idx] = float(ros_positions[ros_idx])

    return out


def omron_to_ros_positions(
    omron_positions: List[float],
    ros_joint_names: List[str],
    joint_order_ros_names: List[str],
) -> List[float]:
    """
    Mappt Joint-Werte aus Omron-Reihenfolge (J1..J6) in eine ROS-Reihenfolge
    passend zu ros_joint_names.

    Args:
        omron_positions: Positionen in Omron-Reihenfolge (selbe Länge wie joint_order_ros_names).
        ros_joint_names: gewünschte ROS-Reihenfolge (Output-Order).
        joint_order_ros_names: Reihenfolge, die zu omron_positions gehört (typisch joint_1..joint_6).

    Returns:
        Positionen in der Reihenfolge von ros_joint_names.

    Raises:
        ValueError: Bei Längenfehlern oder fehlenden Joint-Namen.
    """
    if len(omron_positions) != len(joint_order_ros_names):
        raise ValueError(
            f"omron_to_ros_positions: len(omron_positions)={len(omron_positions)} "
            f"!= len(joint_order_ros_names)={len(joint_order_ros_names)}"
        )

    if len(set(ros_joint_names)) != len(ros_joint_names):
        raise ValueError("omron_to_ros_positions: ros_joint_names enthält Duplikate.")

    if len(set(joint_order_ros_names)) != len(joint_order_ros_names):
        raise ValueError("omron_to_ros_positions: joint_order_ros_names enthält Duplikate.")

    positions_ros: List[float] = [0.0] * len(ros_joint_names)

    for om_idx, name in enumerate(joint_order_ros_names):
        try:
            ros_idx = ros_joint_names.index(name)
        except ValueError as exc:
            raise ValueError(
                f"omron_to_ros_positions: ROS-Jointname '{name}' nicht in ros_joint_names vorhanden."
            ) from exc
        positions_ros[ros_idx] = float(omron_positions[om_idx])

    return positions_ros
