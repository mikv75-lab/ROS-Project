# -*- coding: utf-8 -*-
# spraycoater_nodes_py/utils/joint_mapping.py
from __future__ import annotations

from typing import Dict, List


def _index_map(names: List[str], label: str) -> Dict[str, int]:
    if not names:
        raise ValueError(f"{label} ist leer.")
    if len(set(names)) != len(names):
        # Duplikate sauber ausgeben
        seen = set()
        dups = []
        for n in names:
            if n in seen and n not in dups:
                dups.append(n)
            seen.add(n)
        raise ValueError(f"{label} enthält Duplikate: {dups}")
    return {n: i for i, n in enumerate(names)}


def ros_to_omron_positions(
    ros_joint_names: List[str],
    ros_positions: List[float],
    joint_order_ros_names: List[str],
) -> List[float]:
    """
    Mappt Joint-Werte aus der ROS-Reihenfolge (ros_joint_names/ros_positions)
    in die Zielreihenfolge joint_order_ros_names (typisch joint_1..joint_6).
    """
    if len(ros_joint_names) != len(ros_positions):
        raise ValueError(
            f"ros_to_omron_positions: len(ros_joint_names)={len(ros_joint_names)} "
            f"!= len(ros_positions)={len(ros_positions)}"
        )

    ros_idx = _index_map(ros_joint_names, "ros_joint_names")
    _index_map(joint_order_ros_names, "joint_order_ros_names")  # nur Duplikat-Check

    out: List[float] = [0.0] * len(joint_order_ros_names)
    missing = [n for n in joint_order_ros_names if n not in ros_idx]
    if missing:
        raise ValueError(
            f"ros_to_omron_positions: Joint(s) fehlen in ros_joint_names: {missing}"
        )

    for out_i, name in enumerate(joint_order_ros_names):
        out[out_i] = float(ros_positions[ros_idx[name]])

    return out


def omron_to_ros_positions(
    omron_positions: List[float],
    ros_joint_names: List[str],
    joint_order_ros_names: List[str],
) -> List[float]:
    """
    Mappt Joint-Werte aus joint_order_ros_names-Reihenfolge (omron_positions)
    in die ROS-Reihenfolge ros_joint_names.
    """
    if len(omron_positions) != len(joint_order_ros_names):
        raise ValueError(
            f"omron_to_ros_positions: len(omron_positions)={len(omron_positions)} "
            f"!= len(joint_order_ros_names)={len(joint_order_ros_names)}"
        )

    ros_idx = _index_map(ros_joint_names, "ros_joint_names")
    src_idx = _index_map(joint_order_ros_names, "joint_order_ros_names")

    # sicherstellen dass src joint names auch in ros_joint_names existieren
    missing = [n for n in src_idx.keys() if n not in ros_idx]
    if missing:
        raise ValueError(
            f"omron_to_ros_positions: Joint(s) fehlen in ros_joint_names: {missing}"
        )

    positions_ros: List[float] = [0.0] * len(ros_joint_names)
    for name, src_i in src_idx.items():
        positions_ros[ros_idx[name]] = float(omron_positions[src_i])

    return positions_ros
