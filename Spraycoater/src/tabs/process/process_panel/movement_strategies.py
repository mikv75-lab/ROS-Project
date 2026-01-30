# -*- coding: utf-8 -*-
# File: src/tabs/process/process_panel/movement_strategies.py
from __future__ import annotations

import time
import copy
from abc import ABC, abstractmethod
from typing import List, Any, Optional

# Wir brauchen PoseArray, um die Daten korrekt für die Bridge zu verpacken
from geometry_msgs.msg import PoseArray, PoseStamped

# WICHTIG: Import passend zu deinem Dateinamen 'segment_runner.py'
from .segment_runner import SegmentRunner, StepSpec

class MoveStrategy(ABC):
    """
    Abstrakte Basisklasse für Bewegungsstrategien.
    Jede Strategie weiß selbst, wie sie eine Liste von Posen in Steps umwandelt.
    """
    def __init__(self, runner: SegmentRunner, run_id: str):
        self.runner = runner
        self.run_id = run_id

    @abstractmethod
    def create_steps(self, seg_name: str, poses: List[Any]) -> List[StepSpec]:
        """Wandelt eine Liste von PoseStamped in StepSpecs um."""
        pass

    def _new_id(self, i: int) -> int:
        return int(time.time() * 1000) + i


class PtpStrategy(MoveStrategy):
    """
    Klassisches Point-to-Point (PTP) in einer Schleife.
    Verhalten: Stop-and-Go bei jedem Punkt.
    Nutzt: plan_pose (Einzelpunkte)
    """
    def create_steps(self, seg_name: str, poses: List[Any]) -> List[StepSpec]:
        steps = []
        for i, pose in enumerate(poses):
            rid = self._new_id(i)
            
            # 1. Planen (Einzelner Punkt)
            steps.append(self.runner.make_plan_pose_step_bound(
                run=self.run_id, req_id=rid, seg=seg_name, pose=pose, label=f"{seg_name}:ptp:plan[{i}]"
            ))
            
            # 2. Ausführen
            steps.append(self.runner.make_execute_last_planned_step_bound(
                run=self.run_id, req_id=rid, seg=seg_name, label=f"{seg_name}:ptp:exec[{i}]"
            ))
        return steps


class CartesianStrategy(MoveStrategy):
    """
    Linearer Pfad (LIN/Cartesian) via MoveIt Cartesian Interpolator.
    Verhalten: Eine flüssige Bewegung durch alle Punkte (strikte Linie).
    Nutzt: plan_cartesian
    """
    def create_steps(self, seg_name: str, poses: List[Any]) -> List[StepSpec]:
        if not poses: return []
        rid = self._new_id(0)
        
        # Cartesian Path Interpolator akzeptiert meist direkt eine Liste von Posen
        steps = [self.runner.make_plan_cartesian_step_bound(
            run=self.run_id, req_id=rid, seg=seg_name, poses=poses, label=f"{seg_name}:cart:plan"
        )]
        
        steps.append(self.runner.make_execute_last_planned_step_bound(
            run=self.run_id, req_id=rid, seg=seg_name, label=f"{seg_name}:cart:exec"
        ))
        return steps


class PilzSequenceStrategy(MoveStrategy):
    """
    Pilz Sequence: Sendet alle Punkte als PoseArray.
    Der Pilz-Planer verbindet sie flüssig (PTP/LIN/CIRC Blending).
    
    INTELLIGENT (TCP UPDATE): 
    - Wenn < 2 Punkte: Hole aktuelle TCP-Position vom Roboter (via RobotBridge).
    - Baue Sequenz: [Current_Pos, Target_Pos].
    - Sende LIN Sequenz.
    """
    def create_steps(self, seg_name: str, poses: List[Any]) -> List[StepSpec]:
        if not poses: return []
        
        rid = self._new_id(0)
        final_poses = list(poses)

        # --- INTELLIGENTER STARTPUNKT ---
        # Wenn wir nur einen Zielpunkt haben (z.B. Predispense),
        # bauen wir uns eine Linie vom aktuellen Roboter-Standort zum Ziel.
        if len(final_poses) < 2:
            current_pose = self._try_get_current_pose()
            if current_pose:
                # Wir fügen die aktuelle Position VORNE an
                final_poses.insert(0, current_pose)
            else:
                # Fallback, falls wir die Position nicht lesen können -> Single Point PTP
                return self._fallback_single_ptp(seg_name, poses[0], rid)

        # --- NORMALFALL: SEQUENZ (jetzt >= 2 Punkte) ---
        pa = PoseArray()
        
        # Header vom ersten Punkt übernehmen (Frame ID ist wichtig!)
        if hasattr(final_poses[0], "header"):
            pa.header.frame_id = final_poses[0].header.frame_id
            pa.header.stamp = final_poses[0].header.stamp
        
        # Nur die .pose Anteile extrahieren
        pa.poses = [p.pose for p in final_poses]

        steps = [self.runner.make_plan_pose_array_step_bound(
            run=self.run_id, req_id=rid, seg=seg_name, poses=pa, label=f"{seg_name}:seq:plan"
        )]

        steps.append(self.runner.make_execute_last_planned_step_bound(
            run=self.run_id, req_id=rid, seg=seg_name, label=f"{seg_name}:seq:exec"
        ))
        return steps

    def _try_get_current_pose(self) -> Optional[PoseStamped]:
        """
        Versucht, die aktuelle TCP-Pose aus der RobotBridge zu lesen.
        Pfad: ros_bridge -> robot (RobotBridge) -> tcp_pose
        """
        try:
            ros = self.runner._ros
            # Zugriff auf die RobotBridge Instanz
            if hasattr(ros, "robot") and ros.robot is not None:
                # Zugriff auf das tcp_pose Attribut (wie in robot_bridge.py definiert)
                if hasattr(ros.robot, "tcp_pose") and ros.robot.tcp_pose is not None:
                    return copy.deepcopy(ros.robot.tcp_pose)
        except Exception:
            return None
        return None

    def _fallback_single_ptp(self, seg_name: str, pose: Any, rid: int) -> List[StepSpec]:
        """Alter Fallback für Einzelpunkte."""
        return [
            self.runner.make_plan_pose_step_bound(
                run=self.run_id, req_id=rid, seg=seg_name, pose=pose, label=f"{seg_name}:seq:single"
            ),
            self.runner.make_execute_last_planned_step_bound(
                run=self.run_id, req_id=rid, seg=seg_name, label=f"{seg_name}:seq:exec"
            )
        ]