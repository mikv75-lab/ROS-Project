# -*- coding: utf-8 -*-
# File: src/app/tabs/process/robot_init_statemachine.py
from __future__ import annotations

import math
import logging
from typing import Optional, Callable

from PyQt6 import QtCore
from PyQt6.QtCore import QTimer

from geometry_msgs.msg import PoseStamped

_LOG = logging.getLogger("tabs.process.robot_init_sm")


class RobotInitStatemachine(QtCore.QObject):
    """
    Robot Init + Home (Run-once Worker)

    Ziel:
      - deterministisch: init -> home, mit sauberer Diagnose
      - nutzt PoseBridge (poses_state.home) und MoveItBridge (moveit_last_result/plan/execute)

    Wichtige Fixes:
      - wartet auf MoveIt ready (sonst gehen Plan/Execute gerne ins Leere)
      - sendet HOME explizit über MoveIt-Bridge (plan_named 'home' oder plan_pose(home_pose)+execute)
      - WAIT_HOME loggt Distanz (mm) periodisch, damit du sofort siehst ob TCP "stuck" ist
      - ACK: snapshot last_result vor send; nur Änderung zählt als ACK
    """

    notifyFinished = QtCore.pyqtSignal()
    notifyError = QtCore.pyqtSignal(str)

    stateChanged = QtCore.pyqtSignal(str)
    logMessage = QtCore.pyqtSignal(str)

    S_IDLE = "IDLE"
    S_WAIT_HOME_AVAILABLE = "WAIT_HOME_AVAILABLE"
    S_WAIT_MOVEIT_READY = "WAIT_MOVEIT_READY"
    S_INIT_REQUESTED = "INIT_REQUESTED"
    S_WAIT_INITIALIZED = "WAIT_INITIALIZED"
    S_HOME_REQUESTED = "HOME_REQUESTED"
    S_WAIT_MOVEIT_ACK = "WAIT_MOVEIT_ACK"
    S_WAIT_HOME = "WAIT_HOME"
    S_FINISHED = "FINISHED"
    S_ERROR = "ERROR"
    S_STOPPED = "STOPPED"

    def __init__(
        self,
        *,
        ros,
        init_timeout_s: float = 10.0,
        home_timeout_s: float = 60.0,
        home_pose_timeout_s: float = 2.0,
        moveit_ready_timeout_s: float = 10.0,
        moveit_ack_timeout_s: float = 5.0,
        pos_tol_mm: float = 1.0,
        parent: Optional[QtCore.QObject] = None,
    ) -> None:
        super().__init__(parent)
        self._ros = ros

        self._init_timeout_s = float(init_timeout_s)
        self._home_timeout_s = float(home_timeout_s)
        self._home_pose_timeout_s = float(home_pose_timeout_s)
        self._moveit_ready_timeout_s = float(moveit_ready_timeout_s)
        self._moveit_ack_timeout_s = float(moveit_ack_timeout_s)
        self._pos_tol_mm = float(pos_tol_mm)

        self._stop_requested: bool = False
        self._done: bool = False

        self._state: str = self.S_IDLE
        self._deadline_ms: int = 0

        self._init_sent_once = False
        self._home_sent_once = False
        self._home_resend_used = False

        self._last_result_snapshot: str = ""
        self._last_dist_log_ms: int = 0
        self._last_tcp_snapshot: Optional[PoseStamped] = None
        self._tcp_stuck_since_ms: int = 0

        self._timer = QTimer(self)
        self._timer.setInterval(50)
        self._timer.timeout.connect(self._tick)

    # ------------------------------------------------------------------
    # Public
    # ------------------------------------------------------------------

    @QtCore.pyqtSlot()
    def start(self) -> None:
        self._stop_requested = False
        self._done = False
        self._deadline_ms = 0

        self._init_sent_once = False
        self._home_sent_once = False
        self._home_resend_used = False

        self._last_result_snapshot = self._last_motion_result()
        self._last_dist_log_ms = 0
        self._last_tcp_snapshot = None
        self._tcp_stuck_since_ms = 0

        self._set_state(self.S_WAIT_HOME_AVAILABLE)
        self._set_deadline(self._home_pose_timeout_s)

        self._log(
            f"RobotInit: start (init_timeout={self._init_timeout_s:.1f}s, "
            f"home_timeout={self._home_timeout_s:.1f}s, home_pose_timeout={self._home_pose_timeout_s:.1f}s, "
            f"moveit_ready_timeout={self._moveit_ready_timeout_s:.1f}s, moveit_ack_timeout={self._moveit_ack_timeout_s:.1f}s, "
            f"tol={self._pos_tol_mm:.2f}mm)"
        )

        self._timer.start()
        self._tick()

    @QtCore.pyqtSlot()
    def request_stop(self) -> None:
        if self._done:
            return
        self._stop_requested = True
        self._log("RobotInit: stop requested -> stopping motion/robot (NOT ros.stop())")

        try:
            if hasattr(self._ros, "stop_all"):
                self._ros.stop_all()
            elif hasattr(self._ros, "moveit_stop"):
                self._ros.moveit_stop()
            elif hasattr(self._ros, "moveit") and hasattr(self._ros.moveit, "stop"):
                self._ros.moveit.stop()
        except Exception:
            _LOG.exception("RobotInit: moveit stop failed")

        try:
            if hasattr(self._ros, "robot_stop"):
                self._ros.robot_stop()
            elif hasattr(self._ros, "robot") and hasattr(self._ros.robot, "stop"):
                self._ros.robot.stop()
        except Exception:
            _LOG.exception("RobotInit: robot_stop failed")

    # ------------------------------------------------------------------
    # Time
    # ------------------------------------------------------------------

    def _now_ms(self) -> int:
        return int(QtCore.QDateTime.currentMSecsSinceEpoch())

    def _set_deadline(self, timeout_s: float) -> None:
        self._deadline_ms = self._now_ms() + max(0, int(timeout_s * 1000.0))

    def _deadline_passed(self) -> bool:
        return self._deadline_ms > 0 and self._now_ms() >= self._deadline_ms

    # ------------------------------------------------------------------
    # State / log / finish
    # ------------------------------------------------------------------

    def _set_state(self, s: str) -> None:
        if s != self._state:
            self._state = s
            self.stateChanged.emit(s)

    def _log(self, msg: str) -> None:
        if msg:
            self.logMessage.emit(msg)

    def _finish_ok(self) -> None:
        if self._done:
            return
        self._done = True
        self._timer.stop()
        self._set_state(self.S_FINISHED)
        self._log("RobotInit: finished OK")
        self.notifyFinished.emit()

    def _finish_err(self, msg: str) -> None:
        if self._done:
            return
        self._done = True
        self._timer.stop()
        self._set_state(self.S_ERROR)
        self._log(f"RobotInit: ERROR: {msg}")
        self.notifyError.emit(msg)

    def _finish_stopped(self) -> None:
        if self._done:
            return
        self._done = True
        self._timer.stop()
        self._set_state(self.S_STOPPED)
        self._log("RobotInit: stopped")
        self.notifyError.emit("Robot-Init abgebrochen.")

    # ------------------------------------------------------------------
    # Guards
    # ------------------------------------------------------------------

    def _bridge_alive(self) -> bool:
        try:
            for attr in ("is_running", "running", "_running"):
                v = getattr(self._ros, attr, None)
                if callable(v):
                    try:
                        return bool(v())
                    except Exception:
                        continue
                if isinstance(v, bool):
                    return bool(v)

            thr = getattr(self._ros, "_thread", None)
            if thr is not None and hasattr(thr, "is_alive"):
                try:
                    return bool(thr.is_alive())
                except Exception:
                    pass
            return True
        except Exception:
            return True

    def _moveit_ready(self) -> bool:
        """
        Best-effort readiness check.
        Unterstützt unterschiedliche RosBridge-Layouts:
          - ros.moveit.is_ready / ready
          - ros.moveitpy_bridge.is_ready / ready
          - ros.moveit_state.ready
        Wenn nichts vorhanden, nehmen wir "ready", um nicht hart zu failen.
        """
        candidates = []
        try:
            candidates.append(getattr(self._ros, "moveit", None))
            candidates.append(getattr(self._ros, "moveitpy_bridge", None))
            candidates.append(getattr(self._ros, "moveit_state", None))
        except Exception:
            candidates = []

        for obj in candidates:
            if obj is None:
                continue
            for attr in ("is_ready", "ready", "_ready"):
                v = getattr(obj, attr, None)
                if callable(v):
                    try:
                        return bool(v())
                    except Exception:
                        continue
                if isinstance(v, bool):
                    return bool(v)

        return True

    # ------------------------------------------------------------------
    # State reads
    # ------------------------------------------------------------------

    def _is_initialized(self) -> bool:
        try:
            rs = getattr(self._ros, "robot_state", None)
            if rs is None:
                return False

            st = getattr(rs, "st", rs)
            v = getattr(st, "initialized", None)
            if isinstance(v, bool):
                return bool(v)

            fn = getattr(rs, "initialized", None)
            if callable(fn):
                return bool(fn())
            return False
        except Exception:
            return False

    def _robot_is_moving(self) -> Optional[bool]:
        try:
            rs = getattr(self._ros, "robot_state", None)
            if rs is None:
                return None
            st = getattr(rs, "st", rs)
            v = getattr(st, "moving", None)
            if isinstance(v, bool):
                return bool(v)
            fn = getattr(rs, "moving", None)
            if callable(fn):
                return bool(fn())
            return None
        except Exception:
            return None

    def _get_tcp_pose(self) -> Optional[PoseStamped]:
        try:
            rs = getattr(self._ros, "robot_state", None)
            if rs is None:
                return None
            st = getattr(rs, "st", rs)
            p = getattr(st, "tcp_pose", None)
            if p is None:
                fn = getattr(rs, "tcp_pose", None)
                if callable(fn):
                    p = fn()
            return p if isinstance(p, PoseStamped) else None
        except Exception:
            return None

    def _get_home_pose(self) -> Optional[PoseStamped]:
        try:
            ps = getattr(self._ros, "poses_state", None)
            if ps is None:
                return None
            st = getattr(ps, "st", ps)
            p = getattr(st, "home", None)
            if p is None:
                fn = getattr(ps, "home", None)
                if callable(fn):
                    p = fn()
            return p if isinstance(p, PoseStamped) else None
        except Exception:
            return None

    # ------------------------------------------------------------------
    # Pose math
    # ------------------------------------------------------------------

    def _dist_mm(self, a: PoseStamped, b: PoseStamped) -> float:
        dx = float(a.pose.position.x) - float(b.pose.position.x)
        dy = float(a.pose.position.y) - float(b.pose.position.y)
        dz = float(a.pose.position.z) - float(b.pose.position.z)
        return math.sqrt(dx * dx + dy * dy + dz * dz) * 1000.0

    def _poses_close(self, a: PoseStamped, b: PoseStamped) -> bool:
        return self._dist_mm(a, b) <= self._pos_tol_mm

    def _is_at_home(self) -> bool:
        cur = self._get_tcp_pose()
        home = self._get_home_pose()
        if cur is None or home is None:
            return False

        f_cur = (cur.header.frame_id or "").strip()
        f_home = (home.header.frame_id or "").strip()
        if f_cur and f_home and f_cur != f_home:
            # Frame mismatch -> niemals "at home" behaupten
            return False

        return self._poses_close(cur, home)

    # ------------------------------------------------------------------
    # MoveIt result helpers
    # ------------------------------------------------------------------

    def _last_motion_result(self) -> str:
        try:
            if hasattr(self._ros, "moveit_last_result"):
                return (self._ros.moveit_last_result() or "").strip()
        except Exception:
            return ""
        # optional via ros.moveit
        try:
            mv = getattr(self._ros, "moveit", None)
            if mv is not None and hasattr(mv, "last_result"):
                return (mv.last_result() or "").strip()
        except Exception:
            pass
        return ""

    def _has_motion_error(self) -> bool:
        res = self._last_motion_result().upper()
        return bool(res) and (res.startswith("ERROR") or "EXECUTE_FAILED" in res)

    # ------------------------------------------------------------------
    # Actions
    # ------------------------------------------------------------------

    def _send_robot_init_once(self) -> None:
        if self._init_sent_once:
            return
        self._init_sent_once = True
        self._log("RobotInit: sending robot_init()")

        # prefer structured API if available
        if hasattr(self._ros, "robot") and hasattr(self._ros.robot, "init"):
            self._ros.robot.init()
        elif hasattr(self._ros, "robot_init"):
            self._ros.robot_init()
        else:
            raise RuntimeError("RosBridge: robot_init() nicht verfügbar")

    def _send_home_move(self, *, reason: str) -> None:
        """
        Home-Move explizit über MoveItBridge:
          - bevorzugt: plan_named('home') + execute(True)
          - fallback: plan_pose(home_pose) + execute(True)
          - fallback alt: ros.moveit_move_home()
        """
        home = self._get_home_pose()
        if home is None:
            raise RuntimeError("Home-Pose ist None (cannot send home move).")

        self._log(f"RobotInit: sending HOME via MoveIt ({reason})")
        # Snapshot vor dem Senden für ACK-Erkennung
        self._last_result_snapshot = self._last_motion_result()

        mv = getattr(self._ros, "moveit", None)
        if mv is not None:
            # Varianten: plan_named/plan_pose/execute
            if hasattr(mv, "plan_named") and hasattr(mv, "execute"):
                mv.plan_named("home")
                mv.execute(True)
                return
            if hasattr(mv, "plan_pose") and hasattr(mv, "execute"):
                mv.plan_pose(home)
                mv.execute(True)
                return

        # fallback: direkte ros-Methoden (alte API)
        if hasattr(self._ros, "moveit_plan_named") and hasattr(self._ros, "moveit_execute"):
            self._ros.moveit_plan_named("home")
            self._ros.moveit_execute(True)
            return
        if hasattr(self._ros, "moveit_plan_pose") and hasattr(self._ros, "moveit_execute"):
            self._ros.moveit_plan_pose(home)
            self._ros.moveit_execute(True)
            return
        if hasattr(self._ros, "moveit_move_home"):
            self._ros.moveit_move_home()
            return

        raise RuntimeError("RosBridge: keine MoveIt-API gefunden (plan/execute).")

    # ------------------------------------------------------------------
    # Tick
    # ------------------------------------------------------------------

    @QtCore.pyqtSlot()
    def _tick(self) -> None:
        if self._stop_requested:
            self._finish_stopped()
            return

        if not self._bridge_alive():
            self._finish_err("RosBridge läuft nicht (wurde gestoppt/zerstört).")
            return

        # 1) Home pose must be available
        if self._state == self.S_WAIT_HOME_AVAILABLE:
            if self._get_home_pose() is not None:
                self._log("RobotInit: home pose available")
                self._set_state(self.S_WAIT_MOVEIT_READY)
                self._set_deadline(self._moveit_ready_timeout_s)
            elif self._deadline_passed():
                self._finish_err(
                    f"Home-Pose nicht verfügbar nach {self._home_pose_timeout_s:.1f}s. "
                    "Poses-Node publisht keine home_pose (latched) oder falscher Namespace."
                )
            return

        # 2) Wait MoveIt ready (important!)
        if self._state == self.S_WAIT_MOVEIT_READY:
            if self._moveit_ready():
                self._log("RobotInit: moveit ready")
                self._set_state(self.S_IDLE)
                self._deadline_ms = 0
            elif self._deadline_passed():
                self._finish_err(
                    f"MoveIt nicht ready nach {self._moveit_ready_timeout_s:.1f}s. "
                    "MoveItPyMotionNode/Bridge nicht online oder falscher Namespace."
                )
            return

        # 3) Init
        if not self._is_initialized():
            if self._state == self.S_IDLE:
                self._set_state(self.S_INIT_REQUESTED)
                try:
                    self._send_robot_init_once()
                except Exception as e:
                    self._finish_err(f"robot_init() fehlgeschlagen: {e}")
                    return
                self._set_state(self.S_WAIT_INITIALIZED)
                self._set_deadline(self._init_timeout_s)
                return

            if self._state == self.S_WAIT_INITIALIZED:
                if self._is_initialized():
                    self._log("RobotInit: robot initialized")
                    self._set_state(self.S_IDLE)
                    self._deadline_ms = 0
                elif self._deadline_passed():
                    self._finish_err(f"Timeout nach {self._init_timeout_s:.1f}s beim Warten auf initialized.")
                return

            self._set_state(self.S_WAIT_INITIALIZED)
            return

        # 4) Home
        home = self._get_home_pose()
        if home is None:
            self._finish_err("Home-Pose ist None (Poses-State).")
            return

        if self._is_at_home():
            self._finish_ok()
            return

        if self._state == self.S_IDLE:
            try:
                self._send_home_move(reason="not at home")
            except Exception as e:
                self._finish_err(f"Home-Move fehlgeschlagen: {e}")
                return

            self._home_sent_once = True
            self._set_state(self.S_WAIT_MOVEIT_ACK)
            self._set_deadline(self._moveit_ack_timeout_s)
            return

        if self._state == self.S_WAIT_MOVEIT_ACK:
            cur_res = self._last_motion_result()
            moving = self._robot_is_moving()

            if cur_res and cur_res != self._last_result_snapshot:
                self._log(f"RobotInit: moveit result update: {cur_res}")
                self._set_state(self.S_WAIT_HOME)
                self._set_deadline(self._home_timeout_s)
                return

            if moving is True:
                self._log("RobotInit: robot is moving -> wait for home")
                self._set_state(self.S_WAIT_HOME)
                self._set_deadline(self._home_timeout_s)
                return

            if self._deadline_passed():
                if not self._home_resend_used:
                    self._home_resend_used = True
                    self._log("RobotInit: no MoveIt ACK -> resending HOME once")
                    try:
                        self._send_home_move(reason="resend (no ack)")
                    except Exception as e:
                        self._finish_err(f"Home resend fehlgeschlagen: {e}")
                        return
                    self._set_deadline(self._moveit_ack_timeout_s)
                    return

                self._finish_err(
                    "MoveIt hat nicht reagiert (kein motion_result Update) und Robot bewegt sich nicht. "
                    "Sehr wahrscheinlich Topic/Namespace-Mismatch (MoveItBridge <-> MoveItPyMotionNode) "
                    "oder MoveItPyMotionNode war nicht ready."
                )
            return

        if self._state == self.S_WAIT_HOME:
            # Fehler aus MoveIt?
            if self._has_motion_error():
                self._finish_err(f"Home-Execute Fehler: {self._last_motion_result() or 'EXECUTE_FAILED'}")
                return

            tcp = self._get_tcp_pose()
            if tcp is not None:
                # periodische Distanz-Logs (1 Hz)
                now = self._now_ms()
                if self._last_dist_log_ms == 0 or (now - self._last_dist_log_ms) >= 1000:
                    dist = self._dist_mm(tcp, home)
                    self._log(f"RobotInit: dist(TCP->HOME)={dist:.2f}mm (tol={self._pos_tol_mm:.2f}mm)")
                    self._last_dist_log_ms = now

                # TCP-stuck detection (wenn TCP sich praktisch nicht bewegt)
                if self._last_tcp_snapshot is None:
                    self._last_tcp_snapshot = tcp
                    self._tcp_stuck_since_ms = self._now_ms()
                else:
                    d_tcp = self._dist_mm(tcp, self._last_tcp_snapshot)
                    if d_tcp <= 0.2:  # 0.2mm Schwellwert
                        if self._tcp_stuck_since_ms == 0:
                            self._tcp_stuck_since_ms = self._now_ms()
                    else:
                        self._last_tcp_snapshot = tcp
                        self._tcp_stuck_since_ms = self._now_ms()

            if self._is_at_home():
                self._log("RobotInit: TCP is at home pose")
                self._finish_ok()
                return

            if self._deadline_passed():
                tcp_frame = (tcp.header.frame_id if tcp else "") or ""
                home_frame = (home.header.frame_id or "") or ""
                last_moveit = self._last_motion_result()
                self._finish_err(
                    f"Timeout nach {self._home_timeout_s:.1f}s: TCP ≉ Home-Pose. "
                    f"(tcp_frame='{tcp_frame}', home_frame='{home_frame}', last_moveit='{last_moveit}')"
                )
            return

        # converge
        self._set_state(self.S_WAIT_HOME)
