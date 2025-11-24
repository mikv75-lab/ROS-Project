# -*- coding: utf-8 -*-
# File: tabs/process/process_run_statemachine.py
from __future__ import annotations

from typing import Optional, List, Dict, Any
import logging
import math
import yaml

from PyQt6 import QtCore
from PyQt6.QtStateMachine import QStateMachine, QState, QFinalState

from control_msgs.msg import JointJog
from geometry_msgs.msg import PoseStamped

_LOG = logging.getLogger("app.tabs.process.run_statemachine")


class QStatemachine(QStateMachine):
    """Nur für lesbareren Typnamen."""
    def __init__(self, parent: Optional[QtCore.QObject] = None) -> None:
        super().__init__(parent)


class _QtSignalHandler(logging.Handler):
    """
    Logging → Qt-Signal-Weiterleitung (für ProcessTab-Log).
    """
    def __init__(self, owner: "ProcessRunStatemachine") -> None:
        super().__init__()
        self._owner = owner

    def emit(self, record: logging.LogRecord) -> None:
        try:
            msg = self.format(record)
        except Exception:
            return
        try:
            self._owner.logMessage.emit(msg)
        except Exception:
            # beim Shutdown ggf. schon weg – ignorieren
            pass


class ProcessRunStatemachine(QtCore.QObject):
    """
    StateMachine-Worker für das Abspielen eines Run-YAML über Servo (JointJog).

    Erwartet:
      - Eine Run-Datei im YAML-Format
      - Eine Servo-Bridge im UI-Bridge-Objekt (bridge._servo),
        die einen Publisher 'pub_joint' (JointJog) bereitstellt.

    States (ähnlich wie Setup-Statemachine):
      - MOVE_PREDISPENSE   -> spielt Segment "MOVE_PREDISPENSE"
      - MOVE_RECIPE        -> spielt Segment "MOVE_RECIPE"
      - MOVE_RETREAT       -> spielt Segment "MOVE_RETREAT"
      - MOVE_HOME          -> spielt Segment "MOVE_HOME"
      - FINISHED           -> Ende
      - ERROR              -> Fehlerpfad

    Signale (kompatibel zu ProcessSetupStatemachine):
      - notifyFinished(object)   # { "poses": [], "planned_traj": [], "executed_traj": [] }
      - notifyError(str)
      - stateChanged(str)
      - logMessage(str)
    """

    # Ergebnis / Fehler
    notifyFinished = QtCore.pyqtSignal(object)
    notifyError = QtCore.pyqtSignal(str)

    # UI-Hilfssignale
    stateChanged = QtCore.pyqtSignal(str)
    logMessage = QtCore.pyqtSignal(str)

    # interne Transition-Signale
    _sig_predispense_done = QtCore.pyqtSignal()
    _sig_recipe_done = QtCore.pyqtSignal()
    _sig_retreat_done = QtCore.pyqtSignal()
    _sig_home_done = QtCore.pyqtSignal()
    _sig_error = QtCore.pyqtSignal()

    def __init__(
        self,
        *,
        run_yaml_path: str,
        bridge,
        parent: Optional[QtCore.QObject] = None,
    ) -> None:
        super().__init__(parent)

        self._run_yaml_path = run_yaml_path
        self._bridge = bridge

        self._stop_requested: bool = False
        self._error_msg: Optional[str] = None
        self._machine: Optional[QStatemachine] = None

        # Servo-Bridge (UI-Variante)
        self._servo = getattr(bridge, "_servo", None)
        if self._servo is None or not hasattr(self._servo, "pub_joint"):
            _LOG.error("ProcessRunStatemachine: ServoBridge (_servo) fehlt oder hat kein pub_joint.")
        else:
            _LOG.info("ProcessRunStatemachine: ServoBridge gefunden (%s).", type(self._servo).__name__)

        # TCP-Logging via RobotBridge ist optional (für später)
        self._rb = getattr(bridge, "_rb", None)
        self._rb_signals = getattr(self._rb, "signals", None) if self._rb is not None else None

        self._executed_poses: List[PoseStamped] = []

        # Run-Daten
        self._joints: List[str] = []
        self._segments: Dict[str, List[Dict[str, Any]]] = {}

        # Logging → Qt
        self._log_handler: Optional[_QtSignalHandler] = _QtSignalHandler(self)
        self._log_handler.setLevel(logging.INFO)
        formatter = logging.Formatter("%(message)s")
        self._log_handler.setFormatter(formatter)
        _LOG.addHandler(self._log_handler)

        _LOG.info(
            "ProcessRunStatemachine init: run_yaml=%s, rb=%s, servo=%s",
            run_yaml_path,
            type(self._rb).__name__ if self._rb is not None else "None",
            type(self._servo).__name__ if self._servo is not None else "None",
        )

    # ------------------------------------------------------------------ #
    # Öffentliche API (vom ProcessThread aufzurufen)
    # ------------------------------------------------------------------ #

    @QtCore.pyqtSlot()
    def start(self) -> None:
        """
        Startet das Abspielen des Run-YAML im Worker-Thread.
        """
        _LOG.info("ProcessRunStatemachine.start: gestartet.")

        self._stop_requested = False
        self._error_msg = None
        self._executed_poses.clear()

        # Run-YAML laden
        if not self._load_run_yaml(self._run_yaml_path):
            msg = self._error_msg or f"Run-YAML '{self._run_yaml_path}' konnte nicht geladen werden."
            _LOG.error("ProcessRunStatemachine.start: %s", msg)
            self.notifyError.emit(msg)
            return

        # StateMachine aufbauen
        machine = QStatemachine(self)
        self._machine = machine

        s_move_predisp = QState(machine)
        s_move_recipe = QState(machine)
        s_move_retreat = QState(machine)
        s_move_home = QState(machine)
        s_finished = QFinalState(machine)
        s_error = QFinalState(machine)

        machine.setInitialState(s_move_predisp)

        # Transitions
        s_move_predisp.addTransition(self._sig_predispense_done, s_move_recipe)
        s_move_recipe.addTransition(self._sig_recipe_done, s_move_retreat)
        s_move_retreat.addTransition(self._sig_retreat_done, s_move_home)
        s_move_home.addTransition(self._sig_home_done, s_finished)

        for st in (s_move_predisp, s_move_recipe, s_move_retreat, s_move_home):
            st.addTransition(self._sig_error, s_error)

        # Callbacks
        s_move_predisp.entered.connect(self._on_state_move_predispense)
        s_move_recipe.entered.connect(self._on_state_move_recipe)
        s_move_retreat.entered.connect(self._on_state_move_retreat)
        s_move_home.entered.connect(self._on_state_move_home)
        s_error.entered.connect(self._on_state_error)
        s_finished.entered.connect(self._on_state_finished)

        # UI-State-Namen
        s_move_predisp.entered.connect(lambda: self._emit_state("MOVE_PREDISPENSE"))
        s_move_recipe.entered.connect(lambda: self._emit_state("MOVE_RECIPE"))
        s_move_retreat.entered.connect(lambda: self._emit_state("MOVE_RETREAT"))
        s_move_home.entered.connect(lambda: self._emit_state("MOVE_HOME"))

        try:
            machine.start()
            _LOG.info("ProcessRunStatemachine: StateMachine gestartet.")
        except Exception as e:
            _LOG.exception("ProcessRunStatemachine.start: Exception: %s", e)
            self._signal_error(str(e) or "Unbekannter Fehler beim Start der Servo-StateMachine")

    @QtCore.pyqtSlot()
    def request_stop(self) -> None:
        """
        Von außen aufgerufen (z. B. Stop-Button).
        """
        _LOG.info("ProcessRunStatemachine: request_stop()")
        self._stop_requested = True

    # ------------------------------------------------------------------ #
    # Run-YAML laden
    # ------------------------------------------------------------------ #

    def _load_run_yaml(self, path: str) -> bool:
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
        except Exception as e:
            self._error_msg = f"Run-YAML konnte nicht geladen werden: {e}"
            return False

        joints = data.get("joints") or []
        if not isinstance(joints, list) or not joints:
            self._error_msg = "Run-YAML enthält keine gültige 'joints'-Liste."
            return False

        segments_raw = data.get("segments") or []
        if not isinstance(segments_raw, list) or not segments_raw:
            self._error_msg = "Run-YAML enthält keine 'segments'-Liste."
            return False

        seg_map: Dict[str, List[Dict[str, Any]]] = {
            "MOVE_PREDISPENSE": [],
            "MOVE_RECIPE": [],
            "MOVE_RETREAT": [],
            "MOVE_HOME": [],
        }

        for seg in segments_raw:
            name = (seg.get("name") or "").strip().upper()
            pts = seg.get("points") or []
            if not name or not isinstance(pts, list) or not pts:
                continue
            if name in seg_map:
                seg_map[name] = pts

        self._joints = list(joints)
        self._segments = seg_map

        _LOG.info(
            "ProcessRunStatemachine: Run-YAML geladen: joints=%d, segments={%s}",
            len(self._joints),
            ", ".join(f"{k}:{len(v)}" for k, v in self._segments.items()),
        )
        return True

    # ------------------------------------------------------------------ #
    # Hilfsfunktionen
    # ------------------------------------------------------------------ #

    def _should_stop(self) -> bool:
        return self._stop_requested

    def _emit_state(self, name: str) -> None:
        _LOG.info("ProcessRunStatemachine: State=%s", name)
        try:
            self.stateChanged.emit(name)
        except Exception:
            pass

    def _signal_error(self, msg: str) -> None:
        if not msg:
            msg = "Unbekannter Servo-Run-Fehler."
        if not self._error_msg:
            self._error_msg = msg
        _LOG.error("ProcessRunStatemachine: _signal_error: %s", msg)
        QtCore.QTimer.singleShot(0, self._sig_error.emit)

    def _finish_state(self, success_sig: QtCore.pyqtSignal) -> None:
        """
        Gemeinsames Ende eines States:
          - Wenn Stop oder Fehler -> ERROR
          - sonst -> success_sig
        """
        if self._should_stop() and not self._error_msg:
            self._error_msg = "Prozess durch Benutzer gestoppt."
        if self._error_msg:
            self._signal_error(self._error_msg)
        else:
            QtCore.QTimer.singleShot(0, success_sig.emit)

    # ---------------- Servo-Playback einer Joint-Trajektorie --------------

    def _play_segment_joint_traj(self, seg_name: str) -> None:
        """
        Spielt ein Segment aus dem Run-YAML über JointJog ab.

        Hinweis:
          - sehr einfache Umsetzung:
              * für jedes Punkt-Paar (i, i+1):
                  dt = t[i+1] - t[i]
                  v = (q[i+1] - q[i]) / dt
                  -> JointJog mit velocities=v, duration=dt
                  -> QThread.msleep(dt*1000)
          - keine Rückkopplung, keine Kollisionsprüfung – nur Sim/Fake-System!
        """
        if self._servo is None or not hasattr(self._servo, "pub_joint"):
            _LOG.error("ProcessRunStatemachine: _play_segment_joint_traj: ServoBridge/Pub fehlt.")
            if not self._error_msg:
                self._error_msg = "ServoBridge nicht verfügbar."
            return

        pts = self._segments.get(seg_name, [])
        if not pts:
            _LOG.info("ProcessRunStatemachine: Segment '%s' leer – überspringe.", seg_name)
            return

        pub = self._servo.pub_joint  # type: ignore[attr-defined]
        n_j = len(self._joints)

        _LOG.info(
            "ProcessRunStatemachine: spiele Segment '%s' mit %d Punkten.",
            seg_name, len(pts),
        )

        for i in range(len(pts) - 1):
            if self._should_stop():
                _LOG.info("ProcessRunStatemachine: Stop während Segment '%s' erkannt.", seg_name)
                return
            if self._error_msg:
                _LOG.error(
                    "ProcessRunStatemachine: Fehler während Segment '%s': %s",
                    seg_name, self._error_msg,
                )
                return

            p0 = pts[i]
            p1 = pts[i + 1]

            try:
                t0 = float(p0.get("t", 0.0))
                t1 = float(p1.get("t", 0.0))
            except Exception:
                _LOG.warning("ProcessRunStatemachine: Punkt %d/%d hat ungültige Zeitstempel – überspringe.", i, len(pts) - 1)
                continue

            dt = max(0.001, t1 - t0)
            pos0 = p0.get("positions") or []
            pos1 = p1.get("positions") or []

            if not isinstance(pos0, list) or not isinstance(pos1, list):
                _LOG.warning("ProcessRunStatemachine: Punkt %d/%d ohne positions-Liste – überspringe.", i, len(pts) - 1)
                continue
            if len(pos0) != n_j or len(pos1) != n_j:
                _LOG.warning("ProcessRunStatemachine: Punkt %d/%d hat falsche Dimensionszahl – überspringe.", i, len(pts) - 1)
                continue

            vel = [(pos1[j] - pos0[j]) / dt for j in range(n_j)]

            msg = JointJog()
            msg.header.frame_id = "world"  # oder frames.yaml tcp/world
            # Zeitstempel aus rclpy-Node (ServoBridge ist ein BaseBridge/Node)
            try:
                now = self._servo.get_clock().now().to_msg()  # type: ignore[attr-defined]
                msg.header.stamp = now
            except Exception:
                pass

            msg.joint_names = list(self._joints)
            msg.velocities = vel
            msg.displacements = []  # bei command_in_type=speed_units unbenutzt
            msg.duration = dt

            try:
                pub.publish(msg)
            except Exception as e:
                _LOG.error(
                    "ProcessRunStatemachine: JointJog publish in Segment '%s' fehlgeschlagen: %s",
                    seg_name, e,
                )
                if not self._error_msg:
                    self._error_msg = f"Servo publish failed: {e}"
                return

            # einfache Wartezeit – QThread.msleep blockiert nur den Worker-Thread
            ms = int(math.ceil(dt * 1000.0))
            if ms > 0:
                QtCore.QThread.msleep(ms)

        # Am Ende optional einen „Stopp“-Jog (vel=0) senden:
        try:
            stop_msg = JointJog()
            stop_msg.header.frame_id = "world"
            stop_msg.joint_names = list(self._joints)
            stop_msg.velocities = [0.0] * n_j
            stop_msg.displacements = []
            stop_msg.duration = 0.05
            pub.publish(stop_msg)
        except Exception:
            pass

        _LOG.info("ProcessRunStatemachine: Segment '%s' fertig abgespielt.", seg_name)

    # ------------------------------------------------------------------ #
    # State-Handler
    # ------------------------------------------------------------------ #

    def _on_state_move_predispense(self) -> None:
        _LOG.info("ProcessRunStatemachine: ENTER MOVE_PREDISPENSE")
        if not self._error_msg and not self._should_stop():
            self._play_segment_joint_traj("MOVE_PREDISPENSE")
        _LOG.info("ProcessRunStatemachine: LEAVE MOVE_PREDISPENSE")
        self._finish_state(self._sig_predispense_done)

    def _on_state_move_recipe(self) -> None:
        _LOG.info("ProcessRunStatemachine: ENTER MOVE_RECIPE")
        if not self._error_msg and not self._should_stop():
            self._play_segment_joint_traj("MOVE_RECIPE")
        _LOG.info("ProcessRunStatemachine: LEAVE MOVE_RECIPE")
        self._finish_state(self._sig_recipe_done)

    def _on_state_move_retreat(self) -> None:
        _LOG.info("ProcessRunStatemachine: ENTER MOVE_RETREAT")
        if not self._error_msg and not self._should_stop():
            self._play_segment_joint_traj("MOVE_RETREAT")
        _LOG.info("ProcessRunStatemachine: LEAVE MOVE_RETREAT")
        self._finish_state(self._sig_retreat_done)

    def _on_state_move_home(self) -> None:
        _LOG.info("ProcessRunStatemachine: ENTER MOVE_HOME")
        if not self._error_msg and not self._should_stop():
            self._play_segment_joint_traj("MOVE_HOME")
        _LOG.info("ProcessRunStatemachine: LEAVE MOVE_HOME")
        self._finish_state(self._sig_home_done)

    def _on_state_error(self) -> None:
        _LOG.info("ProcessRunStatemachine: ENTER ERROR")
        msg = self._error_msg or "Unbekannter Fehler im Servo-Run."
        _LOG.error("ProcessRunStatemachine: notifyError(%s)", msg)
        self.notifyError.emit(msg)
        self._cleanup()

    def _on_state_finished(self) -> None:
        _LOG.info("ProcessRunStatemachine: ENTER FINISHED")

        result: Dict[str, Any] = {
            "poses": list(self._executed_poses),  # aktuell leer; später über tcpPoseChanged füllen
            "planned_traj": [],      # Servo hat keine MoveIt-Trajs
            "executed_traj": [],
        }
        self.notifyFinished.emit(result)
        self._cleanup()

    # ------------------------------------------------------------------ #
    # Cleanup
    # ------------------------------------------------------------------ #

    def _cleanup(self) -> None:
        m = self._machine
        if m is not None and m.isRunning():
            try:
                _LOG.info("ProcessRunStatemachine: stoppe StateMachine.")
                m.stop()
            except Exception:
                _LOG.exception("ProcessRunStatemachine: Fehler beim Stoppen der StateMachine.")
        self._machine = None

        if self._log_handler is not None:
            try:
                _LOG.removeHandler(self._log_handler)
            except Exception:
                pass
            self._log_handler = None