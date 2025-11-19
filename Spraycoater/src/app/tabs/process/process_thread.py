# -*- coding: utf-8 -*-
# File: tabs/process/process_thread.py
from __future__ import annotations

from typing import Optional, List

import logging

from PyQt6 import QtCore
from PyQt6.QtStateMachine import QStateMachine, QState, QFinalState

from geometry_msgs.msg import PoseStamped

from app.model.recipe.recipe import Recipe

_LOG = logging.getLogger("app.tabs.process.thread")


class _QtSignalHandler(logging.Handler):
    """
    Logging-Handler, der alle Log-Messages dieses Moduls über
    ProcessThread.logMessage nach außen schickt.
    """

    def __init__(self, owner: "ProcessThread") -> None:
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
            # wenn der Thread schon weg ist, einfach ignorieren
            pass


class ProcessThread(QtCore.QThread):
    """
    QThread mit eigener QStateMachine für einen Prozesslauf.

    States:

      - MOVE_PREDISPENSE    : erste/Predispense-Position anfahren (path[0])
      - WAIT_PREDISPENSE    : predispense-Zeit warten
      - MOVE_RECIPE         : alle Zwischenpunkte fahren (path[1:-1])
      - WAIT_POSTDISPENSE   : postdispense-Zeit warten
      - MOVE_RETREAT        : Retreat-Position anfahren (path[-1])
      - MOVE_HOME           : Roboter nach Home
      - FINISHED (QFinalState)
    """

    # Steuer-Signale von außen
    startSignal = QtCore.pyqtSignal()
    stopSignal = QtCore.pyqtSignal()

    # Ergebnis-Signale nach außen
    notifyFinished = QtCore.pyqtSignal()
    notifyError = QtCore.pyqtSignal(str)

    # State-Änderungen nach außen
    stateChanged = QtCore.pyqtSignal(str)

    # Vollständige Log-Messages dieses Threads / Moduls
    logMessage = QtCore.pyqtSignal(str)

    # interne Übergangs-Signale für die StateMachine
    _sig_predispense_move_done = QtCore.pyqtSignal()
    _sig_predispense_wait_done = QtCore.pyqtSignal()
    _sig_move_recipe_done = QtCore.pyqtSignal()
    _sig_postdispense_wait_done = QtCore.pyqtSignal()
    _sig_retreat_move_done = QtCore.pyqtSignal()
    _sig_move_home_done = QtCore.pyqtSignal()

    def __init__(
        self,
        *,
        recipe: Recipe,
        bridge,
        parent: Optional[QtCore.QObject] = None,
        move_timeout_s: float = 60.0,
    ):
        # WICHTIG: KEIN parent an QThread geben → verhindert
        # "wrapped C/C++ object of type ProcessTab has been deleted"
        super().__init__()

        self._recipe = recipe
        self._bridge = bridge
        self._stop_requested = False
        self._error_msg: str | None = None
        self._machine: QStateMachine | None = None
        self._loop: QtCore.QEventLoop | None = None  # EventLoop-Referenz für sauberen Abbruch

        # Logging → Qt-Signal
        self._log_handler: Optional[_QtSignalHandler] = _QtSignalHandler(self)
        self._log_handler.setLevel(logging.INFO)
        formatter = logging.Formatter("%(message)s")
        self._log_handler.setFormatter(formatter)
        _LOG.addHandler(self._log_handler)

        # RobotBridge (für Home etc.), optional
        self._rb = getattr(bridge, "_robot", None) or getattr(bridge, "robot", None)

        # MotionBridge (für Pfad → MoveIt), robust finden
        self._motion = (
            getattr(bridge, "_motion", None)
            or getattr(bridge, "motion", None)
            or getattr(bridge, "_mb", None)
            or getattr(bridge, "motion_bridge", None)
        )
        self._move_timeout_s = float(move_timeout_s)

        # motion_result-Tracking
        self._last_motion_result: Optional[str] = None
        try:
            sigs = getattr(self._motion, "signals", None) if self._motion is not None else None
            if sigs is not None and hasattr(sigs, "motionResultChanged"):
                # WICHTIG: DirectConnection, damit _on_motion_result auch ohne
                # Eventloop im QThread ausgeführt wird (im Sender-Thread).
                sigs.motionResultChanged.connect(
                    self._on_motion_result,
                    QtCore.Qt.ConnectionType.DirectConnection,
                )
                _LOG.info(
                    "ProcessThread: MotionBridge gefunden (%s), motionResultChanged als DirectConnection verbunden.",
                    type(self._motion).__name__,
                )
            else:
                _LOG.warning(
                    "ProcessThread: MotionBridge.signals.motionResultChanged NICHT verfügbar "
                    "(motion=%r, signals=%r).",
                    self._motion,
                    sigs,
                )
        except Exception:
            _LOG.exception("ProcessThread: Konnte motionResultChanged nicht verbinden.")

        # Signale von außen auf Slots verdrahten
        self.startSignal.connect(self._on_start_signal)
        self.stopSignal.connect(self.request_stop)

        _LOG.info(
            "ProcessThread init: recipe=%s, rb=%s, motion=%s, timeout=%.1f s",
            getattr(recipe, "id", None),
            type(self._rb).__name__ if self._rb is not None else "None",
            type(self._motion).__name__ if self._motion is not None else "None",
            self._move_timeout_s,
        )

    # ---------------------------------------------------------
    # Zugriff auf Rezept
    # ---------------------------------------------------------

    @property
    def recipe(self) -> Recipe:
        return self._recipe

    def set_recipe(self, recipe: Recipe) -> None:
        """
        Erlaubt das Aktualisieren des Rezeptes, wenn ein neues geladen wird.
        Nur nutzen, solange der Thread NICHT läuft.
        """
        if self.isRunning():
            _LOG.warning("ProcessThread.set_recipe ignoriert: Thread läuft noch.")
            return
        _LOG.info("ProcessThread.set_recipe: recipe=%s", getattr(recipe, "id", None))
        self._recipe = recipe

    # ---------------------------------------------------------
    # Steuerung von außen
    # ---------------------------------------------------------

    def _on_start_signal(self) -> None:
        """Slot für startSignal – startet den Thread, falls er nicht läuft."""
        if not self.isRunning():
            _LOG.info("ProcessThread: startSignal erhalten, Thread wird gestartet.")
            self.start()
        else:
            _LOG.warning("ProcessThread: startSignal ignoriert, Thread läuft bereits.")

    def request_stop(self) -> None:
        """Von außen aufgerufen, um einen Abbruch anzufordern."""
        _LOG.info("ProcessThread: request_stop() aufgerufen.")
        self._stop_requested = True

    def _should_stop(self) -> bool:
        return self._stop_requested

    def _abort_if_needed(self, msg: str | None = None) -> None:
        """
        Setzt optional eine Fehlermeldung und stoppt die StateMachine + EventLoop.
        Wird sowohl für Stop als auch Fehler benutzt.
        """
        if msg and not self._error_msg:
            self._error_msg = msg
            _LOG.error("ProcessThread: Abbruch mit Fehler: %s", msg)

        # optional: ERROR-State nach außen signalisieren
        if msg:
            try:
                self.stateChanged.emit("ERROR")
            except Exception:
                pass

        # StateMachine stoppen
        if self._machine is not None:
            try:
                _LOG.info("ProcessThread: StateMachine wird gestoppt.")
                self._machine.stop()
            except Exception:
                _LOG.exception("ProcessThread: Fehler beim Stoppen der StateMachine.")

        # EventLoop beenden, damit run() zurückkehrt
        loop = self._loop
        if loop is not None and loop.isRunning():
            try:
                _LOG.info("ProcessThread: EventLoop.quit() aufgrund Abbruch/Fehler.")
                loop.quit()
            except Exception:
                _LOG.exception("ProcessThread: Fehler beim Beenden des EventLoops.")

    # ---------------------------------------------------------
    # Motion-Result-Handling
    # ---------------------------------------------------------

    @QtCore.pyqtSlot(str)
    def _on_motion_result(self, text: str) -> None:
        # Wird durch DirectConnection im Sender-Thread (MotionBridge) aufgerufen
        self._last_motion_result = (text or "").strip()
        _LOG.info("ProcessThread: motion_result empfangen: %r", self._last_motion_result)

    def _has_motion_error(self) -> bool:
        res = (self._last_motion_result or "").strip().upper()
        if not res:
            return False
        return res.startswith("ERROR:")

    def _wait_for_motion(self, timeout_s: float) -> bool:
        """
        Wartet auf EXECUTED:OK oder ein anderes Motion-Result.
        Alles, was NICHT EXECUTED ist, wird im Prozesskontext als Fehler gewertet.
        """
        if self._motion is None:
            _LOG.warning("ProcessThread: _wait_for_motion ohne MotionBridge -> direkt OK.")
            return True

        if timeout_s <= 0.0:
            return True

        _LOG.info("ProcessThread: Warte auf Motion-Ergebnis (timeout=%.1f s)", timeout_s)
        self._last_motion_result = None

        elapsed_ms = 0
        step_ms = 50
        max_ms = int(timeout_s * 1000.0)

        while elapsed_ms < max_ms:
            if self._should_stop():
                _LOG.info("ProcessThread: _wait_for_motion abgebrochen (stop_requested).")
                self._abort_if_needed("Prozess durch Benutzer gestoppt.")
                return False

            res = (self._last_motion_result or "").strip()
            res_u = res.upper()
            if res:
                _LOG.info("ProcessThread: motion_result ausgewertet: %r", res)

                # Erfolg
                if res_u.startswith("EXECUTED:OK") or res_u.startswith("EXECUTED"):
                    return True

                # Fehler oder „unerwartet“
                if not self._error_msg:
                    self._error_msg = res or "UNERWARTETES_MOTION_RESULT"
                self._abort_if_needed(self._error_msg)
                return False

            self.msleep(step_ms)
            elapsed_ms += step_ms

        # Timeout
        if not self._error_msg:
            self._error_msg = (
                f"Motion timeout nach {timeout_s:.1f} s während MoveIt-Planung/Ausführung."
            )
        _LOG.error("ProcessThread: _wait_for_motion -> TIMEOUT: %s", self._error_msg)
        self._abort_if_needed(self._error_msg)
        return False

    # ---------------------------------------------------------
    # Helper: State-Namen emitten
    # ---------------------------------------------------------

    def _emit_state(self, name: str) -> None:
        """Emitiert den aktuellen State-Namen nach außen."""
        _LOG.info("ProcessThread: State gewechselt zu %s", name)
        try:
            self.stateChanged.emit(name)
        except Exception:
            pass

    # ---------------------------------------------------------
    # Pfad aus Recipe holen
    # ---------------------------------------------------------

    def _get_recipe_poses(self) -> List[PoseStamped]:
        """
        Holt die PoseStamped-Liste aus Recipe.paths_compiled.

        Wir nehmen IMMER die **erste Side im Dict** `paths_compiled["sides"]`
        und interpretieren deren `poses_quat` als kompletten Prozesspfad:

          - poses[0]   -> Predispense
          - poses[1:-1]-> Rezeptpfad
          - poses[-1]  -> Retreat
        """
        pc = getattr(self._recipe, "paths_compiled", {}) or {}
        sides = pc.get("sides") or {}

        if not isinstance(sides, dict) or not sides:
            _LOG.warning("ProcessThread: _get_recipe_poses: paths_compiled.sides ist leer.")
            return []

        # Immer die erste Side im Dict verwenden
        first_side, sdata = next(iter(sides.items()))
        sdata = sdata or {}
        poses_quat = sdata.get("poses_quat") or []

        if not poses_quat:
            _LOG.warning(
                "ProcessThread: _get_recipe_poses: erste Side '%s' hat keine poses_quat.",
                first_side,
            )
            return []

        frame_id = pc.get("frame") or "scene"

        out: List[PoseStamped] = []
        for i, p in enumerate(poses_quat):
            ps = PoseStamped()
            ps.header.frame_id = frame_id

            ps.pose.position.x = float(p.get("x", 0.0))
            ps.pose.position.y = float(p.get("y", 0.0))
            ps.pose.position.z = float(p.get("z", 0.0))

            ps.pose.orientation.x = float(p.get("qx", 0.0))
            ps.pose.orientation.y = float(p.get("qy", 0.0))
            ps.pose.orientation.z = float(p.get("qz", 0.0))
            ps.pose.orientation.w = float(p.get("qw", 1.0))

            out.append(ps)

        _LOG.info(
            "ProcessThread: _get_recipe_poses: benutze erste side='%s' mit %d Posen (frame=%s).",
            first_side, len(out), frame_id,
        )
        return out

    def _move_pose_with_motion(self, pose: PoseStamped, label: str = "") -> bool:
        """
        Eine Pose via MotionBridge fahren, mit Fehler-/Timeout-Handling.
        """
        if self._should_stop():
            _LOG.info("ProcessThread: _move_pose_with_motion(%s) abgebrochen (stop_requested).", label)
            self._abort_if_needed("Prozess durch Benutzer gestoppt.")
            return False

        if pose is None:
            _LOG.error("ProcessThread: _move_pose_with_motion(%s): pose ist None.", label)
            self._abort_if_needed("Interner Fehler: Zielpose ist None.")
            return False

        if self._motion is None:
            _LOG.warning(
                "ProcessThread: _move_pose_with_motion(%s): KEINE MotionBridge -> Dummy-Sleep.",
                label,
            )
            for _ in range(80):
                if self._should_stop():
                    _LOG.info("ProcessThread: Dummy-Sleep abgebrochen (stop_requested).")
                    self._abort_if_needed("Prozess durch Benutzer gestoppt.")
                    return False
                self.msleep(25)
            return True

        _LOG.info("ProcessThread: move_to_pose(%s) via MotionBridge wird aufgerufen.", label)
        try:
            self._motion.move_to_pose(pose)
        except Exception as e:
            _LOG.exception("ProcessThread: move_to_pose(%s) failed: %s", label, e)
            self._abort_if_needed(str(e))
            return False

        ok = self._wait_for_motion(self._move_timeout_s)
        if not ok:
            _LOG.error(
                "ProcessThread: _move_pose_with_motion(%s) fehlgeschlagen: %s",
                label,
                self._error_msg,
            )
            return False

        if self._has_motion_error():
            _LOG.error(
                "ProcessThread: _move_pose_with_motion(%s) Motion-Error: %s",
                label,
                self._last_motion_result,
            )
            self._abort_if_needed(self._last_motion_result)
            return False

        _LOG.info("ProcessThread: _move_pose_with_motion(%s) erfolgreich.", label)
        return True

    # ---------------------------------------------------------
    # StateMachine-Logik
    # ---------------------------------------------------------

    def run(self) -> None:
        """
        Thread-Einstieg: baut die QStateMachine auf, startet sie
        und wartet im lokalen EventLoop bis FINISHED oder Abbruch.
        """
        _LOG.info("ProcessThread.run: gestartet.")
        self._stop_requested = False
        self._error_msg = None

        machine = QStateMachine()
        self._machine = machine  # für _abort_if_needed()

        # States
        s_move_predisp = QState()
        s_wait_predisp = QState()
        s_move_recipe = QState()
        s_wait_postdisp = QState()
        s_move_retreat = QState()
        s_move_home = QState()
        s_finished = QFinalState()

        # StateMachine strukturieren
        machine.addState(s_move_predisp)
        machine.addState(s_wait_predisp)
        machine.addState(s_move_recipe)
        machine.addState(s_wait_postdisp)
        machine.addState(s_move_retreat)
        machine.addState(s_move_home)
        machine.addState(s_finished)
        machine.setInitialState(s_move_predisp)

        # Übergänge
        s_move_predisp.addTransition(self._sig_predispense_move_done, s_wait_predisp)
        s_wait_predisp.addTransition(self._sig_predispense_wait_done, s_move_recipe)
        s_move_recipe.addTransition(self._sig_move_recipe_done, s_wait_postdisp)
        s_wait_postdisp.addTransition(self._sig_postdispense_wait_done, s_move_retreat)
        s_move_retreat.addTransition(self._sig_retreat_move_done, s_move_home)
        s_move_home.addTransition(self._sig_move_home_done, s_finished)

        # State-Callbacks
        s_move_predisp.entered.connect(self._on_state_move_predispense)
        s_wait_predisp.entered.connect(self._on_state_wait_predispense)
        s_move_recipe.entered.connect(self._on_state_move_recipe)
        s_wait_postdisp.entered.connect(self._on_state_wait_postdispense)
        s_move_retreat.entered.connect(self._on_state_move_retreat)
        s_move_home.entered.connect(self._on_state_move_home)

        # State-Namen nach außen signalisieren
        s_move_predisp.entered.connect(lambda: self._emit_state("MOVE_PREDISPENSE"))
        s_wait_predisp.entered.connect(lambda: self._emit_state("WAIT_PREDISPENSE"))
        s_move_recipe.entered.connect(lambda: self._emit_state("MOVE_RECIPE"))
        s_wait_postdisp.entered.connect(lambda: self._emit_state("WAIT_POSTDISPENSE"))
        s_move_retreat.entered.connect(lambda: self._emit_state("MOVE_RETREAT"))
        s_move_home.entered.connect(lambda: self._emit_state("MOVE_HOME"))

        # EventLoop für diese StateMachine in diesem Thread
        loop = QtCore.QEventLoop()
        self._loop = loop
        machine.finished.connect(loop.quit)

        try:
            machine.start()
            _LOG.info("ProcessThread.run: StateMachine gestartet, EventLoop läuft.")
            loop.exec()
        except Exception as e:
            _LOG.exception("ProcessThread.run: Exception: %s", e)
            self._error_msg = self._error_msg or str(e)
        finally:
            self._machine = None
            self._loop = None
            _LOG.info(
                "ProcessThread.run: beendet, _error_msg=%r, stop_requested=%s",
                self._error_msg,
                self._stop_requested,
            )
            # Logging-Handler wieder entfernen
            if self._log_handler is not None:
                try:
                    _LOG.removeHandler(self._log_handler)
                except Exception:
                    pass
                self._log_handler = None

        # Ergebnis auswerten
        if self._error_msg:
            _LOG.error("ProcessThread: notifyError(%s)", self._error_msg)
            self.notifyError.emit(self._error_msg)
        elif not self._stop_requested:
            _LOG.info("ProcessThread: notifyFinished() (kein Fehler, kein Stop).")
            self.notifyFinished.emit()
        else:
            _LOG.info("ProcessThread: beendet durch Stop, kein notifyFinished().")

    # ---------------------------------------------------------
    # State-Handler
    # ---------------------------------------------------------

    def _on_state_move_predispense(self) -> None:
        """
        State: Predispense-/erste Rezeptposition anfahren.
          → path[0]
        """
        _LOG.info("ProcessThread: ENTER MOVE_PREDISPENSE")
        if self._should_stop():
            self._abort_if_needed()
            return

        poses = self._get_recipe_poses()
        if not poses:
            _LOG.warning("ProcessThread: MOVE_PREDISPENSE: keine Posen -> direkt weiter.")
            self._sig_predispense_move_done.emit()
            return

        first = poses[0]
        _LOG.info("ProcessThread: MOVE_PREDISPENSE: fahre Pose[0].")

        if not self._move_pose_with_motion(first, label="predispense"):
            return

        if self._should_stop():
            self._abort_if_needed()
            return

        _LOG.info("ProcessThread: LEAVE MOVE_PREDISPENSE")
        self._sig_predispense_move_done.emit()

    def _on_state_wait_predispense(self) -> None:
        """
        State: predispense-time warten.
        """
        _LOG.info("ProcessThread: ENTER WAIT_PREDISPENSE")
        if self._should_stop():
            self._abort_if_needed()
            return

        pre_t = 0.0
        try:
            g = getattr(self._recipe, "globals", None)
            if g is not None:
                pre_t = float(getattr(g, "predispense_time", 0.0) or 0.0)
        except Exception:
            pre_t = 0.0

        _LOG.info("ProcessThread: WAIT_PREDISPENSE: predispense_time=%.3f s", pre_t)

        ms_total = max(0, int(pre_t * 1000))
        elapsed = 0
        step = 50  # ms

        while elapsed < ms_total:
            if self._should_stop():
                self._abort_if_needed()
                return
            self.msleep(step)
            elapsed += step

        if self._should_stop():
            self._abort_if_needed()
            return

        _LOG.info("ProcessThread: LEAVE WAIT_PREDISPENSE")
        self._sig_predispense_wait_done.emit()

    def _on_state_move_recipe(self) -> None:
        """
        State: kompletten Rezeptpfad fahren.
          → alle Zwischen-Pfade path[1:-1]
        """
        _LOG.info("ProcessThread: ENTER MOVE_RECIPE")
        if self._should_stop():
            self._abort_if_needed()
            return

        poses = self._get_recipe_poses()
        n = len(poses)
        _LOG.info("ProcessThread: MOVE_RECIPE: Anzahl Posen=%d", n)

        if n <= 2:
            _LOG.info("ProcessThread: MOVE_RECIPE: <=2 Posen, nichts zu fahren.")
            self._sig_move_recipe_done.emit()
            return

        for idx, pose in enumerate(poses[1:-1], start=1):
            if self._should_stop():
                self._abort_if_needed()
                return

            _LOG.info("ProcessThread: MOVE_RECIPE: fahre Pose[%d] von %d.", idx, n - 2)

            if not self._move_pose_with_motion(pose, label=f"recipe_{idx}"):
                return

        if self._should_stop():
            self._abort_if_needed()
            return

        _LOG.info("ProcessThread: LEAVE MOVE_RECIPE")
        self._sig_move_recipe_done.emit()

    def _on_state_wait_postdispense(self) -> None:
        """
        State: postdispense-time warten.
        """
        _LOG.info("ProcessThread: ENTER WAIT_POSTDISPENSE")
        if self._should_stop():
            self._abort_if_needed()
            return

        post_t = 0.0
        try:
            g = getattr(self._recipe, "globals", None)
            if g is not None:
                post_t = float(getattr(g, "postdispense_time", 0.0) or 0.0)
        except Exception:
            post_t = 0.0

        _LOG.info("ProcessThread: WAIT_POSTDISPENSE: postdispense_time=%.3f s", post_t)

        ms_total = max(0, int(post_t * 1000))
        elapsed = 0
        step = 50  # ms

        while elapsed < ms_total:
            if self._should_stop():
                self._abort_if_needed()
                return
            self.msleep(step)
            elapsed += step

        if self._should_stop():
            self._abort_if_needed()
            return

        _LOG.info("ProcessThread: LEAVE WAIT_POSTDISPENSE")
        self._sig_postdispense_wait_done.emit()

    def _on_state_move_retreat(self) -> None:
        """
        State: Retreat-Position anfahren.
          → path[-1], wenn vorhanden
        """
        _LOG.info("ProcessThread: ENTER MOVE_RETREAT")
        if self._should_stop():
            self._abort_if_needed()
            return

        poses = self._get_recipe_poses()
        n = len(poses)
        if n < 2:
            _LOG.info("ProcessThread: MOVE_RETREAT: <2 Posen, kein separates Retreat.")
            self._sig_retreat_move_done.emit()
            return

        last = poses[-1]
        _LOG.info("ProcessThread: MOVE_RETREAT: fahre Pose[-1].")

        if not self._move_pose_with_motion(last, label="retreat"):
            return

        if self._should_stop():
            self._abort_if_needed()
            return

        _LOG.info("ProcessThread: LEAVE MOVE_RETREAT")
        self._sig_retreat_move_done.emit()

    def _on_state_move_home(self) -> None:
        """
        State: Roboter nach Home fahren.
        Nutzt, falls vorhanden, move_home() oder go_home() der RobotBridge,
        sonst Dummy-Sleep.
        """
        _LOG.info("ProcessThread: ENTER MOVE_HOME")
        if self._should_stop():
            self._abort_if_needed()
            return

        try:
            if self._rb is not None:
                _LOG.info("ProcessThread: MOVE_HOME: benutze RobotBridge (%s).",
                          type(self._rb).__name__)
                if hasattr(self._rb, "move_home"):
                    self._rb.move_home()
                elif hasattr(self._rb, "go_home"):
                    self._rb.go_home()
                else:
                    _LOG.warning(
                        "ProcessThread: MOVE_HOME: RobotBridge hat kein move_home/go_home, Dummy-Sleep."
                    )
                    for _ in range(80):
                        if self._should_stop():
                            self._abort_if_needed()
                            return
                        self.msleep(25)
            else:
                _LOG.warning("ProcessThread: MOVE_HOME: keine RobotBridge, Dummy-Sleep.")
                for _ in range(80):
                    if self._should_stop():
                        self._abort_if_needed()
                        return
                    self.msleep(25)
        except Exception as e:
            _LOG.exception("ProcessThread: MOVE_HOME: Exception: %s", e)
            self._abort_if_needed(str(e))
            return

        if self._should_stop():
            self._abort_if_needed()
            return

        _LOG.info("ProcessThread: LEAVE MOVE_HOME")
        self._sig_move_home_done.emit()
