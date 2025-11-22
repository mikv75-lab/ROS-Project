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
      - ERROR    (QFinalState)

    WICHTIG:
      - KEIN _wait_for_motion mehr.
      - Bewegungen werden eventbasiert über MotionBridge abgewickelt:
          * State emittiert moveToPoseRequested(...)
          * MotionBridge/MoveIt führen aus und publizieren motionResultChanged(...)
          * ProcessThread wertet das Ergebnis aus und triggert State-Transition.
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

    # internes Signal, um in den ERROR-FinalState zu wechseln
    _sig_error = QtCore.pyqtSignal()

    # Motion-Phasen
    PHASE_NONE = "NONE"
    PHASE_MOVE_PREDISPENSE = "MOVE_PREDISPENSE"
    PHASE_MOVE_RECIPE = "MOVE_RECIPE"
    PHASE_MOVE_RETREAT = "MOVE_RETREAT"
    PHASE_MOVE_HOME = "MOVE_HOME"

    def __init__(
        self,
        *,
        recipe: Recipe,
        bridge,
        parent: Optional[QtCore.QObject] = None,
    ):
        # WICHTIG: KEIN parent an QThread geben → verhindert
        # "wrapped C/C++ object of type ProcessTab has been deleted"
        super().__init__()

        self._recipe = recipe
        self._bridge = bridge
        self._stop_requested = False
        self._error_msg: Optional[str] = None
        self._machine: Optional[QStateMachine] = None

        # Merker, ob die StateMachine im ERROR- oder FINISHED-FinalState geendet hat
        self._ended_in_error_state: bool = False
        self._ended_in_finished_state: bool = False

        # Motion-Phasen-State
        self._current_phase: str = self.PHASE_NONE
        self._recipe_poses: List[PoseStamped] = []
        self._recipe_index: int = 0

        # --- Predispense-Safety-Timer ---
        self._predispense_timer: Optional[QtCore.QTimer] = None
        self._predispense_elapsed_ms: int = 0
        self._predispense_timeout_ms: int = 30000  # 30 s Timeout, falls kein Motion-Result kommt

        # Logging → Qt-Signal
        self._log_handler: Optional[_QtSignalHandler] = _QtSignalHandler(self)
        self._log_handler.setLevel(logging.INFO)
        formatter = logging.Formatter("%(message)s")
        self._log_handler.setFormatter(formatter)
        _LOG.addHandler(self._log_handler)

        # Feste Referenzen aus der UIBridge (ohne getattr/hasattr)
        self._rb = bridge._rb            # RobotBridge (falls du später noch was brauchst)
        self._motion = bridge._motion    # MotionBridge
        self._motion_signals = self._motion.signals if self._motion is not None else None
        self._pb = bridge._pb            # PosesBridge
        self._poses_state = bridge.poses # PosesState (Home/Service-Copy)

        # motion_result-Integration (eventbasiert, ohne Polling)
        if self._motion is not None and self._motion_signals is not None:
            try:
                # WICHTIG: keine explizite DirectConnection, damit Qt sauber
                # eine QueuedConnection zwischen Threads macht.
                self._motion_signals.motionResultChanged.connect(self._on_motion_result)
                _LOG.info(
                    "ProcessThread: MotionBridge gefunden (%s), "
                    "motionResultChanged verbunden.",
                    type(self._motion).__name__,
                )
            except Exception as e:
                _LOG.exception("ProcessThread: Konnte motionResultChanged nicht verbinden: %s", e)
        else:
            _LOG.error(
                "ProcessThread: MotionBridge oder deren Signale sind None – "
                "Bewegungen können nicht ausgeführt werden."
            )

        # Signale von außen auf Slots verdrahten
        self.startSignal.connect(self._on_start_signal)
        self.stopSignal.connect(self.request_stop)

        _LOG.info(
            "ProcessThread init: recipe=%s, rb=%s, motion=%s",
            getattr(recipe, "id", None),
            type(self._rb).__name__ if self._rb is not None else "None",
            type(self._motion).__name__ if self._motion is not None else "None",
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

    # ---------------------------------------------------------
    # Zentrale Transition-Hilfe
    # ---------------------------------------------------------

    def _post_transition(self, sig: QtCore.pyqtSignal) -> None:
        """
        Führt einen State-Übergang *asynchron* aus.

        WICHTIG:
          - Niemals .emit() direkt aus einem State-Handler oder Motion-Callback aufrufen.
          - Immer _post_transition(self._sig_...) verwenden.
        """
        if self._machine is None or not self._machine.isRunning():
            _LOG.warning("ProcessThread: _post_transition aufgerufen, aber StateMachine läuft nicht mehr.")
            return

        QtCore.QTimer.singleShot(0, sig.emit)

    # ---------------------------------------------------------
    # Helper: State abschließen
    # ---------------------------------------------------------

    def _finish_state(self, success_sig: QtCore.pyqtSignal) -> None:
        """
        Am ENDE eines *nicht-motionsbasierten* State-Handlers aufrufen
        (oder wenn in diesem State keine Motion mehr aussteht).

        Logik:
          - Wenn stop_requested und noch keine Fehlermeldung: generische Stop-Message setzen.
          - Wenn es eine _error_msg gibt → ERROR-Transition.
          - Sonst → Transition mit success_sig.
        """
        if self._should_stop() and not self._error_msg:
            self._error_msg = "Prozess durch Benutzer gestoppt."

        if self._error_msg:
            _LOG.error("ProcessThread: Fehler erkannt, wechsle in ERROR: %s", self._error_msg)
            self._post_transition(self._sig_error)
        else:
            self._post_transition(success_sig)

    def _signal_error(self, msg: Optional[str] = None) -> None:
        """
        Setzt (optional) eine Fehlermeldung und triggert den ERROR-State.
        Kann sowohl aus State-Handlern als auch aus Motion-Callbacks gerufen werden.
        """
        if msg and not self._error_msg:
            self._error_msg = msg
        if not self._error_msg:
            self._error_msg = "Unbekannter Prozessfehler."
        _LOG.error("ProcessThread: _signal_error: %s", self._error_msg)
        self._post_transition(self._sig_error)

    # ---------------------------------------------------------
    # Helper: Maschine + Thread-Loop beenden
    # ---------------------------------------------------------

    def _stop_machine_and_quit(self) -> None:
        """Stoppt die StateMachine (falls noch laufend) und beendet den Thread-Eventloop."""
        m = self._machine
        if m is not None and m.isRunning():
            try:
                _LOG.info("ProcessThread: stoppe StateMachine.")
                m.stop()
            except Exception:
                _LOG.exception("ProcessThread: Fehler beim Stoppen der StateMachine.")
        try:
            _LOG.info("ProcessThread: quit() aus _stop_machine_and_quit.")
            self.quit()
        except Exception:
            _LOG.exception("ProcessThread: Fehler bei quit().")

    # ---------------------------------------------------------
    # Motion-Result-Handling (eventbasiert)
    # ---------------------------------------------------------

    @QtCore.pyqtSlot(str)
    def _on_motion_result(self, text: str) -> None:
        """
        Wird von MotionBridge.signals.motionResultChanged aufgerufen.

        Kein Polling, keine While-Loops:
          - PLANNED:OK...      -> Info, ignorieren, auf EXECUTED/ERROR warten
          - EXECUTED:OK...     -> _handle_motion_success()
          - ERROR/FAILED/...   -> _signal_error(...)
        """
        res = (text or "").strip()
        if not res:
            return

        _LOG.info("ProcessThread: motion_result empfangen: %r", res)

        if self._machine is None or not self._machine.isRunning():
            _LOG.info("ProcessThread: motion_result ignoriert (StateMachine läuft nicht mehr).")
            return

        if self._current_phase == self.PHASE_NONE:
            _LOG.info("ProcessThread: motion_result ignoriert (keine aktive Motion-Phase).")
            return

        if self._should_stop():
            if not self._error_msg:
                self._error_msg = "Prozess durch Benutzer gestoppt."
            self._signal_error(self._error_msg)
            return

        res_u = res.upper()

        # 1) PLANNED:OK... -> nur Info
        if res_u.startswith("PLANNED"):
            _LOG.info("ProcessThread: Motion-Status PLANNED..., warte auf EXECUTED/ERROR.")
            return

        # 2) EXECUTED... -> Erfolg für aktuelle Phase
        if res_u.startswith("EXECUTED:OK") or res_u.startswith("EXECUTED"):
            self._handle_motion_success()
            return

        # 3) Klare Fehler-Präfixe
        if res_u.startswith(("ERROR", "FAILED", "ABORTED", "CANCELLED", "UNKNOWN")):
            if not self._error_msg:
                self._error_msg = res
            _LOG.error("ProcessThread: Motion-Fehler: %s", self._error_msg)
            self._signal_error(self._error_msg)
            return

        # 4) Alles andere ebenfalls als Fehler behandeln
        if not self._error_msg:
            self._error_msg = res
        _LOG.error(
            "ProcessThread: Unerwartetes Motion-Result, behandle als Fehler: %s",
            self._error_msg,
        )
        self._signal_error(self._error_msg)

    def _handle_motion_success(self) -> None:
        """
        Wird aufgerufen, wenn Motion EXECUTED:OK gemeldet hat.
        Entscheidet anhand der aktuellen Phase, was passiert:
          - MOVE_PREDISPENSE: State-Transition auf WAIT_PREDISPENSE
          - MOVE_RECIPE: nächster Waypoint oder Transition auf WAIT_POSTDISPENSE
          - MOVE_RETREAT: Transition auf MOVE_HOME
          - MOVE_HOME: Transition auf FINISHED
        """
        phase = self._current_phase
        _LOG.info("ProcessThread: Motion EXECUTED:OK in Phase %s", phase)

        if phase == self.PHASE_MOVE_PREDISPENSE:
            # Safety-Timer stoppen
            if self._predispense_timer is not None:
                self._predispense_timer.stop()
            self._predispense_timer = None
            self._predispense_elapsed_ms = 0

            self._current_phase = self.PHASE_NONE
            self._post_transition(self._sig_predispense_move_done)
            return

        if phase == self.PHASE_MOVE_RECIPE:
            if not self._recipe_poses:
                _LOG.warning("ProcessThread: MOVE_RECIPE: keine Posen mehr, beende State.")
                self._current_phase = self.PHASE_NONE
                self._post_transition(self._sig_move_recipe_done)
                return

            # nächsten Waypoint fahren oder fertig
            self._recipe_index += 1
            if self._recipe_index >= len(self._recipe_poses) - 1:
                # Letzter Wegpunkt (Retreat) wird im MOVE_RETREAT-State gefahren,
                # daher hier nur bis poses[-2]
                _LOG.info("ProcessThread: MOVE_RECIPE: alle Zwischen-Posen abgefahren.")
                self._current_phase = self.PHASE_NONE
                self._post_transition(self._sig_move_recipe_done)
                return

            pose = self._recipe_poses[self._recipe_index]
            _LOG.info(
                "ProcessThread: MOVE_RECIPE: nächster Wegpunkt %d/%d.",
                self._recipe_index,
                len(self._recipe_poses) - 2,
            )
            self._send_motion_pose(pose, label=f"recipe_{self._recipe_index}")
            return

        if phase == self.PHASE_MOVE_RETREAT:
            self._current_phase = self.PHASE_NONE
            self._post_transition(self._sig_retreat_move_done)
            return

        if phase == self.PHASE_MOVE_HOME:
            self._current_phase = self.PHASE_NONE
            self._post_transition(self._sig_move_home_done)
            return

        # Fallback
        self._current_phase = self.PHASE_NONE

    # ---------------------------------------------------------
    # State-Namen emitten
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
        try:
            pc = self._recipe.paths_compiled or {}
        except Exception:
            _LOG.error("ProcessThread: _get_recipe_poses: recipe.paths_compiled nicht verfügbar.")
            return []

        sides = pc.get("sides") or {}
        if not isinstance(sides, dict) or not sides:
            _LOG.warning("ProcessThread: _get_recipe_poses: paths_compiled.sides ist leer.")
            return []

        # Immer die erste Side im Dict verwenden
        first_side = next(iter(sides.keys()))
        sdata = sides[first_side] or {}
        poses_quat = sdata.get("poses_quat") or []

        if not poses_quat:
            _LOG.warning(
                "ProcessThread: _get_recipe_poses: erste Side '%s' hat keine poses_quat.",
                first_side,
            )
            return []

        frame_id = pc.get("frame") or "scene"

        out: List[PoseStamped] = []
        for p in poses_quat:
            ps = PoseStamped()
            ps.header.frame_id = frame_id

            ps.pose.position.x = float(p.get("x", 0.0))
            ps.pose.position.y = float(p.get("y", 0.0))
            ps.pose.position.z = float(p.get("z", 0.0))

            # aktuell: Orientierung auf Identität (rx=ry=rz=0)
            ps.pose.orientation.x = 0.0
            ps.pose.orientation.y = 0.0
            ps.pose.orientation.z = 0.0
            ps.pose.orientation.w = 1.0

            out.append(ps)

        _LOG.info(
            "ProcessThread: _get_recipe_poses: benutze erste side='%s' mit %d Posen "
            "(frame=%s, quat=identity).",
            first_side, len(out), frame_id,
        )
        return out

    # ---------------------------------------------------------
    # Home-Pose holen (PosesBridge/PosesState)
    # ---------------------------------------------------------

    def _get_home_pose(self) -> Optional[PoseStamped]:
        """
        Versucht, eine Home-Pose als PoseStamped zu bekommen:

          1. PosesBridge.get_last_home_pose()
          2. PosesState.home()

        Wenn nichts da ist, None zurück.
        """
        # 1) PosesBridge (Signal-/Bridge-Objekt)
        if self._pb is not None:
            try:
                pose = self._pb.get_last_home_pose()
                if isinstance(pose, PoseStamped):
                    return pose
            except Exception:
                pass

        # 2) State-Objekt bridge.poses
        if self._poses_state is not None:
            try:
                pose2 = self._poses_state.home()
                if isinstance(pose2, PoseStamped):
                    return pose2
            except Exception:
                pass

        _LOG.warning("ProcessThread: _get_home_pose: keine Home-Pose verfügbar.")
        return None

    # ---------------------------------------------------------
    # Motion-Senden (ohne Blockieren)
    # ---------------------------------------------------------

    def _send_motion_pose(self, pose: PoseStamped, label: str) -> None:
        """
        Sendet eine Pose an die MotionBridge, ohne zu blockieren.
        Das eigentliche Ergebnis kommt asynchron via motionResultChanged.
        """
        if pose is None:
            msg = f"Interner Fehler: Zielpose ist None (label={label})."
            _LOG.error("ProcessThread: _send_motion_pose: %s", msg)
            self._signal_error(msg)
            return

        if self._motion is None or self._motion_signals is None:
            msg = f"MotionBridge fehlt für label={label}."
            _LOG.error("ProcessThread: _send_motion_pose: %s", msg)
            self._signal_error(msg)
            return

        try:
            _LOG.info(
                "ProcessThread: moveToPoseRequested(%s) via MotionBridge-Signal wird emittiert.",
                label,
            )
            self._motion_signals.moveToPoseRequested.emit(pose)
        except Exception as e:
            msg = f"moveToPoseRequested({label}) emit failed: {e}"
            _LOG.exception("ProcessThread: %s", msg)
            self._signal_error(msg)

    # ---------------------------------------------------------
    # StateMachine-Logik
    # ---------------------------------------------------------

    def run(self) -> None:
        """
        Thread-Einstieg: baut die QStateMachine auf, startet sie
        und startet den Thread-Eventloop.
        """
        _LOG.info("ProcessThread.run: gestartet.")
        self._stop_requested = False
        self._error_msg = None
        self._ended_in_error_state = False
        self._ended_in_finished_state = False
        self._current_phase = self.PHASE_NONE
        self._recipe_poses = []
        self._recipe_index = 0
        self._predispense_elapsed_ms = 0

        machine = QStateMachine()
        self._machine = machine  # für _finish_state / _stop_machine_and_quit()

        # States
        s_move_predisp = QState()
        s_wait_predisp = QState()
        s_move_recipe = QState()
        s_wait_postdisp = QState()
        s_move_retreat = QState()
        s_move_home = QState()
        s_finished = QFinalState()
        s_error = QFinalState()   # ERROR-FinalState

        # StateMachine strukturieren
        machine.addState(s_move_predisp)
        machine.addState(s_wait_predisp)
        machine.addState(s_move_recipe)
        machine.addState(s_wait_postdisp)
        machine.addState(s_move_retreat)
        machine.addState(s_move_home)
        machine.addState(s_finished)
        machine.addState(s_error)
        machine.setInitialState(s_move_predisp)

        # Übergänge
        s_move_predisp.addTransition(self._sig_predispense_move_done, s_wait_predisp)
        s_wait_predisp.addTransition(self._sig_predispense_wait_done, s_move_recipe)
        s_move_recipe.addTransition(self._sig_move_recipe_done, s_wait_postdisp)
        s_wait_postdisp.addTransition(self._sig_postdispense_wait_done, s_move_retreat)
        s_move_retreat.addTransition(self._sig_retreat_move_done, s_move_home)
        s_move_home.addTransition(self._sig_move_home_done, s_finished)

        # von JEDEM normalen State in den ERROR-FinalState springen können
        for st in (
            s_move_predisp,
            s_wait_predisp,
            s_move_recipe,
            s_wait_postdisp,
            s_move_retreat,
            s_move_home,
        ):
            st.addTransition(self._sig_error, s_error)

        # State-Callbacks
        s_move_predisp.entered.connect(self._on_state_move_predispense)
        s_wait_predisp.entered.connect(self._on_state_wait_predispense)
        s_move_recipe.entered.connect(self._on_state_move_recipe)
        s_wait_postdisp.entered.connect(self._on_state_wait_postdispense)
        s_move_retreat.entered.connect(self._on_state_move_retreat)
        s_move_home.entered.connect(self._on_state_move_home)
        s_error.entered.connect(self._on_state_error)
        s_finished.entered.connect(self._on_state_finished)

        # State-Namen nach außen signalisieren
        s_move_predisp.entered.connect(lambda: self._emit_state("MOVE_PREDISPENSE"))
        s_wait_predisp.entered.connect(lambda: self._emit_state("WAIT_PREDISPENSE"))
        s_move_recipe.entered.connect(lambda: self._emit_state("MOVE_RECIPE"))
        s_wait_postdisp.entered.connect(lambda: self._emit_state("WAIT_POSTDISPENSE"))
        s_move_retreat.entered.connect(lambda: self._emit_state("MOVE_RETREAT"))
        s_move_home.entered.connect(lambda: self._emit_state("MOVE_HOME"))

        try:
            machine.start()
            _LOG.info("ProcessThread.run: StateMachine gestartet, Thread-Eventloop läuft.")
            self.exec()
        except Exception as e:
            _LOG.exception("ProcessThread.run: Exception: %s", e)
            if not self._error_msg:
                self._error_msg = str(e)
        finally:
            self._machine = None
            _LOG.info(
                "ProcessThread.run: beendet, _error_msg=%r, stop_requested=%s, "
                "ended_in_error=%s, ended_in_finished=%s",
                self._error_msg,
                self._stop_requested,
                self._ended_in_error_state,
                self._ended_in_finished_state,
            )
            if self._log_handler is not None:
                try:
                    _LOG.removeHandler(self._log_handler)
                except Exception:
                    pass
                self._log_handler = None

    # ---------------------------------------------------------
    # State-Handler
    # ---------------------------------------------------------

    def _on_state_move_predispense(self) -> None:
        """
        State: Predispense-/erste Rezeptposition anfahren.
          → path[0]
        """
        _LOG.info("ProcessThread: ENTER MOVE_PREDISPENSE")

        # ggf. alten Safety-Timer aufräumen
        if self._predispense_timer is not None:
            self._predispense_timer.stop()
        self._predispense_timer = None
        self._predispense_elapsed_ms = 0

        # Wenn schon vorher Fehler/Stop → direkt weiter/Fehler
        if self._error_msg or self._should_stop():
            _LOG.info("ProcessThread: MOVE_PREDISPENSE: Fehler/Stop vor Start erkannt.")
            self._finish_state(self._sig_predispense_move_done)
            _LOG.info("ProcessThread: LEAVE MOVE_PREDISPENSE")
            return

        poses = self._get_recipe_poses()
        self._recipe_poses = poses
        self._recipe_index = 0

        if not poses:
            _LOG.warning("ProcessThread: MOVE_PREDISPENSE: keine Posen -> direkt weiter.")
            _LOG.info("ProcessThread: LEAVE MOVE_PREDISPENSE")
            self._finish_state(self._sig_predispense_move_done)
            return

        first = poses[0]
        _LOG.info("ProcessThread: MOVE_PREDISPENSE: fahre Pose[0].")
        self._current_phase = self.PHASE_MOVE_PREDISPENSE
        self._send_motion_pose(first, label="predispense")

        # --- Safety-Timer aktivieren: falls aus irgendeinem Grund
        #     kein motionResultChanged ankommt, hängen wir nicht ewig.
        self._predispense_timer = QtCore.QTimer()
        self._predispense_timer.setInterval(100)  # 100 ms
        self._predispense_timer.timeout.connect(self._on_predispense_tick)
        self._predispense_timer.start()

        _LOG.info("ProcessThread: LEAVE MOVE_PREDISPENSE (warte auf EXECUTED:OK oder Timeout).")

    def _on_predispense_tick(self) -> None:
        """
        Safety-Tick während MOVE_PREDISPENSE:
          - wenn Phase gewechselt wurde oder StateMachine gestoppt ist → Timer stoppen
          - wenn Stop angefordert → Fehler
          - wenn Timeout erreicht → Warnung + weiter in WAIT_PREDISPENSE
        """
        if self._machine is None or not self._machine.isRunning():
            if self._predispense_timer is not None:
                self._predispense_timer.stop()
            self._predispense_timer = None
            return

        if self._current_phase != self.PHASE_MOVE_PREDISPENSE:
            # Motion ist fertig oder in Fehler gelaufen → Timer aus
            if self._predispense_timer is not None:
                self._predispense_timer.stop()
            self._predispense_timer = None
            return

        if self._should_stop():
            if not self._error_msg:
                self._error_msg = "Prozess durch Benutzer gestoppt."
            if self._predispense_timer is not None:
                self._predispense_timer.stop()
            self._predispense_timer = None
            self._signal_error(self._error_msg)
            return

        self._predispense_elapsed_ms += 100
        if self._predispense_elapsed_ms >= self._predispense_timeout_ms:
            _LOG.warning(
                "ProcessThread: MOVE_PREDISPENSE Timeout nach %.1f s – "
                "wechsle trotzdem in WAIT_PREDISPENSE.",
                self._predispense_elapsed_ms / 1000.0,
            )
            if self._predispense_timer is not None:
                self._predispense_timer.stop()
            self._predispense_timer = None
            self._current_phase = self.PHASE_NONE
            self._finish_state(self._sig_predispense_move_done)

    def _on_state_wait_predispense(self) -> None:
        """
        State: predispense-time warten.
        """
        _LOG.info("ProcessThread: ENTER WAIT_PREDISPENSE")

        if not self._error_msg:
            pre_t = 0.0
            try:
                pre_t = float(self._recipe.globals.predispose_time or 0.0)  # type: ignore[attr-defined]
            except Exception:
                pre_t = 0.0

            _LOG.info("ProcessThread: WAIT_PREDISPENSE: predispense_time=%.3f s", pre_t)

            ms_total = max(0, int(pre_t * 1000))
            elapsed = 0
            step = 50  # ms

            while elapsed < ms_total:
                if self._should_stop():
                    if not self._error_msg:
                        self._error_msg = "Prozess durch Benutzer gestoppt."
                    break
                self.msleep(step)
                elapsed += step

        _LOG.info("ProcessThread: LEAVE WAIT_PREDISPENSE")
        self._finish_state(self._sig_predispense_wait_done)

    def _on_state_move_recipe(self) -> None:
        """
        State: kompletten Rezeptpfad fahren.
          → alle Zwischen-Pfade path[1:-1]
        """
        _LOG.info("ProcessThread: ENTER MOVE_RECIPE")

        if self._error_msg or self._should_stop():
            _LOG.info("ProcessThread: MOVE_RECIPE: Fehler/Stop vor Start erkannt.")
            self._finish_state(self._sig_move_recipe_done)
            _LOG.info("ProcessThread: LEAVE MOVE_RECIPE")
            return

        if not self._recipe_poses:
            self._recipe_poses = self._get_recipe_poses()
            self._recipe_index = 0

        n = len(self._recipe_poses)
        _LOG.info("ProcessThread: MOVE_RECIPE: Anzahl Posen=%d", n)

        if n <= 2:
            _LOG.info("ProcessThread: MOVE_RECIPE: <=2 Posen, nichts zu fahren.")
            _LOG.info("ProcessThread: LEAVE MOVE_RECIPE")
            self._finish_state(self._sig_move_recipe_done)
            return

        # Wir fahren Posen[1] bis Posen[-2]
        self._recipe_index = 1
        pose = self._recipe_poses[self._recipe_index]
        _LOG.info("ProcessThread: MOVE_RECIPE: starte mit Pose[1] von %d.", n - 2)
        self._current_phase = self.PHASE_MOVE_RECIPE
        self._send_motion_pose(pose, label=f"recipe_{self._recipe_index}")

        _LOG.info("ProcessThread: LEAVE MOVE_RECIPE (warte auf EXECUTED:OK / Folgewaypoints).")

    def _on_state_wait_postdispense(self) -> None:
        """
        State: postdispense-time warten.
        """
        _LOG.info("ProcessThread: ENTER WAIT_POSTDISPENSE")

        if not self._error_msg:
            post_t = 0.0
            try:
                post_t = float(self._recipe.globals.postdispense_time or 0.0)  # type: ignore[attr-defined]
            except Exception:
                post_t = 0.0

            _LOG.info("ProcessThread: WAIT_POSTDISPENSE: postdispense_time=%.3f s", post_t)

            ms_total = max(0, int(post_t * 1000))
            elapsed = 0
            step = 50  # ms

            while elapsed < ms_total:
                if self._should_stop():
                    if not self._error_msg:
                        self._error_msg = "Prozess durch Benutzer gestoppt."
                    break
                self.msleep(step)
                elapsed += step

        _LOG.info("ProcessThread: LEAVE WAIT_POSTDISPENSE")
        self._finish_state(self._sig_postdispense_wait_done)

    def _on_state_move_retreat(self) -> None:
        """
        State: Retreat-Position anfahren.
          → path[-1], wenn vorhanden
        """
        _LOG.info("ProcessThread: ENTER MOVE_RETREAT")

        if self._error_msg or self._should_stop():
            _LOG.info("ProcessThread: MOVE_RETREAT: Fehler/Stop vor Start erkannt.")
            self._finish_state(self._sig_retreat_move_done)
            _LOG.info("ProcessThread: LEAVE MOVE_RETREAT")
            return

        if not self._recipe_poses:
            self._recipe_poses = self._get_recipe_poses()
            self._recipe_index = 0

        n = len(self._recipe_poses)
        if n < 2:
            _LOG.info("ProcessThread: MOVE_RETREAT: <2 Posen, kein separates Retreat.")
            _LOG.info("ProcessThread: LEAVE MOVE_RETREAT")
            self._finish_state(self._sig_retreat_move_done)
            return

        last = self._recipe_poses[-1]
        _LOG.info("ProcessThread: MOVE_RETREAT: fahre Pose[-1].")
        self._current_phase = self.PHASE_MOVE_RETREAT
        self._send_motion_pose(last, label="retreat")

        _LOG.info("ProcessThread: LEAVE MOVE_RETREAT (warte auf EXECUTED:OK).")

    def _on_state_move_home(self) -> None:
        """
        State: Roboter nach Home fahren.

        NICHT mehr über RobotBridge.move_home(),
        sondern konsistent:
          - Home-Pose aus Poses holen
          - via MotionBridge fahren
          - auf EXECUTED:OK warten (Phase MOVE_HOME)
        """
        _LOG.info("ProcessThread: ENTER MOVE_HOME")

        if self._error_msg or self._should_stop():
            _LOG.info("ProcessThread: MOVE_HOME: Fehler/Stop vor Start erkannt.")
            self._finish_state(self._sig_move_home_done)
            _LOG.info("ProcessThread: LEAVE MOVE_HOME")
            return

        home = self._get_home_pose()
        if home is None:
            _LOG.warning(
                "ProcessThread: MOVE_HOME: keine Home-Pose verfügbar, "
                "überspringe Home-Fahrt."
            )
            _LOG.info("ProcessThread: LEAVE MOVE_HOME")
            self._finish_state(self._sig_move_home_done)
            return

        _LOG.info("ProcessThread: MOVE_HOME: sende Home-Pose via MotionBridge.")
        self._current_phase = self.PHASE_MOVE_HOME
        self._send_motion_pose(home, label="home")

        _LOG.info("ProcessThread: LEAVE MOVE_HOME (warte auf EXECUTED:OK).")

    def _on_state_error(self) -> None:
        """
        Finaler ERROR-State.
        """
        _LOG.info("ProcessThread: ENTER ERROR")
        self._ended_in_error_state = True
        self._emit_state("ERROR")

        msg = self._error_msg or "Unbekannter Prozessfehler."
        _LOG.error("ProcessThread: notifyError(%s) im ERROR-State", msg)
        self.notifyError.emit(msg)

        self._stop_machine_and_quit()

    def _on_state_finished(self) -> None:
        """
        Finaler FINISHED-State.
        """
        _LOG.info("ProcessThread: ENTER FINISHED")
        self._ended_in_finished_state = True
        self._emit_state("FINISHED")

        _LOG.info("ProcessThread: notifyFinished() im FINISHED-State")
        self.notifyFinished.emit()

        self._stop_machine_and_quit()
