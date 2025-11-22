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

        # Merker, ob die StateMachine im ERROR- oder FINISHED-FinalState geendet hat
        self._ended_in_error_state: bool = False
        self._ended_in_finished_state: bool = False

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
                # DirectConnection, damit _on_motion_result auch ohne separaten
                # Eventloop im QThread ausgeführt wird (im Sender-Thread).
                sigs.motionResultChanged.connect(
                    self._on_motion_result,
                    QtCore.Qt.ConnectionType.DirectConnection,
                )
                _LOG.info(
                    "ProcessThread: MotionBridge gefunden (%s), "
                    "motionResultChanged als DirectConnection verbunden.",
                    type(self._motion).__name__ if self._motion is not None else "None",
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

    # ---------------------------------------------------------
    # Zentrale Transition-Hilfe
    # ---------------------------------------------------------

    def _post_transition(self, sig: QtCore.pyqtSignal) -> None:
        """
        Führt einen State-Übergang *asynchron* aus.

        WICHTIG:
          - Niemals .emit() direkt aus einem State-Handler aufrufen.
          - Immer _post_transition(self._sig_...) verwenden.
        """
        if self._machine is None or not self._machine.isRunning():
            _LOG.warning("ProcessThread: _post_transition aufgerufen, aber StateMachine läuft nicht mehr.")
            return

        QtCore.QTimer.singleShot(0, sig.emit)

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
    # Abbruch / Fehler
    # ---------------------------------------------------------

    def _abort_if_needed(self, msg: str | None = None) -> None:
        """
        Setzt optional eine Fehlermeldung und beendet den Ablauf.

        Verhalten:
          - setzt _error_msg, wenn msg übergeben wird
          - wenn StateMachine läuft: wechselt via _sig_error asynchron in den ERROR-FinalState
          - wenn StateMachine nicht mehr läuft, wird der Thread-Eventloop via quit() beendet

        WICHTIG:
          - notifyError() wird NICHT hier aufgerufen, sondern ausschließlich
            im ERROR-FinalState (_on_state_error).
        """
        if msg and not self._error_msg:
            self._error_msg = msg

        if self._error_msg:
            _LOG.error("ProcessThread: Abbruch mit Fehler: %s", self._error_msg)

        machine = self._machine

        # Wenn die StateMachine noch läuft: in den ERROR-FinalState wechseln (best effort)
        if machine is not None and machine.isRunning():
            try:
                _LOG.info(
                    "ProcessThread: Fehler erkannt -> wechsle in ERROR-FinalState "
                    "via Signal (async)."
                )
                self._post_transition(self._sig_error)
                return
            except Exception:
                _LOG.exception("ProcessThread: Fehler beim Signalisieren des ERROR-States.")

        # Falls die StateMachine schon beendet ist, Thread-Eventloop direkt beenden
        self._stop_machine_and_quit()

    # ---------------------------------------------------------
    # Motion-Result-Handling
    # ---------------------------------------------------------

    @QtCore.pyqtSlot(str)
    def _on_motion_result(self, text: str) -> None:
        """
        Wird durch DirectConnection im Sender-Thread (MotionBridge) aufgerufen.
        Speichert nur den Text; ausgewertet wird er im Thread-Kontext in _wait_for_motion.
        """
        self._last_motion_result = (text or "").strip()
        _LOG.info("ProcessThread: motion_result empfangen: %r", self._last_motion_result)

    def _wait_for_motion(self, timeout_s: float) -> bool:
        """
        Wartet auf ein Motion-Ergebnis.

        Strikt / fail-fast:
          - Nur EXECUTED:OK / EXECUTED... gilt als Erfolg.
          - JEDES andere Ergebnis (inkl. ERROR:..., UNKNOWN:..., etc.)
            ist ein Fehler.
          - Kein MotionBridge oder timeout_s <= 0 → ebenfalls Fehler.
        """
        # MotionBridge muss da sein
        if self._motion is None:
            msg = "MotionBridge nicht verfügbar (motion=None)."
            _LOG.error("ProcessThread: _wait_for_motion: %s", msg)
            self._abort_if_needed(msg)
            return False

        # Timeout muss sinnvoll sein
        if timeout_s <= 0.0:
            msg = "Motion timeout ist <= 0 s konfiguriert."
            _LOG.error("ProcessThread: _wait_for_motion: %s", msg)
            self._abort_if_needed(msg)
            return False

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
            if res:
                res_u = res.upper()
                _LOG.info("ProcessThread: motion_result ausgewertet: %r", res)

                # Einziger Erfolgsfall
                if res_u.startswith("EXECUTED:OK") or res_u.startswith("EXECUTED"):
                    return True

                # ALLES andere ist Fehler, ohne Fallback-Mapping
                if not self._error_msg:
                    self._error_msg = res

                _LOG.error("ProcessThread: Motion-Fehler: %s", self._error_msg)
                self._abort_if_needed(self._error_msg)
                return False

            self.msleep(step_ms)
            elapsed_ms += step_ms

        # Timeout -> ebenfalls HARTE Fehlerbedingung
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

        Strikt:
          - Wenn _wait_for_motion() False zurückgibt, wurde bereits ein Fehler
            gesetzt und _abort_if_needed() aufgerufen.
          - Es gibt KEINE nachträgliche zweite Auswertung von _last_motion_result.

        Kommunikation mit der MotionBridge erfolgt ausschließlich über
        Qt-Signale (moveToPoseRequested), NICHT über direkte Methodenaufrufe.
        """
        if self._should_stop():
            _LOG.info("ProcessThread: _move_pose_with_motion(%s) abgebrochen (stop_requested).", label)
            self._abort_if_needed("Prozess durch Benutzer gestoppt.")
            return False

        if pose is None:
            msg = f"Interner Fehler: Zielpose ist None (label={label})."
            _LOG.error("ProcessThread: _move_pose_with_motion(%s): pose ist None.", label)
            self._abort_if_needed(msg)
            return False

        if self._motion is None:
            msg = f"MotionBridge fehlt (motion=None) für label={label}."
            _LOG.error("ProcessThread: _move_pose_with_motion: %s", msg)
            self._abort_if_needed(msg)
            return False

        sigs = getattr(self._motion, "signals", None)
        if sigs is None or not hasattr(sigs, "moveToPoseRequested"):
            msg = f"MotionBridge.signals.moveToPoseRequested fehlt für label={label}."
            _LOG.error(
                "ProcessThread: _move_pose_with_motion: %s (motion=%r, signals=%r)",
                msg,
                self._motion,
                sigs,
            )
            self._abort_if_needed(msg)
            return False

        _LOG.info("ProcessThread: moveToPoseRequested(%s) via MotionBridge-Signal wird emittiert.", label)
        try:
            # Qt kümmert sich um Thread-Grenzen (QueuedConnection),
            # MotionBridge führt dann _move_to_pose_impl im eigenen Thread aus.
            sigs.moveToPoseRequested.emit(pose)
        except Exception as e:
            msg = f"moveToPoseRequested({label}) emit failed: {e}"
            _LOG.exception("ProcessThread: %s", msg)
            self._abort_if_needed(msg)
            return False

        ok = self._wait_for_motion(self._move_timeout_s)
        if not ok:
            # _wait_for_motion hat bereits _error_msg gesetzt und _abort_if_needed() aufgerufen.
            _LOG.error(
                "ProcessThread: _move_pose_with_motion(%s) fehlgeschlagen: %s",
                label,
                self._error_msg,
            )
            return False

        _LOG.info("ProcessThread: _move_pose_with_motion(%s) erfolgreich.", label)
        return True

    # ---------------------------------------------------------
    # StateMachine-Logik
    # ---------------------------------------------------------

    def run(self) -> None:
        """
        Thread-Einstieg: baut die QStateMachine auf, startet sie
        und startet den Thread-Eventloop.

        WICHTIG:
        - Diese Methode wertet das Ergebnis NICHT mehr aus.
        - Alle Ergebnis-Signale (notifyFinished / notifyError)
          werden ausschließlich in den State-Handlern
          _on_state_finished() bzw. _on_state_error() emittiert.
        """
        _LOG.info("ProcessThread.run: gestartet.")
        self._stop_requested = False
        self._error_msg = None
        self._ended_in_error_state = False
        self._ended_in_finished_state = False

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
        # ERROR-State-Name + notifyError kommen aus _on_state_error
        # FINISHED-State-Name + notifyFinished kommen aus _on_state_finished

        try:
            machine.start()  # <<< Maschine wird hier gestartet
            _LOG.info("ProcessThread.run: StateMachine gestartet, Thread-Eventloop läuft.")
            # Thread-eigene Eventloop, nötig für QStateMachine + QTimer.singleShot
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
            # Logging-Handler wieder entfernen
            if self._log_handler is not None:
                try:
                    _LOG.removeHandler(self._log_handler)
                except Exception:
                    pass
                self._log_handler = None
        # KEINE Ergebnis-Auswertung / KEIN notifyFinished/notifyError hier!

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
            self._post_transition(self._sig_predispense_move_done)
            return

        first = poses[0]
        _LOG.info("ProcessThread: MOVE_PREDISPENSE: fahre Pose[0].")

        if not self._move_pose_with_motion(first, label="predispense"):
            return

        if self._should_stop():
            self._abort_if_needed()
            return

        _LOG.info("ProcessThread: LEAVE MOVE_PREDISPENSE")
        self._post_transition(self._sig_predispense_move_done)

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
        self._post_transition(self._sig_predispense_wait_done)

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
            self._post_transition(self._sig_move_recipe_done)
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
        self._post_transition(self._sig_move_recipe_done)

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
        self._post_transition(self._sig_postdispense_wait_done)

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
            self._post_transition(self._sig_retreat_move_done)
            return

        last = poses[-1]
        _LOG.info("ProcessThread: MOVE_RETREAT: fahre Pose[-1].")

        if not self._move_pose_with_motion(last, label="retreat"):
            return

        if self._should_stop():
            self._abort_if_needed()
            return

        _LOG.info("ProcessThread: LEAVE MOVE_RETREAT")
        self._post_transition(self._sig_retreat_move_done)

    def _on_state_move_home(self) -> None:
        """
        State: Roboter nach Home fahren.
        Nutzt, falls vorhanden, move_home() oder go_home() der RobotBridge,
        sonst Dummy-Sleep (das ist bewusst KEIN Motion-Fallback).
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
        self._post_transition(self._sig_move_home_done)

    def _on_state_error(self) -> None:
        """
        Finaler ERROR-State.

        Hier wird:
          - der ERROR-State nach außen gemeldet
          - genau EINMAL notifyError() emittiert.
          - die StateMachine gestoppt und der Thread beendet.
        """
        _LOG.info("ProcessThread: ENTER ERROR")
        self._ended_in_error_state = True
        self._emit_state("ERROR")

        # Wenn es keine spezifische Fehlermeldung gibt, generische Nachricht setzen
        msg = self._error_msg or "Unbekannter Prozessfehler."
        _LOG.error("ProcessThread: notifyError(%s) im ERROR-State", msg)
        self.notifyError.emit(msg)

        # <<< Maschine stoppen + Thread-Loop beenden
        self._stop_machine_and_quit()

    def _on_state_finished(self) -> None:
        """
        Finaler FINISHED-State.

        Hier wird:
          - der FINISHED-State nach außen gemeldet
          - genau EINMAL notifyFinished() emittiert.
          - die StateMachine gestoppt und der Thread beendet.
        """
        _LOG.info("ProcessThread: ENTER FINISHED")
        self._ended_in_finished_state = True
        self._emit_state("FINISHED")

        _LOG.info("ProcessThread: notifyFinished() im FINISHED-State")
        self.notifyFinished.emit()

        # <<< Maschine stoppen + Thread-Loop beenden
        self._stop_machine_and_quit()
