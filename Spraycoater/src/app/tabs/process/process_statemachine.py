# -*- coding: utf-8 -*-
# File: tabs/process/process_statemachine.py
from __future__ import annotations

from typing import Optional, List, Dict, Any
from queue import Queue, Empty
import logging

from PyQt6 import QtCore
from PyQt6.QtStateMachine import QStateMachine, QState, QFinalState

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg

from app.model.recipe.recipe import Recipe

_LOG = logging.getLogger("app.tabs.process.statemachine")


class QStatemachine(QStateMachine):
    """
    Dünner Wrapper um QStateMachine – nur damit der Typ im Code lesbarer ist.
    """
    def __init__(self, parent: Optional[QtCore.QObject] = None) -> None:
        super().__init__(parent)


class _QtSignalHandler(logging.Handler):
    """
    Leitet Log-Messages dieses Loggers über ein Qt-Signal nach außen,
    damit sie im UI angezeigt werden können.
    """
    def __init__(self, owner: "ProcessStatemachine") -> None:
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
            # Wenn das Signal nicht mehr existiert (Worker/Thread beendet), ignorieren.
            pass


class ProcessStatemachine(QtCore.QObject):
    """
    StateMachine-Worker, der im Hintergrund-Thread läuft.

    Enthält:
      - QStatemachine mit den Prozess-States
      - Logging
      - Aufzeichnung von TCP-Posen und MoveIt-Trajektorien

    API für den ProcessThread:
      - Slots:
          start()
          request_stop()
      - Properties/Methoden:
          recipe
          set_recipe(recipe)
      - Signale:
          notifyFinished(object)
          notifyError(str)
          stateChanged(str)
          logMessage(str)

    Result-Objekt bei notifyFinished:
      {
        "poses": List[PoseStamped],              # TCP-Stream (executed)
        "planned_traj": List[RobotTrajectoryMsg],
        "executed_traj": List[RobotTrajectoryMsg],
      }
    """

    # Ergebnis / Fehler
    notifyFinished = QtCore.pyqtSignal(object)  # Dict[str, Any]
    notifyError = QtCore.pyqtSignal(str)

    # UI-Hilfssignale
    stateChanged = QtCore.pyqtSignal(str)
    logMessage = QtCore.pyqtSignal(str)

    # interne StateMachine-Signale
    _sig_predispense_move_done = QtCore.pyqtSignal()
    _sig_predispense_wait_done = QtCore.pyqtSignal()
    _sig_move_recipe_done = QtCore.pyqtSignal()
    _sig_postdispense_wait_done = QtCore.pyqtSignal()
    _sig_retreat_move_done = QtCore.pyqtSignal()
    _sig_move_home_done = QtCore.pyqtSignal()
    _sig_error = QtCore.pyqtSignal()

    # Phasen-Konstanten
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
    ) -> None:
        super().__init__(parent)

        self._recipe = recipe
        self._bridge = bridge

        self._stop_requested: bool = False
        self._error_msg: Optional[str] = None
        self._machine: Optional[QStatemachine] = None

        self._ended_in_error_state: bool = False
        self._ended_in_finished_state: bool = False

        # Aktive Motion-Phase (für Auswertung in _on_motion_result, Polling usw.)
        self._current_phase: str = self.PHASE_NONE

        # Rezept-Posen (aus paths_compiled)
        self._recipe_poses: List[PoseStamped] = []
        self._recipe_index: int = 0

        # Aufgezeichnete TCP-Posen (aus tcpPoseChanged)
        self._executed_poses: List[PoseStamped] = []

        # Geplante / ausgeführte MoveIt-Trajektorien (Segmentliste pro Run)
        self._planned_traj_list: List[RobotTrajectoryMsg] = []
        self._executed_traj_list: List[RobotTrajectoryMsg] = []

        # Queues für entkoppeltes Logging (TCP + Trajs)
        self._tcp_pose_queue: "Queue[PoseStamped]" = Queue()
        self._planned_traj_queue: "Queue[RobotTrajectoryMsg]" = Queue()
        self._executed_traj_queue: "Queue[RobotTrajectoryMsg]" = Queue()

        # Polling-Timer (lebt im Worker-Thread)
        self._poll_timer: Optional[QtCore.QTimer] = None

        # Logging -> Qt
        self._log_handler: Optional[_QtSignalHandler] = _QtSignalHandler(self)
        self._log_handler.setLevel(logging.INFO)
        formatter = logging.Formatter("%(message)s")
        self._log_handler.setFormatter(formatter)
        _LOG.addHandler(self._log_handler)

        # Bridges
        self._rb = getattr(bridge, "_rb", None)
        self._rb_signals = None

        self._motion = getattr(bridge, "_motion", None)
        self._motion_signals = self._motion.signals if self._motion is not None else None

        self._pb = getattr(bridge, "_pb", None)
        self._poses_state = getattr(bridge, "poses", None)

        # Motion-Result / Trajektorien verbinden
        if self._motion is not None and self._motion_signals is not None:
            try:
                if hasattr(self._motion_signals, "motionResultChanged"):
                    self._motion_signals.motionResultChanged.connect(self._on_motion_result)
                    _LOG.info(
                        "ProcessStatemachine: MotionBridge gefunden (%s), motionResultChanged verbunden.",
                        type(self._motion).__name__,
                    )

                if hasattr(self._motion_signals, "plannedTrajectoryChanged"):
                    self._motion_signals.plannedTrajectoryChanged.connect(self._on_planned_traj)
                    _LOG.info(
                        "ProcessStatemachine: plannedTrajectoryChanged mit _on_planned_traj verbunden."
                    )

                if hasattr(self._motion_signals, "executedTrajectoryChanged"):
                    self._motion_signals.executedTrajectoryChanged.connect(self._on_executed_traj)
                    _LOG.info(
                        "ProcessStatemachine: executedTrajectoryChanged mit _on_executed_traj verbunden."
                    )
            except Exception as e:
                _LOG.exception("ProcessStatemachine: Konnte Motion-Signale nicht verbinden: %s", e)
        else:
            _LOG.error(
                "ProcessStatemachine: MotionBridge oder deren Signale sind None – "
                "Bewegungen können nicht ausgeführt/geloggt werden."
            )

        # RobotBridge tcpPoseChanged verbinden (für Trajektorien-Logging)
        if self._rb is not None:
            try:
                sig_r = getattr(self._rb, "signals", None)
                if sig_r is None:
                    _LOG.warning("ProcessStatemachine: RobotBridge.signals ist None – kein TCP-Logging.")
                else:
                    self._rb_signals = sig_r
                    if hasattr(sig_r, "tcpPoseChanged"):
                        sig_r.tcpPoseChanged.connect(self._on_tcp_stream)
                        _LOG.info(
                            "ProcessStatemachine: RobotBridge tcpPoseChanged verbunden – "
                            "gefahrene Trajektorie wird mitgeloggt (Queue)."
                        )
                    else:
                        _LOG.warning(
                            "ProcessStatemachine: RobotBridge.signals.tcpPoseChanged fehlt – "
                            "kein Live-Trajektorien-Logging möglich."
                        )
            except Exception:
                _LOG.exception("ProcessStatemachine: Fehler beim Verbinden von tcpPoseChanged.")
        else:
            _LOG.warning("ProcessStatemachine: RobotBridge ist None – kein TCP-Logging möglich.")

        _LOG.info(
            "ProcessStatemachine init: recipe=%s, rb=%s, motion=%s",
            getattr(recipe, "id", None),
            type(self._rb).__name__ if self._rb is not None else "None",
            type(self._motion).__name__ if self._motion is not None else "None",
        )

    # ------------------------------------------------------------------ #
    # Öffentliche API – vom ProcessThread via QueuedConnection aufgerufen
    # ------------------------------------------------------------------ #

    @QtCore.pyqtSlot()
    def start(self) -> None:
        """
        Startet einen neuen Prozesslauf.
        Läuft im Worker-Thread.
        """
        _LOG.info(
            "ProcessStatemachine.start: gestartet (Thread=%r)",
            int(QtCore.QThread.currentThreadId()),
        )

        # Reset Lauf-Status
        self._stop_requested = False
        self._error_msg = None
        self._ended_in_error_state = False
        self._ended_in_finished_state = False
        self._current_phase = self.PHASE_NONE
        self._recipe_poses = []
        self._recipe_index = 0
        self._executed_poses = []
        self._planned_traj_list = []
        self._executed_traj_list = []

        self._drain_all_queues()

        # StateMachine im Worker-Thread erzeugen
        machine = QStatemachine(self)
        self._machine = machine

        # States mit Parent = machine erzeugen
        s_move_predisp = QState(machine)
        s_wait_predisp = QState(machine)
        s_move_recipe = QState(machine)
        s_wait_postdisp = QState(machine)
        s_move_retreat = QState(machine)
        s_move_home = QState(machine)
        s_finished = QFinalState(machine)
        s_error = QFinalState(machine)

        machine.setInitialState(s_move_predisp)

        # Transitions (Happy Path)
        s_move_predisp.addTransition(self._sig_predispense_move_done, s_wait_predisp)
        s_wait_predisp.addTransition(self._sig_predispense_wait_done, s_move_recipe)
        s_move_recipe.addTransition(self._sig_move_recipe_done, s_wait_postdisp)
        s_wait_postdisp.addTransition(self._sig_postdispense_wait_done, s_move_retreat)
        s_move_retreat.addTransition(self._sig_retreat_move_done, s_move_home)
        s_move_home.addTransition(self._sig_move_home_done, s_finished)

        # Fehler-Transitionen
        for st in (
            s_move_predisp,
            s_wait_predisp,
            s_move_recipe,
            s_wait_postdisp,
            s_move_retreat,
            s_move_home,
        ):
            st.addTransition(self._sig_error, s_error)

        # State-Callbacks (Logik)
        s_move_predisp.entered.connect(self._on_state_move_predispense)
        s_wait_predisp.entered.connect(self._on_state_wait_predispense)
        s_move_recipe.entered.connect(self._on_state_move_recipe)
        s_wait_postdisp.entered.connect(self._on_state_wait_postdispense)
        s_move_retreat.entered.connect(self._on_state_move_retreat)
        s_move_home.entered.connect(self._on_state_move_home)
        s_error.entered.connect(self._on_state_error)
        s_finished.entered.connect(self._on_state_finished)

        # UI-State-Namen
        s_move_predisp.entered.connect(lambda: self._emit_state("MOVE_PREDISPENSE"))
        s_wait_predisp.entered.connect(lambda: self._emit_state("WAIT_PREDISPENSE"))
        s_move_recipe.entered.connect(lambda: self._emit_state("MOVE_RECIPE"))
        s_wait_postdisp.entered.connect(lambda: self._emit_state("WAIT_POSTDISPENSE"))
        s_move_retreat.entered.connect(lambda: self._emit_state("MOVE_RETREAT"))
        s_move_home.entered.connect(lambda: self._emit_state("MOVE_HOME"))
        # ERROR/FINISHED werden explizit in den Handlern gesetzt

        # Polling-Timer für Queues (läuft im Worker-Thread)
        if self._poll_timer is not None:
            try:
                self._poll_timer.stop()
                self._poll_timer.deleteLater()
            except Exception:
                pass
            self._poll_timer = None

        self._poll_timer = QtCore.QTimer(self)
        self._poll_timer.setInterval(50)  # 50 ms
        self._poll_timer.timeout.connect(self._poll_queues)
        self._poll_timer.start()

        try:
            machine.start()
            _LOG.info("ProcessStatemachine.start: StateMachine gestartet.")
        except Exception as e:
            _LOG.exception("ProcessStatemachine.start: Exception: %s", e)
            if not self._error_msg:
                self._error_msg = str(e)
            self._signal_error(self._error_msg)

    @QtCore.pyqtSlot()
    def request_stop(self) -> None:
        """
        Stop-Flag setzen; die States prüfen regelmäßig _should_stop().
        """
        _LOG.info("ProcessStatemachine: request_stop() aufgerufen.")
        self._stop_requested = True

    @property
    def recipe(self) -> Recipe:
        return self._recipe

    def set_recipe(self, recipe: Recipe) -> None:
        """
        Nur verwenden, wenn kein Run aktiv ist.
        """
        _LOG.info("ProcessStatemachine.set_recipe: recipe=%s", getattr(recipe, "id", None))
        self._recipe = recipe

    # ------------------------------------------------------------------ #
    # Interne Helper
    # ------------------------------------------------------------------ #

    def _should_stop(self) -> bool:
        return self._stop_requested

    def _post_transition(self, sig: QtCore.pyqtSignal) -> None:
        """
        State-Wechsel async über QTimer.singleShot, um den Ablauf zu entkoppeln.
        Läuft im Worker-Thread.
        """
        if self._machine is None or not self._machine.isRunning():
            _LOG.warning("ProcessStatemachine: _post_transition, aber StateMachine läuft nicht mehr.")
            return
        QtCore.QTimer.singleShot(0, sig.emit)

    def _finish_state(self, success_sig: QtCore.pyqtSignal) -> None:
        """
        Gemeinsame Logik am Ende eines States:
        - Wenn Fehler oder Stop -> ERROR
        - sonst -> success_sig
        """
        if self._should_stop() and not self._error_msg:
            self._error_msg = "Prozess durch Benutzer gestoppt."
        if self._error_msg:
            _LOG.error("ProcessStatemachine: Fehler erkannt, wechsle in ERROR: %s", self._error_msg)
            self._post_transition(self._sig_error)
        else:
            self._post_transition(success_sig)

    def _signal_error(self, msg: Optional[str] = None) -> None:
        """
        Zentraler Fehlerpfad.
        """
        if msg and not self._error_msg:
            self._error_msg = msg
        if not self._error_msg:
            self._error_msg = "Unbekannter Prozessfehler."
        _LOG.error("ProcessStatemachine: _signal_error: %s", self._error_msg)
        self._post_transition(self._sig_error)

    def _disconnect_signals(self) -> None:
        """
        Externe Signalverbindungen lösen (für finalen Shutdown).
        Wird vom ProcessThread im Shutdown/memory-cleanup aufgerufen.
        """
        if self._rb_signals is not None:
            try:
                if hasattr(self._rb_signals, "tcpPoseChanged"):
                    self._rb_signals.tcpPoseChanged.disconnect(self._on_tcp_stream)
            except (TypeError, RuntimeError):
                pass
            self._rb_signals = None

        if self._motion_signals is not None:
            try:
                if hasattr(self._motion_signals, "motionResultChanged"):
                    self._motion_signals.motionResultChanged.disconnect(self._on_motion_result)
            except (TypeError, RuntimeError):
                pass

            try:
                if hasattr(self._motion_signals, "plannedTrajectoryChanged"):
                    self._motion_signals.plannedTrajectoryChanged.disconnect(self._on_planned_traj)
            except (TypeError, RuntimeError):
                pass

            try:
                if hasattr(self._motion_signals, "executedTrajectoryChanged"):
                    self._motion_signals.executedTrajectoryChanged.disconnect(self._on_executed_traj)
            except (TypeError, RuntimeError):
                pass

            self._motion_signals = None

        if self._log_handler is not None:
            try:
                _LOG.removeHandler(self._log_handler)
            except Exception:
                pass
            self._log_handler = None

    def _stop_machine_and_quit(self) -> None:
        """
        StateMachine + Polling-Timer stoppen.

        Der QThread selbst wird NICHT hier beendet – das macht der ProcessThread.
        """
        if self._poll_timer is not None:
            try:
                self._poll_timer.stop()
                self._poll_timer.deleteLater()
            except Exception:
                pass
            self._poll_timer = None

        m = self._machine
        if m is not None and m.isRunning():
            try:
                _LOG.info("ProcessStatemachine: stoppe StateMachine.")
                m.stop()
            except Exception:
                _LOG.exception("ProcessStatemachine: Fehler beim Stoppen der StateMachine.")

        self._machine = None

    # ------------------------------------------------------------------ #
    # Motion-Result Handling
    # ------------------------------------------------------------------ #

    @QtCore.pyqtSlot(str)
    def _on_motion_result(self, text: str) -> None:
        res = (text or "").strip()
        if not res:
            return

        _LOG.info("ProcessStatemachine: motion_result empfangen: %r", res)

        if self._machine is None or not self._machine.isRunning():
            _LOG.info("ProcessStatemachine: motion_result ignoriert (StateMachine läuft nicht mehr).")
            return

        if self._current_phase == self.PHASE_NONE:
            _LOG.info("ProcessStatemachine: motion_result ignoriert (keine aktive Motion-Phase).")
            return

        if self._should_stop():
            if not self._error_msg:
                self._error_msg = "Prozess durch Benutzer gestoppt."
            self._signal_error(self._error_msg)
            return

        res_u = res.upper()

        # PLANNED... kommt zuerst, danach EXECUTED/ERROR
        if res_u.startswith("PLANNED"):
            _LOG.info("ProcessStatemachine: Motion-Status PLANNED..., warte auf EXECUTED/ERROR.")
            return

        # EXECUTED -> Success-Pfad der aktuellen Phase
        if res_u.startswith("EXECUTED:OK") or res_u.startswith("EXECUTED"):
            self._handle_motion_success()
            return

        # Alles mit ERROR/FAILED/ABORTED/CANCELLED/UNKNOWN = Fehler
        if res_u.startswith(("ERROR", "FAILED", "ABORTED", "CANCELLED", "UNKNOWN")):
            if not self._error_msg:
                self._error_msg = res
            _LOG.error("ProcessStatemachine: Motion-Fehler: %s", self._error_msg)
            self._signal_error(self._error_msg)
            return

        # Unerwarteter String -> ebenfalls als Fehler behandeln
        if not self._error_msg:
            self._error_msg = res
        _LOG.error(
            "ProcessStatemachine: Unerwartetes Motion-Result, behandle als Fehler: %s",
            self._error_msg,
        )
        self._signal_error(self._error_msg)

    def _handle_motion_success(self) -> None:
        """
        Wird aufgerufen, wenn EXECUTED:OK (oder EXECUTED...) rein kommt.
        Entscheidet anhand von self._current_phase, wie es weitergeht.
        """
        phase = self._current_phase
        _LOG.info("ProcessStatemachine: Motion EXECUTED:OK in Phase %s", phase)

        # 1) Predispense-Fahrt fertig -> Warte-State
        if phase == self.PHASE_MOVE_PREDISPENSE:
            self._current_phase = self.PHASE_NONE
            self._post_transition(self._sig_predispense_move_done)
            return

        # 2) Rezept-Fahrt (Zwischenpunkte)
        if phase == self.PHASE_MOVE_RECIPE:
            if not self._recipe_poses:
                _LOG.error("ProcessStatemachine: MOVE_RECIPE: _recipe_poses leer – Rezept inkonsistent.")
                if not self._error_msg:
                    self._error_msg = "Rezept inkonsistent: keine Posen für MOVE_RECIPE."
                self._post_transition(self._sig_error)
                return

            self._recipe_index += 1
            if self._recipe_index >= len(self._recipe_poses) - 1:
                _LOG.info("ProcessStatemachine: MOVE_RECIPE: alle Zwischen-Posen abgefahren.")
                self._current_phase = self.PHASE_NONE
                self._post_transition(self._sig_move_recipe_done)
                return

            pose = self._recipe_poses[self._recipe_index]
            _LOG.info(
                "ProcessStatemachine: MOVE_RECIPE: nächster Wegpunkt %d/%d.",
                self._recipe_index,
                len(self._recipe_poses) - 2,
            )
            self._send_motion_pose(pose, label=f"recipe_{self._recipe_index}")
            return

        # 3) Retreat-Fahrt fertig -> Home
        if phase == self.PHASE_MOVE_RETREAT:
            self._current_phase = self.PHASE_NONE
            self._post_transition(self._sig_retreat_move_done)
            return

        # 4) Home-Fahrt fertig -> FINISHED
        if phase == self.PHASE_MOVE_HOME:
            self._current_phase = self.PHASE_NONE
            self._post_transition(self._sig_move_home_done)
            return

        # Fallback: nur Phase zurücksetzen
        self._current_phase = self.PHASE_NONE

    # ------------------------------------------------------------------ #
    # TCP-Stream & Traj-Queues
    # ------------------------------------------------------------------ #

    @QtCore.pyqtSlot(object)
    def _on_tcp_stream(self, msg: object) -> None:
        """
        TCP-Posen werden zunächst nur in eine threadsichere Queue gelegt.
        Die eigentliche Filterung (Phase, Fehler, Stop) passiert im Worker-Thread
        im _poll_queues().
        """
        if not isinstance(msg, PoseStamped):
            return

        snap = PoseStamped()
        snap.header.frame_id = msg.header.frame_id
        snap.header.stamp = msg.header.stamp
        snap.pose = msg.pose

        try:
            self._tcp_pose_queue.put_nowait(snap)
        except Exception as e:
            _LOG.exception("ProcessStatemachine: _on_tcp_stream: Queue-Fehler: %s", e)

    @QtCore.pyqtSlot(object)
    def _on_planned_traj(self, msg: object) -> None:
        """
        MotionBridge.plannedTrajectoryChanged -> Queue.
        """
        if not isinstance(msg, RobotTrajectoryMsg):
            return

        try:
            self._planned_traj_queue.put_nowait(msg)
        except Exception as e:
            _LOG.exception("ProcessStatemachine: _on_planned_traj: Queue-Fehler: %s", e)

    @QtCore.pyqtSlot(object)
    def _on_executed_traj(self, msg: object) -> None:
        """
        MotionBridge.executedTrajectoryChanged -> Queue.
        """
        if not isinstance(msg, RobotTrajectoryMsg):
            return

        try:
            self._executed_traj_queue.put_nowait(msg)
        except Exception as e:
            _LOG.exception("ProcessStatemachine: _on_executed_traj: Queue-Fehler: %s", e)

    def _poll_queues(self) -> None:
        """
        Läuft im Worker-Thread (QTimer). Holt TCP-Posen und Trajs aus den Queues
        und hängt sie – sofern sinnvoll – an die internen Listen an.
        """
        if self._machine is None or not self._machine.isRunning():
            self._drain_all_queues()
            return

        if self._should_stop() or self._error_msg:
            self._drain_all_queues()
            return

        active_phases = (
            self.PHASE_MOVE_PREDISPENSE,
            self.PHASE_MOVE_RECIPE,
            self.PHASE_MOVE_RETREAT,
            self.PHASE_MOVE_HOME,
        )

        # TCP-Posen
        while True:
            try:
                snap = self._tcp_pose_queue.get_nowait()
            except Empty:
                break

            if self._current_phase in active_phases:
                self._executed_poses.append(snap)

        # geplante Trajs
        while True:
            try:
                traj = self._planned_traj_queue.get_nowait()
            except Empty:
                break

            if self._current_phase in active_phases:
                self._planned_traj_list.append(traj)
                _LOG.info(
                    "ProcessStatemachine: _poll_queues: geplantes Segment #%d (points=%d).",
                    len(self._planned_traj_list),
                    len(traj.joint_trajectory.points),
                )

        # ausgeführte Trajs
        while True:
            try:
                traj = self._executed_traj_queue.get_nowait()
            except Empty:
                break

            if self._current_phase in active_phases:
                self._executed_traj_list.append(traj)
                _LOG.info(
                    "ProcessStatemachine: _poll_queues: ausgeführtes Segment #%d (points=%d).",
                    len(self._executed_traj_list),
                    len(traj.joint_trajectory.points),
                )

    def _drain_all_queues(self) -> None:
        """
        Leert alle internen Queues, ohne etwas zu speichern.
        Verhindert Memory-Leaks, wenn Maschine gestoppt oder im Fehler ist.
        """
        for q in (self._tcp_pose_queue, self._planned_traj_queue, self._executed_traj_queue):
            while True:
                try:
                    q.get_nowait()
                except Empty:
                    break

    # ------------------------------------------------------------------ #
    # Utils
    # ------------------------------------------------------------------ #

    def _emit_state(self, name: str) -> None:
        _LOG.info("ProcessStatemachine: State gewechselt zu %s", name)
        try:
            self.stateChanged.emit(name)
        except Exception:
            pass

    def _get_recipe_poses(self) -> List[PoseStamped]:
        """
        Extrahiert die Posen aus recipe.paths_compiled['sides'][first_side]['poses_quat'].

        Keine stillen Fallbacks:
          - Wenn leer/nicht vorhanden -> [] + Log-Warnung
          - Die States behandeln [] als Fehler und setzen _error_msg.
        """
        try:
            pc = self._recipe.paths_compiled or {}
        except Exception:
            _LOG.error("ProcessStatemachine: _get_recipe_poses: recipe.paths_compiled nicht verfügbar.")
            return []

        sides = pc.get("sides") or {}
        if not isinstance(sides, dict) or not sides:
            _LOG.warning("ProcessStatemachine: _get_recipe_poses: paths_compiled.sides ist leer.")
            return []

        first_side = next(iter(sides.keys()))
        sdata = sides[first_side] or {}
        poses_quat = sdata.get("poses_quat") or []

        if not poses_quat:
            _LOG.warning(
                "ProcessStatemachine: _get_recipe_poses: erste Side '%s' hat keine poses_quat.",
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
            # Orientierung hier neutral – Pfadbewertung nutzt nur Position
            ps.pose.orientation.x = 0.0
            ps.pose.orientation.y = 0.0
            ps.pose.orientation.z = 0.0
            ps.pose.orientation.w = 1.0
            out.append(ps)

        _LOG.info(
            "ProcessStatemachine: _get_recipe_poses: benutze erste side='%s' mit %d Posen (frame=%s).",
            first_side, len(out), frame_id,
        )
        return out

    def _get_home_pose(self) -> Optional[PoseStamped]:
        """
        Holt die Home-Pose, zuerst aus PoseBridge, dann aus poses_state.
        Keine generierte Fallback-Pose – wenn nichts da ist, ist das ein Fehler.
        """
        if self._pb is not None:
            try:
                pose = self._pb.get_last_home_pose()
                if isinstance(pose, PoseStamped):
                    return pose
            except Exception:
                pass

        if self._poses_state is not None:
            try:
                pose2 = self._poses_state.home()
                if isinstance(pose2, PoseStamped):
                    return pose2
            except Exception:
                pass

        _LOG.warning("ProcessStatemachine: _get_home_pose: keine Home-Pose verfügbar.")
        return None

    def _send_motion_pose(self, pose: PoseStamped, label: str) -> None:
        """
        Schickt eine Pose über die MotionBridge in Richtung motion-Node.
        """
        if pose is None:
            msg = f"Interner Fehler: Zielpose ist None (label={label})."
            _LOG.error("ProcessStatemachine: _send_motion_pose: %s", msg)
            self._signal_error(msg)
            return

        if self._motion is None or self._motion_signals is None:
            msg = f"MotionBridge fehlt für label={label}."
            _LOG.error("ProcessStatemachine: _send_motion_pose: %s", msg)
            self._signal_error(msg)
            return

        try:
            _LOG.info(
                "ProcessStatemachine: moveToPoseRequested(%s) via MotionBridge-Signal wird emittiert.",
                label,
            )
            self._motion_signals.moveToPoseRequested.emit(pose)
        except Exception as e:
            msg = f"moveToPoseRequested({label}) emit failed: {e}"
            _LOG.exception("ProcessStatemachine: %s", msg)
            self._signal_error(msg)

    # ------------------------------------------------------------------ #
    # State-Callbacks
    # ------------------------------------------------------------------ #

    def _on_state_move_predispense(self) -> None:
        _LOG.info("ProcessStatemachine: ENTER MOVE_PREDISPENSE")

        if self._error_msg or self._should_stop():
            _LOG.info("ProcessStatemachine: MOVE_PREDISPENSE: Fehler/Stop vor Start erkannt.")
            self._finish_state(self._sig_predispense_move_done)
            _LOG.info("ProcessStatemachine: LEAVE MOVE_PREDISPENSE")
            return

        poses = self._get_recipe_poses()
        self._recipe_poses = poses
        self._recipe_index = 0

        if not poses:
            # Kein Fallback mehr: ohne Posen ist das Rezept ungültig
            msg = "Rezept enthält keine gültigen Posen für Predispense."
            _LOG.error("ProcessStatemachine: MOVE_PREDISPENSE: %s", msg)
            if not self._error_msg:
                self._error_msg = msg
            self._finish_state(self._sig_predispense_move_done)
            _LOG.info("ProcessStatemachine: LEAVE MOVE_PREDISPENSE (ERROR)")
            return

        first = poses[0]
        _LOG.info("ProcessStatemachine: MOVE_PREDISPENSE: fahre Pose[0].")
        self._current_phase = self.PHASE_MOVE_PREDISPENSE
        self._send_motion_pose(first, label="predispense")

        _LOG.info("ProcessStatemachine: LEAVE MOVE_PREDISPENSE (warte auf EXECUTED:OK).")

    def _on_state_wait_predispense(self) -> None:
        _LOG.info("ProcessStatemachine: ENTER WAIT_PREDISPENSE")

        if not self._error_msg:
            pre_t = 0.0
            try:
                pre_t = float(self._recipe.globals.predispose_time or 0.0)
            except Exception:
                pre_t = 0.0

            _LOG.info("ProcessStatemachine: WAIT_PREDISPENSE: predispense_time=%.3f s", pre_t)

            ms_total = max(0, int(pre_t * 1000))
            elapsed = 0
            step = 50  # 50 ms

            while elapsed < ms_total:
                if self._should_stop():
                    if not self._error_msg:
                        self._error_msg = "Prozess durch Benutzer gestoppt."
                    break
                QtCore.QThread.msleep(step)
                elapsed += step

        _LOG.info("ProcessStatemachine: LEAVE WAIT_PREDISPENSE")
        self._finish_state(self._sig_predispense_wait_done)

    def _on_state_move_recipe(self) -> None:
        _LOG.info("ProcessStatemachine: ENTER MOVE_RECIPE")

        if self._error_msg or self._should_stop():
            _LOG.info("ProcessStatemachine: MOVE_RECIPE: Fehler/Stop vor Start erkannt.")
            self._finish_state(self._sig_move_recipe_done)
            _LOG.info("ProcessStatemachine: LEAVE MOVE_RECIPE")
            return

        if not self._recipe_poses:
            self._recipe_poses = self._get_recipe_poses()
            self._recipe_index = 0

        n = len(self._recipe_poses)
        _LOG.info("ProcessStatemachine: MOVE_RECIPE: Anzahl Posen=%d", n)

        # Keine „stummen“ Fallbacks mehr:
        # Für eine sinnvolle Trajektorie brauchen wir mind. 3 Posen (Start, mind. 1 Innenpunkt, Ende)
        if n <= 2:
            msg = "Rezept enthält zu wenige Posen für MOVE_RECIPE (mind. 3 erforderlich)."
            _LOG.error("ProcessStatemachine: MOVE_RECIPE: %s", msg)
            if not self._error_msg:
                self._error_msg = msg
            self._finish_state(self._sig_move_recipe_done)
            _LOG.info("ProcessStatemachine: LEAVE MOVE_RECIPE (ERROR)")
            return

        self._recipe_index = 1
        pose = self._recipe_poses[self._recipe_index]
        _LOG.info("ProcessStatemachine: MOVE_RECIPE: starte mit Pose[1] von %d.", n - 2)
        self._current_phase = self.PHASE_MOVE_RECIPE
        self._send_motion_pose(pose, label=f"recipe_{self._recipe_index}")

        _LOG.info("ProcessStatemachine: LEAVE MOVE_RECIPE (warte auf EXECUTED:OK / Folgewaypoints).")

    def _on_state_wait_postdispense(self) -> None:
        _LOG.info("ProcessStatemachine: ENTER WAIT_POSTDISPENSE")

        if not self._error_msg:
            post_t = 0.0
            try:
                post_t = float(self._recipe.globals.postdispense_time or 0.0)
            except Exception:
                post_t = 0.0

            _LOG.info("ProcessStatemachine: WAIT_POSTDISPENSE: postdispense_time=%.3f s", post_t)

            ms_total = max(0, int(post_t * 1000))
            elapsed = 0
            step = 50

            while elapsed < ms_total:
                if self._should_stop():
                    if not self._error_msg:
                        self._error_msg = "Prozess durch Benutzer gestoppt."
                    break
                QtCore.QThread.msleep(step)
                elapsed += step

        _LOG.info("ProcessStatemachine: LEAVE WAIT_POSTDISPENSE")
        self._finish_state(self._sig_postdispense_wait_done)

    def _on_state_move_retreat(self) -> None:
        _LOG.info("ProcessStatemachine: ENTER MOVE_RETREAT")

        if self._error_msg or self._should_stop():
            _LOG.info("ProcessStatemachine: MOVE_RETREAT: Fehler/Stop vor Start erkannt.")
            self._finish_state(self._sig_retreat_move_done)
            _LOG.info("ProcessStatemachine: LEAVE MOVE_RETREAT")
            return

        if not self._recipe_poses:
            self._recipe_poses = self._get_recipe_poses()
            self._recipe_index = 0

        n = len(self._recipe_poses)
        if n < 2:
            msg = "Rezept enthält zu wenige Posen für MOVE_RETREAT (mind. 2 erforderlich)."
            _LOG.error("ProcessStatemachine: MOVE_RETREAT: %s", msg)
            if not self._error_msg:
                self._error_msg = msg
            self._finish_state(self._sig_retreat_move_done)
            _LOG.info("ProcessStatemachine: LEAVE MOVE_RETREAT (ERROR)")
            return

        last = self._recipe_poses[-1]
        _LOG.info("ProcessStatemachine: MOVE_RETREAT: fahre Pose[-1].")
        self._current_phase = self.PHASE_MOVE_RETREAT
        self._send_motion_pose(last, label="retreat")

        _LOG.info("ProcessStatemachine: LEAVE MOVE_RETREAT (warte auf EXECUTED:OK).")

    def _on_state_move_home(self) -> None:
        _LOG.info("ProcessStatemachine: ENTER MOVE_HOME")

        if self._error_msg or self._should_stop():
            _LOG.info("ProcessStatemachine: MOVE_HOME: Fehler/Stop vor Start erkannt.")
            self._finish_state(self._sig_move_home_done)
            _LOG.info("ProcessStatemachine: LEAVE MOVE_HOME")
            return

        home = self._get_home_pose()
        if home is None:
            msg = "Keine Home-Pose verfügbar – MOVE_HOME kann nicht ausgeführt werden."
            _LOG.error("ProcessStatemachine: MOVE_HOME: %s", msg)
            if not self._error_msg:
                self._error_msg = msg
            _LOG.info("ProcessStatemachine: LEAVE MOVE_HOME (ERROR)")
            self._finish_state(self._sig_move_home_done)
            return

        _LOG.info("ProcessStatemachine: MOVE_HOME: sende Home-Pose via MotionBridge.")
        self._current_phase = self.PHASE_MOVE_HOME
        self._send_motion_pose(home, label="home")

        _LOG.info("ProcessStatemachine: LEAVE MOVE_HOME (warte auf EXECUTED:OK).")

    def _on_state_error(self) -> None:
        _LOG.info("ProcessStatemachine: ENTER ERROR")
        self._ended_in_error_state = True
        self._emit_state("ERROR")

        msg = self._error_msg or "Unbekannter Prozessfehler."
        _LOG.error("ProcessStatemachine: notifyError(%s) im ERROR-State", msg)
        self.notifyError.emit(msg)

        _LOG.info(
            "ProcessStatemachine: run beendet (ERROR), stop_requested=%s, "
            "executed_poses=%d, planned_traj_segments=%d, executed_traj_segments=%d",
            self._stop_requested,
            len(self._executed_poses),
            len(self._planned_traj_list),
            len(self._executed_traj_list),
        )

        self._stop_machine_and_quit()

    def _on_state_finished(self) -> None:
        _LOG.info("ProcessStatemachine: ENTER FINISHED")
        self._ended_in_finished_state = True
        self._emit_state("FINISHED")

        poses_copy: List[PoseStamped] = list(self._executed_poses)

        _LOG.info(
            "ProcessStatemachine: notifyFinished() im FINISHED-State, %d Posen (TCP executed), "
            "%d geplante Trajs, %d ausgeführte Trajs.",
            len(poses_copy),
            len(self._planned_traj_list),
            len(self._executed_traj_list),
        )

        result: Dict[str, Any] = {
            "poses": poses_copy,
            "planned_traj": list(self._planned_traj_list),
            "executed_traj": list(self._executed_traj_list),
        }

        self.notifyFinished.emit(result)

        _LOG.info(
            "ProcessStatemachine: run beendet (FINISHED), stop_requested=%s, "
            "executed_poses=%d, planned_traj_segments=%d, executed_traj_segments=%d",
            self._stop_requested,
            len(self._executed_poses),
            len(self._planned_traj_list),
            len(self._executed_traj_list),
        )

        self._stop_machine_and_quit()
