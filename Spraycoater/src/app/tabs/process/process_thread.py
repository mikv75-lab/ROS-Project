# -*- coding: utf-8 -*-
# File: tabs/process/process_thread.py
from __future__ import annotations

from typing import Optional, List, Dict, Any
from queue import Queue, Empty

import logging
from PyQt6 import QtCore
from PyQt6.QtStateMachine import QStateMachine, QState, QFinalState
from PyQt6.QtCore import QCoreApplication  # <--- NEU: für aboutToQuit-Hook

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg

from app.model.recipe.recipe import Recipe

_LOG = logging.getLogger("app.tabs.process.thread")


class _QtSignalHandler(logging.Handler):
    """
    Leitet Log-Messages dieses Loggers über ein Qt-Signal nach außen,
    damit sie im UI angezeigt werden können.
    """
    def __init__(self, owner: "ProcessWorker") -> None:
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


class ProcessWorker(QtCore.QObject):
    """
    Reiner QObject-Worker, der im Hintergrund-Thread läuft.
    Enthält:
      - QStateMachine mit den Prozess-States
      - Logging
      - Aufzeichnung von TCP-Posen und MoveIt-Trajektorien

    Threading:
      - Dieses Objekt wird via moveToThread(...) in einen QThread verschoben.
      - Alle Slots (_on_motion_result, _on_tcp_stream, ...) laufen im Worker-Thread.
      - QStateMachine + QTimer leben im selben Thread -> thread-sicher.

    Am Ende sendet notifyFinished ein Dict:
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
        self._machine: Optional[QStateMachine] = None

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
                    # Cross-Thread -> Qt macht QueuedConnection, Slot läuft im Worker-Thread
                    self._motion_signals.motionResultChanged.connect(self._on_motion_result)
                    _LOG.info(
                        "ProcessWorker: MotionBridge gefunden (%s), motionResultChanged verbunden.",
                        type(self._motion).__name__,
                    )

                if hasattr(self._motion_signals, "plannedTrajectoryChanged"):
                    self._motion_signals.plannedTrajectoryChanged.connect(self._on_planned_traj)
                    _LOG.info(
                        "ProcessWorker: plannedTrajectoryChanged mit _on_planned_traj verbunden."
                    )

                if hasattr(self._motion_signals, "executedTrajectoryChanged"):
                    self._motion_signals.executedTrajectoryChanged.connect(self._on_executed_traj)
                    _LOG.info(
                        "ProcessWorker: executedTrajectoryChanged mit _on_executed_traj verbunden."
                    )
            except Exception as e:
                _LOG.exception("ProcessWorker: Konnte Motion-Signale nicht verbinden: %s", e)
        else:
            _LOG.error(
                "ProcessWorker: MotionBridge oder deren Signale sind None – "
                "Bewegungen können nicht ausgeführt/geloggt werden."
            )

        # RobotBridge tcpPoseChanged verbinden (für Trajektorien-Logging)
        if self._rb is not None:
            try:
                sig_r = getattr(self._rb, "signals", None)
                if sig_r is None:
                    _LOG.warning("ProcessWorker: RobotBridge.signals ist None – kein TCP-Logging.")
                else:
                    self._rb_signals = sig_r
                    if hasattr(sig_r, "tcpPoseChanged"):
                        sig_r.tcpPoseChanged.connect(self._on_tcp_stream)
                        _LOG.info(
                            "ProcessWorker: RobotBridge tcpPoseChanged verbunden – "
                            "gefahrene Trajektorie wird mitgeloggt (Queue)."
                        )
                    else:
                        _LOG.warning(
                            "ProcessWorker: RobotBridge.signals.tcpPoseChanged fehlt – "
                            "kein Live-Trajektorien-Logging möglich."
                        )
            except Exception:
                _LOG.exception("ProcessWorker: Fehler beim Verbinden von tcpPoseChanged.")
        else:
            _LOG.warning("ProcessWorker: RobotBridge ist None – kein TCP-Logging möglich.")

        _LOG.info(
            "ProcessWorker init: recipe=%s, rb=%s, motion=%s",
            getattr(recipe, "id", None),
            type(self._rb).__name__ if self._rb is not None else "None",
            type(self._motion).__name__ if self._motion is not None else "None",
        )

    # ------------------------------------------------------------------ #
    # Öffentliche API (aus GUI-Thread via QueuedConnection aufrufen)
    # ------------------------------------------------------------------ #

    @QtCore.pyqtSlot()
    def start(self) -> None:
        """
        Startet einen neuen Prozesslauf.
        Wird über QMetaObject.invokeMethod(..., QueuedConnection) aufgerufen.
        Läuft im Worker-Thread.
        """
        _LOG.info(
            "ProcessWorker.start: gestartet (Thread=%r)",
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
        machine = QStateMachine(self)
        self._machine = machine

        s_move_predisp = QState(machine)
        s_wait_predisp = QState(machine)
        s_move_recipe = QState(machine)
        s_wait_postdisp = QState(machine)
        s_move_retreat = QState(machine)
        s_move_home = QState(machine)
        s_finished = QFinalState(machine)
        s_error = QFinalState(machine)

        machine.addState(s_move_predisp)
        machine.addState(s_wait_predisp)
        machine.addState(s_move_recipe)
        machine.addState(s_wait_postdisp)
        machine.addState(s_move_retreat)
        machine.addState(s_move_home)
        machine.addState(s_finished)
        machine.addState(s_error)
        machine.setInitialState(s_move_predisp)

        # Transitions
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

        # State-Enter-Handler
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
            _LOG.info("ProcessWorker.start: StateMachine gestartet.")
        except Exception as e:
            _LOG.exception("ProcessWorker.start: Exception: %s", e)
            if not self._error_msg:
                self._error_msg = str(e)
            self._signal_error(self._error_msg)

    @QtCore.pyqtSlot()
    def request_stop(self) -> None:
        """
        Stop-Flag setzen; die States prüfen regelmäßig _should_stop().
        """
        _LOG.info("ProcessWorker: request_stop() aufgerufen.")
        self._stop_requested = True

    @property
    def recipe(self) -> Recipe:
        return self._recipe

    def set_recipe(self, recipe: Recipe) -> None:
        """
        Rezept nur wechseln, wenn der Thread nicht läuft.
        (Die Prüfung, ob der Thread läuft, macht der Wrapper.)
        """
        _LOG.info("ProcessWorker.set_recipe: recipe=%s", getattr(recipe, "id", None))
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
            _LOG.warning("ProcessWorker: _post_transition, aber StateMachine läuft nicht mehr.")
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
            _LOG.error("ProcessWorker: Fehler erkannt, wechsle in ERROR: %s", self._error_msg)
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
        _LOG.error("ProcessWorker: _signal_error: %s", self._error_msg)
        self._post_transition(self._sig_error)

    def _disconnect_signals(self) -> None:
        """
        Externe Signalverbindungen lösen (vor allem für Shutdown).
        Wird i.d.R. nur einmal am Ende des Lebenszyklus gebraucht.
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
        StateMachine stoppen, Polling-Timer stoppen, Signale trennen,
        und den zugehörigen Thread-Eventloop beenden.
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
                _LOG.info("ProcessWorker: stoppe StateMachine.")
                m.stop()
            except Exception:
                _LOG.exception("ProcessWorker: Fehler beim Stoppen der StateMachine.")

        self._machine = None

        self._disconnect_signals()

        try:
            th = self.thread()
            if isinstance(th, QtCore.QThread):
                _LOG.info("ProcessWorker: thread().quit() aufrufen.")
                th.quit()
        except Exception:
            _LOG.exception("ProcessWorker: Fehler bei thread().quit().")

    # ------------------------------------------------------------------ #
    # Motion-Result Handling
    # ------------------------------------------------------------------ #

    @QtCore.pyqtSlot(str)
    def _on_motion_result(self, text: str) -> None:
        res = (text or "").strip()
        if not res:
            return

        _LOG.info("ProcessWorker: motion_result empfangen: %r", res)

        if self._machine is None or not self._machine.isRunning():
            _LOG.info("ProcessWorker: motion_result ignoriert (StateMachine läuft nicht mehr).")
            return

        if self._current_phase == self.PHASE_NONE:
            _LOG.info("ProcessWorker: motion_result ignoriert (keine aktive Motion-Phase).")
            return

        if self._should_stop():
            if not self._error_msg:
                self._error_msg = "Prozess durch Benutzer gestoppt."
            self._signal_error(self._error_msg)
            return

        res_u = res.upper()

        # PLANNED... kommt zuerst, danach EXECUTED/ERROR
        if res_u.startswith("PLANNED"):
            _LOG.info("ProcessWorker: Motion-Status PLANNED..., warte auf EXECUTED/ERROR.")
            return

        # EXECUTED -> Success-Pfad der aktuellen Phase
        if res_u.startswith("EXECUTED:OK") or res_u.startswith("EXECUTED"):
            self._handle_motion_success()
            return

        # Alles mit ERROR/FAILED/ABORTED/CANCELLED/UNKNOWN = Fehler
        if res_u.startswith(("ERROR", "FAILED", "ABORTED", "CANCELLED", "UNKNOWN")):
            if not self._error_msg:
                self._error_msg = res
            _LOG.error("ProcessWorker: Motion-Fehler: %s", self._error_msg)
            self._signal_error(self._error_msg)
            return

        # Unerwarteter String -> ebenfalls als Fehler behandeln
        if not self._error_msg:
            self._error_msg = res
        _LOG.error(
            "ProcessWorker: Unerwartetes Motion-Result, behandle als Fehler: %s",
            self._error_msg,
        )
        self._signal_error(self._error_msg)

    def _handle_motion_success(self) -> None:
        """
        Wird aufgerufen, wenn EXECUTED:OK (oder EXECUTED...) rein kommt.
        Entscheidet anhand von self._current_phase, wie es weitergeht.
        """
        phase = self._current_phase
        _LOG.info("ProcessWorker: Motion EXECUTED:OK in Phase %s", phase)

        # 1) Predispense-Fahrt fertig -> Warte-State
        if phase == self.PHASE_MOVE_PREDISPENSE:
            self._current_phase = self.PHASE_NONE
            self._post_transition(self._sig_predispense_move_done)
            return

        # 2) Rezept-Fahrt (Zwischenpunkte)
        if phase == self.PHASE_MOVE_RECIPE:
            if not self._recipe_poses:
                _LOG.warning("ProcessWorker: MOVE_RECIPE: keine Posen mehr, beende State.")
                self._current_phase = self.PHASE_NONE
                self._post_transition(self._sig_move_recipe_done)
                return

            self._recipe_index += 1
            if self._recipe_index >= len(self._recipe_poses) - 1:
                _LOG.info("ProcessWorker: MOVE_RECIPE: alle Zwischen-Posen abgefahren.")
                self._current_phase = self.PHASE_NONE
                self._post_transition(self._sig_move_recipe_done)
                return

            pose = self._recipe_poses[self._recipe_index]
            _LOG.info(
                "ProcessWorker: MOVE_RECIPE: nächster Wegpunkt %d/%d.",
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

        # Fallback (nur Phase zurücksetzen, keine weiteren Aktionen)
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
            _LOG.exception("ProcessWorker: _on_tcp_stream: Queue-Fehler: %s", e)

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
            _LOG.exception("ProcessWorker: _on_planned_traj: Queue-Fehler: %s", e)

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
            _LOG.exception("ProcessWorker: _on_executed_traj: Queue-Fehler: %s", e)

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
                    "ProcessWorker: _poll_queues: geplantes Segment #%d (points=%d).",
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
                    "ProcessWorker: _poll_queues: ausgeführtes Segment #%d (points=%d).",
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
        _LOG.info("ProcessWorker: State gewechselt zu %s", name)
        try:
            self.stateChanged.emit(name)
        except Exception:
            pass

    def _get_recipe_poses(self) -> List[PoseStamped]:
        """
        Extrahiert die Posen aus recipe.paths_compiled['sides'][first_side]['poses_quat'].

        Wichtig:
          - Keine Fallbacks
          - Wenn leer/nicht vorhanden -> [] und Log-Warnung
        """
        try:
            pc = self._recipe.paths_compiled or {}
        except Exception:
            _LOG.error("ProcessWorker: _get_recipe_poses: recipe.paths_compiled nicht verfügbar.")
            return []

        sides = pc.get("sides") or {}
        if not isinstance(sides, dict) or not sides:
            _LOG.warning("ProcessWorker: _get_recipe_poses: paths_compiled.sides ist leer.")
            return []

        first_side = next(iter(sides.keys()))
        sdata = sides[first_side] or {}
        poses_quat = sdata.get("poses_quat") or []

        if not poses_quat:
            _LOG.warning(
                "ProcessWorker: _get_recipe_poses: erste Side '%s' hat keine poses_quat.",
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
            # Orientierung hier neutral lassen – Pfadbewertung nutzt nur Position
            ps.pose.orientation.x = 0.0
            ps.pose.orientation.y = 0.0
            ps.pose.orientation.z = 0.0
            ps.pose.orientation.w = 1.0
            out.append(ps)

        _LOG.info(
            "ProcessWorker: _get_recipe_poses: benutze erste side='%s' mit %d Posen (frame=%s).",
            first_side, len(out), frame_id,
        )
        return out

    def _get_home_pose(self) -> Optional[PoseStamped]:
        """
        Holt die Home-Pose, zuerst aus PoseBridge, dann aus poses_state.
        Keine Fallback-Posen generieren.
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

        _LOG.warning("ProcessWorker: _get_home_pose: keine Home-Pose verfügbar.")
        return None

    def _send_motion_pose(self, pose: PoseStamped, label: str) -> None:
        """
        Schickt eine Pose über die MotionBridge in Richtung motion-Node.
        """
        if pose is None:
            msg = f"Interner Fehler: Zielpose ist None (label={label})."
            _LOG.error("ProcessWorker: _send_motion_pose: %s", msg)
            self._signal_error(msg)
            return

        if self._motion is None or self._motion_signals is None:
            msg = f"MotionBridge fehlt für label={label}."
            _LOG.error("ProcessWorker: _send_motion_pose: %s", msg)
            self._signal_error(msg)
            return

        try:
            _LOG.info(
                "ProcessWorker: moveToPoseRequested(%s) via MotionBridge-Signal wird emittiert.",
                label,
            )
            self._motion_signals.moveToPoseRequested.emit(pose)
        except Exception as e:
            msg = f"moveToPoseRequested({label}) emit failed: {e}"
            _LOG.exception("ProcessWorker: %s", msg)
            self._signal_error(msg)

    # ------------------------------------------------------------------ #
    # State-Callbacks
    # ------------------------------------------------------------------ #

    def _on_state_move_predispense(self) -> None:
        _LOG.info("ProcessWorker: ENTER MOVE_PREDISPENSE")

        if self._error_msg or self._should_stop():
            _LOG.info("ProcessWorker: MOVE_PREDISPENSE: Fehler/Stop vor Start erkannt.")
            self._finish_state(self._sig_predispense_move_done)
            _LOG.info("ProcessWorker: LEAVE MOVE_PREDISPENSE")
            return

        poses = self._get_recipe_poses()
        self._recipe_poses = poses
        self._recipe_index = 0

        if not poses:
            _LOG.warning("ProcessWorker: MOVE_PREDISPENSE: keine Posen -> direkt weiter.")
            _LOG.info("ProcessWorker: LEAVE MOVE_PREDISPENSE")
            self._finish_state(self._sig_predispense_move_done)
            return

        first = poses[0]
        _LOG.info("ProcessWorker: MOVE_PREDISPENSE: fahre Pose[0].")
        self._current_phase = self.PHASE_MOVE_PREDISPENSE
        self._send_motion_pose(first, label="predispense")

        _LOG.info("ProcessWorker: LEAVE MOVE_PREDISPENSE (warte auf EXECUTED:OK).")

    def _on_state_wait_predispense(self) -> None:
        _LOG.info("ProcessWorker: ENTER WAIT_PREDISPENSE")

        if not self._error_msg:
            pre_t = 0.0
            try:
                pre_t = float(self._recipe.globals.predispose_time or 0.0)
            except Exception:
                pre_t = 0.0

            _LOG.info("ProcessWorker: WAIT_PREDISPENSE: predispense_time=%.3f s", pre_t)

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

        _LOG.info("ProcessWorker: LEAVE WAIT_PREDISPENSE")
        self._finish_state(self._sig_predispense_wait_done)

    def _on_state_move_recipe(self) -> None:
        _LOG.info("ProcessWorker: ENTER MOVE_RECIPE")

        if self._error_msg or self._should_stop():
            _LOG.info("ProcessWorker: MOVE_RECIPE: Fehler/Stop vor Start erkannt.")
            self._finish_state(self._sig_move_recipe_done)
            _LOG.info("ProcessWorker: LEAVE MOVE_RECIPE")
            return

        if not self._recipe_poses:
            self._recipe_poses = self._get_recipe_poses()
            self._recipe_index = 0

        n = len(self._recipe_poses)
        _LOG.info("ProcessWorker: MOVE_RECIPE: Anzahl Posen=%d", n)

        # Wir nutzen nur die "Innenpunkte" (1..n-2) für die Rezeptfahrt.
        if n <= 2:
            _LOG.info("ProcessWorker: MOVE_RECIPE: <=2 Posen, nichts zu fahren.")
            _LOG.info("ProcessWorker: LEAVE MOVE_RECIPE")
            self._finish_state(self._sig_move_recipe_done)
            return

        self._recipe_index = 1
        pose = self._recipe_poses[self._recipe_index]
        _LOG.info("ProcessWorker: MOVE_RECIPE: starte mit Pose[1] von %d.", n - 2)
        self._current_phase = self.PHASE_MOVE_RECIPE
        self._send_motion_pose(pose, label=f"recipe_{self._recipe_index}")

        _LOG.info("ProcessWorker: LEAVE MOVE_RECIPE (warte auf EXECUTED:OK / Folgewaypoints).")

    def _on_state_wait_postdispense(self) -> None:
        _LOG.info("ProcessWorker: ENTER WAIT_POSTDISPENSE")

        if not self._error_msg:
            post_t = 0.0
            try:
                post_t = float(self._recipe.globals.postdispense_time or 0.0)
            except Exception:
                post_t = 0.0

            _LOG.info("ProcessWorker: WAIT_POSTDISPENSE: postdispense_time=%.3f s", post_t)

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

        _LOG.info("ProcessWorker: LEAVE WAIT_POSTDISPENSE")
        self._finish_state(self._sig_postdispense_wait_done)

    def _on_state_move_retreat(self) -> None:
        _LOG.info("ProcessWorker: ENTER MOVE_RETREAT")

        if self._error_msg or self._should_stop():
            _LOG.info("ProcessWorker: MOVE_RETREAT: Fehler/Stop vor Start erkannt.")
            self._finish_state(self._sig_retreat_move_done)
            _LOG.info("ProcessWorker: LEAVE MOVE_RETREAT")
            return

        if not self._recipe_poses:
            self._recipe_poses = self._get_recipe_poses()
            self._recipe_index = 0

        n = len(self._recipe_poses)
        if n < 2:
            _LOG.info("ProcessWorker: MOVE_RETREAT: <2 Posen, kein separates Retreat.")
            _LOG.info("ProcessWorker: LEAVE MOVE_RETREAT")
            self._finish_state(self._sig_retreat_move_done)
            return

        last = self._recipe_poses[-1]
        _LOG.info("ProcessWorker: MOVE_RETREAT: fahre Pose[-1].")
        self._current_phase = self.PHASE_MOVE_RETREAT
        self._send_motion_pose(last, label="retreat")

        _LOG.info("ProcessWorker: LEAVE MOVE_RETREAT (warte auf EXECUTED:OK).")

    def _on_state_move_home(self) -> None:
        _LOG.info("ProcessWorker: ENTER MOVE_HOME")

        if self._error_msg or self._should_stop():
            _LOG.info("ProcessWorker: MOVE_HOME: Fehler/Stop vor Start erkannt.")
            self._finish_state(self._sig_move_home_done)
            _LOG.info("ProcessWorker: LEAVE MOVE_HOME")
            return

        home = self._get_home_pose()
        if home is None:
            _LOG.warning(
                "ProcessWorker: MOVE_HOME: keine Home-Pose verfügbar, "
                "überspringe Home-Fahrt."
            )
            _LOG.info("ProcessWorker: LEAVE MOVE_HOME")
            self._finish_state(self._sig_move_home_done)
            return

        _LOG.info("ProcessWorker: MOVE_HOME: sende Home-Pose via MotionBridge.")
        self._current_phase = self.PHASE_MOVE_HOME
        self._send_motion_pose(home, label="home")

        _LOG.info("ProcessWorker: LEAVE MOVE_HOME (warte auf EXECUTED:OK).")

    def _on_state_error(self) -> None:
        _LOG.info("ProcessWorker: ENTER ERROR")
        self._ended_in_error_state = True
        self._emit_state("ERROR")

        msg = self._error_msg or "Unbekannter Prozessfehler."
        _LOG.error("ProcessWorker: notifyError(%s) im ERROR-State", msg)
        self.notifyError.emit(msg)

        _LOG.info(
            "ProcessWorker: run beendet (ERROR), stop_requested=%s, "
            "executed_poses=%d, planned_traj_segments=%d, executed_traj_segments=%d",
            self._stop_requested,
            len(self._executed_poses),
            len(self._planned_traj_list),
            len(self._executed_traj_list),
        )

        self._stop_machine_and_quit()

    def _on_state_finished(self) -> None:
        _LOG.info("ProcessWorker: ENTER FINISHED")
        self._ended_in_finished_state = True
        self._emit_state("FINISHED")

        poses_copy: List[PoseStamped] = list(self._executed_poses)

        _LOG.info(
            "ProcessWorker: notifyFinished() im FINISHED-State, %d Posen (TCP executed), "
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
            "ProcessWorker: run beendet (FINISHED), stop_requested=%s, "
            "executed_poses=%d, planned_traj_segments=%d, executed_traj_segments=%d",
            self._stop_requested,
            len(self._executed_poses),
            len(self._planned_traj_list),
            len(self._executed_traj_list),
        )

        self._stop_machine_and_quit()


# ====================================================================== #
# Wrapper für GUI: ProcessThread
# ====================================================================== #


class ProcessThread(QtCore.QObject):
    """
    Wrapper, der von außen wie ein "Thread" aussieht, aber intern
    aus einem QThread + ProcessWorker (QObject im Worker-Thread) besteht.

    Öffentliche API (wie vorher):
      - startSignal (pyqtSignal)
      - stopSignal (pyqtSignal)
      - notifyFinished(object)
      - notifyError(str)
      - stateChanged(str)
      - logMessage(str)
      - recipe property + set_recipe(...)
      - isRunning()
      - request_stop()
      - wait(timeout_ms)
    """

    # externe Steuer-Signale (GUI-Seite verwendet diese wie bisher)
    startSignal = QtCore.pyqtSignal()
    stopSignal = QtCore.pyqtSignal()

    # Ergebnis / Fehler (vom Worker durchgereicht)
    notifyFinished = QtCore.pyqtSignal(object)  # Dict[str, Any]
    notifyError = QtCore.pyqtSignal(str)

    # UI-Hilfssignale
    stateChanged = QtCore.pyqtSignal(str)
    logMessage = QtCore.pyqtSignal(str)

    # "Thread finished" an GUI
    finished = QtCore.pyqtSignal()

    def __init__(
        self,
        *,
        recipe: Recipe,
        bridge,
        parent: Optional[QtCore.QObject] = None,
    ) -> None:
        super().__init__(parent)

        # WICHTIG: QThread OHNE Elternteil – wir managen Lebensdauer selbst,
        # damit Qt ihn nicht "nebenbei" zerstört, während er noch läuft.
        self._thread = QtCore.QThread()

        self._worker = ProcessWorker(recipe=recipe, bridge=bridge)

        # Worker in den Hintergrund-Thread verschieben
        self._worker.moveToThread(self._thread)

        # Signals vom Wrapper -> Worker
        self.startSignal.connect(self._on_start_signal)
        self.stopSignal.connect(self._on_stop_signal)

        # Signals vom Worker -> Wrapper (einfach durchreichen)
        self._worker.notifyFinished.connect(self.notifyFinished)
        self._worker.notifyError.connect(self.notifyError)
        self._worker.stateChanged.connect(self.stateChanged)
        self._worker.logMessage.connect(self.logMessage)

        # Wenn der QThread fertig ist, Worker/Wrapper aufräumen / Info
        self._thread.finished.connect(self._on_thread_finished)

        # Zusätzliche Sicherheit: beim App-Beenden Thread sauber runterfahren
        app = QCoreApplication.instance()
        if app is not None:
            try:
                app.aboutToQuit.connect(self._on_app_about_to_quit)
            except Exception:
                # falls mehrfach verbunden oder kein QApp – einfach ignorieren
                pass

        _LOG.info(
            "ProcessThread-Wrapper init: Thread=%r",
            int(self._thread.currentThreadId())
            if self._thread.currentThread() is not None
            else None,
        )

    # ------------------------------------------------------------------ #
    # Öffentliche API
    # ------------------------------------------------------------------ #

    @property
    def recipe(self) -> Recipe:
        return self._worker.recipe

    def set_recipe(self, recipe: Recipe) -> None:
        """
        Rezept nur wechseln, wenn der Thread nicht läuft.
        """
        if self.isRunning():
            _LOG.warning("ProcessThread.set_recipe ignoriert: Thread läuft noch.")
            return
        self._worker.set_recipe(recipe)

    def isRunning(self) -> bool:
        return self._thread.isRunning()

    def request_stop(self) -> None:
        """
        Kompatible Methode zur alten QThread-basierten Version.
        Setzt im Worker das Stop-Flag (im Worker-Thread).
        """
        QtCore.QMetaObject.invokeMethod(
            self._worker,
            "request_stop",
            QtCore.Qt.ConnectionType.QueuedConnection,
        )

    def wait(self, timeout_ms: int = 2000) -> None:
        """
        Wartet (optional mit Timeout in ms) auf das Ende des QThreads.
        """
        try:
            self._thread.wait(timeout_ms)
        except Exception:
            _LOG.exception("ProcessThread.wait: Fehler beim Warten auf Thread-Ende.")

    # ------------------------------------------------------------------ #
    # Interne Slots für start/stop
    # ------------------------------------------------------------------ #

    @QtCore.pyqtSlot()
    def _on_start_signal(self) -> None:
        """
        Reaktion auf startSignal (aus GUI). Startet den QThread (falls nötig)
        und ruft Worker.start() via QueuedConnection auf.
        """
        if not self._thread.isRunning():
            _LOG.info("ProcessThread: starte Hintergrund-Thread.")
            self._thread.start()
        else:
            _LOG.info("ProcessThread: Hintergrund-Thread läuft bereits.")

        # Worker.start im Worker-Thread ausführen
        QtCore.QMetaObject.invokeMethod(
            self._worker,
            "start",
            QtCore.Qt.ConnectionType.QueuedConnection,
        )

    @QtCore.pyqtSlot()
    def _on_stop_signal(self) -> None:
        """
        Reaktion auf stopSignal (aus GUI). Setzt im Worker das Stop-Flag.
        """
        self.request_stop()

    @QtCore.pyqtSlot()
    def _on_thread_finished(self) -> None:
        """
        Wird aufgerufen, wenn der QThread sein Eventloop beendet hat.
        """
        _LOG.info("ProcessThread: QThread.finished empfangen – Hintergrund-Thread beendet.")
        try:
            self.finished.emit()
        except Exception:
            pass

    @QtCore.pyqtSlot()
    def _on_app_about_to_quit(self) -> None:
        """
        Wird aufgerufen, wenn die QApplication beendet wird.
        Hier sorgen wir dafür, dass der Hintergrund-Thread garantiert
        nicht mehr läuft, bevor Qt beginnt, Objekte zu zerstören.
        """
        _LOG.info("ProcessThread: QApplication.aboutToQuit – stoppe Worker und Thread.")
        try:
            self.request_stop()
        except Exception:
            pass

        try:
            if self._thread.isRunning():
                self._thread.quit()
                # etwas großzügiger warten – wir sind ohnehin im Shutdown
                self._thread.wait(5000)
        except Exception:
            _LOG.exception("ProcessThread._on_app_about_to_quit: Fehler beim Thread-Shutdown.")

    def deleteLater(self) -> None:
        """
        Beim Löschen sicherstellen, dass der Thread sauber beendet wird.
        Da der QThread keinen Parent hat, managen wir seine Lebensdauer selbst.
        """
        try:
            if self._thread.isRunning():
                _LOG.info("ProcessThread.deleteLater: Thread läuft noch, quit() + wait().")
                # Worker sanft stoppen
                try:
                    self.request_stop()
                except Exception:
                    pass

                self._thread.quit()
                # etwas längeres Timeout, damit laufende Motion-Zyklen noch fertig werden
                self._thread.wait(5000)
        except Exception:
            _LOG.exception("ProcessThread.deleteLater: Fehler beim Thread-Shutdown.")

        # QThread-Objekt freigeben
        try:
            self._thread.deleteLater()
        except Exception:
            pass

        super().deleteLater()
