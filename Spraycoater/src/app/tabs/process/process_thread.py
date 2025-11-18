# -*- coding: utf-8 -*-
# File: tabs/process/process_thread.py
from __future__ import annotations

from typing import Optional

from PyQt6 import QtCore
from PyQt6.QtStateMachine import QStateMachine, QState, QFinalState

from app.model.recipe.recipe import Recipe


class ProcessThread(QtCore.QThread):
    """
    QThread mit eigener QStateMachine für einen Prozesslauf.

    States:

      - MOVE_PREDISPENSE    : erste/Predispense-Position anfahren
      - WAIT_PREDISPENSE    : predispense-Zeit warten
      - MOVE_RECIPE         : komplettes Rezept fahren
      - WAIT_POSTDISPENSE   : postdispense-Zeit warten
      - MOVE_RETREAT        : Retreat-Position anfahren
      - MOVE_HOME           : Roboter nach Home
      - FINISHED (QFinalState)

    Ein-/Ausgangssignale:
      - startSignal()            : von außen aufgerufen, um den Thread zu starten
      - stopSignal()             : von außen aufgerufen, um Stop anzufordern
      - notifyFinished()         : Prozess sauber fertig
      - notifyError(str message) : Prozess mit Fehler abgebrochen
    """

    # Steuer-Signale von außen
    startSignal = QtCore.pyqtSignal()
    stopSignal = QtCore.pyqtSignal()

    # Ergebnis-Signale nach außen
    notifyFinished = QtCore.pyqtSignal()
    notifyError = QtCore.pyqtSignal(str)

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
    ):
        super().__init__(parent)
        self._recipe = recipe
        self._bridge = bridge
        self._stop_requested = False
        self._error_msg: str | None = None
        self._machine: QStateMachine | None = None

        # Signale von außen auf Slots verdrahten
        self.startSignal.connect(self._on_start_signal)
        self.stopSignal.connect(self.request_stop)

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
            return
        self._recipe = recipe

    # ---------------------------------------------------------
    # Steuerung von außen
    # ---------------------------------------------------------

    def _on_start_signal(self) -> None:
        """
        Slot für startSignal – startet den Thread, falls er nicht läuft.
        """
        if not self.isRunning():
            self.start()

    def request_stop(self) -> None:
        """Von außen aufgerufen, um einen Abbruch anzufordern."""
        self._stop_requested = True
        self._abort_if_needed()

    def _should_stop(self) -> bool:
        return self._stop_requested

    def _abort_if_needed(self, msg: str | None = None) -> None:
        """
        Setzt optional eine Fehlermeldung und stoppt die StateMachine.
        Wird sowohl für Stop als auch Fehler benutzt.
        """
        if msg and not self._error_msg:
            self._error_msg = msg

        if self._machine is not None:
            try:
                self._machine.stop()
            except Exception:
                pass

    # ---------------------------------------------------------
    # StateMachine-Logik
    # ---------------------------------------------------------

    def run(self) -> None:
        """
        Thread-Einstieg: baut die QStateMachine auf, startet sie
        und wartet im lokalen EventLoop bis FINISHED oder Abbruch.
        """
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

        # EventLoop für diese StateMachine in diesem Thread
        loop = QtCore.QEventLoop()
        machine.finished.connect(loop.quit)

        try:
            machine.start()
            loop.exec()  # blockiert, bis Machine fertig oder gestoppt ist
        except Exception as e:
            self._error_msg = self._error_msg or str(e)
        finally:
            self._machine = None  # aufräumen

        # Ergebnis auswerten
        if self._error_msg:
            self.notifyError.emit(self._error_msg)
        elif not self._stop_requested:
            # nur "fertig" melden, wenn kein Stop angefordert wurde
            self.notifyFinished.emit()

    # ---------------------------------------------------------
    # State-Handler
    # ---------------------------------------------------------

    def _on_state_move_predispense(self) -> None:
        """
        State: Predispense-/erste Rezeptposition anfahren.
        """
        if self._should_stop():
            self._abort_if_needed()
            return

        try:
            robot = getattr(self._bridge, "_robot", None) or getattr(self._bridge, "robot", None)
            if robot is not None and hasattr(robot, "move_to_predispense"):
                # Vorschlag-Signatur:
                #   move_to_predispense(recipe: Recipe, should_stop: Callable[[], bool])
                robot.move_to_predispense(self._recipe, self._should_stop)
            else:
                # Dummy-Workload als Platzhalter
                for _ in range(40):
                    if self._should_stop():
                        self._abort_if_needed()
                        return
                    self.msleep(25)
        except Exception as e:
            self._abort_if_needed(str(e))
            return

        if self._should_stop():
            self._abort_if_needed()
            return

        self._sig_predispense_move_done.emit()

    def _on_state_wait_predispense(self) -> None:
        """
        State: predispense-time warten (Blocking im Thread).
        Holt sich die Zeit aus dem Rezept, sonst 0.
        """
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

        self._sig_predispense_wait_done.emit()

    def _on_state_move_recipe(self) -> None:
        """
        State: kompletten Rezeptpfad fahren.
        """
        if self._should_stop():
            self._abort_if_needed()
            return

        try:
            # TODO: Hier später echte Motion-API / MotionBridge aufrufen:
            # z.B.: robot.execute_recipe(self._recipe, self._should_stop)
            for _ in range(200):
                if self._should_stop():
                    self._abort_if_needed()
                    return
                self.msleep(25)
        except Exception as e:
            self._abort_if_needed(str(e))
            return

        if self._should_stop():
            self._abort_if_needed()
            return

        self._sig_move_recipe_done.emit()

    def _on_state_wait_postdispense(self) -> None:
        """
        State: postdispense-time warten.
        """
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

        self._sig_postdispense_wait_done.emit()

    def _on_state_move_retreat(self) -> None:
        """
        State: Retreat-Position anfahren (z.B. safe retreat über Substrat).
        """
        if self._should_stop():
            self._abort_if_needed()
            return

        try:
            robot = getattr(self._bridge, "_robot", None) or getattr(self._bridge, "robot", None)
            if robot is not None and hasattr(robot, "move_retreat"):
                # Vorschlag:
                #   move_retreat(recipe: Recipe, should_stop: Callable[[], bool])
                robot.move_retreat(self._recipe, self._should_stop)
            else:
                # Dummy: Retreat-Bewegung simulieren
                for _ in range(80):
                    if self._should_stop():
                        self._abort_if_needed()
                        return
                    self.msleep(25)
        except Exception as e:
            self._abort_if_needed(str(e))
            return

        if self._should_stop():
            self._abort_if_needed()
            return

        self._sig_retreat_move_done.emit()

    def _on_state_move_home(self) -> None:
        """
        State: Roboter nach Home fahren.
        """
        if self._should_stop():
            self._abort_if_needed()
            return

        try:
            robot = getattr(self._bridge, "_robot", None) or getattr(self._bridge, "robot", None)
            if robot is not None:
                if hasattr(robot, "move_home"):
                    robot.move_home()
                elif hasattr(robot, "go_home"):
                    robot.go_home()
            else:
                # Dummy: Home-Bewegung simulieren
                for _ in range(80):
                    if self._should_stop():
                        self._abort_if_needed()
                        return
                    self.msleep(25)
        except Exception as e:
            self._abort_if_needed(str(e))
            return

        if self._should_stop():
            self._abort_if_needed()
            return

        self._sig_move_home_done.emit()
