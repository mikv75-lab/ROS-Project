# -*- coding: utf-8 -*-
# File: tabs/process/process_thread.py
from __future__ import annotations

from typing import Optional

import logging
from PyQt6 import QtCore
from PyQt6.QtCore import QCoreApplication

from app.model.recipe.recipe import Recipe
from .process_setup_statemachine import ProcessSetupStatemachine   # Setup/MoveIt-Variante
from .process_run_statemachine import ProcessRunStatemachine       # Servo-Run-Playback-Variante

_LOG = logging.getLogger("app.tabs.process.thread")


class ProcessThread(QtCore.QObject):
    """
    Wrapper, der in der GUI wie ein "Thread" genutzt wird, intern aber:
      - einen QThread besitzt (Eventloop im Hintergrund)
      - pro Start einen Worker (StateMachine QObject) erzeugt und in den QThread verschiebt

    Unterstützte Modi:
      - MODE_RECIPE ("setup"):
          Worker = ProcessSetupStatemachine
          Quelle = Recipe / UI-Spraypath / Motion-Bridge
      - MODE_RUN ("servo"):
          Worker = ProcessRunStatemachine
          Quelle = Run-YAML (Joint-Trajektorie) -> Servo-JointJog Playback
    """

    MODE_RECIPE = "setup"
    MODE_RUN = "servo"

    # GUI steuert den Wrapper über diese Signale (Start/Stop wie bei einer Thread-API)
    startSignal = QtCore.pyqtSignal()
    stopSignal = QtCore.pyqtSignal()

    # Ergebnis/Fehler vom Worker (wird 1:1 weitergereicht)
    notifyFinished = QtCore.pyqtSignal(object)
    notifyError = QtCore.pyqtSignal(str)

    # UI-Hilfssignale vom Worker (State + Log)
    stateChanged = QtCore.pyqtSignal(str)
    logMessage = QtCore.pyqtSignal(str)

    # "Thread finished": wird emittiert, wenn der QThread beendet wurde
    finished = QtCore.pyqtSignal()

    def __init__(
        self,
        *,
        recipe: Optional[Recipe],
        bridge,
        parent: Optional[QtCore.QObject] = None,
        mode: str = MODE_RECIPE,
        run_yaml_path: Optional[str] = None,
    ) -> None:
        super().__init__(parent)

        # QThread ohne Parent: Lebensdauer wird manuell über deleteLater() gemanagt
        self._thread = QtCore.QThread()
        self._bridge = bridge
        self._recipe: Optional[Recipe] = recipe

        # Betriebsmodus + optionale Run-Datei
        self._mode: str = mode or self.MODE_RECIPE
        self._run_yaml_path: Optional[str] = run_yaml_path

        # Aktueller Worker für den aktuell/letzten Run
        self._worker: Optional[QtCore.QObject] = None

        # Steuer-Signale aus der GUI werden intern auf Slots gemappt
        self.startSignal.connect(self._on_start_signal)
        self.stopSignal.connect(self._on_stop_signal)

        # Wenn der QThread endet, emitten wir ein Wrapper-finished
        self._thread.finished.connect(self._on_thread_finished)

        # Beim App-Shutdown versuchen wir, Worker/Thread sauber zu stoppen
        app = QCoreApplication.instance()
        if app is not None:
            try:
                app.aboutToQuit.connect(self._on_app_about_to_quit)
            except Exception:
                pass

        _LOG.info(
            "ProcessThread-Wrapper init: mode=%s, Thread=%r",
            self._mode,
            int(self._thread.currentThreadId())
            if self._thread.currentThread() is not None
            else None,
        )

    # ------------------------------------------------------------------ #
    # Convenience-Konstruktor für Servo-Run-Playback
    # ------------------------------------------------------------------ #

    @classmethod
    def for_run(
        cls,
        *,
        run_yaml_path: str,
        bridge,
        parent: Optional[QtCore.QObject] = None,
    ) -> "ProcessThread":
        # Factory: erzeugt einen Wrapper im MODE_RUN, ohne Recipe, aber mit run_yaml_path
        return cls(
            recipe=None,
            bridge=bridge,
            parent=parent,
            mode=cls.MODE_RUN,
            run_yaml_path=run_yaml_path,
        )

    # ------------------------------------------------------------------ #
    # Öffentliche API
    # ------------------------------------------------------------------ #

    @property
    def recipe(self) -> Optional[Recipe]:
        # Zugriff auf das aktuell gesetzte Rezept (kann None sein)
        return self._recipe

    def set_recipe(self, recipe: Recipe) -> None:
        """
        Setzt das Rezept (typisch im MODE_RECIPE).
        Wenn der Thread gerade läuft, wird der Wechsel ignoriert.
        """
        if self.isRunning():
            _LOG.warning("ProcessThread.set_recipe ignoriert: Thread/Run läuft noch.")
            return

        self._recipe = recipe

        # Wenn ein Setup-Worker existiert und eine set_recipe API hätte, wird best-effort weitergereicht
        if self._worker is not None and isinstance(self._worker, ProcessSetupStatemachine):
            if hasattr(self._worker, "set_recipe"):
                try:
                    self._worker.set_recipe(recipe)  # type: ignore[attr-defined]
                except Exception:
                    _LOG.exception("ProcessThread.set_recipe: Fehler beim set_recipe im Worker.")

    def isRunning(self) -> bool:
        # True, wenn der QThread-Eventloop läuft
        return self._thread.isRunning()

    def request_stop(self) -> None:
        """
        Setzt im Worker das Stop-Flag über QueuedConnection.
        (läuft damit garantiert im Worker-Thread, nicht im GUI-Thread)
        """
        worker = self._worker
        if worker is None:
            return

        QtCore.QMetaObject.invokeMethod(
            worker,
            "request_stop",
            QtCore.Qt.ConnectionType.QueuedConnection,
        )

    def wait(self, timeout_ms: int = 2000) -> None:
        # Wartet auf das Ende des QThreads (best-effort)
        try:
            self._thread.wait(timeout_ms)
        except Exception:
            _LOG.exception("ProcessThread.wait: Fehler beim Warten auf Thread-Ende.")

    # ------------------------------------------------------------------ #
    # Interne Worker-Verwaltung
    # ------------------------------------------------------------------ #

    def _create_worker(self) -> QtCore.QObject:
        """
        Erzeugt einen Worker passend zum Modus, verschiebt ihn in den QThread
        und verbindet Worker-Signale auf Wrapper-Signale/Slots.
        """
        if self._mode == self.MODE_RUN:
            # Servo-Modus: Worker bekommt run_yaml_path + Bridge (für Zugriff auf servo bridge)
            worker = ProcessRunStatemachine(
                run_yaml_path=self._run_yaml_path or "",
                bridge=self._bridge,
            )
            _LOG.info(
                "ProcessThread: Worker=ProcessRunStatemachine, run_yaml_path=%r",
                self._run_yaml_path,
            )
        else:
            # Setup-Modus: Worker arbeitet mit recipe + UIBridge (spraypath/motion/robot/...)
            worker = ProcessSetupStatemachine(
                recipe=self._recipe,
                ui_bridge=self._bridge,
            )
            _LOG.info(
                "ProcessThread: Worker=ProcessSetupStatemachine, recipe=%r",
                getattr(self._recipe, "id", None),
            )

        # Worker-Objekt in den Hintergrund-Thread verschieben
        worker.moveToThread(self._thread)

        # Worker -> Wrapper: Status + Log direkt durchreichen
        worker.stateChanged.connect(self.stateChanged)           # type: ignore[attr-defined]
        worker.logMessage.connect(self.logMessage)               # type: ignore[attr-defined]

        # Worker -> Wrapper: Ergebnis/Fehler wird zusätzlich über Wrapper-Slots abgefangen,
        # damit danach zentral cleanup passiert
        worker.notifyFinished.connect(self._on_worker_finished)  # type: ignore[attr-defined]
        worker.notifyError.connect(self._on_worker_error)        # type: ignore[attr-defined]

        self._worker = worker
        return worker

    def _cleanup_worker(self) -> None:
        """
        Räumt den aktuellen Worker nach einem Run auf.
        Der QThread bleibt bestehen und kann für den nächsten Start wieder genutzt werden.
        """
        worker = self._worker
        if worker is None:
            return

        _LOG.info("ProcessThread: cleanup Worker nach Run (mode=%s).", self._mode)

        # Falls der Worker eigene Disconnect-Logik anbietet: best-effort ausführen
        try:
            if hasattr(worker, "_disconnect_signals"):
                worker._disconnect_signals()  # type: ignore[attr-defined]
        except Exception:
            _LOG.exception("ProcessThread._cleanup_worker: Fehler beim Disconnect der Worker-Signale.")

        # Worker zur Löschung vormerken (läuft im Qt-Eventloop)
        try:
            worker.deleteLater()
        except Exception:
            pass

        self._worker = None

    # ------------------------------------------------------------------ #
    # Interne Slots für start/stop
    # ------------------------------------------------------------------ #

    @QtCore.pyqtSlot()
    def _on_start_signal(self) -> None:
        """
        Start-Handling:
          - QThread starten, falls noch nicht läuft
          - ggf. vorhandenen Worker aufräumen
          - neuen Worker erzeugen
          - Worker.start() per QueuedConnection im Worker-Thread aufrufen
        """
        if not self._thread.isRunning():
            _LOG.info("ProcessThread: starte Hintergrund-Thread (mode=%s).", self._mode)
            self._thread.start()
        else:
            _LOG.info("ProcessThread: Hintergrund-Thread läuft bereits (mode=%s).", self._mode)

        # Safety: falls noch ein Worker übrig ist, vor dem neuen Lauf entfernen
        if self._worker is not None:
            _LOG.warning("ProcessThread: alter Worker war noch vorhanden, räume ihn jetzt auf.")
            self._cleanup_worker()

        worker = self._create_worker()

        # Worker.start() im Worker-Thread ausführen
        QtCore.QMetaObject.invokeMethod(
            worker,
            "start",
            QtCore.Qt.ConnectionType.QueuedConnection,
        )

    @QtCore.pyqtSlot()
    def _on_stop_signal(self) -> None:
        # Stop-Handling: delegiert auf request_stop (setzt Stop-Flag im Worker)
        self.request_stop()

    # ------------------------------------------------------------------ #
    # Worker -> Wrapper (Ergebnis/Fehler)
    # ------------------------------------------------------------------ #

    @QtCore.pyqtSlot(object)
    def _on_worker_finished(self, result_obj: object) -> None:
        # Ergebnis nach außen weitergeben und anschließend Worker aufräumen
        self.notifyFinished.emit(result_obj)
        self._cleanup_worker()

    @QtCore.pyqtSlot(str)
    def _on_worker_error(self, msg: str) -> None:
        # Fehler nach außen weitergeben und anschließend Worker aufräumen
        self.notifyError.emit(msg)
        self._cleanup_worker()

    # ------------------------------------------------------------------ #
    # Thread-Shutdown / App-Shutdown
    # ------------------------------------------------------------------ #

    @QtCore.pyqtSlot()
    def _on_thread_finished(self) -> None:
        # Wird getriggert, wenn der QThread beendet ist (Eventloop stoppt)
        _LOG.info("ProcessThread: QThread.finished empfangen – Hintergrund-Thread beendet.")
        try:
            self.finished.emit()
        except Exception:
            pass

    @QtCore.pyqtSlot()
    def _on_app_about_to_quit(self) -> None:
        # Shutdown-Pfad: Stop anfordern, QThread beenden, Worker aufräumen
        _LOG.info("ProcessThread: QApplication.aboutToQuit – stoppe Worker und Thread.")

        try:
            self.request_stop()
        except Exception:
            pass

        try:
            if self._thread.isRunning():
                self._thread.quit()
                self._thread.wait(5000)
        except Exception:
            _LOG.exception("ProcessThread._on_app_about_to_quit: Fehler beim Thread-Shutdown.")

        self._cleanup_worker()

    def deleteLater(self) -> None:
        """
        Beim Löschen sicherstellen, dass:
          - Worker gestoppt wird
          - QThread sauber beendet wird
          - QThread-Objekt freigegeben wird
        """
        try:
            if self._thread.isRunning():
                _LOG.info("ProcessThread.deleteLater: Thread läuft noch, quit() + wait().")

                try:
                    self.request_stop()
                except Exception:
                    pass

                self._thread.quit()
                self._thread.wait(5000)
        except Exception:
            _LOG.exception("ProcessThread.deleteLater: Fehler beim Thread-Shutdown.")

        self._cleanup_worker()

        try:
            self._thread.deleteLater()
        except Exception:
            pass

        super().deleteLater()
