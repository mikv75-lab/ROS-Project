# -*- coding: utf-8 -*-
# File: tabs/process/process_thread.py
from __future__ import annotations

from typing import Optional

import logging
from PyQt6 import QtCore
from PyQt6.QtCore import QCoreApplication

from app.model.recipe.recipe import Recipe
from .process_statemachine import ProcessStatemachine  # <--- neues Modul + Klasse

_LOG = logging.getLogger("app.tabs.process.thread")


class ProcessThread(QtCore.QObject):
    """
    Wrapper, der von außen wie ein "Thread" aussieht, aber intern
    aus einem QThread + ProcessStatemachine (QObject im Worker-Thread) besteht.

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
    notifyFinished = QtCore.pyqtSignal(object)  # Dict[str, Any] oder List[PoseStamped]
    notifyError = QtCore.pyqtSignal(str)

    # UI-Hilfssignale
    stateChanged = QtCore.pyqtSignal(str)
    logMessage = QtCore.pyqtSignal(str)

    # "Thread finished" an GUI (wird emittiert, wenn der QThread beendet wird)
    finished = QtCore.pyqtSignal()

    def __init__(
        self,
        *,
        recipe: Recipe,
        bridge,
        parent: Optional[QtCore.QObject] = None,
    ) -> None:
        super().__init__(parent)

        self._thread = QtCore.QThread()   # kein Parent, Lebensdauer selbst verwalten
        self._bridge = bridge
        self._recipe: Recipe = recipe

        self._worker: Optional[ProcessStatemachine] = None

        # Signals vom Wrapper -> Worker (indirekt via Slots)
        self.startSignal.connect(self._on_start_signal)
        self.stopSignal.connect(self._on_stop_signal)

        # QThread.finished -> Wrapper.finished
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
        return self._recipe

    def set_recipe(self, recipe: Recipe) -> None:
        """
        Rezept nur wechseln, wenn kein Run aktiv ist.
        """
        if self.isRunning():
            _LOG.warning("ProcessThread.set_recipe ignoriert: Thread/Run läuft noch.")
            return

        self._recipe = recipe

        # Falls bereits ein Worker existiert (aber Thread nicht läuft),
        # geben wir das Rezept weiter.
        if self._worker is not None:
            self._worker.set_recipe(recipe)

    def isRunning(self) -> bool:
        """
        True, wenn der QThread läuft (unabhängig davon, ob gerade ein Run aktiv ist).
        """
        return self._thread.isRunning()

    def request_stop(self) -> None:
        """
        Kompatible Methode zur alten QThread-basierten Version.
        Setzt im Worker das Stop-Flag (im Worker-Thread).
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
        """
        Wartet (optional mit Timeout in ms) auf das Ende des QThreads.
        """
        try:
            self._thread.wait(timeout_ms)
        except Exception:
            _LOG.exception("ProcessThread.wait: Fehler beim Warten auf Thread-Ende.")

    # ------------------------------------------------------------------ #
    # Interne Worker-Verwaltung
    # ------------------------------------------------------------------ #

    def _create_worker(self) -> ProcessStatemachine:
        """
        Erzeugt ein neues ProcessStatemachine-Objekt für einen Run,
        verschiebt es in den QThread und verdrahtet die Signals.
        """
        worker = ProcessStatemachine(recipe=self._recipe, bridge=self._bridge)

        # Worker in den Hintergrund-Thread verschieben
        worker.moveToThread(self._thread)

        # Signals vom Worker -> Wrapper (mit Cleanup)
        worker.stateChanged.connect(self.stateChanged)
        worker.logMessage.connect(self.logMessage)

        worker.notifyFinished.connect(self._on_worker_finished)
        worker.notifyError.connect(self._on_worker_error)

        self._worker = worker
        return worker

    def _cleanup_worker(self) -> None:
        """
        Nach einem Run den aktuellen Worker aufräumen.
        Der QThread bleibt bestehen, damit ein neuer Worker für den nächsten
        Run genutzt werden kann.
        """
        worker = self._worker
        if worker is None:
            return

        _LOG.info("ProcessThread: cleanup Worker nach Run.")

        try:
            # externe Signale lösen, Logging etc.
            if hasattr(worker, "_disconnect_signals"):
                worker._disconnect_signals()  # type: ignore[attr-defined]
        except Exception:
            _LOG.exception("ProcessThread._cleanup_worker: Fehler beim Disconnect der Worker-Signale.")

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
        Reaktion auf startSignal (aus GUI).
        - startet den QThread (falls nötig)
        - erzeugt für diesen Run einen neuen Worker
        - triggert Worker.start() via QueuedConnection
        """
        if not self._thread.isRunning():
            _LOG.info("ProcessThread: starte Hintergrund-Thread.")
            self._thread.start()
        else:
            _LOG.info("ProcessThread: Hintergrund-Thread läuft bereits.")

        # ggf. alten Worker wegräumen (sollte nach Run ohnehin passiert sein)
        if self._worker is not None:
            _LOG.warning("ProcessThread: alter Worker war noch vorhanden, räume ihn jetzt auf.")
            self._cleanup_worker()

        worker = self._create_worker()

        # Worker.start im Worker-Thread ausführen
        QtCore.QMetaObject.invokeMethod(
            worker,
            "start",
            QtCore.Qt.ConnectionType.QueuedConnection,
        )

    @QtCore.pyqtSlot()
    def _on_stop_signal(self) -> None:
        """
        Reaktion auf stopSignal (aus GUI). Setzt im Worker das Stop-Flag.
        """
        self.request_stop()

    # ------------------------------------------------------------------ #
    # Worker -> Wrapper (Ergebnis/Fehler)
    # ------------------------------------------------------------------ #

    @QtCore.pyqtSlot(object)
    def _on_worker_finished(self, result_obj: object) -> None:
        """
        Weiterreichen von notifyFinished + Aufräumen des Workers nach dem Run.
        """
        self.notifyFinished.emit(result_obj)
        self._cleanup_worker()

    @QtCore.pyqtSlot(str)
    def _on_worker_error(self, msg: str) -> None:
        """
        Weiterreichen von notifyError + Aufräumen des Workers nach einem Fehler.
        """
        self.notifyError.emit(msg)
        self._cleanup_worker()

    # ------------------------------------------------------------------ #
    # Thread-Shutdown / App-Shutdown
    # ------------------------------------------------------------------ #

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

        # Worker-Signale final lösen
        self._cleanup_worker()

    def deleteLater(self) -> None:
        """
        Beim Löschen sicherstellen, dass der Thread sauber beendet wird.
        Da der QThread keinen Parent hat, managen wir seine Lebensdauer selbst.
        """
        try:
            if self._thread.isRunning():
                _LOG.info("ProcessThread.deleteLater: Thread läuft noch, quit() + wait().")

                try:
                    self.request_stop()
                except Exception:
                    pass

                self._thread.quit()
                # etwas längeres Timeout, damit laufende Motion-Zyklen noch fertig werden
                self._thread.wait(5000)
        except Exception:
            _LOG.exception("ProcessThread.deleteLater: Fehler beim Thread-Shutdown.")

        # Worker final aufräumen
        self._cleanup_worker()

        # QThread-Objekt freigeben
        try:
            self._thread.deleteLater()
        except Exception:
            pass

        super().deleteLater()
