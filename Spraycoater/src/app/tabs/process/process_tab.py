# -*- coding: utf-8 -*-
# File: tabs/process/process_tab.py
from __future__ import annotations

import os
import math
import logging
from typing import Optional, List, Dict, Any

from PyQt6 import QtCore
from PyQt6.QtWidgets import (
    QWidget, QPushButton, QLabel, QVBoxLayout, QHBoxLayout,
    QGroupBox, QSizePolicy, QFileDialog, QMessageBox, QTextEdit
)

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray

from app.model.recipe.recipe import Recipe
from app.model.recipe import recipe_markers
from app.widgets.robot_status_box import RobotStatusInfoBox
from app.widgets.info_groupbox import InfoGroupBox
from .process_thread import ProcessThread
from .robot_init_thread import RobotInitThread
from .process_view import ProcessTabView

_LOG = logging.getLogger("app.tabs.process")


class ProcessTab(QWidget):
    """
    Process-Tab (nur Code, keine .ui):

      VBOX(
        HBOX(
          [GroupBox] Process Control,
          [GroupBox] Startbedingungen,
          [GroupBox] Setup (Tool / Substrate / Mount),
          [GroupBox] Robot Status
        ),
        [InfoGroupBox]
        [GroupBox] Process Status / Log
           ├─ QLabel    (aktueller State)
           └─ QTextEdit (Log)
        [GroupBox] Recipe
           ├─ QTextEdit  (Summary)
           └─ QTextEdit  (Poses)
      )
    """

    def __init__(self, *, ctx, bridge, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        self.bridge = bridge

        # Bridges (fest gecacht)
        self._rb = bridge._rb if bridge is not None else None
        self._pb = bridge._pb if bridge is not None else None
        self._poses_state = bridge.poses if bridge is not None else None
        self._sb = bridge._sb if bridge is not None else None  # SceneBridge

        self._sig = self._rb.signals if self._rb is not None else None

        # -------- interner Zustand --------
        self._robot_initialized: bool = False
        self._robot_at_home: bool = False

        self._recipe_model: Optional[Recipe] = None
        self._recipe_name: str | None = None

        self._tool_ok: bool = True
        self._substrate_ok: bool = False
        self._mount_ok: bool = False

        self._process_active: bool = False
        self._process_thread: Optional[ProcessThread] = None
        self._robot_init_thread: Optional[RobotInitThread] = None

        # Ergebnis eines Prozesslaufs:
        #   None      -> noch nichts
        #   "success" -> notifyFinished verarbeitet
        #   "error"   -> notifyError verarbeitet (hat Vorrang)
        self._process_outcome: Optional[str] = None

        # Info-Box Cache
        self._info_values: Dict[str, Any] = {}

        # Reentrancy-Guard
        self._in_update_start_conditions: bool = False

        # ==================================================================
        # View / Layout auslagern
        # ==================================================================
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        self.view = ProcessTabView(self)
        root.addWidget(self.view)

        # Kurz-Referenzen, damit der restliche Code unverändert bleiben kann
        self.grpProcess: QGroupBox = self.view.grpProcess
        self.grpStatus: QGroupBox = self.view.grpStatus
        self.grpSetup: QGroupBox = self.view.grpSetup
        self.robotStatusBox: RobotStatusInfoBox = self.view.robotStatusBox
        self.infoBox: InfoGroupBox = self.view.infoBox
        self.grpProcessInfo: QGroupBox = self.view.grpProcessInfo
        self.grpRecipe: QGroupBox = self.view.grpRecipe

        self.btnInit: QPushButton = self.view.btnInit
        self.btnLoadRecipe: QPushButton = self.view.btnLoadRecipe
        self.btnStart: QPushButton = self.view.btnStart
        self.btnStop: QPushButton = self.view.btnStop

        self.lblStatus: QLabel = self.view.lblStatus
        self.lblTool: QLabel = self.view.lblTool
        self.lblSubstrate: QLabel = self.view.lblSubstrate
        self.lblMount: QLabel = self.view.lblMount
        self.lblProcessState: QLabel = self.view.lblProcessState
        self.txtProcessLog: QTextEdit = self.view.txtProcessLog
        self.txtRecipeSummary: QTextEdit = self.view.txtRecipeSummary
        self.txtRecipePoses: QTextEdit = self.view.txtRecipePoses

        # ==================================================================
        # Threads
        # ==================================================================
        if self.bridge is not None:
            self._robot_init_thread = RobotInitThread(
                bridge=self.bridge,
                parent=self,
                init_timeout_s=10.0,
                home_timeout_s=60.0,
                pos_tol_mm=1.0,
            )
            self._robot_init_thread.notifyFinished.connect(self._on_robot_init_finished)
            self._robot_init_thread.notifyError.connect(self._on_robot_init_error)

        # ==================================================================
        # Buttons -> Methoden
        # ==================================================================
        self.btnInit.clicked.connect(self._on_init_clicked)
        self.btnLoadRecipe.clicked.connect(self._on_load_recipe_clicked)
        self.btnStart.clicked.connect(self._on_start_clicked)
        self.btnStop.clicked.connect(self._on_stop_clicked)

        # Wiring
        self._wire_scene_bridge()
        self._wire_robot_status()
        self._wire_poses_bridge()

        self._info_values.clear()
        self.infoBox.set_values(None)

        self._condTimer = QtCore.QTimer(self)
        self._condTimer.setInterval(500)
        self._condTimer.timeout.connect(self._update_start_conditions)

        self._update_start_conditions()
        self._update_timer_state()

    # =====================================================================
    # Hilfsfunktionen für InfoBox / ProcessLog
    # =====================================================================

    def _info_set_all(self, values: Optional[Dict[str, Any]]) -> None:
        """Gesamten Info-Cache ersetzen und InfoBox updaten."""
        self._info_values = dict(values or {})
        try:
            if self._info_values:
                self.infoBox.set_values(dict(self._info_values))
            else:
                self.infoBox.set_values(None)
        except RuntimeError:
            pass

    def _info_update(self, key: str, value: Any | None) -> None:
        """Einzelnen Key im Info-Cache setzen/löschen und InfoBox updaten."""
        if value is None:
            self._info_values.pop(key, None)
        else:
            self._info_values[key] = value

        try:
            if self._info_values:
                self.infoBox.set_values(dict(self._info_values))
            else:
                self.infoBox.set_values(None)
        except RuntimeError:
            pass

    def _append_process_log(self, text: str) -> None:
        """Text in die Process-Logbox anhängen."""
        if not text:
            return
        try:
            self.txtProcessLog.append(text)
        except RuntimeError:
            pass

    @QtCore.pyqtSlot(str)
    def _on_process_log_message(self, msg: str) -> None:
        """Slot für ProcessThread.logMessage."""
        self._append_process_log(msg)

    # =====================================================================
    # Robot-Status Wiring
    # =====================================================================

    @QtCore.pyqtSlot(object)
    def _on_joints(self, js):
        try:
            if js is None or not hasattr(js, "position"):
                self.robotStatusBox.set_joints(None)
            else:
                self.robotStatusBox.set_joints(list(js.position or []))
        except RuntimeError:
            pass

    def _wire_robot_status(self) -> None:
        if self._sig is None:
            return

        sig = self._sig
        sb = self.robotStatusBox

        if hasattr(sig, "connectionChanged"):
            sig.connectionChanged.connect(sb.set_connection)
        if hasattr(sig, "modeChanged"):
            sig.modeChanged.connect(sb.set_mode)
        if hasattr(sig, "initializedChanged"):
            sig.initializedChanged.connect(sb.set_initialized)
            sig.initializedChanged.connect(self.set_robot_initialized)
        if hasattr(sig, "movingChanged"):
            sig.movingChanged.connect(sb.set_moving)
        if hasattr(sig, "powerChanged"):
            sig.powerChanged.connect(sb.set_power)
        if hasattr(sig, "servoEnabledChanged"):
            sig.servoEnabledChanged.connect(sb.set_servo_enabled)
        if hasattr(sig, "estopChanged"):
            sig.estopChanged.connect(sb.set_estop)
        if hasattr(sig, "errorsChanged"):
            sig.errorsChanged.connect(sb.set_errors)
        if hasattr(sig, "tcpPoseChanged"):
            sig.tcpPoseChanged.connect(sb.set_tcp_from_ps)
            sig.tcpPoseChanged.connect(self._on_tcp_pose_changed)
        if hasattr(sig, "jointsChanged"):
            sig.jointsChanged.connect(self._on_joints)

    # =====================================================================
    # Poses-Bridge: Home-Pose beobachten
    # =====================================================================

    def _wire_poses_bridge(self) -> None:
        if self._pb is None:
            return
        sigp = getattr(self._pb, "signals", None)
        if sigp is None:
            return
        if hasattr(sigp, "homePoseChanged"):
            sigp.homePoseChanged.connect(self._on_home_pose_changed)

    @QtCore.pyqtSlot(object)
    def _on_tcp_pose_changed(self, _msg: object) -> None:
        self._recompute_robot_at_home()

    @QtCore.pyqtSlot(object)
    def _on_home_pose_changed(self, _msg: object) -> None:
        self._recompute_robot_at_home()

    # =====================================================================
    # Pose-basierter Home-Check
    # =====================================================================

    def _get_tcp_pose(self) -> Optional[PoseStamped]:
        if self._rb is None:
            return None
        pose = getattr(self._rb, "tcp_pose", None)
        if isinstance(pose, PoseStamped) or pose is None:
            return pose
        return None

    def _get_home_pose(self) -> Optional[PoseStamped]:
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

        return None

    @staticmethod
    def _poses_close(a: PoseStamped, b: PoseStamped, pos_tol_mm: float = 1.0) -> bool:
        try:
            dx = float(a.pose.position.x) - float(b.pose.position.x)
            dy = float(a.pose.position.y) - float(b.pose.position.y)
            dz = float(a.pose.position.z) - float(b.pose.position.z)
        except Exception:
            return False

        dist_m = math.sqrt(dx * dx + dy * dy + dz * dz)
        dist_mm = dist_m * 1000.0
        return dist_mm <= pos_tol_mm

    def _recompute_robot_at_home(self) -> None:
        cur = self._get_tcp_pose()
        home = self._get_home_pose()
        if cur is None or home is None:
            self.set_robot_at_home(False)
            return

        at_home = self._poses_close(cur, home, pos_tol_mm=1.0)
        self.set_robot_at_home(at_home)

    # =====================================================================
    # ProcessThread-Handling
    # =====================================================================

    def _cleanup_process_thread(self) -> None:
        thr = self._process_thread
        self._process_thread = None

        if thr is None:
            return

        try:
            if thr.isRunning():
                thr.request_stop()
                thr.wait(2000)
            thr.deleteLater()
        except RuntimeError:
            pass

    def _setup_process_thread_for_recipe(self, recipe: Recipe) -> None:
        """
        Erstellt einen neuen ProcessThread für das aktuelle Rezept.
        Wird bei jedem 'Load Recipe' aufgerufen.
        """
        self._cleanup_process_thread()

        # neuer Lauf-Kontext -> Outcome zurücksetzen
        self._process_outcome = None

        thr = ProcessThread(recipe=recipe, bridge=self.bridge)
        thr.notifyFinished.connect(self._on_process_finished_success)
        thr.notifyError.connect(self._on_process_finished_error)
        thr.finished.connect(self._on_process_thread_finished)
        # WICHTIG: Verbindung über Lambda, um Qt-Slot-Signatur-Probleme zu vermeiden
        thr.stateChanged.connect(lambda s, self=self: self._on_process_state_changed(s))
        thr.logMessage.connect(lambda m, self=self: self._on_process_log_message(m))

        self._process_thread = thr

        # Beim Laden eines neuen Rezepts das Log leeren
        try:
            self.txtProcessLog.clear()
        except RuntimeError:
            pass

    # =====================================================================
    # SprayPath-Ansteuerung
    # =====================================================================

    def _send_recipe_to_spraypath(self, recipe: Recipe) -> None:
        if self.bridge is None or recipe is None:
            return

        try:
            ma: MarkerArray = recipe_markers.build_marker_array_from_recipe(
                recipe,
                sides=None,
                frame_id="scene",
            )

            if ma and isinstance(ma, MarkerArray) and ma.markers:
                try:
                    self.bridge.set_spraypath(ma)
                    _LOG.info(
                        "ProcessTab: MarkerArray mit %d Markern an SprayPath gesendet (frame=scene)",
                        len(ma.markers),
                    )
                except Exception:
                    _LOG.exception("ProcessTab: bridge.set_spraypath failed")
            else:
                _LOG.warning(
                    "ProcessTab: build_marker_array_from_recipe lieferte kein/nur leeres MarkerArray"
                )
        except Exception:
            _LOG.exception(
                "ProcessTab: Fehler beim Erzeugen/Publizieren des MarkerArray für SprayPath"
            )

    # =====================================================================
    # Button-Slots
    # =====================================================================

    def _on_init_clicked(self) -> None:
        if self._robot_init_thread is None:
            QMessageBox.warning(self, "Robot Init", "RobotInitThread nicht verfügbar.")
            return

        self.set_robot_initialized(False)
        self.set_robot_at_home(False)

        try:
            self.btnInit.setEnabled(False)
        except RuntimeError:
            pass

        self._info_update("process", "Robot-Init läuft (Init + Home via Pose-Vergleich)...")

        self._robot_init_thread.startSignal.emit()

    def _on_load_recipe_clicked(self) -> None:
        """
        Teil 1: Datei auswählen + Rezeptmodell laden.
        Der eigentliche UI-/Thread-/SprayPath-Kram passiert danach
        in _apply_loaded_recipe(), optional verzögert.
        """
        start_dir = getattr(getattr(self.ctx, "paths", None), "recipe_dir", os.getcwd())

        fname, _ = QFileDialog.getOpenFileName(
            self,
            "Rezept laden (Process)",
            start_dir,
            "YAML (*.yaml *.yml)"
        )
        if not fname:
            return

        try:
            # *** Nur Laden + Model-Erstellung ***
            model = Recipe.load_yaml(fname)
        except Exception as e:
            QMessageBox.critical(self, "Ladefehler", f"Rezept konnte nicht geladen werden:\n{e}")
            return

        _LOG.info("ProcessTab: Rezept '%s' geladen, wende nun auf UI/Threads an.", fname)

        # Variante B: leicht verzögert im nächsten Eventloop-Tick
        QtCore.QTimer.singleShot(
            0,
            lambda m=model, f=fname, self=self: self._apply_loaded_recipe(m, f)
        )

    def _apply_loaded_recipe(self, model: Recipe, fname: str) -> None:
        """
        Teil 2: Das fertig geladene Rezept-Model in den UI- und
        Prozesszustand übernehmen, ProcessThread/SprayPath anstoßen.
        """
        if model is None:
            return

        # 1) Model & Name setzen
        self._recipe_model = model

        name = (model.id or "").strip() or os.path.basename(fname)
        self.set_recipe_name(name)

        # 2) Recipe-Textfelder aktualisieren
        try:
            self._update_recipe_info_text()
        except RuntimeError:
            # UI evtl. schon zerstört (z.B. beim Schließen) -> einfach abbrechen
            return

        # 3) ProcessThread vorbereiten (neuer Thread pro Load)
        try:
            self._setup_process_thread_for_recipe(model)
        except Exception:
            _LOG.exception("ProcessTab: _setup_process_thread_for_recipe() hat eine Exception geworfen")
            return

        # 4) SprayPath (MarkerArray) an Bridge schicken
        try:
            self._send_recipe_to_spraypath(model)
        except Exception:
            _LOG.exception("ProcessTab: _send_recipe_to_spraypath() hat eine Exception geworfen")

        # 5) Setup/Scene prüfen
        try:
            self._update_setup_from_scene()
            self._evaluate_scene_match()
        except Exception:
            _LOG.exception("ProcessTab: Szene-/Setup-Update nach Rezeptladen fehlgeschlagen")

        # 6) Status/Log zurücksetzen
        try:
            self.lblProcessState.setText("Kein Prozess aktiv.")
        except RuntimeError:
            pass

        _LOG.info("ProcessTab: Rezept '%s' erfolgreich angewendet (id=%r).", fname, model.id)

    def _on_start_clicked(self) -> None:
        try:
            if not self.btnStart.isEnabled():
                return
        except RuntimeError:
            return

        if self._recipe_model is None or self._process_thread is None:
            QMessageBox.warning(self, "Kein Rezept", "Es ist kein Rezept geladen.")
            return

        # Ergebnis-Status für diesen Lauf zurücksetzen
        self._process_outcome = None

        # Safety: sicherstellen, dass Thread das aktuelle Model kennt
        if getattr(self._process_thread, "recipe", None) is not self._recipe_model:
            try:
                self._process_thread.set_recipe(self._recipe_model)
            except Exception:
                _LOG.exception("ProcessTab: set_recipe im ProcessThread fehlgeschlagen")
                QMessageBox.critical(self, "Process-Fehler", "Rezept konnte nicht an den Prozess übergeben werden.")
                return

        self.set_process_active(True)
        self._append_process_log("=== Prozess gestartet ===")

        try:
            self._process_thread.startSignal.emit()
        except Exception:
            _LOG.exception("ProcessTab: startSignal.emit() fehlgeschlagen")
            self.set_process_active(False)

    def _on_stop_clicked(self) -> None:
        """
        Stop-Button:
          - setzt das Stop-Flag im ProcessThread *direkt* (ohne auf die
            QThread-Eventloop zu warten),
          - schickt zusätzlich das stopSignal (Abwärtskompatibilität),
          - stoppt ggf. auch den RobotInitThread.
        """
        self._append_process_log("=== Stop angefordert ===")

        thr = self._process_thread
        if thr is not None:
            try:
                # Direkt das Flag setzen (thread-safe genug, nur bool),
                # damit _wait_for_motion() nicht auf Events angewiesen ist.
                thr.request_stop()
            except Exception:
                _LOG.exception("ProcessTab: request_stop() auf ProcessThread fehlgeschlagen.")

            try:
                if thr.isRunning():
                    thr.stopSignal.emit()
            except Exception:
                _LOG.exception("ProcessTab: stopSignal.emit() auf ProcessThread fehlgeschlagen.")

        if self._robot_init_thread is not None and self._robot_init_thread.isRunning():
            self._robot_init_thread.stopSignal.emit()

    # ---------------------------------------------------------------------
    # Callbacks Robot-Init
    # ---------------------------------------------------------------------

    def _on_robot_init_finished(self) -> None:
        self.set_robot_initialized(True)
        self._recompute_robot_at_home()

        try:
            self.btnInit.setEnabled(True)
        except RuntimeError:
            pass

        text = "Roboter initialisiert und Home-Pose erreicht (TCP≈Home)."
        self._info_update("robot_init", text)
        if self._recipe_name:
            self._info_update("recipe", self._recipe_name)

    def _on_robot_init_error(self, msg: str) -> None:
        self.set_robot_initialized(False)
        self.set_robot_at_home(False)

        try:
            self.btnInit.setEnabled(True)
        except RuntimeError:
            pass

        QMessageBox.warning(self, "Robot Init fehlgeschlagen", msg or "Unbekannter Fehler.")

    # ---------------------------------------------------------------------
    # Callbacks vom ProcessThread
    # ---------------------------------------------------------------------

    def _on_process_finished_success(self) -> None:
        """
        Wird vom ProcessThread.notifyFinished() gerufen.

        Wichtig:
          - Wenn bereits ein Fehler gemeldet wurde (_process_outcome == "error"),
            wird der Erfolgs-Callback ignoriert.
        """
        if self._process_outcome == "error":
            _LOG.info("ProcessTab: notifyFinished ignoriert, da bereits ein Fehler gemeldet wurde.")
            return

        # Nur setzen, wenn noch nichts feststeht
        if self._process_outcome is None:
            self._process_outcome = "success"

        self.set_process_active(False)
        self._info_update("process", "Prozess erfolgreich abgeschlossen.")
        self._append_process_log("=== Prozess erfolgreich abgeschlossen ===")

        try:
            self.lblProcessState.setText("Prozess erfolgreich abgeschlossen.")
        except RuntimeError:
            pass

    def _on_process_finished_error(self, msg: str) -> None:
        """
        Wird vom ProcessThread.notifyError(msg) gerufen.

        - Wenn bereits Erfolg gesetzt war, gewinnt der Fehler und überschreibt.
        - Wenn bereits ein Fehler verarbeitet wurde, wird der zweite ignoriert.
        """
        if self._process_outcome == "error":
            _LOG.info("ProcessTab: zusätzlicher Fehler-Callback ignoriert: %s", msg)
            return

        if self._process_outcome == "success":
            _LOG.warning(
                "ProcessTab: Fehler-Callback nach notifyFinished erhalten, "
                "Fehler hat Vorrang: %s", msg
            )

        self._process_outcome = "error"

        self.set_process_active(False)
        self._info_update("process", f"Fehler: {msg}")
        self._append_process_log(f"=== Prozessfehler: {msg} ===")

        try:
            self.lblProcessState.setText(f"Fehler: {msg}")
        except RuntimeError:
            pass

        try:
            QMessageBox.critical(self, "Prozessfehler", f"Prozess abgebrochen:\n{msg}")
        except RuntimeError:
            pass

    def _on_process_thread_finished(self) -> None:
        self.set_process_active(False)

    @QtCore.pyqtSlot(str)
    def _on_process_state_changed(self, state: str) -> None:
        pretty = {
            "MOVE_PREDISPENSE": "Predispense-Position anfahren",
            "WAIT_PREDISPENSE": "Predispense-Zeit warten",
            "MOVE_RECIPE": "Rezeptpfad fahren",
            "WAIT_POSTDISPENSE": "Postdispense-Zeit warten",
            "MOVE_RETREAT": "Retreat-Position anfahren",
            "MOVE_HOME": "Home-Position anfahren",
            "ERROR": "Fehler – Prozess abgebrochen",
            "FINISHED": "Prozess abgeschlossen",
        }.get(state, state)

        try:
            self.lblProcessState.setText(f"Aktueller Schritt: {pretty}")
        except RuntimeError:
            pass

        try:
            prefix = "Prozess läuft.\n" if self._process_active else ""
            self.lblStatus.setText(f"{prefix}Aktueller Schritt: {pretty}")
        except RuntimeError:
            pass

        self._info_update("process_state", pretty)
        if self._recipe_name:
            self._info_update("recipe", self._recipe_name)

    # =====================================================================
    # Public API
    # =====================================================================

    def set_recipe_name(self, name: str | None) -> None:
        txt = (name or "").strip()
        self._recipe_name = txt if txt else None
        self._update_start_conditions()

    def _update_recipe_info_text(self) -> None:
        if not self._recipe_model:
            try:
                self.txtRecipeSummary.setPlainText("")
                self.txtRecipePoses.setPlainText("")
            except RuntimeError:
                return
            self._info_set_all(None)
            return

        try:
            full_text = str(self._recipe_model)
        except Exception:
            full_text = f"id: {self._recipe_model.id}\n(no detailed dump available)"

        lines: List[str] = full_text.splitlines()
        split_idx = None
        for i, line in enumerate(lines):
            if line.strip().startswith("# compiled poses"):
                split_idx = i
                break

        if split_idx is None:
            summary_lines = lines
            poses_lines: List[str] = []
        else:
            summary_lines = lines[:split_idx]
            poses_lines = lines[split_idx:]

        summary_text = "\n".join(summary_lines).rstrip()
        poses_text = "\n".join(poses_lines).rstrip()

        try:
            self.txtRecipeSummary.setPlainText(summary_text)
            self.txtRecipeSummary.moveCursor(self.txtRecipeSummary.textCursor().Start)

            self.txtRecipePoses.setPlainText(poses_text)
            self.txtRecipePoses.moveCursor(self.txtRecipePoses.textCursor().Start)
        except RuntimeError:
            return

        base_info = self._recipe_model.info or {}
        self._info_set_all(base_info)

    def set_robot_initialized(self, flag: bool) -> None:
        self._robot_initialized = bool(flag)
        self._update_start_conditions()

    def set_robot_at_home(self, flag: bool) -> None:
        self._robot_at_home = bool(flag)
        self._update_start_conditions()

    def set_tool_ok(self, flag: bool) -> None:
        self._tool_ok = bool(flag)
        self._update_start_conditions()

    def set_substrate_ok(self, flag: bool) -> None:
        self._substrate_ok = bool(flag)
        self._update_start_conditions()

    def set_mount_ok(self, flag: bool) -> None:
        self._mount_ok = bool(flag)
        self._update_start_conditions()

    def set_process_active(self, active: bool) -> None:
        self._process_active = bool(active)
        self._update_timer_state()

        if self._process_active:
            try:
                self.lblStatus.setText(
                    "Prozess läuft.\nStartbedingungen werden während der Ausführung nicht geprüft."
                )
            except RuntimeError:
                pass
        else:
            self._update_start_conditions()

    # =====================================================================
    # Startbedingungen + Timer
    # =====================================================================

    @staticmethod
    def _norm_mesh_name(name: str) -> str:
        n = os.path.basename(name or "").strip()
        if n.lower().endswith(".stl"):
            n = n[:-4]
        return n

    def _update_start_conditions(self) -> None:
        if self._in_update_start_conditions:
            return

        self._in_update_start_conditions = True
        try:
            self._update_setup_from_scene()
            self._evaluate_scene_match()

            missing: list[str] = []

            if not self._robot_initialized:
                missing.append("Roboter nicht initialisiert")

            if not self._robot_at_home:
                missing.append("Roboter nicht in Home-Position (TCP ≉ Home-Pose)")

            if not self._recipe_name:
                missing.append("Kein Rezept geladen")

            if not self._substrate_ok:
                missing.append("Substrate-Konfiguration stimmt nicht mit dem Rezept überein")

            if not self._mount_ok:
                missing.append("Mount-Konfiguration stimmt nicht mit dem Rezept überein")

            can_start = (len(missing) == 0)

            try:
                self.btnStart.setEnabled(can_start and not self._process_active)
            except RuntimeError:
                pass

            if self._process_active:
                return

            if can_start:
                txt = "Startbereit.\nAlle Startbedingungen sind erfüllt."
            else:
                msg_lines = ["Start nicht möglich.", "Fehlende Bedingungen:"]
                msg_lines += [f"- {m}" for m in missing]
                txt = "\n".join(msg_lines)

            try:
                self.lblStatus.setText(txt)
            except RuntimeError:
                pass
        finally:
            self._in_update_start_conditions = False

    def _update_timer_state(self) -> None:
        if self._process_active:
            if self._condTimer.isActive():
                self._condTimer.stop()
        else:
            if not self._condTimer.isActive():
                self._condTimer.start()

    # =====================================================================
    # Scene-Match + Setup-Anzeige
    # =====================================================================

    def _wire_scene_bridge(self) -> None:
        if self._sb is None:
            self.set_substrate_ok(False)
            self.set_mount_ok(False)
            self._set_setup_labels("-", "-", "-")
            return

        sig = getattr(self._sb, "signals", None)
        if sig is None:
            self.set_substrate_ok(False)
            self.set_mount_ok(False)
            self._set_setup_labels("-", "-", "-")
            return

        if hasattr(sig, "substrateCurrentChanged"):
            sig.substrateCurrentChanged.connect(self._on_scene_changed)
        if hasattr(sig, "mountCurrentChanged"):
            sig.mountCurrentChanged.connect(self._on_scene_changed)
        if hasattr(sig, "toolCurrentChanged"):
            sig.toolCurrentChanged.connect(self._on_scene_changed)

        self._update_setup_from_scene()
        self._evaluate_scene_match()

    def _on_scene_changed(self, *_args) -> None:
        self._update_setup_from_scene()
        self._evaluate_scene_match()

    def _get_scene_currents(self) -> tuple[str, str, str]:
        if self._sb is None:
            return "", "", ""

        tool = getattr(self._sb, "tool_current", "") or getattr(self._sb, "tool", "") or ""
        substrate = getattr(self._sb, "substrate_current", "") or ""
        mount = getattr(self._sb, "mount_current", "") or ""

        return str(tool), str(substrate), str(mount)

    def _update_setup_from_scene(self) -> None:
        tool, substrate, mount = self._get_scene_currents()

        if not tool and self._rb is not None:
            tool = getattr(self._rb, "current_tool", "") or getattr(self._rb, "tool", "")

        self._set_setup_labels(tool or "-", substrate or "-", mount or "-")

    def _set_setup_labels(self, tool: str, substrate: str, mount: str) -> None:
        try:
            self.lblTool.setText(f"Tool: {tool}")
            self.lblSubstrate.setText(f"Substrate: {substrate}")
            self.lblMount.setText(f"Mount: {mount}")
        except RuntimeError:
            pass

    def _evaluate_scene_match(self) -> None:
        if not self._recipe_model:
            self.set_substrate_ok(False)
            self.set_mount_ok(False)
            return

        _, cur_sub, cur_mount = self._get_scene_currents()

        cur_sub_norm = self._norm_mesh_name(cur_sub)
        cur_mount_norm = self._norm_mesh_name(cur_mount)

        rec_sub_norm = self._norm_mesh_name(self._recipe_model.substrate or "")
        rec_mount_norm = self._norm_mesh_name(self._recipe_model.substrate_mount or "")

        sub_ok = bool(rec_sub_norm) and (cur_sub_norm == rec_sub_norm)
        # sub_ok wird derzeit nur für Startbedingungen verwendet
        self.set_substrate_ok(sub_ok)

        mount_ok = bool(rec_mount_norm) and (cur_mount_norm == rec_mount_norm)
        self.set_mount_ok(mount_ok)
