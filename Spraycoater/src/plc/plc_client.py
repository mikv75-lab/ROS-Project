# -*- coding: utf-8 -*-
# src/app/plc_client.py
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Dict, Optional, Callable

from plc.plc_config import PlcConfig

LOG = logging.getLogger(__name__)

try:
    import pyads  # type: ignore
except Exception:  # pragma: no cover
    pyads = None

try:
    import yaml  # für Spec-Export
except Exception:  # pragma: no cover
    yaml = None


# ======================================================================
# Datenstrukturen
# ======================================================================

@dataclass
class PlcStatus:
    connected: bool = False
    process_active: bool = False
    process_done: bool = False
    process_error: bool = False
    interlock_ok: bool = False
    vacuum_ok: bool = False
    alarm_code: int = 0


# ======================================================================
# Basis-Interface
# ======================================================================

class PlcClientBase:
    """
    Abstraktes SPS-Interface. UI und Prozesslogik reden nur hiermit.

    WICHTIG:
      - self._spec    : aktuelle PLC-Spezifikation (aus plc.yaml über PlcConfig.spec)
        → hier kannst du beliebige Sections nutzen, z.B.:
            common:   {...}   # Commands, Defaults, Meta
            error:    {...}   # Fehlercodes/-texte
            io:
              outputs: {...}  # Signale mit Typ, Beschreibung, ads_symbol, min/max…
              inputs:  {...}
            ads_mapping:
              recipe_params: {...}
              outputs:       {...}
              inputs:        {...}

      - self._ads_mapping:
          {
            "recipe_params": { key -> ads_symbol },
            "outputs":      { key -> ads_symbol },
            "inputs":       { key -> ads_symbol },
          }

      Min/Max, Units usw. stehen einfach in self._spec und werden 1:1
      beim Export nach YAML wieder ausgegeben.
    """

    def __init__(self, cfg: PlcConfig):
        self._cfg = cfg
        self._connected = False

        # Roh-Spec aus Config (z.B. aus plc.yaml)
        self._spec: Dict[str, Any] = dict(cfg.spec or {})
        self._ads_mapping: Dict[str, Dict[str, str]] = self._spec.get("ads_mapping", {}) or {}
        self._recipe_map: Dict[str, str] = self._ads_mapping.get("recipe_params", {}) or {}
        self._out_map: Dict[str, str] = self._ads_mapping.get("outputs", {}) or {}
        self._in_map: Dict[str, str] = self._ads_mapping.get("inputs", {}) or {}

    # ------------------------------------------------------------------
    # Basis-Status / Spec-Zugriff
    # ------------------------------------------------------------------

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def spec(self) -> Dict[str, Any]:
        """
        Aktuelle PLC-Spezifikation (aus plc.yaml und evtl. modifiziert zur Laufzeit).
        """
        return self._spec

    def _update_spec(self, new_spec: Dict[str, Any]) -> None:
        """
        Interne Helper-Funktion, um die Spec konsistent auszutauschen.
        (Aktuell nicht von der SPS überschrieben, aber z.B. nutzbar,
         falls du später dynamisch etwas änderst.)
        """
        self._spec = dict(new_spec or {})
        self._ads_mapping = self._spec.get("ads_mapping", {}) or {}
        self._recipe_map = self._ads_mapping.get("recipe_params", {}) or {}
        self._out_map = self._ads_mapping.get("outputs", {}) or {}
        self._in_map = self._ads_mapping.get("inputs", {}) or {}

    # --- Interface-Methoden, von Unterklassen zu implementieren ---

    def connect(self) -> None:
        raise NotImplementedError

    def disconnect(self) -> None:
        raise NotImplementedError

    def write_recipe_params(self, params: Dict[str, Any]) -> None:
        """Rezeptparameter in die SPS schreiben (z.B. beim Laden)."""
        raise NotImplementedError

    def set_media_state(self, media_on: bool, syringe_only: bool = False) -> None:
        """
        Medienkette ein/aus.
        Vereinfachung:
            media_on=True  -> N2 + US + Kartusche ON
            media_on=False -> Kartusche STOP + US OFF + N2 OFF
        """
        raise NotImplementedError

    def send_process_start(self) -> None:
        """Prozessstart-Bit zur SPS schicken."""
        raise NotImplementedError

    def send_process_abort(self) -> None:
        """Abbruchanforderung an die SPS schicken."""
        raise NotImplementedError

    def read_status(self) -> PlcStatus:
        """
        Status-Bits aus der SPS lesen und als PlcStatus zurückgeben.
        Wird z.B. in init_plc() der Tabs verwendet.
        """
        raise NotImplementedError

    # ------------------------------------------------------------------
    # Spec-Export (für System-Tab-Button)
    # ------------------------------------------------------------------

    def export_spec_to_yaml(self, path: str) -> None:
        """
        Exportiert die aktuelle Spec (self._spec) als YAML-Datei.

        Usecase: System-Tab "PLC-Spec nach YAML exportieren" → ruft
          plc.export_spec_to_yaml(ctx.paths.plc_yaml_path)

        Die Datei enthält ALLE Sections (common, error, io, ads_mapping, …),
        genau so wie sie aktuell im Speicher stehen.
        """
        if yaml is None:
            LOG.warning("export_spec_to_yaml: yaml-Modul nicht verfügbar, Export übersprungen.")
            return

        try:
            with open(path, "w", encoding="utf-8") as f:
                yaml.safe_dump(
                    self._spec,
                    f,
                    sort_keys=False,
                    allow_unicode=True,
                    default_flow_style=False,
                )
            LOG.info("PlcClientBase: Spec nach '%s' exportiert.", path)
        except Exception as e:
            LOG.error("PlcClientBase: export_spec_to_yaml(%s) fehlgeschlagen: %s", path, e)

    # ------------------------------------------------------------------
    # Zusätzliche Convenience-API für ProcessTab
    # ------------------------------------------------------------------

    def _recipe_to_param_dict(self, recipe: Any) -> Dict[str, Any]:
        """
        Versucht, aus einem Recipe-Objekt ein Dict[str, Any] für die SPS zu erzeugen.

        Erwartete Konventionen (alles optional):
          - recipe.to_plc_params() -> Dict[str, Any]
          - recipe.plc_params      -> Dict[str, Any]
          - recipe.to_dict()["plc_params"] -> Dict[str, Any]
        """
        if recipe is None:
            return {}

        # 1) to_plc_params()
        if hasattr(recipe, "to_plc_params"):
            try:
                d = recipe.to_plc_params()
                if isinstance(d, dict):
                    return d
            except Exception as e:
                LOG.warning("PlcClientBase: recipe.to_plc_params() fehlgeschlagen: %s", e)

        # 2) Attribut plc_params
        if hasattr(recipe, "plc_params"):
            try:
                d = getattr(recipe, "plc_params")
                if isinstance(d, dict):
                    return d
            except Exception as e:
                LOG.warning("PlcClientBase: Zugriff auf recipe.plc_params fehlgeschlagen: %s", e)

        # 3) to_dict()["plc_params"]
        if hasattr(recipe, "to_dict"):
            try:
                d = recipe.to_dict()
                if isinstance(d, dict):
                    pp = d.get("plc_params")
                    if isinstance(pp, dict):
                        return pp
            except Exception as e:
                LOG.warning("PlcClientBase: recipe.to_dict()['plc_params'] fehlgeschlagen: %s", e)

        LOG.info("PlcClientBase: Keine PLC-Parameter im Recipe gefunden, sende nichts.")
        return {}

    def send_recipe_params(self, recipe: Any) -> None:
        """
        Convenience für den ProcessTab:
        - extrahiert PLC-Parameter aus dem Recipe (siehe _recipe_to_param_dict)
        - ruft write_recipe_params(params)
        """
        params = self._recipe_to_param_dict(recipe)
        if not params:
            return
        LOG.debug("PlcClientBase: sende %d Rezept-Parameter an SPS", len(params))
        self.write_recipe_params(params)

    def send_process_init(self) -> None:
        """
        Separates Init-Kommando an die SPS.
        Default-Implementierung macht nichts – AdsPlcClient
        kann das überschreiben.
        """
        LOG.debug("PlcClientBase.send_process_init(): nicht implementiert (no-op).")

    def send_process_stop(self) -> None:
        """
        Separates Stop-Kommando an die SPS (unabhängig vom Prozess-Abbruch).
        Default-Implementierung macht nichts – AdsPlcClient
        kann das überschreiben.
        """
        LOG.debug("PlcClientBase.send_process_stop(): nicht implementiert (no-op).")

    # ------------------------------------------------------------------
    # Callback-Registrierung (für Tabs)
    # ------------------------------------------------------------------

    def register_bool_callback(self, key: str, cb: Callable[[bool], None]) -> None:
        """
        Default: keine Callbacks.
        AdsPlcClient implementiert das via pyads-device-notification.
        """
        LOG.debug("PlcClientBase.register_bool_callback(%s, %s) – nicht implementiert", key, cb)

    def register_int_callback(self, key: str, cb: Callable[[int], None]) -> None:
        LOG.debug("PlcClientBase.register_int_callback(%s, %s) – nicht implementiert", key, cb)


# ======================================================================
# ADS-Implementierung (einzige konkrete PLC-Klasse)
# ======================================================================

class AdsPlcClient(PlcClientBase):
    """
    Echte Beckhoff-SPS via PyADS.

    Erwartet:
      - cfg.ads_ams_net_id
      - cfg.ads_ip
      - cfg.ads_port (standard 851)
      - cfg.spec mit:
          * ads_mapping.* aus plc.yaml
          * io.inputs/outputs, common, error, ... (frei definierbar)
    """

    def __init__(self, cfg: PlcConfig):
        if pyads is None:
            raise RuntimeError("pyads ist nicht verfügbar, AdsPlcClient kann nicht verwendet werden.")
        super().__init__(cfg)

        if not cfg.ads_ams_net_id or not cfg.ads_ip:
            raise ValueError("AdsPlcClient: AMS-Net-ID oder IP in PlcConfig fehlen.")

        self._net_id = cfg.ads_ams_net_id
        self._ip = cfg.ads_ip
        self._port = cfg.ads_port or 851
        self._conn: Optional[pyads.Connection] = None

        # Notification-Handles nach key (inputs-keys aus ads_mapping.inputs)
        self._bool_notification_handles: Dict[str, Any] = {}
        self._int_notification_handles: Dict[str, Any] = {}

    # interne Helfer -----------------------------------------------------

    def _ensure_conn(self) -> pyads.Connection:
        if self._conn is None:
            raise RuntimeError("ADS-Verbindung ist nicht aufgebaut.")
        return self._conn

    def _write_by_symbol(self, symbol: str, value: Any) -> None:
        conn = self._ensure_conn()
        try:
            conn.write_by_name(symbol, value)
        except Exception as e:
            LOG.error("ADS write_by_name(%s, %r) fehlgeschlagen: %s", symbol, value, e)
            raise

    def _read_by_symbol(self, symbol: str) -> Any:
        conn = self._ensure_conn()
        try:
            return conn.read_by_name(symbol)
        except Exception as e:
            LOG.error("ADS read_by_name(%s) fehlgeschlagen: %s", symbol, e)
            raise

    # Public API ---------------------------------------------------------

    def connect(self) -> None:
        if self._conn is not None:
            return
        LOG.info(
            "AdsPlcClient: Verbinde zu %s:%s (AMS=%s)",
            self._ip,
            self._port,
            self._net_id,
        )
        self._conn = pyads.Connection(self._net_id, self._port, self._ip)
        self._conn.open()
        self._connected = True

    def disconnect(self) -> None:
        if self._conn is None:
            return
        LOG.info("AdsPlcClient: Verbindung trennen.")
        try:
            self._conn.close()
        finally:
            self._conn = None
            self._connected = False

    # ------------------------------------------------------------------
    # Schreiben / Lesen der Prozessdaten
    # ------------------------------------------------------------------

    def write_recipe_params(self, params: Dict[str, Any]) -> None:
        if not self._connected:
            LOG.warning("AdsPlcClient: write_recipe_params() ohne Connection")

        for key, val in params.items():
            sym = self._recipe_map.get(key)
            if not sym:
                LOG.debug("AdsPlcClient: Kein ADS-Mapping für recipe_param '%s'", key)
                continue
            LOG.debug("AdsPlcClient: Schreibe %s -> %s = %r", key, sym, val)
            try:
                self._write_by_symbol(sym, val)
            except Exception:
                # Fehler schon geloggt
                continue

    def set_media_state(self, media_on: bool, syringe_only: bool = False) -> None:
        if not self._connected:
            LOG.warning("AdsPlcClient: set_media_state() ohne Connection")

        if media_on:
            # ON-Sequenz
            if not syringe_only:
                if "media_n2_on" in self._out_map:
                    try:
                        self._write_by_symbol(self._out_map["media_n2_on"], True)
                    except Exception:
                        pass
                if "media_us_on" in self._out_map:
                    try:
                        self._write_by_symbol(self._out_map["media_us_on"], True)
                    except Exception:
                        pass
            if "media_syringe_start" in self._out_map:
                try:
                    self._write_by_symbol(self._out_map["media_syringe_start"], True)
                except Exception:
                    pass
            if "media_syringe_stop" in self._out_map:
                try:
                    self._write_by_symbol(self._out_map["media_syringe_stop"], False)
                except Exception:
                    pass
        else:
            # OFF-Sequenz
            if "media_syringe_start" in self._out_map:
                try:
                    self._write_by_symbol(self._out_map["media_syringe_start"], False)
                except Exception:
                    pass
            if "media_syringe_stop" in self._out_map:
                try:
                    self._write_by_symbol(self._out_map["media_syringe_stop"], True)
                except Exception:
                    pass
            if "media_us_on" in self._out_map:
                try:
                    self._write_by_symbol(self._out_map["media_us_on"], False)
                except Exception:
                    pass
            if "media_n2_on" in self._out_map:
                try:
                    self._write_by_symbol(self._out_map["media_n2_on"], False)
                except Exception:
                    pass

    def send_process_start(self) -> None:
        if not self._connected:
            LOG.warning("AdsPlcClient: send_process_start() ohne Connection")
        sym = self._out_map.get("process_start")
        if not sym:
            LOG.warning("AdsPlcClient: Kein ADS-Symbol für process_start")
            return
        LOG.info("AdsPlcClient: Prozessstart (Symbol=%s)", sym)
        try:
            self._write_by_symbol(sym, True)
        except Exception:
            pass

    def send_process_abort(self) -> None:
        if not self._connected:
            LOG.warning("AdsPlcClient: send_process_abort() ohne Connection")
        sym = self._out_map.get("process_abort")
        if not sym:
            LOG.warning("AdsPlcClient: Kein ADS-Symbol für process_abort")
            return
        LOG.info("AdsPlcClient: Prozessabbruch (Symbol=%s)", sym)
        try:
            self._write_by_symbol(sym, True)
        except Exception:
            pass

    def read_status(self) -> PlcStatus:
        st = PlcStatus(connected=self._connected)

        if not self._connected:
            # Keine Verbindung -> default Status (nur connected=False)
            return st

        def _read_bool(key: str, attr: str):
            sym = self._in_map.get(key)
            if not sym:
                return
            try:
                val = bool(self._read_by_symbol(sym))
                setattr(st, attr, val)
            except Exception:
                # Fehler wurde im _read_by_symbol geloggt
                pass

        def _read_int(key: str, attr: str):
            sym = self._in_map.get(key)
            if not sym:
                return
            try:
                val = int(self._read_by_symbol(sym))
                setattr(st, attr, val)
            except Exception:
                pass

        _read_bool("process_active", "process_active")
        _read_bool("process_done", "process_done")
        _read_bool("process_error", "process_error")
        _read_bool("interlock_ok", "interlock_ok")
        _read_bool("vacuum_ok", "vacuum_ok")
        _read_int("alarm_code", "alarm_code")

        return st

    # Neue Convenience-Methoden ----------------------------------------

    def send_process_init(self) -> None:
        if not self._connected:
            LOG.warning("AdsPlcClient: send_process_init() ohne Connection")
        sym = self._out_map.get("process_init")
        if not sym:
            LOG.warning("AdsPlcClient: Kein ADS-Symbol für process_init")
            return
        LOG.info("AdsPlcClient: Prozess-Init (Symbol=%s)", sym)
        try:
            self._write_by_symbol(sym, True)
        except Exception:
            pass

    def send_process_stop(self) -> None:
        if not self._connected:
            LOG.warning("AdsPlcClient: send_process_stop() ohne Connection")
        sym = self._out_map.get("process_stop")
        if not sym:
            LOG.warning("AdsPlcClient: Kein ADS-Symbol für process_stop")
            return
        LOG.info("AdsPlcClient: Prozess-Stop (Symbol=%s)", sym)
        try:
            self._write_by_symbol(sym, True)
        except Exception:
            pass

    def send_recipe_params(self, recipe: Any) -> None:
        """
        Verwendet die Basis-Logik (_recipe_to_param_dict + write_recipe_params),
        loggt nur etwas detaillierter.
        """
        params = self._recipe_to_param_dict(recipe)
        if not params:
            LOG.info("AdsPlcClient: send_recipe_params() – keine Parameter gefunden.")
            return
        LOG.info("AdsPlcClient: sende Rezeptparameter (keys=%s)", list(params.keys()))
        self.write_recipe_params(params)

    # ------------------------------------------------------------------
    # Callback-Registrierung via pyads add_device_notification
    # ------------------------------------------------------------------

    def register_bool_callback(self, key: str, cb: Callable[[bool], None]) -> None:
        if not self._connected:
            LOG.warning("AdsPlcClient: register_bool_callback(%s) ohne Connection", key)
            return

        sym = self._in_map.get(key)
        if not sym:
            LOG.warning("AdsPlcClient: Kein ADS-Mapping für bool-input '%s'", key)
            return

        conn = self._ensure_conn()

        def _notify(handle, name, value):
            try:
                cb(bool(value))
            except Exception as e:
                LOG.error("Bool-Callback für '%s' fehlgeschlagen: %s", key, e)

        try:
            handle = conn.add_device_notification(
                sym,
                pyads.PLCTYPE_BOOL,
                _notify
            )
            self._bool_notification_handles[key] = handle
            LOG.info("AdsPlcClient: Bool-Notification für '%s' (%s) registriert", key, sym)
        except Exception as e:
            LOG.error("AdsPlcClient: add_device_notification(%s) fehlgeschlagen: %s", sym, e)

    def register_int_callback(self, key: str, cb: Callable[[int], None]) -> None:
        if not self._connected:
            LOG.warning("AdsPlcClient: register_int_callback(%s) ohne Connection", key)
            return

        sym = self._in_map.get(key)
        if not sym:
            LOG.warning("AdsPlcClient: Kein ADS-Mapping für int-input '%s'", key)
            return

        conn = self._ensure_conn()

        def _notify(handle, name, value):
            try:
                cb(int(value))
            except Exception as e:
                LOG.error("Int-Callback für '%s' fehlgeschlagen: %s", key, e)

        try:
            handle = conn.add_device_notification(
                sym,
                pyads.PLCTYPE_INT,
                _notify
            )
            self._int_notification_handles[key] = handle
            LOG.info("AdsPlcClient: Int-Notification für '%s' (%s) registriert", key, sym)
        except Exception as e:
            LOG.error("AdsPlcClient: add_device_notification(%s) fehlgeschlagen: %s", sym, e)


# ======================================================================
# Factory-Funktion
# ======================================================================

def create_plc_client(cfg: PlcConfig) -> PlcClientBase:
    """
    Erzeugt immer einen AdsPlcClient.

    Wenn:
      - pyads nicht verfügbar ist ODER
      - Konfiguration unvollständig ist

    wird eine Exception geworfen. Der Aufrufer (StartupMachine) kann das
    abfangen und die Anwendung dann ohne PLC-Funktionalität starten.

    Fallback-Logik (deine Seite):
      - Wenn create_plc_client() / connect() fehlschlägt:
          * plc = None
          * Tabs sehen plc=None → reine Anzeige / lokale Spec
      - Wenn Verbindung klappt:
          * Spec bleibt wie aus plc.yaml geladen
          * export_spec_to_yaml() kann sie auf Wunsch wieder überschreiben
    """
    LOG.info("create_plc_client: Erzeuge AdsPlcClient (mode=%r)", getattr(cfg, "mode", None))
    return AdsPlcClient(cfg)
