# -*- coding: utf-8 -*-
# src/config/plc_config.py

from __future__ import annotations

import io
import os
from dataclasses import dataclass
from typing import Any, Dict, Optional

import yaml


@dataclass(frozen=True)
class PlcConfig:
    """
    Konfiguration der SPS-Anbindung (aus startup.yaml + plc.yaml).

    Aktuell gibt es nur noch den Modus 'ads':
      - ADS-Verbindung wird versucht
      - falls pyads fehlt oder Connect fehlschlägt, arbeitet der PlcClient
        im Offline-/YAML-Modus weiter (lokale Spec).
    """

    # Modus: aktuell nur 'ads' unterstützt (für Zukunft evtl. 'local', etc.)
    mode: str

    # Ob beim Programmstart automatisch versucht werden soll, die SPS zu verbinden
    connect_on_start: bool

    # ADS-Verbindungsdaten
    ads_ams_net_id: Optional[str] = None
    ads_ip: Optional[str] = None
    ads_port: int = 851

    # PLC-Spezifikation (plc.yaml) – Typen, Min/Max, Mapping etc.
    spec_path: Optional[str] = None
    spec: Optional[Dict[str, Any]] = None


# ----------------------------------------------------------------------
# Hilfsfunktionen
# ----------------------------------------------------------------------


def _load_yaml(path: str) -> Dict[str, Any]:
    path = os.path.abspath(os.path.normpath(path))
    if not os.path.exists(path):
        raise FileNotFoundError(f"YAML nicht gefunden: {path}")
    with io.open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f"YAML ist kein Mapping: {path}")
    return data


def load_plc_config(startup_yaml_path: str) -> PlcConfig:
    """
    Lies die PLC-Konfiguration aus startup.yaml und plc.yaml.

    Erwartete Struktur in startup.yaml:

      plc:
        mode: ads                # (optional, default 'ads')
        connect_on_start: true   # (optional, default True)
        ads:
          ams_net_id: "1.2.3.4.5.6"
          ip: "192.168.0.10"
          port: 851
        spec_file: "plc.yaml"    # relativ zu startup.yaml

    Hinweis:
      - Es gibt keinen 'mock'-Modus mehr.
      - Fällt die ADS-Verbindung aus, übernimmt der PlcClient selbst
        den Fallback auf die lokale Spec/YAML.
    """
    startup_yaml_path = os.path.abspath(os.path.normpath(startup_yaml_path))
    su = _load_yaml(startup_yaml_path)

    plc_block = su.get("plc") or {}
    if not isinstance(plc_block, dict):
        raise ValueError("startup.yaml: Abschnitt 'plc' fehlt oder ist ungültig.")

    # Standard: 'ads'
    mode = str(plc_block.get("mode", "ads")).strip().lower()

    # Aktuell nur 'ads' erlaubt – alles andere ist Konfigurationsfehler
    if mode != "ads":
        raise ValueError(
            f"startup.yaml: plc.mode muss 'ads' sein (mock wird nicht mehr unterstützt, ist {mode!r})."
        )

    connect_on_start = bool(plc_block.get("connect_on_start", True))

    ads_block = plc_block.get("ads") or {}
    if not isinstance(ads_block, dict):
        ads_block = {}

    ams_net_id = ads_block.get("ams_net_id")
    ads_ip = ads_block.get("ip")
    ads_port = int(ads_block.get("port", 851))

    # plc.yaml (Spec) laden
    spec_file_rel = plc_block.get("spec_file", "plc.yaml")
    base = os.path.dirname(startup_yaml_path)
    spec_path = os.path.abspath(os.path.join(base, spec_file_rel))
    spec = _load_yaml(spec_path)

    return PlcConfig(
        mode=mode,
        connect_on_start=connect_on_start,
        ads_ams_net_id=ams_net_id,
        ads_ip=ads_ip,
        ads_port=ads_port,
        spec_path=spec_path,
        spec=spec,
    )
