#!/usr/bin/env python3
# source/app/main_gui.py

import os
import sys

# Stelle sicher, dass /root/app im Pythonpfad liegt
APP_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
if APP_ROOT not in sys.path:
    sys.path.insert(0, APP_ROOT)

from PyQt5.QtWidgets import QApplication, QTabWidget
from PyQt5.QtCore import QCoreApplication
import rclpy

# Bridge (kapselt Tool/Motion/Jog/Scene usw.)
from src.ros.bridge.ui_bridge import UIBridge

# Tabs
from src.gui.tabs.process import ProcessTab
from src.gui.tabs.spray_path import SprayPathTab
from src.gui.tabs.service import ServiceTab
from src.gui.tabs.system import SystemTab

# Bringup
from src.app.bringup import ensure_clean_graph_then_launch, shutdown_bringup
from src.app.startup import LaunchConfig

import sys
from PyQt5 import QtCore
QtCore.pyqtRemoveInputHook()

LAUNCH_CMD = ["ros2", "launch", "mecademic_bringup", "bringup.launch.py"]


def main():
        
    # -------- ENV Fix für PyQt / ROS in Docker --------
    os.environ.setdefault("XDG_RUNTIME_DIR", "/tmp/runtime-root")
    os.makedirs(os.environ["XDG_RUNTIME_DIR"], exist_ok=True)

    # -------- Startup Config Laden --------
    cfg = LaunchConfig.load()
    print(f"[app] startup config loaded (launch={cfg.launch})", flush=True)

    # -------- Bringup (optional) --------
    if cfg.launch:
        extra_args = cfg.as_launch_args()
        print("[app] launching bringup…", flush=True)
        ensure_clean_graph_then_launch(LAUNCH_CMD, extra_launch_args=extra_args)
    else:
        print("[app] launch disabled by startup.yaml", flush=True)

    # -------- ROS Node --------
    rclpy.init()
    bridge = UIBridge()

    # -------- Qt --------
    app = QApplication(sys.argv)

    # -------- ROS Spin Mode auswählen --------
    USE_THREAD_SPINNER = os.getenv("ROS_SPIN_MODE", "qt").lower() == "thread"
    timer = None
    spinner = None

    if USE_THREAD_SPINNER:
        from src.ros.exec.spin import spin_in_thread
        spinner = spin_in_thread(bridge, rate_hz=100)
        print("[app] ROS spin mode: THREAD (100 Hz)", flush=True)
    else:
        from src.ros.exec.spin import start_qt_spin
        timer = start_qt_spin(bridge, interval_ms=10)
        print("[app] ROS spin mode: QT (10ms interval)", flush=True)

    # -------- Cleanup --------
    def _cleanup():
        try:
            if spinner:
                spinner.stop()
        except:
            pass
        try:
            if timer:
                timer.stop()
        except:
            pass
        try:
            bridge.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass
        try:
            shutdown_bringup()
        except:
            pass

    QCoreApplication.instance().aboutToQuit.connect(_cleanup)

    # -------- Tabs --------
    tabs = QTabWidget()
    tabs.setWindowTitle("Meca – Control Suite")
    tabs.addTab(ProcessTab(bridge), "Prozess")
    tabs.addTab(SprayPathTab(bridge), "Spray Path")
    tabs.addTab(ServiceTab(bridge), "Service")
    tabs.addTab(SystemTab(bridge), "System")
    tabs.resize(1200, 900)
    tabs.show()

    # -------- Start --------
    code = app.exec_()
    _cleanup()
    sys.exit(code)


if __name__ == "__main__":
    main()
