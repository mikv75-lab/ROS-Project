# scripts/pyqt_embed_rviz.py
import os, sys, ctypes
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout
from PyQt5.QtGui import QWindow
from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtCore import Qt

def main():
    app = QApplication(sys.argv)

    # Load our C++ host library (installed by the package)
    libpath = os.path.join(os.environ.get("COLCON_CURRENT_PREFIX", "/root/ws_moveit/install"),
                           "embedded_rviz", "lib", "libembedded_rviz_host.so")
    lib = ctypes.CDLL(libpath)

    # C bindings
    lib.erviz_create_panel.restype  = ctypes.c_int
    lib.erviz_create_panel.argtypes = [ctypes.c_char_p, ctypes.c_char_p]

    lib.erviz_get_wid.restype  = ctypes.c_ulonglong
    lib.erviz_get_wid.argtypes = [ctypes.c_int]

    lib.erviz_destroy_panel.restype  = None
    lib.erviz_destroy_panel.argtypes = [ctypes.c_int]

    # Create RViz widget instance
    node = b"embedded_rviz_py"
    cfg  = b""  # or b"/path/to/config.rviz"
    pid = lib.erviz_create_panel(node, cfg)

    wid = lib.erviz_get_wid(pid)
    if wid == 0:
        print("Failed to create RViz widget")
        sys.exit(1)

    # Wrap native WId as QWindow, then into QWidget for layouts
    rviz_window = QWindow.fromWinId(wid)
    container = QWidget.createWindowContainer(rviz_window)
    container.setFocusPolicy(Qt.TabFocus)

    root = QWidget()
    lay  = QVBoxLayout(root)
    lay.setContentsMargins(0,0,0,0)
    lay.addWidget(container)
    root.resize(1000, 700)
    root.setWindowTitle("PyQt5 + Embedded RViz")
    root.show()

    code = app.exec_()
    lib.erviz_destroy_panel(pid)
    sys.exit(code)

if __name__ == "__main__":
    main()
