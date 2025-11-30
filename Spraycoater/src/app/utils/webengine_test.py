# webengine_test.py
from __future__ import annotations
import os
import sys

# Sandbox im Container ausschalten (root + Docker)
os.environ.setdefault("QTWEBENGINE_DISABLE_SANDBOX", "1")

from PyQt6.QtCore import QUrl
from PyQt6.QtWidgets import QApplication
from PyQt6.QtWebEngineWidgets import QWebEngineView


def main() -> None:
    app = QApplication(sys.argv)

    view = QWebEngineView()
    view.setWindowTitle("Qt WebEngine Test")
    view.resize(1024, 768)

    # Zum Testen erstmal was Einfaches:
    # später z.B. QUrl("https://app.foxglove.dev")
    view.load(QUrl("https://example.com"))
    view.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
