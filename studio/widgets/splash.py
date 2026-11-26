"""
Lightweight Startup Splash Screen for pyCub Studio.

Uses only core PySide6 widgets so it renders immediately while the GUI theme
and Studio mode panels initialize. Heavy physics/3D engines run in worker
subprocesses and are not imported in the GUI main process.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Optional

from PySide6.QtCore import Qt
from PySide6.QtGui import QIcon, QPixmap
from PySide6.QtWidgets import (
    QApplication,
    QFrame,
    QHBoxLayout,
    QLabel,
    QProgressBar,
    QVBoxLayout,
    QWidget,
)


def locate_app_icon() -> Optional[str]:
    """Resolve `studio/assets/icons/pycub.png` across source and frozen builds."""
    candidates = [
        Path(__file__).resolve().parents[1] / "assets" / "icons" / "pycub.png",
    ]
    meipass = getattr(sys, "_MEIPASS", None)
    if meipass:
        candidates.append(Path(meipass) / "studio" / "assets" / "icons" / "pycub.png")
    if getattr(sys, "frozen", False):
        exe_dir = Path(sys.executable).resolve().parent
        candidates.append(exe_dir / "_internal" / "studio" / "assets" / "icons" / "pycub.png")
        candidates.append(exe_dir / "studio" / "assets" / "icons" / "pycub.png")

    for cand in candidates:
        if cand.is_file():
            return str(cand)
    return None


class StudioSplashScreen(QWidget):
    """Frameless dark-navy splash card with pyCub icon, status text, and progress bar."""

    def __init__(self, icon_path: Optional[str] = None, lang: str = "en"):
        """Initialize the splash screen widget with optional icon path and UI language."""
        super().__init__(
            None,
            Qt.WindowType.SplashScreen
            | Qt.WindowType.FramelessWindowHint
            | Qt.WindowType.WindowStaysOnTopHint,
        )
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)
        self.setFixedSize(460, 230)

        resolved_icon = icon_path if (icon_path and os.path.isfile(icon_path)) else locate_app_icon()
        if resolved_icon:
            self.setWindowIcon(QIcon(resolved_icon))

        outer = QVBoxLayout(self)
        outer.setContentsMargins(10, 10, 10, 10)

        card = QFrame(self)
        card.setObjectName("splashCard")
        card.setStyleSheet(
            """
            QFrame#splashCard {
                background: #0f172a;
                border: 2px solid #2563eb;
                border-radius: 18px;
            }
            """
        )
        outer.addWidget(card)

        lay = QVBoxLayout(card)
        lay.setContentsMargins(24, 20, 24, 20)
        lay.setSpacing(14)

        top = QHBoxLayout()
        top.setSpacing(16)

        icon_lbl = QLabel(card)
        icon_lbl.setFixedSize(84, 84)
        icon_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        if resolved_icon:
            raw_pix = QPixmap(resolved_icon)
            if not raw_pix.isNull():
                pix = raw_pix.scaled(
                    84,
                    84,
                    Qt.AspectRatioMode.KeepAspectRatio,
                    Qt.TransformationMode.SmoothTransformation,
                )
                icon_lbl.setPixmap(pix)
        top.addWidget(icon_lbl)

        title_col = QVBoxLayout()
        title_col.setSpacing(4)
        title_lbl = QLabel("pyCub Studio", card)
        title_lbl.setStyleSheet("font-size: 26px; font-weight: 900; color: #f8fafc; background: transparent;")
        title_col.addWidget(title_lbl)

        sub_txt = (
            "Simulátor humanoidního robota iCub"
            if lang == "cs"
            else "iCub Humanoid Robot Simulator"
        )
        sub_lbl = QLabel(sub_txt, card)
        sub_lbl.setStyleSheet("font-size: 13px; color: #94a3b8; background: transparent;")
        title_col.addWidget(sub_lbl)
        title_col.addStretch(1)
        top.addLayout(title_col, 1)

        lay.addLayout(top)
        lay.addStretch(1)

        init_txt = "Spouštím pyCub Studio…" if lang == "cs" else "Starting pyCub Studio…"
        self.status_lbl = QLabel(init_txt, card)
        self.status_lbl.setStyleSheet("font-size: 12px; font-weight: 600; color: #cbd5e1; background: transparent;")
        lay.addWidget(self.status_lbl)

        self.progress = QProgressBar(card)
        self.progress.setRange(0, 100)
        self.progress.setValue(5)
        self.progress.setTextVisible(False)
        self.progress.setFixedHeight(10)
        self.progress.setStyleSheet(
            """
            QProgressBar {
                background: #1e293b;
                border: 1px solid #334155;
                border-radius: 5px;
            }
            QProgressBar::chunk {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #2563eb, stop:1 #38bdf8);
                border-radius: 4px;
            }
            """
        )
        lay.addWidget(self.progress)

        self._center_on_screen()

    def _center_on_screen(self) -> None:
        """Center the splash window within the primary screen's available geometry."""
        screen = QApplication.primaryScreen()
        if screen is not None:
            geo = screen.availableGeometry()
            self.move(
                geo.x() + (geo.width() - self.width()) // 2,
                geo.y() + (geo.height() - self.height()) // 2,
            )

    def set_progress(self, percent: int, message: str = "") -> None:
        """Update the progress bar percentage and optional status message."""
        self.progress.setValue(max(0, min(100, int(percent))))
        if message:
            self.status_lbl.setText(message)
        QApplication.processEvents()

    def finish(self, main_window: QWidget) -> None:
        """Complete progress, close the splash screen, and focus ``main_window``."""
        self.set_progress(100)
        self.close()
        self.deleteLater()
        if main_window is not None:
            main_window.raise_()
            main_window.activateWindow()
