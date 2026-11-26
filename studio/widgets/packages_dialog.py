"""
Python Environment & Extra Packages Dialog for pyCub Studio.

Allows users (even in frozen standalone builds) to:
  1. Inspect/open `~/Documents/pyCub/extra_packages` (always added to worker `sys.path`).
  2. Add/remove extra package directories or import `site-packages` from an existing venv.
  3. Install PyPI packages directly into `extra_packages` via `pip install --target`.
  4. Optionally select an external Python executable (venv / conda) for running Mode 4 scripts
     with full embedded 3D viewport & eye camera streaming.
"""
from __future__ import annotations

import os
import shlex
import subprocess
import sys
from typing import Optional

from PySide6.QtCore import QProcess, QUrl
from PySide6.QtGui import QDesktopServices
from PySide6.QtWidgets import (
    QDialog,
    QFileDialog,
    QFrame,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from icub_pybullet.runtime_paths import get_extra_packages_dir, is_frozen
from studio.i18n import I18nManager
from studio.settings import (
    StudioSettings,
    build_pip_install_command,
    discover_site_packages_in_venv,
    load_studio_settings,
    save_studio_settings,
)
from studio.theme import get_active_theme, get_theme_tokens


class PackagesDialog(QDialog):
    """Modal dialog for managing extra Python packages and optional external Python interpreter."""

    def __init__(self, i18n: I18nManager, parent: Optional[QWidget] = None):
        """Initialize the packages and interpreter settings dialog."""
        super().__init__(parent)
        self.i18n = i18n
        self.settings: StudioSettings = load_studio_settings()
        self._pip_proc: Optional[QProcess] = None
        self.resize(720, 620)
        self._build_ui()
        self.apply_theme(get_active_theme())
        self._populate_from_settings()

    def _build_ui(self) -> None:
        """Construct the dialog layout, package list, pip runner, and interpreter selector."""
        self.setWindowTitle(self.i18n.t("pkg_dialog_title"))
        lay = QVBoxLayout(self)
        lay.setSpacing(10)

        # Runtime banner
        self._banner = QFrame()
        self._banner.setObjectName("pkgBanner")
        b_lay = QVBoxLayout(self._banner)
        b_lay.setContentsMargins(10, 8, 10, 8)
        ver = sys.version.split()[0]
        extra_dir = str(get_extra_packages_dir())
        self._info_lbl = QLabel(self.i18n.t("pkg_runtime_lbl", ver=ver, extra_dir=extra_dir))
        self._info_lbl.setWordWrap(True)
        self._info_lbl.setTextInteractionFlags(self._info_lbl.textInteractionFlags())
        b_lay.addWidget(self._info_lbl)

        open_extra_btn = QPushButton(self.i18n.t("pkg_open_extra"))
        open_extra_btn.clicked.connect(lambda: QDesktopServices.openUrl(QUrl.fromLocalFile(extra_dir)))
        b_lay.addWidget(open_extra_btn)
        lay.addWidget(self._banner)

        # Extra paths list
        lay.addWidget(QLabel(self.i18n.t("pkg_paths_heading")))
        self.paths_list = QListWidget()
        self.paths_list.setMinimumHeight(110)
        lay.addWidget(self.paths_list)

        p_btns = QHBoxLayout()
        add_dir_btn = QPushButton(self.i18n.t("pkg_add_folder"))
        add_dir_btn.clicked.connect(self._add_folder)
        p_btns.addWidget(add_dir_btn)

        add_venv_btn = QPushButton(self.i18n.t("pkg_add_venv"))
        add_venv_btn.clicked.connect(self._add_venv)
        p_btns.addWidget(add_venv_btn)

        rem_btn = QPushButton(self.i18n.t("pkg_remove"))
        rem_btn.clicked.connect(self._remove_selected_path)
        p_btns.addWidget(rem_btn)
        p_btns.addStretch(1)
        lay.addLayout(p_btns)

        # Pip install box
        lay.addWidget(QLabel(self.i18n.t("pkg_pip_heading")))
        pip_row = QHBoxLayout()
        self.pip_input = QLineEdit()
        self.pip_input.setPlaceholderText(self.i18n.t("pkg_pip_placeholder"))
        self.pip_input.returnPressed.connect(self._run_pip_install)
        pip_row.addWidget(self.pip_input, 1)

        self.pip_btn = QPushButton(self.i18n.t("pkg_pip_install"))
        self.pip_btn.setStyleSheet(
            "QPushButton { background: #16a34a; color: white; font-weight: 700; padding: 6px 14px; border: none; }"
            "QPushButton:hover { background: #15803d; }"
        )
        self.pip_btn.clicked.connect(self._run_pip_install)
        pip_row.addWidget(self.pip_btn)
        lay.addLayout(pip_row)

        self.pip_log = QPlainTextEdit()
        self.pip_log.setReadOnly(True)
        self.pip_log.setMaximumHeight(140)
        lay.addWidget(self.pip_log)

        # Optional custom python executable for Mode 4
        lay.addWidget(QLabel(self.i18n.t("pkg_interp_heading")))
        exe_row = QHBoxLayout()
        self.exe_edit = QLineEdit()
        self.exe_edit.setPlaceholderText("(Bundled Python — recommended)" if self.i18n.lang == "en" else "(Vestavěný Python — doporučeno)")
        exe_row.addWidget(self.exe_edit, 1)

        browse_exe_btn = QPushButton(self.i18n.t("pkg_browse_exe"))
        browse_exe_btn.clicked.connect(self._browse_python_exe)
        exe_row.addWidget(browse_exe_btn)

        reset_exe_btn = QPushButton(self.i18n.t("pkg_use_bundled"))
        reset_exe_btn.clicked.connect(lambda: self.exe_edit.clear())
        exe_row.addWidget(reset_exe_btn)
        lay.addLayout(exe_row)

        # Save / Close
        bottom = QHBoxLayout()
        bottom.addStretch(1)
        save_btn = QPushButton(self.i18n.t("pkg_save_close"))
        save_btn.setStyleSheet(
            "QPushButton { background: #2563eb; color: white; font-weight: 700; padding: 8px 18px; border: none; }"
            "QPushButton:hover { background: #1d4ed8; }"
        )
        save_btn.clicked.connect(self._save_and_close)
        bottom.addWidget(save_btn)
        lay.addLayout(bottom)

    def apply_theme(self, theme: str) -> None:
        """Apply ``theme`` colors to the runtime banner and pip output console."""
        t = get_theme_tokens(theme)
        self._banner.setStyleSheet(
            f"QFrame#pkgBanner {{ background: {t.palette_btn_bg}; border: 1px solid {t.palette_btn_border}; border-radius: 8px; }}"
        )
        self._info_lbl.setStyleSheet(f"font-weight: 600; color: {t.heading_accent};")
        self.pip_log.setStyleSheet(
            f"background: {t.console_bg}; color: {t.console_fg}; font-family: monospace; font-size: 11px; "
            f"border: 1px solid {t.console_border}; border-radius: 6px;"
        )

    def _populate_from_settings(self) -> None:
        """Populate the UI widgets from the loaded studio settings."""
        self.paths_list.clear()
        for p in self.settings.extra_python_paths:
            self.paths_list.addItem(p)
        self.exe_edit.setText(self.settings.custom_python_exe)

    def _add_folder(self) -> None:
        """Prompt for a directory and append its absolute path to the extra paths list."""
        d = QFileDialog.getExistingDirectory(self, self.i18n.t("pkg_add_folder"), str(get_extra_packages_dir()))
        if d:
            ap = os.path.abspath(d)
            if ap not in [self.paths_list.item(i).text() for i in range(self.paths_list.count())]:
                self.paths_list.addItem(ap)

    def _add_venv(self) -> None:
        """Prompt for a virtual environment directory and add its ``site-packages`` folders."""
        d = QFileDialog.getExistingDirectory(self, self.i18n.t("pkg_add_venv"), os.path.expanduser("~"))
        if not d:
            return
        sites = discover_site_packages_in_venv(d)
        if not sites:
            QMessageBox.information(
                self,
                "No site-packages Found",
                f"Could not locate a `site-packages` folder inside:\n{d}",
            )
            return
        existing = {self.paths_list.item(i).text() for i in range(self.paths_list.count())}
        for sp in sites:
            if sp not in existing:
                self.paths_list.addItem(sp)

    def _remove_selected_path(self) -> None:
        """Remove currently selected entries from the extra paths list."""
        for item in self.paths_list.selectedItems():
            self.paths_list.takeItem(self.paths_list.row(item))

    def _browse_python_exe(self) -> None:
        """Open a file dialog to select an external Python executable."""
        filt = "Python (python.exe)" if sys.platform == "win32" else "Python (python*)"
        p, _ = QFileDialog.getOpenFileName(self, self.i18n.t("pkg_browse_exe"), os.path.expanduser("~"), filt)
        if p and os.path.isfile(p):
            self.exe_edit.setText(os.path.abspath(p))

    def _run_pip_install(self) -> None:
        """Run ``pip install`` asynchronously into the user's extra packages directory."""
        raw = self.pip_input.text().strip()
        if not raw:
            return
        try:
            pkgs = shlex.split(raw)
        except ValueError as exc:
            self.pip_log.appendPlainText(f"Invalid package input: {exc}")
            return

        target_dir = str(get_extra_packages_dir())
        chosen_exe = self.exe_edit.text().strip()
        cmd_spec = build_pip_install_command(pkgs, target_dir, chosen_exe)
        if not cmd_spec:
            self.pip_log.appendPlainText(
                "❌ Bundled pip is unavailable and no external Python+pip was found on PATH.\n"
                "Please install Python 3 or copy packages into:\n"
                f"  {target_dir}"
            )
            return

        if self._pip_proc is not None and self._pip_proc.state() != QProcess.ProcessState.NotRunning:
            return

        pip_program, args, mode_label = cmd_spec
        self.pip_log.clear()
        self.pip_log.appendPlainText(f"[{mode_label}] $ {pip_program} {' '.join(args)}\n")
        self.pip_btn.setEnabled(False)

        proc = QProcess(self)
        proc.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        proc.readyReadStandardOutput.connect(
            lambda: self.pip_log.insertPlainText(bytes(proc.readAllStandardOutput()).decode("utf-8", errors="replace"))
        )
        proc.finished.connect(self._on_pip_finished)
        self._pip_proc = proc
        proc.start(pip_program, args)

    def _on_pip_finished(self, exit_code: int, _status) -> None:
        """Re-enable the install button and append the pip process exit status."""
        self.pip_btn.setEnabled(True)
        self.pip_log.appendPlainText(f"\n=== pip exited with code {exit_code} ===")

    def _save_and_close(self) -> None:
        """Validate settings, persist them to disk, and accept the dialog."""
        paths = [self.paths_list.item(i).text() for i in range(self.paths_list.count())]
        exe = self.exe_edit.text().strip()
        if exe and not os.path.isfile(exe):
            QMessageBox.warning(self, "Invalid Python Path", f"Executable not found:\n{exe}")
            return
        self.settings.extra_python_paths = paths
        self.settings.custom_python_exe = exe
        save_studio_settings(self.settings)
        self.accept()
