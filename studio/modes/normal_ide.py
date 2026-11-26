"""
Mode 4: Normal Mode (Full `pyCub` Python IDE, Exercise Tester & External `.py` Script Runner).

Provides a multi-tab Python/YAML editor with `pyCub` API autocomplete, live stdout/stderr console,
bundled Examples & Exercises workspace installer + HTML result viewer, external `.py` runner with
optional file-save watcher, and an embedded hardware-accelerated 3D viewport with `l_eye`/`r_eye`
camera docks, coordinate axes, and RGB/Depth export.
"""
from __future__ import annotations

import os
import re
import shutil
import sys
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple, get_args

from PySide6.QtCore import QFileSystemWatcher, Qt, QUrl
from PySide6.QtGui import QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QCheckBox,
    QDialog,
    QFileDialog,
    QFrame,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMenu,
    QPlainTextEdit,
    QPushButton,
    QSplitter,
    QTabWidget,
    QTextBrowser,
    QVBoxLayout,
    QWidget,
)

import icub_pybullet as icub_pkg
from icub_pybullet.runtime_paths import ensure_user_workspace
from icub_pybullet.utils import CHAINS, JOINTS
from studio.i18n import I18nManager
from studio.resource_paths import iter_studio_data_roots
from studio.theme import get_active_theme, get_theme_tokens
from studio.sim_bridge import SimProcessSupervisor
from studio.widgets.code_editor import PythonCodeEditor
from studio.widgets.flow_layout import FlowLayout
from studio.widgets.viewport import EmbeddedGLViewer, EyeCameraPanel


_ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")

_DEFAULT_FULL_SCRIPT = '''"""
Full pyCub Simulator Script (Normal Mode).
Press F5 or click "Run Script" to execute inside the embedded 3D viewport!
"""
from icub_pybullet.pycub import pyCub, Pose

def main():
    client = pyCub(config="with_ball.yaml")

    # Move neck to look at the table and raise right arm
    client.move_position("neck_pitch", -0.35)
    pose = client.end_effector.get_position()
    pose.pos[0] -= 0.08
    pose.pos[2] += 0.05
    client.move_cartesian(pose, timeout=4.0)

    # Keep simulation alive until Stop is clicked
    while client.is_alive():
        client.update_simulation()

if __name__ == "__main__":
    main()
'''

_PYCUB_COMPLETIONS: List[str] = [
    "pyCub",
    "Pose",
    "Config",
    "URDF",
    "move_position",
    "move_velocity",
    "move_cartesian",
    "update_simulation",
    "wait_motion_done",
    "motion_done",
    "stop_robot",
    "get_joint_state",
    "compute_jacobian",
    "get_camera_images",
    "get_camera_depth_images",
    "is_alive",
    "toggle_gravity",
    "end_effector",
    "free_objects",
    "other_objects",
    "skin_activations",
    "activated_skin_points",
    "visualizer",
    *list(get_args(JOINTS)),
    *list(get_args(CHAINS)),
]


def _locate_bundled_exercises_root() -> Optional[Path]:
    """Return the directory containing bundled course exercises, or ``None`` if absent."""
    pkg_dir = Path(os.path.abspath(icub_pkg.__file__)).parent
    studio_root = Path(__file__).resolve().parents[2]
    candidates: List[Path] = [
        studio_root / "exercises",
        pkg_dir.parent / "exercises",
    ]
    meipass = getattr(sys, "_MEIPASS", None)
    if meipass:
        candidates.append(Path(meipass) / "exercises")
    if getattr(sys, "frozen", False):
        exe_dir = Path(sys.executable).resolve().parent
        candidates.append(exe_dir / "exercises")
        candidates.append(exe_dir / "_internal" / "exercises")
    for data_root in iter_studio_data_roots():
        candidates.append(data_root / "exercises")

    for c in candidates:
        if c.is_dir() and any(c.glob("exercise_*")):
            return c
    return None


def _locate_bundled_examples_dir() -> Optional[Path]:
    """Return the directory containing built-in example Python scripts, or ``None`` if absent."""
    pkg_dir = Path(os.path.abspath(icub_pkg.__file__)).parent
    studio_root = Path(__file__).resolve().parents[2]
    candidates: List[Path] = [
        pkg_dir / "examples",
        studio_root / "icub_pybullet" / "examples",
    ]
    meipass = getattr(sys, "_MEIPASS", None)
    if meipass:
        candidates.append(Path(meipass) / "icub_pybullet" / "examples")
    if getattr(sys, "frozen", False):
        exe_dir = Path(sys.executable).resolve().parent
        candidates.append(exe_dir / "icub_pybullet" / "examples")
        candidates.append(exe_dir / "_internal" / "icub_pybullet" / "examples")
    for data_root in iter_studio_data_roots():
        candidates.append(data_root / "examples")

    for c in candidates:
        if c.is_dir() and any(c.glob("*.py")):
            return c
    return None


class NormalIDEMode(QWidget):
    """Mode 4 widget: Full Python IDE + Script Runner + Embedded 3D & Eye Cameras."""

    def __init__(
        self,
        supervisor: SimProcessSupervisor,
        i18n: I18nManager,
        parent: Optional[QWidget] = None,
    ):
        """Initialize the Normal IDE mode UI, file watcher, signals, and default tab."""
        super().__init__(parent)
        self.sup = supervisor
        self.i18n = i18n
        self._active = False
        self._tab_paths: Dict[QWidget, Optional[str]] = {}
        self._external_script: Optional[str] = None
        self._pending_tester_results_dir: Optional[str] = None
        self._enabled_eyes: Set[str] = set()

        self._watcher = QFileSystemWatcher(self)
        self._watcher.fileChanged.connect(self._on_watched_file_changed)

        self._build_ui()
        self.apply_theme(get_active_theme())
        self._connect_signals()
        self.retranslate_ui()
        self._new_tab("main.py", _DEFAULT_FULL_SCRIPT, None)

    def _build_ui(self) -> None:
        """Construct the IDE toolbar, tabbed editor, console, 3D viewport, and eye docks."""
        root = QVBoxLayout(self)
        root.setContentsMargins(10, 8, 10, 10)
        root.setSpacing(8)

        # ================= TOP IDE TOOLBAR =================
        tb = FlowLayout(h_spacing=6, v_spacing=6)

        self.run_btn = QPushButton()
        self.run_btn.setStyleSheet(
            "background: #16a34a; color: white; font-weight: 800; padding: 7px 14px; border-radius: 8px;"
        )
        self.run_btn.clicked.connect(self.run_current_tab)
        tb.addWidget(self.run_btn)

        self.stop_btn = QPushButton()
        self.stop_btn.setStyleSheet(
            "background: #dc2626; color: white; font-weight: 800; padding: 7px 12px; border-radius: 8px;"
        )
        self.stop_btn.clicked.connect(self.stop_execution)
        tb.addWidget(self.stop_btn)

        self.new_btn = QPushButton()
        self.new_btn.clicked.connect(lambda: self._new_tab("untitled.py", _DEFAULT_FULL_SCRIPT, None))
        tb.addWidget(self.new_btn)

        self.open_btn = QPushButton()
        self.open_btn.clicked.connect(self._open_file_dialog)
        tb.addWidget(self.open_btn)

        self.save_btn = QPushButton()
        self.save_btn.clicked.connect(self._save_current_tab)
        tb.addWidget(self.save_btn)

        self.save_as_btn = QPushButton()
        self.save_as_btn.clicked.connect(self._save_current_tab_as)
        tb.addWidget(self.save_as_btn)

        self.examples_btn = QPushButton()
        self.examples_menu = QMenu(self)
        self.examples_btn.setMenu(self.examples_menu)
        tb.addWidget(self.examples_btn)

        self.tester_btn = QPushButton()
        self.tester_btn.setStyleSheet(
            "background: #7c3aed; color: white; font-weight: 700; padding: 7px 12px; border-radius: 8px;"
        )
        self.tester_btn.setVisible(False)
        self.tester_btn.clicked.connect(self._run_active_exercise_tester)
        tb.addWidget(self.tester_btn)

        tb.addStretch(1)

        self.ext_btn = QPushButton()
        self.ext_btn.setStyleSheet(
            "background: #0284c7; color: white; font-weight: 700; padding: 7px 12px; border-radius: 8px;"
        )
        self.ext_btn.clicked.connect(self._choose_and_run_external)
        tb.addWidget(self.ext_btn)

        self.watch_chk = QCheckBox()
        self.watch_chk.toggled.connect(self._sync_watcher)
        tb.addWidget(self.watch_chk)

        root.addLayout(tb)

        # ================= MAIN HORIZONTAL SPLITTER =================
        main_split = QSplitter(Qt.Orientation.Horizontal, self)
        root.addWidget(main_split, 1)

        # --- Left: Multi-tab Editor + Find Bar + Console ---
        left_widget = QWidget()
        left_lay = QVBoxLayout(left_widget)
        left_lay.setContentsMargins(0, 0, 4, 0)
        left_lay.setSpacing(6)

        left_vsplit = QSplitter(Qt.Orientation.Vertical)

        editor_container = QWidget()
        ec_lay = QVBoxLayout(editor_container)
        ec_lay.setContentsMargins(0, 0, 0, 0)
        ec_lay.setSpacing(4)

        self.tabs = QTabWidget()
        self.tabs.setUsesScrollButtons(True)
        self.tabs.setTabsClosable(True)
        self.tabs.tabCloseRequested.connect(self._close_tab)
        self.tabs.currentChanged.connect(self._on_tab_switched)
        ec_lay.addWidget(self.tabs, 1)

        # Quick Find Bar
        self.find_bar = QFrame()
        self.find_bar.setVisible(False)
        fb_lay = QHBoxLayout(self.find_bar)
        fb_lay.setContentsMargins(4, 2, 4, 2)
        fb_lay.addWidget(QLabel("Find:"))
        self.find_input = QLineEdit()
        self.find_input.setMinimumWidth(80)
        self.find_input.returnPressed.connect(self._find_next)
        fb_lay.addWidget(self.find_input, 1)
        find_next_btn = QPushButton("Next")
        find_next_btn.clicked.connect(self._find_next)
        fb_lay.addWidget(find_next_btn)
        close_find_btn = QPushButton("✕")
        close_find_btn.setFixedWidth(28)
        close_find_btn.clicked.connect(lambda: self.find_bar.setVisible(False))
        fb_lay.addWidget(close_find_btn)
        ec_lay.addWidget(self.find_bar)

        left_vsplit.addWidget(editor_container)

        # Console Output Pane
        console_container = QWidget()
        cc_lay = QVBoxLayout(console_container)
        cc_lay.setContentsMargins(0, 0, 0, 0)
        cc_lay.setSpacing(4)
        c_hdr = QHBoxLayout()
        self.console_lbl = QLabel()
        c_hdr.addWidget(self.console_lbl)
        self.status_lbl = QLabel()
        c_hdr.addWidget(self.status_lbl)
        c_hdr.addStretch(1)
        self.clear_console_btn = QPushButton()
        self.clear_console_btn.clicked.connect(lambda: self.console.clear())
        c_hdr.addWidget(self.clear_console_btn)
        cc_lay.addLayout(c_hdr)

        self.console = QPlainTextEdit()
        self.console.setReadOnly(True)
        cc_lay.addWidget(self.console, 1)
        left_vsplit.addWidget(console_container)
        left_vsplit.setSizes([500, 190])

        left_lay.addWidget(left_vsplit, 1)
        main_split.addWidget(left_widget)

        # --- Right: Embedded 3D Viewport + Visualizer Controls + Eye Camera Docks ---
        right_widget = QWidget()
        right_lay = QVBoxLayout(right_widget)
        right_lay.setContentsMargins(4, 0, 0, 0)
        right_lay.setSpacing(6)

        vis_tb = FlowLayout(h_spacing=6, v_spacing=6)

        self.l_eye_btn = QPushButton()
        self.l_eye_btn.setCheckable(True)
        self.l_eye_btn.toggled.connect(lambda on: self._toggle_eye("l_eye", on))
        vis_tb.addWidget(self.l_eye_btn)

        self.r_eye_btn = QPushButton()
        self.r_eye_btn.setCheckable(True)
        self.r_eye_btn.toggled.connect(lambda on: self._toggle_eye("r_eye", on))
        vis_tb.addWidget(self.r_eye_btn)

        self.axes_btn = QPushButton()
        self.axes_btn.setCheckable(True)
        self.axes_btn.toggled.connect(lambda on: self.viewer.set_show_axes(on))
        vis_tb.addWidget(self.axes_btn)

        self.save_rgb_btn = QPushButton()
        self.save_rgb_btn.clicked.connect(self._export_rgb)
        vis_tb.addWidget(self.save_rgb_btn)

        self.save_depth_btn = QPushButton()
        self.save_depth_btn.clicked.connect(self._export_depth)
        vis_tb.addWidget(self.save_depth_btn)

        self.preview_btn = QPushButton()
        self.preview_btn.clicked.connect(lambda: self.sup.start_interactive("ball"))
        vis_tb.addWidget(self.preview_btn)

        self.reset_view_btn = QPushButton()
        self.reset_view_btn.clicked.connect(lambda: self.viewer.reset_view())
        vis_tb.addWidget(self.reset_view_btn)
        vis_tb.addStretch(1)
        right_lay.addLayout(vis_tb)

        right_vsplit = QSplitter(Qt.Orientation.Vertical)
        self._viewer_frame = QFrame()
        vf_lay = QVBoxLayout(self._viewer_frame)
        vf_lay.setContentsMargins(2, 2, 2, 2)
        self.viewer = EmbeddedGLViewer(self._viewer_frame)
        vf_lay.addWidget(self.viewer, 1)
        right_vsplit.addWidget(self._viewer_frame)

        self.eyes_container = QWidget()
        eyes_lay = QHBoxLayout(self.eyes_container)
        eyes_lay.setContentsMargins(0, 0, 0, 0)
        eyes_lay.setSpacing(8)
        self.l_eye_panel = EyeCameraPanel("l_eye", "Left Eye (l_eye)")
        self.l_eye_panel.setVisible(False)
        self.r_eye_panel = EyeCameraPanel("r_eye", "Right Eye (r_eye)")
        self.r_eye_panel.setVisible(False)
        eyes_lay.addWidget(self.l_eye_panel)
        eyes_lay.addWidget(self.r_eye_panel)
        self.eyes_container.setVisible(False)
        right_vsplit.addWidget(self.eyes_container)
        right_vsplit.setSizes([520, 220])

        right_lay.addWidget(right_vsplit, 1)
        main_split.addWidget(right_widget)
        main_split.setSizes([600, 580])

        # Keyboard shortcuts
        QShortcut(QKeySequence("F5"), self, activated=self.run_current_tab)
        QShortcut(QKeySequence("Ctrl+S"), self, activated=self._save_current_tab)
        QShortcut(QKeySequence("Ctrl+F"), self, activated=self._open_find_bar)

        self._populate_examples_menu()

    def apply_theme(self, theme: str) -> None:
        """Apply ``theme`` colors to the IDE labels, console, viewer frame, and eye panels."""
        t = get_theme_tokens(theme)
        self.console_lbl.setStyleSheet(f"font-weight: 700; color: {t.text_secondary};")
        self.status_lbl.setStyleSheet(f"color: {t.section_accent}; font-weight: 700;")
        self.console.setStyleSheet(
            f"background: {t.console_bg}; color: {t.console_fg}; font-family: monospace; font-size: 12px; "
            f"border: 1px solid {t.console_border}; border-radius: 8px;"
        )
        self._viewer_frame.setStyleSheet(
            f"border: 2px solid {t.border}; border-radius: 10px; background: {t.surface_alt_bg};"
        )
        self.l_eye_panel.apply_theme(theme)
        self.r_eye_panel.apply_theme(theme)

    def _connect_signals(self) -> None:
        """Connect localization and simulator supervisor signals to IDE handlers."""
        self.i18n.languageChanged.connect(lambda _l: self.retranslate_ui())
        self.sup.sceneInitialized.connect(self._on_scene_init)
        self.sup.frameReady.connect(self._on_frame)
        self.sup.eyesUpdated.connect(self._on_eyes)
        self.sup.statusChanged.connect(self._on_status)
        self.sup.consoleOutput.connect(self._on_console)
        self.sup.scriptFinished.connect(self._on_script_finished)

    # --- Activation ---

    def activate_mode(self) -> None:
        """Activate the mode, sync enabled eye cameras, and start an idle preview if needed."""
        self._active = True
        self.sup.set_enabled_eyes(self._enabled_eyes)
        if not self.sup.is_running():
            self.viewer.set_overlay_status(self.i18n.t("status_loading"))
            self.sup.start_interactive("ball")

    def deactivate_mode(self) -> None:
        """Mark the mode inactive."""
        self._active = False

    def open_and_run_script(self, script_path: str) -> None:
        """Used by `--run-script <file.py>` CLI flag to open and immediately execute a user script."""
        abs_p = os.path.abspath(script_path)
        if os.path.isfile(abs_p):
            self.open_file_in_tab(abs_p)
            self.run_script_path(abs_p)

    # --- Tabs & File Management ---

    def _current_editor(self) -> Optional[PythonCodeEditor]:
        """Return the active tab's ``PythonCodeEditor``, or ``None``."""
        w = self.tabs.currentWidget()
        return w if isinstance(w, PythonCodeEditor) else None

    def _new_tab(self, title: str, content: str, file_path: Optional[str]) -> PythonCodeEditor:
        """Create and focus a new editor tab with the given title, content, and path."""
        ed = PythonCodeEditor(completions=_PYCUB_COMPLETIONS, extra_highlight_symbols=_PYCUB_COMPLETIONS, i18n=self.i18n)
        ed.source_path = file_path
        ed.setPlainText(content)
        ed.textChanged.connect(ed.clear_error_line)
        self._tab_paths[ed] = file_path
        idx = self.tabs.addTab(ed, title)
        self.tabs.setCurrentIndex(idx)
        self._update_tester_button_visibility()
        return ed

    def open_file_in_tab(self, path: str) -> None:
        """
        Open a file in an editor tab, or focus its existing tab if already open.

        :param path: Filesystem path to the Python or YAML file to open.
        """
        abs_p = os.path.abspath(path)
        for widget, p in self._tab_paths.items():
            if p == abs_p:
                self.tabs.setCurrentWidget(widget)
                return
        with open(abs_p, "r", encoding="utf-8", errors="replace") as f:
            txt = f.read()
        self._new_tab(os.path.basename(abs_p), txt, abs_p)

    def _close_tab(self, index: int) -> None:
        """Close the tab at ``index`` unless it is the last remaining tab."""
        if self.tabs.count() <= 1:
            return
        w = self.tabs.widget(index)
        self.tabs.removeTab(index)
        self._tab_paths.pop(w, None)
        self._update_tester_button_visibility()

    def _on_tab_switched(self, _index: int) -> None:
        """Update exercise tester button visibility when switching editor tabs."""
        self._update_tester_button_visibility()

    def _open_file_dialog(self) -> None:
        """Prompt for a Python or YAML file and open it in a tab."""
        ws = str(ensure_user_workspace())
        path, _ = QFileDialog.getOpenFileName(
            self,
            self.i18n.t("open_file"),
            ws,
            "Python / YAML (*.py *.yaml *.yml);;All Files (*)",
        )
        if path and os.path.isfile(path):
            self.open_file_in_tab(path)

    def _save_current_tab(self) -> bool:
        """Save the active editor tab to disk, prompting for a path if unsaved."""
        ed = self._current_editor()
        if ed is None:
            return False
        cur_path = self._tab_paths.get(ed)
        if not cur_path:
            return self._save_current_tab_as()
        with open(cur_path, "w", encoding="utf-8") as f:
            f.write(ed.toPlainText())
        self.tabs.setTabText(self.tabs.currentIndex(), os.path.basename(cur_path))
        return True

    def _save_current_tab_as(self) -> bool:
        """Prompt for a new file path and save the active editor tab."""
        ed = self._current_editor()
        if ed is None:
            return False
        scripts_dir = str(ensure_user_workspace() / "scripts")
        cur_path = self._tab_paths.get(ed) or os.path.join(scripts_dir, "my_pycub_script.py")
        path, _ = QFileDialog.getSaveFileName(
            self,
            self.i18n.t("save_as_file"),
            cur_path,
            "Python / YAML (*.py *.yaml *.yml)",
        )
        if not path:
            return False
        abs_p = os.path.abspath(path)
        self._tab_paths[ed] = abs_p
        ed.source_path = abs_p
        return self._save_current_tab()

    def _open_find_bar(self) -> None:
        """Show and focus the inline text search bar."""
        self.find_bar.setVisible(True)
        self.find_input.setFocus()
        self.find_input.selectAll()

    def _find_next(self) -> None:
        """Find the next occurrence of the search term in the active editor, wrapping around."""
        ed = self._current_editor()
        term = self.find_input.text()
        if ed and term:
            if not ed.find(term):
                cur = ed.textCursor()
                cur.movePosition(cur.MoveOperation.Start)
                ed.setTextCursor(cur)
                ed.find(term)

    # --- Bundled Examples & Exercises ---

    def _populate_examples_menu(self) -> None:
        """Populate the Examples & Exercises menu with bundled scripts and coursework."""
        self.examples_menu.clear()
        ex_dir = _locate_bundled_examples_dir()
        if ex_dir is not None:
            sub_ex = self.examples_menu.addMenu("🚀 Built-in Examples")
            for py_f in sorted(ex_dir.glob("*.py")):
                act = sub_ex.addAction(py_f.name)
                act.triggered.connect(lambda _c=False, src=py_f: self._open_bundled_example(src))

        ex_root = _locate_bundled_exercises_root()
        if ex_root is not None:
            sub_hw = self.examples_menu.addMenu("🎓 Course Exercises (1–5)")
            for folder in sorted(ex_root.glob("exercise_*")):
                if folder.is_dir():
                    act = sub_hw.addAction(folder.name.replace("_", " ").title())
                    act.triggered.connect(lambda _c=False, f=folder: self._open_bundled_exercise(f))

    def _open_bundled_example(self, src_file: Path) -> None:
        """Copy a bundled example into the user workspace if missing and open it."""
        dst_dir = ensure_user_workspace() / "examples"
        dst_dir.mkdir(parents=True, exist_ok=True)
        dst_file = dst_dir / src_file.name
        if not dst_file.exists():
            shutil.copy2(str(src_file), str(dst_file))
        self.open_file_in_tab(str(dst_file))

    def _open_bundled_exercise(self, src_folder: Path) -> None:
        """Copy a bundled exercise folder into the user workspace if missing and open it."""
        dst_folder = ensure_user_workspace() / "exercises" / src_folder.name
        if not dst_folder.exists():
            shutil.copytree(str(src_folder), str(dst_folder))
        main_py = dst_folder / f"{src_folder.name}.py"
        if main_py.is_file():
            self.open_file_in_tab(str(main_py))

    def _active_tester_path(self) -> Optional[str]:
        """Return the matching ``*_tester.py`` path for the active tab, or ``None``."""
        ed = self._current_editor()
        if ed is None:
            return None
        p = self._tab_paths.get(ed)
        if not p:
            return None
        folder = os.path.dirname(p)
        stem = Path(p).stem
        cand1 = os.path.join(folder, f"{stem}_tester.py")
        if os.path.isfile(cand1):
            return cand1
        # If viewing tester itself
        if stem.endswith("_tester") and os.path.isfile(p):
            return p
        return None

    def _update_tester_button_visibility(self) -> None:
        """Show the exercise tester button only when an exercise tester is available."""
        self.tester_btn.setVisible(self._active_tester_path() is not None)

    # --- Script & Tester Execution ---

    def run_current_tab(self) -> None:
        """Save or stage the active tab script and execute it in the simulator worker."""
        ed = self._current_editor()
        if ed is None:
            return
        ed.clear_error_line()
        cur_path = self._tab_paths.get(ed)
        if cur_path and cur_path.endswith(".py"):
            self._save_current_tab()
            target_py = cur_path
        else:
            scratch_dir = ensure_user_workspace() / "scripts"
            target_py = str(scratch_dir / "_unsaved_ide_run.py")
            with open(target_py, "w", encoding="utf-8") as f:
                f.write(ed.toPlainText())
        self._pending_tester_results_dir = None
        self.run_script_path(target_py)

    def _run_active_exercise_tester(self) -> None:
        """Save the active exercise file and run its associated tester script."""
        tester = self._active_tester_path()
        if not tester:
            return
        self._save_current_tab()
        ex_dir = os.path.dirname(tester)
        self._pending_tester_results_dir = os.path.join(ex_dir, "results")
        # Pass small rep count ("3") for interactive responsiveness if exercise_3_tester
        args = ["3"] if os.path.basename(tester) == "exercise_3_tester.py" else []
        self.run_script_path(tester, args=args)

    def run_script_path(self, script_path: str, args: Optional[List[str]] = None) -> None:
        """
        Execute a Python script inside the simulator worker process.

        :param script_path: Path to the Python script to run.
        :param args: Optional command-line arguments forwarded to the script.
        """
        abs_p = os.path.abspath(script_path)
        self.console.clear()
        self.console.appendPlainText(f"▶ Running {abs_p} ...\n")
        self.status_lbl.setText(f"▶ {os.path.basename(abs_p)}")
        self.viewer.set_overlay_status(f"Running {os.path.basename(abs_p)}…")
        self.sup.set_enabled_eyes(self._enabled_eyes)
        self.sup.start_user_script(abs_p, working_dir=os.path.dirname(abs_p), args=args)

    def stop_execution(self) -> None:
        """Stop the currently running worker script and reset status indicators."""
        self._pending_tester_results_dir = None
        self.sup.stop_worker()
        self.status_lbl.setText(self.i18n.t("status_stopped"))
        self.viewer.set_overlay_status("")

    def _choose_and_run_external(self) -> None:
        """Prompt for an external Python script, attach the watcher if enabled, and run it."""
        ws = str(ensure_user_workspace())
        path, _ = QFileDialog.getOpenFileName(self, self.i18n.t("run_external"), ws, "Python Script (*.py)")
        if path and os.path.isfile(path):
            self._external_script = os.path.abspath(path)
            self._sync_watcher()
            self._pending_tester_results_dir = None
            self.run_script_path(self._external_script)

    def _sync_watcher(self) -> None:
        """Update ``QFileSystemWatcher`` paths to match the auto-run checkbox state."""
        watched = self._watcher.files()
        if watched:
            self._watcher.removePaths(watched)
        if self.watch_chk.isChecked() and self._external_script and os.path.isfile(self._external_script):
            self._watcher.addPath(self._external_script)

    def _on_watched_file_changed(self, path: str) -> None:
        """Re-run the watched external script when it is modified on disk."""
        if os.path.isfile(path) and self.watch_chk.isChecked():
            if path not in self._watcher.files():
                self._watcher.addPath(path)
            self.run_script_path(path)

    # --- Eye Docks & Screenshot Exports ---

    def _toggle_eye(self, eye: str, enabled: bool) -> None:
        """Toggle an eye camera dock panel and update active eye streams on the worker."""
        if enabled:
            self._enabled_eyes.add(eye)
        else:
            self._enabled_eyes.discard(eye)
        self.l_eye_panel.setVisible("l_eye" in self._enabled_eyes)
        self.r_eye_panel.setVisible("r_eye" in self._enabled_eyes)
        self.eyes_container.setVisible(bool(self._enabled_eyes))
        self.sup.set_enabled_eyes(self._enabled_eyes)

    def _export_rgb(self) -> None:
        """Prompt for a PNG path and save the current 3D viewport RGB image."""
        img_dir = str(ensure_user_workspace() / "images")
        path, _ = QFileDialog.getSaveFileName(
            self, self.i18n.t("save_rgb"), os.path.join(img_dir, "scene_rgb.png"), "PNG (*.png)"
        )
        if path:
            self.viewer.save_rgb_image(path)

    def _export_depth(self) -> None:
        """Prompt for a PNG path and save the current 3D viewport depth image."""
        img_dir = str(ensure_user_workspace() / "images")
        path, _ = QFileDialog.getSaveFileName(
            self, self.i18n.t("save_depth"), os.path.join(img_dir, "scene_depth.png"), "PNG (*.png)"
        )
        if path:
            self.viewer.save_depth_image(path)

    # --- Worker Signal Handlers ---

    def _on_scene_init(self, manifest: Dict) -> None:
        """Load the scene manifest into the 3D viewer and clear the overlay status."""
        if not self._active:
            return
        self.viewer.load_scene_manifest(manifest)
        self.viewer.set_overlay_status("")

    def _on_frame(self, frame: Dict) -> None:
        """Update the 3D viewer with a newly received simulation frame."""
        if self._active:
            self.viewer.update_frame(frame)

    def _on_eyes(self, eyes: Dict) -> None:
        """Update eye camera panels with new RGB/depth images and reveal active panels."""
        if not self._active:
            return
        if "l_eye" in eyes:
            self.l_eye_panel.update_eye_frames(eyes["l_eye"].get("rgb"), eyes["l_eye"].get("depth"))
            if not self.l_eye_panel.isVisible():
                self.l_eye_btn.setChecked(True)
        if "r_eye" in eyes:
            self.r_eye_panel.update_eye_frames(eyes["r_eye"].get("rgb"), eyes["r_eye"].get("depth"))
            if not self.r_eye_panel.isVisible():
                self.r_eye_btn.setChecked(True)

    def _on_status(self, st: Dict) -> None:
        """Update the status label, viewer overlay, and console from worker status updates."""
        if not self._active:
            return
        state = st.get("state", "")
        detail = st.get("detail", "")
        if state == "loading":
            msg = f"{self.i18n.t('status_loading')} {detail}".strip()
            self.status_lbl.setText(msg)
            self.viewer.set_overlay_status(msg)
        elif state == "running_script":
            self.status_lbl.setText(f"▶ {detail}")
            self.viewer.set_overlay_status("")
        elif state == "error":
            self.status_lbl.setText("❌ Error loading world")
            self.viewer.set_overlay_status("Error loading world")
            if detail:
                self.console.appendPlainText(detail + "\n")
        elif state == "ready":
            self.status_lbl.setText(self.i18n.t("status_ready"))
            self.viewer.set_overlay_status("")

    def _on_console(self, stream: str, text: str) -> None:
        """Strip ANSI codes, append output to the console, and highlight stderr error lines."""
        if not self._active or not text:
            return
        clean = _ANSI_RE.sub("", text)
        self.console.moveCursor(self.console.textCursor().MoveOperation.End)
        self.console.insertPlainText(clean)

        if stream == "stderr":
            ed = self._current_editor()
            cur_path = self._tab_paths.get(ed) if ed else None
            if ed and cur_path:
                for m in re.finditer(r'File "([^"]+)", line (\d+)', clean):
                    if os.path.abspath(m.group(1)) == os.path.abspath(cur_path):
                        ed.set_error_line(int(m.group(2)))

    def _on_script_finished(self, exit_code: int) -> None:
        """Update status on script exit and open any generated exercise results HTML."""
        if not self._active:
            return
        self.viewer.set_overlay_status("")
        status_msg = f"Finished (exit code {exit_code})"
        self.status_lbl.setText(status_msg)
        self.console.appendPlainText(f"\n=== Process exited with code {exit_code} ===\n")

        if self._pending_tester_results_dir:
            res_html = os.path.join(self._pending_tester_results_dir, "results.html")
            self._pending_tester_results_dir = None
            if os.path.isfile(res_html):
                self._show_results_dialog(res_html)

    def _show_results_dialog(self, html_path: str) -> None:
        """Display an exercise tester ``results.html`` report in a modal dialog."""
        dlg = QDialog(self)
        dlg.setWindowTitle("Exercise Tester Results")
        dlg.resize(720, 540)
        lay = QVBoxLayout(dlg)
        browser = QTextBrowser()
        browser.setSearchPaths([os.path.dirname(html_path)])
        browser.setSource(QUrl.fromLocalFile(html_path))
        lay.addWidget(browser)
        dlg.exec()

    def retranslate_ui(self) -> None:
        """Update all toolbar, console, and viewport labels for the active language."""
        self.run_btn.setText(self.i18n.t("run_script"))
        self.stop_btn.setText(self.i18n.t("stop_program"))
        self.new_btn.setText(self.i18n.t("new_file"))
        self.open_btn.setText(self.i18n.t("open_file"))
        self.save_btn.setText(self.i18n.t("save_file"))
        self.save_as_btn.setText(self.i18n.t("save_as_file"))
        self.examples_btn.setText(self.i18n.t("examples_menu"))
        self.tester_btn.setText(self.i18n.t("run_tester"))
        self.ext_btn.setText(self.i18n.t("run_external"))
        self.watch_chk.setText(self.i18n.t("watch_external"))
        self.console_lbl.setText(self.i18n.t("console_output"))
        self.clear_console_btn.setText(self.i18n.t("clear_console"))
        self.l_eye_btn.setText(self.i18n.t("show_l_eye"))
        self.r_eye_btn.setText(self.i18n.t("show_r_eye"))
        self.axes_btn.setText(self.i18n.t("show_axes"))
        self.save_rgb_btn.setText(self.i18n.t("save_rgb"))
        self.save_depth_btn.setText(self.i18n.t("save_depth"))
        self.preview_btn.setText(self.i18n.t("idle_preview"))
        self.reset_view_btn.setText(self.i18n.t("reset_view"))
