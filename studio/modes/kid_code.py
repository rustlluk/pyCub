"""
Mode 3: Kid GUI "Programming" (High-Level Python Coding Environment).

Beginner-friendly Python editor powered by `icub_pybullet.kid_api` with clickable Command Palette,
autocomplete, starter templates, friendly error explanations + red line highlighting, `.py` file
open/save, and embedded 3D view.
"""
from __future__ import annotations

import os
from typing import Dict, List, Optional, Tuple

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QComboBox,
    QFileDialog,
    QFrame,
    QHBoxLayout,
    QLabel,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

import icub_pybullet.kid_api as kid_api
from icub_pybullet.runtime_paths import ensure_user_workspace
from studio.i18n import I18nManager
from studio.theme import get_active_theme, get_theme_tokens
from studio.sim_bridge import SimProcessSupervisor, discover_all_worlds
from studio.widgets.code_editor import PythonCodeEditor
from studio.widgets.confetti import ConfettiOverlay
from studio.widgets.flow_layout import FlowLayout
from studio.widgets.viewport import EmbeddedGLViewer


_PALETTE_ITEMS = [
    (
        ("🤖 Hotové pohyby", "🤖 Preset Actions"),
        [
            ("👋", ('zamávej("pravá")', 'wave("right")')),
            ("👋", ('zamávej("levá")', 'wave("left")')),
            ("🙌", ("zvedni_obě_ruce()", "both_arms_up()")),
            ("🙇", ("spusť_obě_ruce()", "both_arms_down()")),
            ("🎯", ("podívej_se_před_sebe()", "look_center()")),
            ("🧍", ("narovnej_se()", "stand_straight()")),
            ("🔄", ("obnov_polohu()", "reset_pose()")),
            ("⏱️", ("počkej(1.0)", "wait(1.0)")),
        ],
    ),
    (
        ("💪 Ruce", "💪 Arms"),
        [
            ("⬆️", ('zvedni_rameno("pravá", 20)', 'shoulder_up("right", 20)')),
            ("⬇️", ('spusť_rameno("pravá", 20)', 'shoulder_down("right", 20)')),
            ("↔️", ('upaž("pravá", 20)', 'shoulder_out("right", 20)')),
            ("↩️", ('připaž("pravá", 20)', 'shoulder_in("right", 20)')),
            ("💪", ('ohni_loket("pravá", 25)', 'bend_elbow("right", 25)')),
            ("📏", ('narovnej_loket("pravá", 25)', 'straighten_elbow("right", 25)')),
            ("↻", ('otoč_paži("pravá", 20)', 'rotate_arm("right", 20)')),
            ("↻", ('otoč_zápěstí("pravá", 25)', 'rotate_wrist("right", 25)')),
        ],
    ),
    (
        ("👀 Hlava a tělo", "👀 Head & Body"),
        [
            ("👇", ("podívej_se_dolů(20)", "look_down(20)")),
            ("👆", ("podívej_se_nahoru(15)", "look_up(15)")),
            ("👈", ("otoč_hlavu(20)", "turn_head(20)")),
            ("🙇", ("předkloň_se(20)", "bend_forward(20)")),
            ("🧍", ("zakloň_se(15)", "bend_back(15)")),
            ("↪️", ("otoč_trup(20)", "turn_body(20)")),
        ],
    ),
    (
        ("⚽ Senzory a cykly", "⚽ Sensors & Loops"),
        [
            ("🔁", ("for i in range(3):\n    ohni_loket(\"pravá\", 20)\n    narovnej_loket(\"pravá\", 20)", "for i in range(3):\n    bend_elbow(\"right\", 20)\n    straighten_elbow(\"right\", 20)")),
            ("⚽", ("if dotkl_se_míčku():\n    print(\"Zásah! Vzdálenost:\", vzdálenost_míčku())\n    zamávej(\"levá\")", "if ball_touched():\n    print(\"Hit! Distance:\", ball_distance())\n    wave(\"left\")")),
            ("📏", ('print("Vzdálenost míčku (cm):", vzdálenost_míčku())', 'print("Ball distance (cm):", ball_distance())')),
        ],
    ),
]

_STARTER_TEMPLATES = [
    (
        ("👋 Zamávat a strčit do míčku", "👋 Wave & Push the Ball"),
        (
            """from icub_pybullet.kid_api import *

# Podívej se na stůl a zamávej!
podívej_se_dolů(20)
zamávej("levá")

# Natáhni pravou ruku směrem k míčku
předkloň_se(18)
zvedni_rameno("pravá", 25)
narovnej_loket("pravá", 25)
počkej(0.5)

print("Zásahy:", počet_zásahů(), "Vzdálenost (cm):", vzdálenost_míčku())
""",
            """from icub_pybullet.kid_api import *

# Look at the table and wave!
look_down(20)
wave("left")

# Reach forward with the right arm toward the ball
bend_forward(18)
shoulder_up("right", 25)
straighten_elbow("right", 25)
wait(0.5)

print("Hits:", ball_hits(), "Distance (cm):", ball_distance())
""",
        ),
    ),
    (
        ("🎉 Oslava po zásahu míčku", "🎉 Celebrate When Ball Touched"),
        (
            """from icub_pybullet.kid_api import *

podívej_se_dolů(20)
for krok in range(4):
    předkloň_se(8)
    zvedni_rameno("pravá", 10)
    if dotkl_se_míčku():
        print("Trefili jsme míček!")
        zvedni_obě_ruce()
        break
""",
            """from icub_pybullet.kid_api import *

look_down(20)
for step in range(4):
    bend_forward(8)
    shoulder_up("right", 10)
    if ball_touched():
        print("We hit the ball!")
        both_arms_up()
        break
""",
        ),
    ),
    (
        ("💃 Taneční sestava", "💃 Robot Dance Routine"),
        (
            """from icub_pybullet.kid_api import *

for i in range(2):
    zvedni_obě_ruce()
    otoč_trup(20)
    otoč_trup(-40)
    otoč_trup(20)
    spusť_obě_ruce()
obnov_polohu()
""",
            """from icub_pybullet.kid_api import *

for i in range(2):
    both_arms_up()
    turn_body(20)
    turn_body(-40)
    turn_body(20)
    both_arms_down()
reset_pose()
""",
        ),
    ),
]


def _friendly_kid_error(err: Dict, lang: str) -> str:
    """Format a Python error dictionary into a kid-friendly localized hint."""
    etype = err.get("type", "Error")
    msg = err.get("message", "")
    line = err.get("lineno")
    loc_cs = f" na řádku {line}" if line else ""
    loc_en = f" on line {line}" if line else ""

    if lang == "cs":
        hints = {
            "NameError": "Zkontroluj překlep v názvu příkazu nebo proměnné (např. zamávej, zvedni_rameno, dotkl_se_míčku).",
            "SyntaxError": "Zkontroluj závorky (), uvozovky \"\" a dvojtečku : na konci řádku s for / if / while.",
            "IndentationError": "Zkontroluj odsazení mezerami pod příkazy for:, if: nebo while:.",
            "TypeError": "Zkontroluj parametry v závorce příkazu (např. zamávej(\"pravá\") nebo počkej(1.0)).",
        }
        hint = hints.get(etype, "Podívej se na zvýrazněný řádek v editoru.")
        return f"❌ Chyba ({etype}){loc_cs}: {msg}\n💡 Tip: {hint}\n"
    else:
        hints = {
            "NameError": "Check for a typo in the command or variable name (e.g. wave, shoulder_up, ball_touched).",
            "SyntaxError": "Check parentheses (), quotes \"\", and the colon : after for / if / while.",
            "IndentationError": "Check indentation (spaces) inside your for:, if:, or while: block.",
            "TypeError": "Check the arguments passed to the command (e.g. wave(\"right\") or wait(1.0)).",
        }
        hint = hints.get(etype, "Check the highlighted line in the editor.")
        return f"❌ Error ({etype}){loc_en}: {msg}\n💡 Hint: {hint}\n"


class KidCodeMode(QWidget):
    """Mode 3 widget: High-level Python coding environment using `kid_api`."""

    def __init__(
        self,
        supervisor: SimProcessSupervisor,
        i18n: I18nManager,
        parent: Optional[QWidget] = None,
    ):
        """Initialize the Kid Programming mode UI, signals, and default template."""
        super().__init__(parent)
        self.sup = supervisor
        self.i18n = i18n
        self._active = False
        self._last_hits = 0
        self._custom_world_path: Optional[str] = None
        self._palette_headers: List[Tuple[QLabel, Tuple[str, str]]] = []

        self._palette_buttons = []
        self._template_index: Optional[int] = None

        self._build_ui()
        self.apply_theme(get_active_theme())
        self._connect_signals()
        self.retranslate_ui()
        self._load_template(0)

    def _build_ui(self) -> None:
        """Construct the three-column palette, editor/console, and 3D viewer layout."""
        root = QHBoxLayout(self)
        root.setContentsMargins(10, 10, 10, 10)

        splitter = QSplitter(Qt.Orientation.Horizontal, self)
        root.addWidget(splitter)

        # ================= COLUMN 1: CLICKABLE COMMAND PALETTE =================
        self._palette_frame = QFrame()
        self._palette_frame.setObjectName("kidPalette")
        self._palette_frame.setMinimumWidth(135)
        self._palette_frame.setMaximumWidth(275)
        pal_lay = QVBoxLayout(self._palette_frame)
        pal_lay.setContentsMargins(10, 10, 10, 10)
        pal_lay.setSpacing(6)

        self.pal_title = QLabel()
        self.pal_title.setWordWrap(True)
        pal_lay.addWidget(self.pal_title)

        pal_scroll = QScrollArea()
        pal_scroll.setWidgetResizable(True)
        pal_scroll.setFrameShape(QFrame.Shape.NoFrame)
        self._pal_body = QWidget()
        self._pal_body.setObjectName("kidPalBody")
        pb_lay = QVBoxLayout(self._pal_body)
        pb_lay.setContentsMargins(0, 4, 4, 4)
        pb_lay.setSpacing(6)

        for cat_pair, items in _PALETTE_ITEMS:
            hdr = QLabel()
            pb_lay.addWidget(hdr)
            self._palette_headers.append((hdr, cat_pair))
            for icon, snippets in items:
                b = QPushButton()
                self._palette_buttons.append((b, icon, snippets))
                b.clicked.connect(lambda _c=False, s=snippets: self.editor.insert_snippet(s[self.i18n.lang_idx]))
                pb_lay.addWidget(b)
        pb_lay.addStretch(1)
        pal_scroll.setWidget(self._pal_body)
        pal_lay.addWidget(pal_scroll, 1)
        splitter.addWidget(self._palette_frame)

        # ================= COLUMN 2: PYTHON EDITOR + FRIENDLY CONSOLE =================
        center_panel = QWidget()
        c_lay = QVBoxLayout(center_panel)
        c_lay.setContentsMargins(4, 0, 4, 0)
        c_lay.setSpacing(8)

        tb = FlowLayout(h_spacing=6, v_spacing=6)
        self.run_btn = QPushButton()
        self.run_btn.setStyleSheet(
            "background: #16a34a; color: white; font-weight: 800; padding: 8px 14px; border-radius: 9px;"
        )
        self.run_btn.clicked.connect(self.run_code)
        tb.addWidget(self.run_btn)

        self.stop_btn = QPushButton()
        self.stop_btn.setStyleSheet(
            "background: #dc2626; color: white; font-weight: 800; padding: 8px 14px; border-radius: 9px;"
        )
        self.stop_btn.clicked.connect(self.stop_code)
        tb.addWidget(self.stop_btn)

        self.template_combo = QComboBox()
        self.template_combo.setMinimumWidth(100)
        self.template_combo.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        self.template_combo.setMinimumContentsLength(10)
        self.template_combo.activated.connect(self._load_template)
        tb.addWidget(self.template_combo)

        self.open_btn = QPushButton()
        self.open_btn.clicked.connect(self._open_py_file)
        tb.addWidget(self.open_btn)

        self.save_btn = QPushButton()
        self.save_btn.clicked.connect(self._save_py_file)
        tb.addWidget(self.save_btn)

        self.save_as_btn = QPushButton()
        self.save_as_btn.clicked.connect(self._save_py_file_as)
        tb.addWidget(self.save_as_btn)
        tb.addStretch(1)
        c_lay.addLayout(tb)

        editor_split = QSplitter(Qt.Orientation.Vertical)
        self.editor = PythonCodeEditor(extra_highlight_symbols=kid_api.__all__, kid_mode=True, i18n=self.i18n)
        self.editor.textChanged.connect(self.editor.clear_error_line)
        editor_split.addWidget(self.editor)

        console_box = QWidget()
        cb_lay = QVBoxLayout(console_box)
        cb_lay.setContentsMargins(0, 0, 0, 0)
        cb_lay.setSpacing(4)
        ch_row = QHBoxLayout()
        self.console_lbl = QLabel()
        ch_row.addWidget(self.console_lbl)
        ch_row.addStretch(1)
        self.clear_console_btn = QPushButton()
        self.clear_console_btn.clicked.connect(lambda: self.console.clear())
        ch_row.addWidget(self.clear_console_btn)
        cb_lay.addLayout(ch_row)

        self.console = QPlainTextEdit()
        self.console.setReadOnly(True)
        cb_lay.addWidget(self.console, 1)
        editor_split.addWidget(console_box)
        editor_split.setSizes([470, 160])
        c_lay.addWidget(editor_split, 1)

        splitter.addWidget(center_panel)

        # ================= COLUMN 3: EMBEDDED 3D VIEWER + WORLD & TELEMETRY =================
        right_panel = QWidget()
        r_lay = QVBoxLayout(right_panel)
        r_lay.setContentsMargins(4, 0, 0, 0)
        r_lay.setSpacing(8)

        r_top = FlowLayout(h_spacing=6, v_spacing=6)
        self.status_lbl = QLabel()
        r_top.addWidget(self.status_lbl)

        self.world_combo = QComboBox()
        self.world_combo.setMinimumWidth(100)
        self.world_combo.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        self.world_combo.setMinimumContentsLength(10)
        self.world_combo.activated.connect(self._on_world_selected)
        r_top.addWidget(self.world_combo)

        self.custom_world_btn = QPushButton("📄 .yaml")
        self.custom_world_btn.clicked.connect(self._browse_custom_world)
        r_top.addWidget(self.custom_world_btn)

        self.restart_btn = QPushButton()
        self.restart_btn.clicked.connect(self._restart_world)
        r_top.addWidget(self.restart_btn)

        self.reset_view_btn = QPushButton()
        self.reset_view_btn.clicked.connect(lambda: self.viewer.reset_view())
        r_top.addWidget(self.reset_view_btn)
        r_lay.addLayout(r_top)

        self.ball_lbl = QLabel()
        self.ball_lbl.setWordWrap(True)
        r_lay.addWidget(self.ball_lbl)

        self.viewer_frame = QFrame()
        self.viewer_frame.setObjectName("viewerFrame")
        vf_lay = QVBoxLayout(self.viewer_frame)
        vf_lay.setContentsMargins(2, 2, 2, 2)
        self.viewer = EmbeddedGLViewer(self.viewer_frame)
        vf_lay.addWidget(self.viewer, 1)
        self.confetti = ConfettiOverlay(self.viewer_frame)
        r_lay.addWidget(self.viewer_frame, 1)

        splitter.addWidget(right_panel)
        splitter.setSizes([235, 510, 460])

    def apply_theme(self, theme: str) -> None:
        """Apply ``theme`` colors to the command palette, console, banners, and viewer frame."""
        t = get_theme_tokens(theme)
        self._palette_frame.setStyleSheet(
            f"QFrame#kidPalette {{ background: {t.surface_bg}; border: 1px solid {t.card_border}; border-radius: 12px; }}"
        )
        self._pal_body.setStyleSheet(f"QWidget#kidPalBody {{ background: {t.surface_bg}; }}")
        self.pal_title.setStyleSheet(f"font-weight: 800; color: {t.heading_accent}; font-size: 13px;")
        for hdr, _pair in self._palette_headers:
            hdr.setStyleSheet(f"font-weight: 700; color: {t.indigo_accent}; margin-top: 4px;")
        pal_btn_qss = (
            f"QPushButton {{ text-align: left; background: {t.palette_btn_bg}; color: {t.palette_btn_fg}; "
            f"font-family: monospace; font-weight: 600; padding: 6px 8px; "
            f"border: 1px solid {t.palette_btn_border}; border-radius: 7px; }}"
            f"QPushButton:hover {{ background: {t.palette_btn_hover}; }}"
        )
        for btn, _icon, _snippets in self._palette_buttons:
            btn.setStyleSheet(pal_btn_qss)
        self.console_lbl.setStyleSheet(f"font-weight: 700; color: {t.text_secondary};")
        self.console.setStyleSheet(
            f"background: {t.console_bg}; color: {t.console_fg}; font-family: monospace; font-size: 12px; "
            f"border: 1px solid {t.console_border}; border-radius: 8px;"
        )
        self.status_lbl.setStyleSheet(f"font-weight: 700; color: {t.heading_accent};")
        self.ball_lbl.setStyleSheet(
            f"background: {t.banner_warn_bg}; border: 1px solid {t.banner_warn_border}; border-radius: 8px; "
            f"padding: 5px 10px; font-weight: 700; color: {t.banner_warn_fg};"
        )
        self.viewer_frame.setStyleSheet(
            f"QFrame#viewerFrame {{ border: 2px solid {t.viewer_border}; border-radius: 12px; background: {t.surface_alt_bg}; }}"
        )

    def _connect_signals(self) -> None:
        """Connect localization and simulator supervisor signals to UI slots."""
        self.i18n.languageChanged.connect(lambda _l: self.retranslate_ui())
        self.sup.sceneInitialized.connect(self._on_scene_init)
        self.sup.frameReady.connect(self._on_frame)
        self.sup.statusChanged.connect(self._on_status)
        self.sup.ballUpdated.connect(self._on_ball)
        self.sup.consoleOutput.connect(self._on_console)
        self.sup.kidScriptStateChanged.connect(self._on_kid_script_state)

    def activate_mode(self) -> None:
        """Activate the mode, populate worlds, and start the interactive simulator."""
        self._active = True
        self._populate_worlds()
        self.viewer.set_overlay_status(self.i18n.t("status_loading"))
        self.sup.set_enabled_eyes(set())
        self.sup.start_interactive(self._selected_world())

    def deactivate_mode(self) -> None:
        """Stop any running kid script and mark the mode inactive."""
        if self._active:
            self.sup.stop_kid_code()
        self._active = False

    def load_code_from_scratch(self, code: str) -> None:
        """
        Load Python code exported from Scratch mode into the editor.

        :param code: Generated Python source code to display in the editor.
        """
        self.editor.source_path = None
        self._template_index = None
        self.editor.setPlainText(code)
        self.editor.clear_error_line()

    def run_code(self) -> None:
        """Clear previous output and execute the current editor code."""
        self.console.clear()
        self.editor.clear_error_line()
        code = self.editor.toPlainText()
        fname = self.editor.source_path or "<kid_program.py>"
        self.sup.run_kid_code(code, fname)

    def stop_code(self) -> None:
        """Send a stop command to interrupt the running simulator script."""
        self.sup.send_command({"cmd": "stop"})

    def _load_template(self, idx: int) -> None:
        """Load a starter template by index into the editor."""
        if 0 <= idx < len(_STARTER_TEMPLATES):
            self.editor.source_path = None
            self._template_index = idx
            self.editor.setPlainText(_STARTER_TEMPLATES[idx][1][self.i18n.lang_idx])
            self.editor.clear_error_line()

    def _open_py_file(self) -> None:
        """Prompt for a Python script file and load its contents into the editor."""
        scripts_dir = str(ensure_user_workspace() / "scripts")
        path, _ = QFileDialog.getOpenFileName(self, self.i18n.t("open_file"), scripts_dir, "Python (*.py)")
        if path and os.path.isfile(path):
            self._template_index = None
            with open(path, "r", encoding="utf-8") as f:
                self.editor.setPlainText(f.read())
            self.editor.source_path = os.path.abspath(path)
            self.editor.clear_error_line()

    def _save_py_file(self) -> None:
        """Save the editor contents to its bound path, or prompt via Save As."""
        if not self.editor.source_path:
            self._save_py_file_as()
            return
        with open(self.editor.source_path, "w", encoding="utf-8") as f:
            f.write(self.editor.toPlainText())

    def _save_py_file_as(self) -> None:
        """Prompt for a file path and save the current editor script."""
        scripts_dir = str(ensure_user_workspace() / "scripts")
        default_p = self.editor.source_path or os.path.join(scripts_dir, "my_kid_robot.py")
        path, _ = QFileDialog.getSaveFileName(self, self.i18n.t("save_as_file"), default_p, "Python (*.py)")
        if path:
            self.editor.source_path = os.path.abspath(path)
            self._save_py_file()

    def _populate_worlds(self) -> None:
        """Refresh the world selector combo box with built-in and custom worlds."""
        prev = self.world_combo.currentData()
        self.world_combo.blockSignals(True)
        self.world_combo.clear()
        li = self.i18n.lang_idx
        for wid, (labels, _cfg) in discover_all_worlds().items():
            self.world_combo.addItem(labels[li], wid)
        if self._custom_world_path and os.path.isfile(self._custom_world_path):
            self.world_combo.addItem(f"📄 {os.path.basename(self._custom_world_path)}", self._custom_world_path)
        idx = self.world_combo.findData(prev or self.sup.current_world or "ball")
        self.world_combo.setCurrentIndex(max(0, idx))
        self.world_combo.blockSignals(False)

    def _selected_world(self) -> str:
        """Return the currently selected world identifier or custom YAML path."""
        return str(self.world_combo.currentData() or "ball")

    def _on_world_selected(self, _idx: int) -> None:
        """Restart the interactive simulator using the newly selected world."""
        self._last_hits = 0
        self.viewer.set_overlay_status(self.i18n.t("status_loading"))
        self.sup.start_interactive(self._selected_world())

    def _browse_custom_world(self) -> None:
        """Prompt for a custom YAML world config and load it in the simulator."""
        ws_worlds = str(ensure_user_workspace() / "worlds")
        path, _ = QFileDialog.getOpenFileName(self, "Select World YAML", ws_worlds, "YAML (*.yaml *.yml)")
        if path and os.path.isfile(path):
            self._custom_world_path = os.path.abspath(path)
            self._populate_worlds()
            self.world_combo.setCurrentIndex(self.world_combo.findData(self._custom_world_path))
            self._on_world_selected(0)

    def _restart_world(self) -> None:
        """Reload the currently selected world in the interactive simulator."""
        self._last_hits = 0
        self.viewer.set_overlay_status(self.i18n.t("status_loading"))
        self.sup.start_interactive(self._selected_world())

    # --- Signals ---

    def _on_scene_init(self, manifest: Dict) -> None:
        """Load scene meshes into the 3D viewer and toggle the ball telemetry banner."""
        if not self._active:
            return
        self.viewer.load_scene_manifest(manifest)
        self.viewer.set_overlay_status("")
        has_ball = any("ball" in os.path.basename(m.get("path", "")).lower() for m in manifest.get("meshes", []))
        self.ball_lbl.setVisible(has_ball)

    def _on_frame(self, frame: Dict) -> None:
        """Update the 3D viewer with a newly rendered simulator frame."""
        if self._active:
            self.viewer.update_frame(frame)

    def _on_status(self, st: Dict) -> None:
        """Update status label, viewer overlay, and console from worker status updates."""
        if not self._active:
            return
        state = st.get("state")
        if state == "loading":
            txt = f"{self.i18n.t('status_loading')} {st.get('detail', '')}".strip()
            self.status_lbl.setText(txt)
            self.viewer.set_overlay_status(txt)
        elif state == "error":
            detail = str(st.get("detail", "")).strip()
            self.status_lbl.setText("❌ Error loading world")
            self.viewer.set_overlay_status("Error loading world")
            if detail:
                self.console.appendPlainText(detail + "\n")
        else:
            self.status_lbl.setText(self.i18n.t("status_ready"))
            self.viewer.set_overlay_status("")

    def _on_kid_script_state(self, ev: Dict) -> None:
        """Reflect script execution state and highlight errors with friendly hints."""
        if not self._active:
            return
        if ev.get("running"):
            self.status_lbl.setText(self.i18n.t("status_running_code"))
            return
        self.status_lbl.setText(self.i18n.t("status_ready"))
        err = ev.get("error")
        if err:
            lineno = err.get("lineno")
            if isinstance(lineno, int):
                self.editor.set_error_line(lineno)
            friendly = _friendly_kid_error(err, self.i18n.lang)
            self.console.moveCursor(self.console.textCursor().MoveOperation.End)
            self.console.insertPlainText("\n" + friendly)

    def _on_ball(self, ball: Dict) -> None:
        """Update ball telemetry text and trigger confetti when hit count increases."""
        if not self._active or not ball:
            return
        hits = int(ball.get("hits", 0))
        if hits > self._last_hits:
            self._last_hits = hits
            self.confetti.burst()
        if not ball.get("touched"):
            self.ball_lbl.setText(self.i18n.t("ball_waiting"))
        else:
            d = ball.get("distance")
            a = ball.get("after")
            self.ball_lbl.setText(
                self.i18n.t(
                    "ball_stats",
                    hits=hits,
                    dist=f"{d:.1f}" if isinstance(d, (int, float)) else "—",
                    after=f"{a:.1f}" if isinstance(a, (int, float)) else "…",
                )
            )

    def _on_console(self, _stream: str, text: str) -> None:
        """Append stdout/stderr text from the simulator worker to the console."""
        if self._active and text:
            self.console.moveCursor(self.console.textCursor().MoveOperation.End)
            self.console.insertPlainText(text)

    def retranslate_ui(self) -> None:
        """Update all UI labels, palette snippets, completions, and templates for the active language."""
        li = self.i18n.lang_idx
        self.pal_title.setText(self.i18n.t("command_palette"))
        for lbl, pair in self._palette_headers:
            lbl.setText(pair[li])
        commands = kid_api.CZECH_COMMANDS
        completions = list(commands.values() if self.i18n.lang == "cs" else commands)
        for button, icon, snippets in self._palette_buttons:
            snippet = snippets[li]
            first_line = snippet.splitlines()[0]
            button.setText(button.fontMetrics().elidedText(f"{icon} {first_line}", Qt.TextElideMode.ElideRight, 190))
            button.setToolTip(snippet)
            if "\n" not in snippet:
                completions.append(snippet)
        self.editor.set_completions(completions)
        if self._template_index is not None and not self.editor.source_path and not self.editor.document().isModified():
            self._load_template(self._template_index)
        self.run_btn.setText(self.i18n.t("run_program"))
        self.stop_btn.setText(self.i18n.t("stop_program"))
        self.open_btn.setText(self.i18n.t("open_file"))
        self.save_btn.setText(self.i18n.t("save_file"))
        self.save_as_btn.setText(self.i18n.t("save_as_file"))
        self.console_lbl.setText(self.i18n.t("console_output"))
        self.clear_console_btn.setText(self.i18n.t("clear_console"))
        self.restart_btn.setText(self.i18n.t("restart_world"))
        self.reset_view_btn.setText(self.i18n.t("reset_view"))
        self.status_lbl.setText(self.i18n.t("status_ready"))
        self.ball_lbl.setText(self.i18n.t("ball_waiting"))

        self.template_combo.blockSignals(True)
        cur_t = max(0, self.template_combo.currentIndex())
        self.template_combo.clear()
        for pair, _code in _STARTER_TEMPLATES:
            self.template_combo.addItem(pair[li])
        self.template_combo.setCurrentIndex(cur_t)
        self.template_combo.blockSignals(False)

        self._populate_worlds()
