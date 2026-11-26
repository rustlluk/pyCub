"""
Mode 1: Kid GUI (Native Desktop).

Native Qt port of `studio/kid_ui/kid_ui.py` with hardware-accelerated embedded 3D rendering,
instant tactile button feedback, custom `.yaml` world loading, live ball telemetry,confetti bursts,
and live Čeština / English switching.
"""
from __future__ import annotations

import os
from typing import Dict, List, Optional, Tuple

from PySide6.QtCore import QEvent, Qt
from PySide6.QtWidgets import (
    QButtonGroup,
    QComboBox,
    QFileDialog,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

from studio.kid_ui.kid_ui import JOGS, SECTIONS
from icub_pybullet.runtime_paths import ensure_user_workspace
from studio.i18n import I18nManager
from studio.theme import get_active_theme, get_theme_tokens
from studio.sim_bridge import SimProcessSupervisor, discover_all_worlds
from studio.widgets.confetti import ConfettiOverlay
from studio.widgets.flow_layout import FlowLayout
from studio.widgets.viewport import EmbeddedGLViewer

class KidNativeMode(QWidget):
    """Mode 1 widget: Native button-driven robot playground."""

    def __init__(
        self,
        supervisor: SimProcessSupervisor,
        i18n: I18nManager,
        parent: Optional[QWidget] = None,
    ):
        """Initialize the Kid Native button-driven playground UI and signal bindings."""
        super().__init__(parent)
        self.sup = supervisor
        self.i18n = i18n
        self._active = False
        self._last_hits = 0
        self._speed_value = 1.2
        self._custom_world_path: Optional[str] = None

        # Translatable widget references
        self._section_titles: List[Tuple[QLabel, Tuple[str, str]]] = []
        self._action_btns: List[Tuple[QPushButton, Tuple[str, str]]] = []
        self._jog_rows: List[Tuple[QLabel, Tuple[str, str], QPushButton, Tuple[str, str], QPushButton, Tuple[str, str]]] = []
        self._all_cards: List[QFrame] = []
        self._cards: List[QFrame] = []
        self._jog_layouts: List[Tuple[QGridLayout, QLabel, QPushButton, QPushButton]] = []
        self._grid_cols: int = 2
        self._jog_stacked: bool = False
        self._relayouting: bool = False

        self._build_ui()
        self.apply_theme(get_active_theme())
        self._connect_signals()
        self.retranslate_ui()

    def _make_card(self) -> Tuple[QFrame, QVBoxLayout]:
        """Create a styled card frame paired with its vertical layout."""
        card = QFrame()
        card.setObjectName("kidCard")
        self._all_cards.append(card)
        lay = QVBoxLayout(card)
        lay.setContentsMargins(12, 10, 12, 10)
        lay.setSpacing(8)
        return card, lay

    def _build_ui(self) -> None:
        """Construct the 3D viewport column and scrollable action/jog button grid."""
        root = QHBoxLayout(self)
        root.setContentsMargins(10, 10, 10, 10)

        self.splitter = QSplitter(Qt.Orientation.Horizontal, self)
        root.addWidget(self.splitter)

        # ================= LEFT COLUMN: 3D VIEWPORT + WORLD & STEP CONTROLS =================
        left_scroll = QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll.setFrameShape(QFrame.Shape.NoFrame)
        self._left_panel = QWidget()
        self._left_panel.setObjectName("kidLeftPanel")
        left_lay = QVBoxLayout(self._left_panel)
        left_lay.setContentsMargins(0, 0, 4, 0)
        left_lay.setSpacing(8)

        # Status & Ball banner
        top_info = QHBoxLayout()
        self.status_label = QLabel()
        top_info.addWidget(self.status_label, 1)

        self.reset_view_btn = QPushButton()
        self.reset_view_btn.setStyleSheet(
            "background: #1e40af; color: white; font-weight: 600; padding: 6px 12px; border-radius: 8px;"
        )
        self.reset_view_btn.clicked.connect(lambda: self.viewer.reset_view())
        top_info.addWidget(self.reset_view_btn)
        left_lay.addLayout(top_info)

        self.ball_card = QFrame()
        self.ball_card.setObjectName("kidBallCard")
        ball_lay = QHBoxLayout(self.ball_card)
        ball_lay.setContentsMargins(10, 6, 10, 6)
        self.ball_label = QLabel()
        self.ball_label.setWordWrap(True)
        ball_lay.addWidget(self.ball_label)
        left_lay.addWidget(self.ball_card)

        # Embedded 3D Viewport + Confetti
        self.viewer_container = QFrame()
        self.viewer_container.setObjectName("kidViewerContainer")
        vc_lay = QVBoxLayout(self.viewer_container)
        vc_lay.setContentsMargins(2, 2, 2, 2)
        self.viewer = EmbeddedGLViewer(self.viewer_container)
        vc_lay.addWidget(self.viewer, 1)
        self.confetti = ConfettiOverlay(self.viewer_container)
        left_lay.addWidget(self.viewer_container, 1)

        # World & Emergency Controls Card
        w_card, w_lay = self._make_card()
        w_row1 = FlowLayout(h_spacing=6, v_spacing=6)
        self.world_title_lbl = QLabel()
        w_row1.addWidget(self.world_title_lbl)

        self.world_combo = QComboBox()
        self.world_combo.setMinimumWidth(110)
        self.world_combo.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        self.world_combo.setMinimumContentsLength(10)
        self.world_combo.activated.connect(self._on_world_selected)
        w_row1.addWidget(self.world_combo)

        self.custom_world_btn = QPushButton()
        self.custom_world_btn.setStyleSheet(
            "background: #0d9488; color: white; font-weight: 600; padding: 6px 10px; border-radius: 8px;"
        )
        self.custom_world_btn.clicked.connect(self._browse_custom_world)
        w_row1.addWidget(self.custom_world_btn)
        w_lay.addLayout(w_row1)

        w_row2 = FlowLayout(h_spacing=6, v_spacing=6)
        self.restart_btn = QPushButton()
        self.restart_btn.setStyleSheet(
            "background: #f59e0b; color: white; font-weight: 700; padding: 9px 12px; border-radius: 10px;"
        )
        self.restart_btn.clicked.connect(self._restart_world)
        w_row2.addWidget(self.restart_btn)

        self.reset_pose_btn = QPushButton()
        self.reset_pose_btn.setStyleSheet(
            "background: #3b82f6; color: white; font-weight: 700; padding: 9px 12px; border-radius: 10px;"
        )
        self.reset_pose_btn.clicked.connect(lambda: self._trigger_preset("reset_pose"))
        w_row2.addWidget(self.reset_pose_btn)

        self.stop_btn = QPushButton()
        self.stop_btn.setStyleSheet(
            "background: #dc2626; color: white; font-weight: 800; padding: 9px 14px; border-radius: 10px;"
        )
        self.stop_btn.clicked.connect(lambda: self.sup.send_command({"cmd": "stop"}))
        w_row2.addWidget(self.stop_btn)
        w_lay.addLayout(w_row2)
        left_lay.addWidget(w_card)

        # Step & Speed Card
        s_card, s_lay = self._make_card()
        step_row = FlowLayout(h_spacing=6, v_spacing=6)
        self.step_lbl = QLabel()
        step_row.addWidget(self.step_lbl)

        self.step_spin = QSpinBox()
        self.step_spin.setRange(1, 90)
        self.step_spin.setValue(15)
        self.step_spin.setSuffix("°")
        self.step_spin.setStyleSheet("QSpinBox { font-weight: 700; }")
        for preset_deg in (5, 15, 30):
            b = QPushButton(f"{preset_deg}°")
            b.setStyleSheet(
                "background: #7c3aed; color: white; font-weight: 700; padding: 5px 12px; border-radius: 8px;"
            )
            b.clicked.connect(lambda _checked=False, d=preset_deg: self.step_spin.setValue(d))
            step_row.addWidget(b)
        step_row.addWidget(self.step_spin)
        step_row.addStretch(1)
        s_lay.addLayout(step_row)

        speed_row = FlowLayout(h_spacing=6, v_spacing=6)
        self.speed_lbl = QLabel()
        speed_row.addWidget(self.speed_lbl)

        self.speed_group = QButtonGroup(self)
        self.speed_group.setExclusive(True)
        self.speed_btns: List[Tuple[QPushButton, str, float]] = []
        for idx, (key, val) in enumerate([("speed_slow", 0.5), ("speed_medium", 1.2), ("speed_fast", 2.5)]):
            btn = QPushButton()
            btn.setCheckable(True)
            if idx == 1:
                btn.setChecked(True)
            btn.clicked.connect(lambda _c=False, v=val: self._set_speed(v))
            self.speed_group.addButton(btn, idx)
            speed_row.addWidget(btn)
            self.speed_btns.append((btn, key, val))
        speed_row.addStretch(1)
        s_lay.addLayout(speed_row)
        left_lay.addWidget(s_card)

        left_scroll.setWidget(self._left_panel)
        self.splitter.addWidget(left_scroll)

        # ================= RIGHT COLUMN: SCROLLABLE ACTION & JOG CARDS =================
        self._right_scroll = QScrollArea()
        self._right_scroll.setWidgetResizable(True)
        self._right_scroll.setFrameShape(QFrame.Shape.NoFrame)
        self._right_scroll.viewport().installEventFilter(self)
        self._scroll_body = QWidget()
        self._scroll_body.setObjectName("kidScrollBody")
        self._right_grid = QGridLayout(self._scroll_body)
        self._right_grid.setContentsMargins(8, 6, 8, 8)
        self._right_grid.setSpacing(10)

        for sec_idx, sec in enumerate(SECTIONS):
            card, c_lay = self._make_card()
            sec_lbl = QLabel()
            sec_lbl.setWordWrap(True)
            c_lay.addWidget(sec_lbl)
            self._section_titles.append((sec_lbl, sec["title"]))

            if sec["buttons"]:
                btn_row = FlowLayout(h_spacing=6, v_spacing=6)
                for act_id, labels in sec["buttons"]:
                    ab = QPushButton()
                    ab.setMinimumHeight(38)
                    ab.setStyleSheet(
                        "background: #2563eb; color: white; font-size: 13px; font-weight: 700; "
                        "padding: 8px 12px; border-radius: 10px;"
                    )
                    ab.clicked.connect(lambda _c=False, a=act_id: self._trigger_preset(a))
                    btn_row.addWidget(ab)
                    self._action_btns.append((ab, labels))
                c_lay.addLayout(btn_row)

            for row_label, jog_id, plus_labels, minus_labels in sec["rows"]:
                r_grid = QGridLayout()
                r_grid.setHorizontalSpacing(6)
                r_grid.setVerticalSpacing(4)
                nl = QLabel()
                nl.setMinimumWidth(120)
                nl.setMaximumWidth(135)
                nl.setWordWrap(True)

                pb = QPushButton()
                pb.setMinimumHeight(34)
                pb.setStyleSheet(
                    "background: #3b82f6; color: white; font-weight: 700; padding: 6px 8px; border-radius: 9px;"
                )
                pb.clicked.connect(lambda _c=False, jid=jog_id: self._trigger_jog(jid, +1))

                mb = QPushButton()
                mb.setMinimumHeight(34)
                mb.setStyleSheet(
                    "background: #3b82f6; color: white; font-weight: 700; padding: 6px 8px; border-radius: 9px;"
                )
                mb.clicked.connect(lambda _c=False, jid=jog_id: self._trigger_jog(jid, -1))

                r_grid.addWidget(nl, 0, 0)
                r_grid.addWidget(pb, 0, 1)
                r_grid.addWidget(mb, 0, 2)
                r_grid.setColumnStretch(0, 0)
                r_grid.setColumnStretch(1, 1)
                r_grid.setColumnStretch(2, 1)

                c_lay.addLayout(r_grid)
                self._jog_layouts.append((r_grid, nl, pb, mb))
                self._jog_rows.append((nl, row_label, pb, plus_labels, mb, minus_labels))

            c_lay.addStretch(1)
            self._cards.append(card)
            self._right_grid.addWidget(card, sec_idx // 2, sec_idx % 2)

        self._right_grid.setColumnStretch(0, 1)
        self._right_grid.setColumnStretch(1, 1)
        self._right_grid.setRowStretch((len(SECTIONS) + 1) // 2, 1)
        self._right_scroll.setWidget(self._scroll_body)
        self.splitter.addWidget(self._right_scroll)
        self.splitter.setSizes([460, 760])

    def apply_theme(self, theme: str) -> None:
        """Apply ``theme`` colors to cards, banners, labels, and speed selector buttons."""
        t = get_theme_tokens(theme)
        card_qss = (
            f"QFrame#kidCard {{ background: {t.surface_bg}; border: 1px solid {t.card_border}; border-radius: 12px; }}"
        )
        for card in self._all_cards:
            card.setStyleSheet(card_qss)
        self._left_panel.setStyleSheet(f"QWidget#kidLeftPanel {{ background: {t.window_bg}; }}")
        self._scroll_body.setStyleSheet(f"QWidget#kidScrollBody {{ background: {t.surface_tint_bg}; }}")
        self.status_label.setStyleSheet(f"background: transparent; font-size: 14px; font-weight: 700; color: {t.heading_accent};")
        self.ball_card.setStyleSheet(
            f"QFrame#kidBallCard {{ background: {t.banner_warn_bg}; border: 2px solid {t.banner_warn_border}; border-radius: 10px; padding: 4px; }}"
        )
        self.ball_label.setStyleSheet(f"background: transparent; font-size: 13px; font-weight: 700; color: {t.banner_warn_fg};")
        self.viewer_container.setStyleSheet(
            f"QFrame#kidViewerContainer {{ border: 2px solid {t.viewer_border}; border-radius: 12px; background: {t.surface_alt_bg}; }}"
        )
        self.world_title_lbl.setStyleSheet(f"font-weight: 700; color: {t.heading_accent};")
        self.step_lbl.setStyleSheet(f"font-weight: 700; color: {t.purple_accent};")
        self.speed_lbl.setStyleSheet(f"font-weight: 700; color: {t.purple_accent};")
        speed_qss = (
            f"QPushButton {{ background: {t.speed_btn_bg}; color: {t.speed_btn_fg}; font-weight: 700; "
            f"padding: 6px 12px; border-radius: 8px; border: 1px solid {t.speed_btn_border}; }}"
            f"QPushButton:checked {{ background: {t.speed_checked_bg}; color: #ffffff; border: 2px solid {t.speed_checked_border}; }}"
        )
        for btn, _key, _val in self.speed_btns:
            btn.setStyleSheet(speed_qss)
        for sec_lbl, _pair in self._section_titles:
            sec_lbl.setStyleSheet(f"font-size: 15px; font-weight: 800; color: {t.section_accent};")
        for nl, _rpair, _pb, _pp, _mb, _mp in self._jog_rows:
            nl.setStyleSheet(f"font-size: 12px; font-weight: 700; color: {t.heading_accent};")

    def eventFilter(self, obj, event) -> bool:
        """Reflow button cards when the right scroll viewport is resized."""
        if obj is self._right_scroll.viewport() and event.type() == QEvent.Type.Resize:
            self._update_responsive_layout()
        return super().eventFilter(obj, event)

    def showEvent(self, event) -> None:
        """Apply responsive column and jog-row layout when shown."""
        super().showEvent(event)
        self._update_responsive_layout()

    def resizeEvent(self, event) -> None:
        """Adapt splitter orientation, card columns, and jog rows to the current width."""
        super().resizeEvent(event)
        self._update_responsive_layout()

    def _update_responsive_layout(self) -> None:
        """Adjust splitter orientation, grid column count, and jog-row wrapping to fit available width."""
        if self._relayouting:
            return
        self._relayouting = True
        try:
            total_w = self.width()
            if total_w > 0:
                want_orient = Qt.Orientation.Vertical if total_w < 560 else Qt.Orientation.Horizontal
                if self.splitter.orientation() != want_orient:
                    self.splitter.setOrientation(want_orient)
                    if want_orient == Qt.Orientation.Vertical:
                        h = max(200, self.height())
                        self.splitter.setSizes([h // 2, h - (h // 2)])
                    else:
                        self.splitter.setSizes([int(total_w * 0.40), int(total_w * 0.60)])

            vp_w = self._right_scroll.viewport().width()
            if vp_w <= 0:
                return

            want_cols = 2 if vp_w >= 730 else 1
            card_avail_w = (vp_w - 16 - (want_cols - 1) * 10) // want_cols
            want_stacked = card_avail_w < 350
            changed = False

            if want_stacked != self._jog_stacked:
                self._jog_stacked = want_stacked
                for r_grid, nl, pb, mb in self._jog_layouts:
                    r_grid.removeWidget(nl)
                    r_grid.removeWidget(pb)
                    r_grid.removeWidget(mb)
                    if want_stacked:
                        nl.setMinimumWidth(0)
                        nl.setMaximumWidth(16777215)
                        r_grid.addWidget(nl, 0, 0, 1, 2)
                        r_grid.addWidget(pb, 1, 0)
                        r_grid.addWidget(mb, 1, 1)
                        r_grid.setColumnStretch(0, 1)
                        r_grid.setColumnStretch(1, 1)
                        r_grid.setColumnStretch(2, 0)
                    else:
                        nl.setMinimumWidth(120)
                        nl.setMaximumWidth(135)
                        r_grid.addWidget(nl, 0, 0)
                        r_grid.addWidget(pb, 0, 1)
                        r_grid.addWidget(mb, 0, 2)
                        r_grid.setColumnStretch(0, 0)
                        r_grid.setColumnStretch(1, 1)
                        r_grid.setColumnStretch(2, 1)
                changed = True

            if want_cols != self._grid_cols:
                self._grid_cols = want_cols
                for card in self._cards:
                    self._right_grid.removeWidget(card)
                for r in range(self._right_grid.rowCount()):
                    self._right_grid.setRowStretch(r, 0)
                for idx, card in enumerate(self._cards):
                    self._right_grid.addWidget(card, idx // want_cols, idx % want_cols)
                self._right_grid.setColumnStretch(0, 1)
                self._right_grid.setColumnStretch(1, 1 if want_cols == 2 else 0)
                rows = (len(self._cards) + want_cols - 1) // want_cols
                self._right_grid.setRowStretch(rows, 1)
                changed = True

            if changed:
                self._right_grid.invalidate()
                self._right_scroll.widget().updateGeometry()
        finally:
            self._relayouting = False

    def _connect_signals(self) -> None:
        """Connect localization and simulator supervisor signals to UI handlers."""
        self.i18n.languageChanged.connect(lambda _l: self.retranslate_ui())
        self.sup.sceneInitialized.connect(self._on_scene_init)
        self.sup.frameReady.connect(self._on_frame)
        self.sup.statusChanged.connect(self._on_status)
        self.sup.ballUpdated.connect(self._on_ball)
        self.sup.resetViewRequested.connect(self.viewer.reset_view)

    # --- Activation Lifecycle ---

    def activate_mode(self) -> None:
        """Activate the mode, refresh worlds, and start the interactive simulator."""
        self._active = True
        self._populate_worlds()
        target_world = self._selected_world_id()
        self.viewer.set_overlay_status(self.i18n.t("status_loading"))
        self.sup.set_enabled_eyes(set())
        self.sup.start_interactive(target_world)

    def deactivate_mode(self) -> None:
        """Mark the mode inactive."""
        self._active = False

    # --- World & Control Handlers ---

    def _populate_worlds(self) -> None:
        """Refresh the world combo box with built-in and custom world entries."""
        prev = self.world_combo.currentData()
        self.world_combo.blockSignals(True)
        self.world_combo.clear()
        idx_lang = self.i18n.lang_idx
        worlds = discover_all_worlds()
        for wid, (labels, _cfg) in worlds.items():
            self.world_combo.addItem(labels[idx_lang], wid)
        if self._custom_world_path and os.path.isfile(self._custom_world_path):
            cname = f"📄 {os.path.basename(self._custom_world_path)}"
            self.world_combo.addItem(cname, self._custom_world_path)

        target = prev or self.sup.current_world or "ball"
        found = self.world_combo.findData(target)
        self.world_combo.setCurrentIndex(max(0, found))
        self.world_combo.blockSignals(False)

    def _selected_world_id(self) -> str:
        """Return the currently selected world ID or custom YAML path."""
        data = self.world_combo.currentData()
        return str(data) if data else "ball"

    def _on_world_selected(self, _index: int) -> None:
        """Start the simulator with the newly selected world."""
        world = self._selected_world_id()
        self._last_hits = 0
        self.viewer.set_overlay_status(self.i18n.t("status_loading"))
        self.sup.start_interactive(world)

    def _browse_custom_world(self) -> None:
        """Prompt for a custom YAML world file and launch it in the simulator."""
        ws_worlds = str(ensure_user_workspace() / "worlds")
        path, _ = QFileDialog.getOpenFileName(
            self,
            self.i18n.t("custom_world_btn"),
            ws_worlds,
            "YAML World Config (*.yaml *.yml)",
        )
        if path and os.path.isfile(path):
            self._custom_world_path = os.path.abspath(path)
            self._populate_worlds()
            idx = self.world_combo.findData(self._custom_world_path)
            if idx >= 0:
                self.world_combo.setCurrentIndex(idx)
            self._last_hits = 0
            self.viewer.set_overlay_status(self.i18n.t("status_loading"))
            self.sup.start_interactive(self._custom_world_path)

    def _restart_world(self) -> None:
        """Reset hit state and reload the currently selected world."""
        self._last_hits = 0
        self.viewer.set_overlay_status(self.i18n.t("status_loading"))
        self.sup.start_interactive(self._selected_world_id())

    def _set_speed(self, val: float) -> None:
        """Store the selected joint movement speed multiplier."""
        self._speed_value = float(val)

    def _trigger_preset(self, action_name: str) -> None:
        """Send a preset action command to the simulator worker."""
        self.sup.send_command({"cmd": "preset", "name": action_name})

    def _trigger_jog(self, jog_id: str, direction: int) -> None:
        """Send a relative joint jog command using the current step angle and speed."""
        if jog_id not in JOGS:
            return
        joint, sign = JOGS[jog_id]
        delta = float(direction * sign * self.step_spin.value())
        self.sup.send_command({"cmd": "jog", "joint": joint, "delta": delta, "speed": self._speed_value})

    # --- Worker Signal Handlers ---

    def _on_scene_init(self, manifest: Dict) -> None:
        """Load the scene manifest into the 3D viewer and configure ball banner visibility."""
        if not self._active:
            return
        self.viewer.load_scene_manifest(manifest)
        self.viewer.set_overlay_status("")
        has_ball = any("ball" in os.path.basename(m.get("path", "")).lower() for m in manifest.get("meshes", []))
        self.ball_card.setVisible(has_ball)
        if has_ball:
            self.ball_label.setText(self.i18n.t("ball_waiting"))

    def _on_frame(self, frame: Dict) -> None:
        """Render the latest pose frame in the embedded 3D viewer."""
        if self._active:
            self.viewer.update_frame(frame)

    def _on_status(self, st: Dict) -> None:
        """Update the status label and viewer overlay from worker state messages."""
        if not self._active:
            return
        state = st.get("state", "ready")
        detail = st.get("detail", "")
        if state == "loading":
            txt = f"{self.i18n.t('status_loading')} {detail}".strip()
            self.status_label.setText(txt)
            self.viewer.set_overlay_status(txt)
        elif state == "busy":
            self.status_label.setText(self.i18n.t("status_busy"))
            self.viewer.set_overlay_status("")
        elif state == "error":
            self.status_label.setText("❌ Error loading world")
            self.viewer.set_overlay_status("Error loading world — check YAML")
        else:
            self.status_label.setText(self.i18n.t("status_ready"))
            self.viewer.set_overlay_status("")

    def _on_ball(self, ball: Dict) -> None:
        """Update ball distance/hit telemetry and trigger confetti on new hits."""
        if not self._active or not ball:
            return
        self.ball_card.setVisible(True)
        hits = int(ball.get("hits", 0))
        if hits > self._last_hits:
            self._last_hits = hits
            self.confetti.burst()
        if not ball.get("touched"):
            self.ball_label.setText(self.i18n.t("ball_waiting"))
        else:
            dist = ball.get("distance")
            after = ball.get("after")
            self.ball_label.setText(
                self.i18n.t(
                    "ball_stats",
                    hits=hits,
                    dist=f"{dist:.1f}" if isinstance(dist, (int, float)) else "—",
                    after=f"{after:.1f}" if isinstance(after, (int, float)) else "…",
                )
            )

    def retranslate_ui(self) -> None:
        """Update all labels, buttons, jog rows, and world names for the active language."""
        li = self.i18n.lang_idx
        self.reset_view_btn.setText(self.i18n.t("reset_view"))
        self.world_title_lbl.setText(self.i18n.t("world_section"))
        self.custom_world_btn.setText(self.i18n.t("custom_world_btn"))
        self.restart_btn.setText(self.i18n.t("restart_world"))
        self.reset_pose_btn.setText(self.i18n.t("reset_pose"))
        self.stop_btn.setText(self.i18n.t("stop_robot"))
        self.step_lbl.setText(self.i18n.t("step_label"))
        self.speed_lbl.setText(self.i18n.t("speed_label"))
        self.status_label.setText(self.i18n.t("status_ready"))
        self.ball_label.setText(self.i18n.t("ball_waiting"))

        for btn, key, _val in self.speed_btns:
            btn.setText(self.i18n.t(key))
        for lbl, pair in self._section_titles:
            lbl.setText(pair[li])
        for ab, pair in self._action_btns:
            ab.setText(pair[li])
        for nl, rpair, pb, ppair, mb, mpair in self._jog_rows:
            nl.setText(rpair[li])
            pb.setText(ppair[li])
            mb.setText(mpair[li])

        self._populate_worlds()
