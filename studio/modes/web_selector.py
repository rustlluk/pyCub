"""
Mode 5: Classroom Web GUI Selector & Multi-Instance Server Manager.

Native desktop dashboard wrapping `studio/kid_ui/kid_ui_landing.py` and supervising
`N` independent `kid_ui.py` web instances with live LAN URLs, QR code, per-instance user counts,
one-click instance restart, and log inspection.
"""
from __future__ import annotations

import io
import os
import subprocess
import threading
import webbrowser
from dataclasses import dataclass
from typing import Dict, List, Optional

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QGuiApplication, QImage, QPixmap
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDialog,
    QFileDialog,
    QFormLayout,
    QFrame,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)
from werkzeug.serving import make_server

from studio.kid_ui.kid_ui_landing import create_landing_app, fetch_instance_status
from studio.kid_ui.kid_ui_multi import build_instance_cmd, popen_session_kwargs, port_free
from icub_pybullet.runtime_paths import (
    ensure_user_workspace,
    get_user_log_dir,
    local_ipv4_addresses,
    terminate_process_tree,
)
from studio.i18n import I18nManager
from studio.theme import get_active_theme, get_theme_tokens
from studio.sim_bridge import discover_all_worlds

try:
    import qrcode
    HAS_QRCODE = True
except Exception:  # pragma: no cover
    qrcode = None
    HAS_QRCODE = False


@dataclass
class _ManagedInstance:
    """Metadata and process handles for a single managed web simulator instance."""

    instance_id: int
    web_port: int
    stream_port: int
    world: str
    headless: bool
    log_path: str
    proc: Optional[subprocess.Popen] = None
    log_handle: Optional[object] = None


class WebSelectorMode(QWidget):
    """Mode 5 widget: Multi-instance web classroom server configurator and live monitor."""

    def __init__(self, i18n: I18nManager, parent: Optional[QWidget] = None):
        """Initialize the classroom web server configurator, poll timer, and UI."""
        super().__init__(parent)
        self.i18n = i18n
        self._instances: List[_ManagedInstance] = []
        self._http_server = None
        self._http_thread: Optional[threading.Thread] = None
        self._custom_world_path: Optional[str] = None
        self._primary_url: str = ""

        self._poll_timer = QTimer(self)
        self._poll_timer.setInterval(1500)
        self._poll_timer.timeout.connect(self._refresh_table)

        self._build_ui()
        self.apply_theme(get_active_theme())
        self.i18n.languageChanged.connect(lambda _l: self.retranslate_ui())
        self.retranslate_ui()

    def _build_ui(self) -> None:
        """Construct the cluster configuration form, QR/URL box, and monitor table."""
        root = QHBoxLayout(self)
        root.setContentsMargins(14, 14, 14, 14)

        splitter = QSplitter(Qt.Orientation.Horizontal, self)
        root.addWidget(splitter)

        # ================= LEFT PANEL: CONFIGURATION + LAN URL + QR CODE =================
        self._left_outer = QFrame()
        self._left_outer.setObjectName("webCard")
        left_outer_lay = QVBoxLayout(self._left_outer)
        left_outer_lay.setContentsMargins(2, 2, 2, 2)
        left_scroll = QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll.setFrameShape(QFrame.Shape.NoFrame)
        self._left_card = QWidget()
        self._left_card.setObjectName("webLeftCard")
        left_lay = QVBoxLayout(self._left_card)
        left_lay.setContentsMargins(16, 14, 16, 14)
        left_lay.setSpacing(10)

        self.cfg_heading = QLabel()
        left_lay.addWidget(self.cfg_heading)

        form = QFormLayout()
        form.setSpacing(8)
        form.setFieldGrowthPolicy(QFormLayout.FieldGrowthPolicy.ExpandingFieldsGrow)

        self.count_lbl = QLabel()
        self.count_spin = QSpinBox()
        self.count_spin.setRange(1, 32)
        self.count_spin.setValue(3)
        form.addRow(self.count_lbl, self.count_spin)

        self.landing_port_lbl = QLabel()
        self.landing_port_spin = QSpinBox()
        self.landing_port_spin.setRange(1024, 65500)
        self.landing_port_spin.setValue(5000)
        form.addRow(self.landing_port_lbl, self.landing_port_spin)

        self.inst_port_lbl = QLabel()
        self.inst_port_spin = QSpinBox()
        self.inst_port_spin.setRange(1024, 65500)
        self.inst_port_spin.setValue(5001)
        form.addRow(self.inst_port_lbl, self.inst_port_spin)

        self.stream_port_lbl = QLabel()
        self.stream_port_spin = QSpinBox()
        self.stream_port_spin.setRange(1024, 65500)
        self.stream_port_spin.setValue(6001)
        form.addRow(self.stream_port_lbl, self.stream_port_spin)

        self.host_lbl = QLabel()
        self.host_combo = QComboBox()
        self.host_combo.addItem("0.0.0.0 (Wi-Fi / LAN)", "0.0.0.0")
        self.host_combo.addItem("127.0.0.1 (Localhost only)", "127.0.0.1")
        form.addRow(self.host_lbl, self.host_combo)

        self.world_lbl = QLabel()
        w_row = QHBoxLayout()
        self.world_combo = QComboBox()
        w_row.addWidget(self.world_combo, 1)
        self.browse_world_btn = QPushButton("…")
        self.browse_world_btn.setFixedWidth(36)
        self.browse_world_btn.clicked.connect(self._browse_world)
        w_row.addWidget(self.browse_world_btn)
        form.addRow(self.world_lbl, w_row)

        left_lay.addLayout(form)

        self.headless_chk = QCheckBox()
        left_lay.addWidget(self.headless_chk)

        btn_row = QHBoxLayout()
        self.start_btn = QPushButton()
        self.start_btn.clicked.connect(self.start_cluster)
        btn_row.addWidget(self.start_btn)

        self.stop_btn = QPushButton()
        self.stop_btn.setEnabled(False)
        self.stop_btn.clicked.connect(self.stop_cluster)
        btn_row.addWidget(self.stop_btn)
        left_lay.addLayout(btn_row)

        # LAN URLs & QR Box
        self.url_box = QFrame()
        self.url_box.setObjectName("urlBox")
        ub_lay = QVBoxLayout(self.url_box)
        ub_lay.setContentsMargins(12, 12, 12, 12)
        ub_lay.setSpacing(8)
        self.url_label = QLabel("Server stopped.")
        self.url_label.setWordWrap(True)
        self.url_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        ub_lay.addWidget(self.url_label)

        url_btns = QHBoxLayout()
        self.open_browser_btn = QPushButton()
        self.open_browser_btn.setEnabled(False)
        self.open_browser_btn.clicked.connect(lambda: webbrowser.open(self._primary_url) if self._primary_url else None)
        url_btns.addWidget(self.open_browser_btn)

        self.copy_url_btn = QPushButton()
        self.copy_url_btn.setEnabled(False)
        self.copy_url_btn.clicked.connect(self._copy_url)
        url_btns.addWidget(self.copy_url_btn)
        ub_lay.addLayout(url_btns)

        self.qr_label = QLabel()
        self.qr_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.qr_label.setMinimumHeight(96)
        ub_lay.addWidget(self.qr_label)
        left_lay.addWidget(self.url_box, 1)

        left_scroll.setWidget(self._left_card)
        left_outer_lay.addWidget(left_scroll)
        splitter.addWidget(self._left_outer)

        # ================= RIGHT PANEL: LIVE INSTANCE MONITOR TABLE =================
        self._right_card = QFrame()
        self._right_card.setObjectName("webCard")
        right_lay = QVBoxLayout(self._right_card)
        right_lay.setContentsMargins(16, 14, 16, 14)

        self.table = QTableWidget(0, 6)
        self.table.setHorizontalHeaderLabels(["#", "Web Port", "Stream Port", "Status", "Users", "Actions"])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.table.verticalHeader().setVisible(False)
        self.table.setAlternatingRowColors(True)
        self.table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        right_lay.addWidget(self.table, 1)

        splitter.addWidget(self._right_card)
        splitter.setSizes([430, 670])

    def apply_theme(self, theme: str) -> None:
        """Apply ``theme`` colors to the Classroom Web configurator cards, buttons, and URL box."""
        t = get_theme_tokens(theme)
        card_qss = f"QFrame#webCard {{ background: {t.surface_bg}; border: 1px solid {t.border}; border-radius: 12px; }}"
        self._left_outer.setStyleSheet(card_qss)
        self._right_card.setStyleSheet(card_qss)
        self._left_card.setStyleSheet(f"QWidget#webLeftCard {{ background: {t.surface_bg}; }}")
        self.cfg_heading.setStyleSheet(f"font-size: 16px; font-weight: 800; color: {t.text_primary};")
        self.url_box.setStyleSheet(
            f"QFrame#urlBox {{ background: {t.surface_alt_bg}; border: 1px solid {t.border}; border-radius: 10px; }}"
        )
        self.url_label.setStyleSheet(f"font-family: monospace; font-size: 13px; font-weight: 700; color: {t.section_accent};")
        self.qr_label.setStyleSheet(f"color: {t.text_muted}; font-size: 12px;")
        self.start_btn.setStyleSheet(
            "QPushButton { background: #16a34a; color: white; font-weight: 800; padding: 10px 14px; border-radius: 10px; border: none; }"
            "QPushButton:hover { background: #15803d; }"
            f"QPushButton:disabled {{ background: {t.disabled_green_bg}; color: {t.disabled_green_fg}; }}"
        )
        self.stop_btn.setStyleSheet(
            "QPushButton { background: #dc2626; color: white; font-weight: 800; padding: 10px 14px; border-radius: 10px; border: none; }"
            "QPushButton:hover { background: #b91c1c; }"
            f"QPushButton:disabled {{ background: {t.disabled_red_bg}; color: {t.disabled_red_fg}; }}"
        )
        self.open_browser_btn.setStyleSheet(
            "QPushButton { background: #2563eb; color: #ffffff; font-weight: 700; padding: 7px 12px; border-radius: 8px; border: none; }"
            "QPushButton:hover { background: #1d4ed8; }"
            f"QPushButton:disabled {{ background: {t.button_disabled_bg}; color: {t.button_disabled_fg}; }}"
        )
        self.copy_url_btn.setStyleSheet(
            "QPushButton { background: #0f766e; color: #ffffff; font-weight: 700; padding: 7px 12px; border-radius: 8px; border: none; }"
            "QPushButton:hover { background: #0d9488; }"
            f"QPushButton:disabled {{ background: {t.button_disabled_bg}; color: {t.button_disabled_fg}; }}"
        )

    # --- Lifecycle ---

    def activate_mode(self) -> None:
        """Refresh the world list when entering Web Selector mode."""
        self._populate_worlds()

    def deactivate_mode(self) -> None:
        """Handle mode deactivation without stopping running classroom servers."""
        pass

    def shutdown_all(self) -> None:
        """Stop the landing HTTP server and all spawned web simulator instances."""
        self.stop_cluster()

    # --- Helpers ---

    def _populate_worlds(self) -> None:
        """Refresh the world selector combo box with built-in and custom worlds."""
        prev = self.world_combo.currentData()
        self.world_combo.clear()
        li = self.i18n.lang_idx
        for wid, (labels, _cfg) in discover_all_worlds().items():
            self.world_combo.addItem(labels[li], wid)
        if self._custom_world_path and os.path.isfile(self._custom_world_path):
            self.world_combo.addItem(f"📄 {os.path.basename(self._custom_world_path)}", self._custom_world_path)
        idx = self.world_combo.findData(prev or "ball")
        self.world_combo.setCurrentIndex(max(0, idx))

    def _browse_world(self) -> None:
        """Prompt for a custom YAML world config and select it in the world dropdown."""
        ws_worlds = str(ensure_user_workspace() / "worlds")
        path, _ = QFileDialog.getOpenFileName(self, "Select World YAML", ws_worlds, "YAML (*.yaml *.yml)")
        if path and os.path.isfile(path):
            self._custom_world_path = os.path.abspath(path)
            self._populate_worlds()
            self.world_combo.setCurrentIndex(self.world_combo.findData(self._custom_world_path))

    def _spawn_single_instance(self, inst: _ManagedInstance) -> None:
        """Start or restart a single ``kid_ui`` subprocess and redirect output to its log."""
        if inst.log_handle is not None:
            try:
                inst.log_handle.close()
            except Exception:
                pass
        os.makedirs(os.path.dirname(inst.log_path), exist_ok=True)
        fh = open(inst.log_path, "a", encoding="utf-8")
        inst.log_handle = fh
        cmd = build_instance_cmd(inst.web_port, inst.stream_port, inst.world, headless=inst.headless)
        inst.proc = subprocess.Popen(cmd, stdout=fh, stderr=fh, **popen_session_kwargs())

    def start_cluster(self) -> None:
        """Validate ports, launch N web simulator instances, and start the landing server."""
        self.stop_cluster()

        n = self.count_spin.value()
        landing_port = self.landing_port_spin.value()
        first_web = self.inst_port_spin.value()
        first_stream = self.stream_port_spin.value()
        host = str(self.host_combo.currentData() or "0.0.0.0")
        world = str(self.world_combo.currentData() or "ball")
        headless = self.headless_chk.isChecked()

        web_ports = [first_web + i for i in range(n)]
        stream_ports = [first_stream + i for i in range(n)]

        if set(web_ports) & set(stream_ports) or landing_port in (set(web_ports) | set(stream_ports)):
            QMessageBox.warning(self, "Port Conflict", "Landing, web, and video port ranges must not overlap.")
            return

        busy = [p for p in [landing_port] + web_ports + stream_ports if not port_free(p)]
        if busy:
            QMessageBox.warning(self, "Ports Busy", f"The following ports are already in use: {busy}")
            return

        log_dir = get_user_log_dir() / "web_instances"
        log_dir.mkdir(parents=True, exist_ok=True)

        self._instances = []
        for idx, (wp, sp) in enumerate(zip(web_ports, stream_ports), start=1):
            inst = _ManagedInstance(
                instance_id=idx,
                web_port=wp,
                stream_port=sp,
                world=world,
                headless=headless,
                log_path=str(log_dir / f"instance_{idx}.log"),
            )
            with open(inst.log_path, "w", encoding="utf-8") as f:
                f.write(f"=== Starting pyCub Web Instance #{idx} (web={wp}, stream={sp}, world={world}) ===\n")
            self._spawn_single_instance(inst)
            self._instances.append(inst)

        app = create_landing_app(
            web_ports=web_ports,
            alive_check=lambda: any(i.proc is not None and i.proc.poll() is None for i in self._instances),
        )
        try:
            self._http_server = make_server(host, landing_port, app, threaded=True)
        except OSError as exc:
            self.stop_cluster()
            QMessageBox.critical(self, "Server Error", f"Failed to bind landing server on {host}:{landing_port}:\n{exc}")
            return

        self._http_thread = threading.Thread(target=self._http_server.serve_forever, daemon=True)
        self._http_thread.start()

        lan_ips = local_ipv4_addresses() if host == "0.0.0.0" else ["127.0.0.1"]
        if not lan_ips:
            lan_ips = ["127.0.0.1"]
        self._primary_url = f"http://{lan_ips[0]}:{landing_port}"
        urls_text = "\n".join(f"• http://{ip}:{landing_port}" for ip in lan_ips)
        fw_note = f"\nFirewall TCP/UDP ports: {landing_port}, {web_ports[0]}–{web_ports[-1]}, {stream_ports[0]}–{stream_ports[-1]}"
        self.url_label.setText(urls_text + fw_note)
        self._render_qr(self._primary_url)

        self._set_controls_running(True)
        self._build_table_rows()
        self._poll_timer.start()

    def stop_cluster(self) -> None:
        """Stop the landing server, terminate all instance processes, and reset the UI."""
        self._poll_timer.stop()
        if self._http_server is not None:
            try:
                self._http_server.shutdown()
                self._http_server.server_close()
            except Exception:
                pass
            self._http_server = None
        if self._http_thread is not None:
            self._http_thread.join(timeout=2.0)
            self._http_thread = None

        procs = [i.proc for i in self._instances if i.proc is not None]
        if procs:
            terminate_process_tree(procs, timeout=5.0)
        for inst in self._instances:
            if inst.log_handle is not None:
                try:
                    inst.log_handle.close()
                except Exception:
                    pass
                inst.log_handle = None
            inst.proc = None
        self._instances = []

        self._primary_url = ""
        self.url_label.setText(self.i18n.t("status_stopped"))
        self.qr_label.clear()
        self.table.setRowCount(0)
        self._set_controls_running(False)

    def _set_controls_running(self, running: bool) -> None:
        """Enable or disable configuration and cluster action controls based on run state."""
        self.start_btn.setEnabled(not running)
        self.stop_btn.setEnabled(running)
        self.open_browser_btn.setEnabled(running)
        self.copy_url_btn.setEnabled(running)
        for w in (
            self.count_spin,
            self.landing_port_spin,
            self.inst_port_spin,
            self.stream_port_spin,
            self.host_combo,
            self.world_combo,
            self.browse_world_btn,
            self.headless_chk,
        ):
            w.setEnabled(not running)

    def _render_qr(self, url: str) -> None:
        """Render a QR code pixmap for ``url`` when ``qrcode`` is available."""
        if not HAS_QRCODE or not url:
            self.qr_label.clear()
            return
        qr = qrcode.QRCode(box_size=5, border=2)
        qr.add_data(url)
        qr.make(fit=True)
        pil_img = qr.make_image(fill_color="black", back_color="white")
        buf = io.BytesIO()
        pil_img.save(buf, format="PNG")
        qimg = QImage.fromData(buf.getvalue(), "PNG")
        self.qr_label.setPixmap(QPixmap.fromImage(qimg).scaled(165, 165, Qt.AspectRatioMode.KeepAspectRatio))

    def _copy_url(self) -> None:
        """Copy the primary classroom landing URL to the system clipboard."""
        if self._primary_url:
            QGuiApplication.clipboard().setText(self._primary_url)

    def _build_table_rows(self) -> None:
        """Populate the monitor table rows and action buttons for all spawned instances."""
        self.table.setRowCount(len(self._instances))
        for row, inst in enumerate(self._instances):
            self.table.setItem(row, 0, QTableWidgetItem(f"#{inst.instance_id}"))
            self.table.setItem(row, 1, QTableWidgetItem(str(inst.web_port)))
            self.table.setItem(row, 2, QTableWidgetItem(str(inst.stream_port)))
            self.table.setItem(row, 3, QTableWidgetItem("starting"))
            self.table.setItem(row, 4, QTableWidgetItem("0"))

            act_widget = QWidget()
            act_lay = QHBoxLayout(act_widget)
            act_lay.setContentsMargins(2, 2, 2, 2)
            act_lay.setSpacing(4)

            open_b = QPushButton("Open")
            open_b.clicked.connect(lambda _c=False, p=inst.web_port: webbrowser.open(f"http://127.0.0.1:{p}"))
            act_lay.addWidget(open_b)

            rest_b = QPushButton("Restart")
            rest_b.clicked.connect(lambda _c=False, idx=row: self._restart_instance(idx))
            act_lay.addWidget(rest_b)

            log_b = QPushButton("Log")
            log_b.clicked.connect(lambda _c=False, idx=row: self._show_instance_log(idx))
            act_lay.addWidget(log_b)

            self.table.setCellWidget(row, 5, act_widget)

    def _refresh_table(self) -> None:
        """Poll each managed instance and update its status and active user count."""
        for row, inst in enumerate(self._instances):
            proc_alive = inst.proc is not None and inst.proc.poll() is None
            info: Dict = fetch_instance_status(inst.instance_id, inst.web_port, proc_alive)
            st_item = self.table.item(row, 3)
            u_item = self.table.item(row, 4)
            if st_item is not None:
                st_item.setText(str(info.get("state", "offline")))
            if u_item is not None:
                u_item.setText(str(info.get("users", 0)))

    def _restart_instance(self, row: int) -> None:
        """Terminate and respawn the managed instance at the given table row."""
        if not (0 <= row < len(self._instances)):
            return
        inst = self._instances[row]
        if inst.proc is not None and inst.proc.poll() is None:
            terminate_process_tree([inst.proc], timeout=4.0)
        self._spawn_single_instance(inst)
        self._refresh_table()

    def _show_instance_log(self, row: int) -> None:
        """Open a modal dialog showing the tail of an instance's log file."""
        if not (0 <= row < len(self._instances)):
            return
        inst = self._instances[row]
        dlg = QDialog(self)
        dlg.setWindowTitle(f"Instance #{inst.instance_id} Log ({inst.log_path})")
        dlg.resize(720, 460)
        lay = QVBoxLayout(dlg)
        view = QPlainTextEdit()
        view.setReadOnly(True)
        try:
            with open(inst.log_path, "r", encoding="utf-8", errors="replace") as f:
                view.setPlainText(f.read()[-24000:])
        except OSError as exc:
            view.setPlainText(str(exc))
        lay.addWidget(view)
        dlg.exec()

    def retranslate_ui(self) -> None:
        """Update all form labels, buttons, and world names for the active language."""
        self.cfg_heading.setText(self.i18n.t("web_cfg_heading"))
        self.count_lbl.setText(self.i18n.t("web_instances_count"))
        self.landing_port_lbl.setText(self.i18n.t("web_landing_port"))
        self.inst_port_lbl.setText(self.i18n.t("web_first_inst_port"))
        self.stream_port_lbl.setText(self.i18n.t("web_first_stream_port"))
        self.host_lbl.setText(self.i18n.t("web_host_bind"))
        self.world_lbl.setText(self.i18n.t("web_initial_world"))
        self.headless_chk.setText(self.i18n.t("web_headless_chk"))
        self.start_btn.setText(self.i18n.t("web_start_server"))
        self.stop_btn.setText(self.i18n.t("web_stop_server"))
        self.open_browser_btn.setText(self.i18n.t("web_open_browser"))
        self.copy_url_btn.setText(self.i18n.t("web_copy_url"))
        if not self._instances:
            self.url_label.setText(self.i18n.t("status_stopped"))
        self._populate_worlds()
