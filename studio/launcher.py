"""
Main Window & Mode Launcher Shell for pyCub Standalone.

Hosts the Home mode-selection grid, all 5 integrated simulator modes, and the full-width
integrated Handbook viewer inside a `QStackedWidget` with a persistent header bar for instant
mode switching, Čeština/English toggling, Python Extra Packages configuration, and opening
the user's writable workspace (`~/Documents/pyCub`).
"""
from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple

from PySide6.QtCore import QEvent, Qt, QUrl, Signal
from PySide6.QtGui import QColor, QDesktopServices, QGuiApplication, QIcon, QOpenGLContext, QPalette, QPixmap
from PySide6.QtOpenGLWidgets import QOpenGLWidget
from PySide6.QtWidgets import (
    QComboBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QListView,
    QMainWindow,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from icub_pybullet.runtime_paths import ensure_user_workspace
from studio.i18n import I18nManager
from studio.settings import load_studio_settings, save_studio_settings
from studio import theme as studio_theme
from studio.theme import arrow_icon_url, get_active_theme, get_theme_tokens, normalize_theme
from studio.modes.kid_code import KidCodeMode
from studio.modes.kid_native import KidNativeMode
from studio.modes.kid_scratch import KidScratchMode
from studio.modes.normal_ide import NormalIDEMode
from studio.modes.web_selector import WebSelectorMode
from studio.sim_bridge import SimProcessSupervisor
from studio.widgets.flow_layout import FlowLayout
from studio.widgets.handbook_viewer import HandbookViewer, HandbookWindow
from studio.widgets.packages_dialog import PackagesDialog
from studio.widgets.splash import locate_app_icon


MODE_SPECS: List[Tuple[str, str, str, str]] = [
    ("kid_native", "mode_kid_native_title", "mode_kid_native_desc", "#2563eb"),
    ("kid_scratch", "mode_kid_scratch_title", "mode_kid_scratch_desc", "#7c3aed"),
    ("kid_code", "mode_kid_code_title", "mode_kid_code_desc", "#0891b2"),
    ("normal", "mode_normal_title", "mode_normal_desc", "#0f766e"),
    ("web", "mode_web_title", "mode_web_desc", "#d97706"),
]


def _should_use_gl_surface_anchor() -> bool:
    """Return True on non-headless Windows and macOS setups unless software viewport is forced."""
    plat = (QGuiApplication.platformName() or "").lower()
    return (
        sys.platform in ("win32", "darwin")
        and plat not in ("offscreen", "minimal")
        and os.environ.get("PYCUB_FORCE_SW_VIEWPORT") != "1"
    )


class _GLSurfaceAnchor(QOpenGLWidget):
    """
    Minimal 1x1 OpenGL widget kept permanently visible inside the Windows/macOS header bar so Qt 6
    initializes the top-level window as `OpenGLSurface` from startup and never destroys/recreates
    the native window surface or tears down OpenGL composition when switching between Home,
    Scratch, Simulator, and Handbook pages in the `QStackedWidget`.
    """

    def __init__(self, parent: Optional[QWidget] = None):
        """Initialize a 1x1 transparent-to-mouse OpenGL surface anchor widget."""
        super().__init__(parent)
        self.setFixedSize(1, 1)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)

    def paintGL(self) -> None:
        """Clear the anchor OpenGL surface with the header background color."""
        try:
            ctx = QOpenGLContext.currentContext()
            if ctx is not None:
                f = ctx.functions()
                col = QColor(get_theme_tokens().header_bg)
                f.glClearColor(col.redF(), col.greenF(), col.blueF(), 1.0)
                f.glClear(0x00004000)  # GL_COLOR_BUFFER_BIT
        except Exception:
            pass


class LauncherHomeWidget(QWidget):
    """Home page displaying 5 large visual mode cards."""

    modeSelected = Signal(str)

    def __init__(self, i18n: I18nManager, parent: Optional[QWidget] = None):
        """Initialize the launcher home screen with localized mode selection cards."""
        super().__init__(parent)
        self.i18n = i18n
        self._cards: List[Tuple[QLabel, str, QLabel, str, QPushButton]] = []
        self._card_frames: List[Tuple[QFrame, str]] = []
        self._grid_cols: int = 3
        self._build_ui()
        self.i18n.languageChanged.connect(lambda _l: self.retranslate_ui())
        self.apply_theme(get_active_theme())
        self.retranslate_ui()

    def _build_ui(self) -> None:
        """Construct the heading and grid of mode launcher cards."""
        self.setObjectName("launcherHome")
        outer_root = QVBoxLayout(self)
        outer_root.setContentsMargins(0, 0, 0, 0)

        self._scroll = QScrollArea(self)
        self._scroll.setWidgetResizable(True)
        self._scroll.setFrameShape(QFrame.Shape.NoFrame)
        self._scroll.viewport().installEventFilter(self)
        self._body = QWidget()
        self._body.setObjectName("launcherHomeBody")
        outer = QVBoxLayout(self._body)
        outer.setContentsMargins(32, 26, 32, 26)
        outer.setSpacing(16)

        self.heading_lbl = QLabel()
        self.heading_lbl.setWordWrap(True)
        self.heading_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        outer.addWidget(self.heading_lbl)

        self.sub_lbl = QLabel()
        self.sub_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.sub_lbl.setWordWrap(True)
        outer.addWidget(self.sub_lbl)

        self._grid = QGridLayout()
        self._grid.setSpacing(18)

        for idx, (mode_id, title_key, desc_key, accent) in enumerate(MODE_SPECS):
            card = QFrame()
            card.setObjectName("modeCard")
            c_lay = QVBoxLayout(card)
            c_lay.setContentsMargins(20, 18, 20, 18)
            c_lay.setSpacing(10)

            t_lbl = QLabel()
            t_lbl.setWordWrap(True)
            c_lay.addWidget(t_lbl)

            d_lbl = QLabel()
            d_lbl.setWordWrap(True)
            c_lay.addWidget(d_lbl, 1)

            btn = QPushButton()
            btn.setCursor(Qt.CursorShape.PointingHandCursor)
            btn.setStyleSheet(
                f"QPushButton {{ background: {accent}; color: white; font-size: 14px; font-weight: 800; "
                "padding: 10px 16px; border-radius: 10px; border: none; }"
            )
            btn.clicked.connect(lambda _c=False, m=mode_id: self.modeSelected.emit(m))
            c_lay.addWidget(btn)

            self._cards.append((t_lbl, title_key, d_lbl, desc_key, btn))
            self._card_frames.append((card, accent))
            self._grid.addWidget(card, idx // 3, idx % 3)

        for c in range(3):
            self._grid.setColumnStretch(c, 1)
        outer.addLayout(self._grid, 1)
        self._scroll.setWidget(self._body)
        outer_root.addWidget(self._scroll)

    def apply_theme(self, theme: str) -> None:
        """Apply ``theme`` colors to the Home launcher background, typography, and cards."""
        t = get_theme_tokens(theme)
        self.setStyleSheet(f"QWidget#launcherHome {{ background: {t.window_bg}; }}")
        self._scroll.setStyleSheet(f"QScrollArea {{ background: {t.window_bg}; border: none; }}")
        self._body.setStyleSheet(f"QWidget#launcherHomeBody {{ background: {t.window_bg}; }}")
        self.heading_lbl.setStyleSheet(f"background: transparent; font-size: 26px; font-weight: 900; color: {t.text_primary};")
        self.sub_lbl.setStyleSheet(f"background: transparent; font-size: 14px; color: {t.text_secondary}; margin-bottom: 8px;")
        for card, accent in self._card_frames:
            card.setStyleSheet(
                f"""
                QFrame#modeCard {{
                    background: {t.surface_bg};
                    border: 2px solid {t.border_subtle};
                    border-top: 6px solid {accent};
                    border-radius: 16px;
                }}
                """
            )
        for t_lbl, _tk, d_lbl, _dk, _btn in self._cards:
            t_lbl.setStyleSheet(f"background: transparent; font-size: 18px; font-weight: 800; color: {t.text_primary};")
            d_lbl.setStyleSheet(f"background: transparent; font-size: 13px; color: {t.text_secondary}; line-height: 1.4;")

    def eventFilter(self, obj, event) -> bool:
        """Reflow home mode cards when the scroll viewport width changes."""
        if obj is self._scroll.viewport() and event.type() == QEvent.Type.Resize:
            self._update_grid_columns()
        return super().eventFilter(obj, event)

    def showEvent(self, event) -> None:
        """Sync card column count when the home page becomes visible."""
        super().showEvent(event)
        self._update_grid_columns()

    def _update_grid_columns(self) -> None:
        """Choose 3, 2, or 1 columns for home launcher cards based on viewport width."""
        vp_w = self._scroll.viewport().width()
        if vp_w <= 0:
            return
        want_cols = 3 if vp_w >= 860 else (2 if vp_w >= 520 else 1)
        if want_cols == self._grid_cols:
            return
        self._grid_cols = want_cols
        for card, _accent in self._card_frames:
            self._grid.removeWidget(card)
        for c in range(3):
            self._grid.setColumnStretch(c, 1 if c < want_cols else 0)
        for idx, (card, _accent) in enumerate(self._card_frames):
            self._grid.addWidget(card, idx // want_cols, idx % want_cols)

    def retranslate_ui(self) -> None:
        """Update all card titles, descriptions, and button labels for the active language."""
        self.heading_lbl.setText(self.i18n.t("launcher_heading"))
        self.sub_lbl.setText(self.i18n.t("launcher_sub"))
        open_txt = self.i18n.t("open_mode")
        for t_lbl, t_key, d_lbl, d_key, btn in self._cards:
            t_lbl.setText(self.i18n.t(t_key))
            d_lbl.setText(self.i18n.t(d_key))
            btn.setText(open_txt)


class PyCubMainWindow(QMainWindow):
    """Top-level standalone application window."""

    def __init__(
        self,
        initial_lang: str = "en",
        initial_theme: Optional[str] = None,
        icon_path: Optional[str] = None,
        progress_cb: Optional[Callable[[int, str], None]] = None,
    ):
        """Initialize the main Studio window, header bar, and all stacked mode views.

        :param initial_lang: Initial UI language code ('en' or 'cs').
        :param initial_theme: Optional initial UI theme ('light' or 'dark').
        :param icon_path: Optional path to the application icon file.
        :param progress_cb: Optional splash progress callback receiving ``(percent, message)``.
        """
        super().__init__()
        ensure_user_workspace()

        def _report(pct: int, msg: str) -> None:
            """Forward startup progress updates to ``progress_cb`` when provided."""
            if progress_cb is not None:
                progress_cb(pct, msg)

        self.i18n = I18nManager(initial_lang=initial_lang, parent=self)
        self.supervisor = SimProcessSupervisor(self)
        self._active_mode_id: str = "home"
        self._work_mode_id: str = "home"
        self._theme: str = normalize_theme(initial_theme if initial_theme is not None else get_active_theme())
        self._handbook_window: Optional[HandbookWindow] = None

        if not icon_path:
            icon_path = locate_app_icon()
        self._icon_path = icon_path

        if icon_path:
            self.setWindowIcon(QIcon(icon_path))
        self.resize(1340, 840)

        central = QWidget(self)
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # Persistent Header Bar
        self.header = QFrame()
        self.header.setObjectName("studioHeader")
        header = self.header
        h_pol = header.sizePolicy()
        h_pol.setVerticalPolicy(QSizePolicy.Policy.Preferred)
        h_pol.setHeightForWidth(True)
        header.setSizePolicy(h_pol)
        h_lay = FlowLayout(header, h_spacing=10, v_spacing=6)
        h_lay.setContentsMargins(14, 8, 14, 8)

        if _should_use_gl_surface_anchor():
            self._gl_anchor = _GLSurfaceAnchor(header)
            self._gl_anchor.move(0, 0)
        else:
            self._gl_anchor = None

        if icon_path:
            brand_icon = QLabel()
            brand_icon.setPixmap(
                QPixmap(icon_path).scaled(
                    26,
                    26,
                    Qt.AspectRatioMode.KeepAspectRatio,
                    Qt.TransformationMode.SmoothTransformation,
                )
            )
            h_lay.addWidget(brand_icon)

        brand = QLabel("pyCub Studio")
        brand.setStyleSheet("font-size: 16px; font-weight: 900; color: #f8fafc;")
        h_lay.addWidget(brand)

        self.home_btn = QPushButton()
        self.home_btn.setStyleSheet(
            "QPushButton { background: #1e293b; color: #f8fafc; font-weight: 700; padding: 6px 12px; border-radius: 8px; border: 1px solid #334155; }"
            "QPushButton:hover { background: #334155; }"
        )
        self.home_btn.clicked.connect(lambda: self.switch_mode("home"))
        h_lay.addWidget(self.home_btn)

        self.mode_combo = QComboBox()
        self.mode_combo.setMinimumWidth(160)
        self.mode_combo.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        self.mode_combo.setMinimumContentsLength(16)
        mode_view = QListView(self.mode_combo)
        self.mode_combo.setView(mode_view)
        mode_view.setStyleSheet(
            "QListView { background-color: #1e293b; color: #f8fafc; border: 1px solid #334155; outline: 0; }"
            "QListView::item { background-color: #1e293b; color: #f8fafc; padding: 4px 10px; min-height: 24px; }"
            "QListView::item:selected, QListView::item:hover { background-color: #2563eb; color: #ffffff; }"
        )
        vpal = mode_view.palette()
        vpal.setColor(QPalette.ColorRole.Base, QColor("#1e293b"))
        vpal.setColor(QPalette.ColorRole.Window, QColor("#1e293b"))
        vpal.setColor(QPalette.ColorRole.Text, QColor("#f8fafc"))
        vpal.setColor(QPalette.ColorRole.WindowText, QColor("#f8fafc"))
        vpal.setColor(QPalette.ColorRole.Highlight, QColor("#2563eb"))
        vpal.setColor(QPalette.ColorRole.HighlightedText, QColor("#ffffff"))
        mode_view.setPalette(vpal)
        mode_container = mode_view.parentWidget()
        if mode_container is not None:
            mode_container.setStyleSheet("QFrame { background-color: #1e293b; border: 1px solid #334155; }")
        self.mode_combo.activated.connect(self._on_combo_mode_selected)
        h_lay.addWidget(self.mode_combo)

        h_lay.addStretch(1)

        self.handbook_btn = QPushButton()
        self.handbook_btn.setCheckable(True)
        self.handbook_btn.setStyleSheet(
            "QPushButton { background: #1e293b; color: #f8fafc; font-weight: 700; padding: 6px 12px; border-radius: 8px; border: 1px solid #334155; }"
            "QPushButton:hover { background: #334155; }"
            "QPushButton:checked { background: #2563eb; border-color: #60a5fa; }"
        )
        self.handbook_btn.toggled.connect(self._toggle_handbook)
        h_lay.addWidget(self.handbook_btn)

        self.packages_btn = QPushButton()
        self.packages_btn.setStyleSheet(
            "QPushButton { background: #475569; color: #f8fafc; font-weight: 600; padding: 6px 12px; border-radius: 8px; border: none; }"
            "QPushButton:hover { background: #64748b; }"
        )
        self.packages_btn.clicked.connect(self.open_packages_dialog)
        h_lay.addWidget(self.packages_btn)

        self.workspace_btn = QPushButton()
        self.workspace_btn.setStyleSheet(
            "QPushButton { background: #334155; color: #f8fafc; font-weight: 600; padding: 6px 12px; border-radius: 8px; border: none; }"
            "QPushButton:hover { background: #475569; }"
        )
        self.workspace_btn.clicked.connect(self._open_workspace_folder)
        h_lay.addWidget(self.workspace_btn)

        self.theme_btn = QPushButton()
        self.theme_btn.setStyleSheet(
            "QPushButton { background: #1e293b; color: #f8fafc; font-weight: 700; padding: 6px 12px; border-radius: 8px; border: 1px solid #334155; }"
            "QPushButton:hover { background: #334155; }"
        )
        self.theme_btn.clicked.connect(self.toggle_theme)
        h_lay.addWidget(self.theme_btn)

        self.lang_btn = QPushButton()
        self.lang_btn.setStyleSheet(
            "QPushButton { background: #2563eb; color: #ffffff; font-weight: 800; padding: 6px 12px; border-radius: 8px; border: none; }"
            "QPushButton:hover { background: #1d4ed8; }"
        )
        self.lang_btn.clicked.connect(self.i18n.toggle_language)
        h_lay.addWidget(self.lang_btn)

        root.addWidget(header)

        # Stacked Views (all simulator modes + full-width Handbook viewer)
        self.stack = QStackedWidget()
        root.addWidget(self.stack, 1)

        _report(52, "Připravuji příručku… " if initial_lang == "cs" else "Initializing Handbook viewer…")
        self.handbook_viewer = HandbookViewer(self.i18n, self, is_popout=False)
        self.handbook_viewer.backRequested.connect(self.close_handbook)
        self.handbook_viewer.popoutRequested.connect(self._popout_handbook_window)

        _report(58, "Spouštím domovskou obrazovku…" if initial_lang == "cs" else "Building Home launcher…")
        self.home_widget = LauncherHomeWidget(self.i18n, self)
        self.home_widget.modeSelected.connect(self.switch_mode)

        _report(66, "Připravuji Dětské ovládání…" if initial_lang == "cs" else "Initializing Kid GUI…")
        self.kid_native = KidNativeMode(self.supervisor, self.i18n, self)

        _report(74, "Připravuji blokové programování…" if initial_lang == "cs" else "Initializing Scratch Blocks…")
        self.kid_scratch = KidScratchMode(self.supervisor, self.i18n, self)

        _report(82, "Připravuji Dětské programování…" if initial_lang == "cs" else "Initializing Kid Programming…")
        self.kid_code = KidCodeMode(self.supervisor, self.i18n, self)

        _report(90, "Připravuji Normální IDE…" if initial_lang == "cs" else "Initializing Normal IDE…")
        self.normal_ide = NormalIDEMode(self.supervisor, self.i18n, self)

        _report(96, "Připravuji webový rozcestník…" if initial_lang == "cs" else "Initializing Classroom Web…")
        self.web_selector = WebSelectorMode(self.i18n, self)

        self.kid_scratch.openInKidCodeRequested.connect(self._open_scratch_in_kid_code)

        self._mode_widgets: Dict[str, QWidget] = {
            "home": self.home_widget,
            "kid_native": self.kid_native,
            "kid_scratch": self.kid_scratch,
            "kid_code": self.kid_code,
            "normal": self.normal_ide,
            "web": self.web_selector,
        }
        for w in self._mode_widgets.values():
            self.stack.addWidget(w)
        self.stack.addWidget(self.handbook_viewer)

        self.i18n.languageChanged.connect(lambda _l: self.retranslate_ui())
        self.apply_theme(self._theme, persist=False)
        self.retranslate_ui()
        self.switch_mode("home")

    @property
    def current_theme(self) -> str:
        """Return the active Studio theme identifier (``'light'`` or ``'dark'``)."""
        return self._theme

    def toggle_theme(self) -> None:
        """Toggle Studio's UI theme between Light and Dark modes and persist the choice."""
        next_theme = "dark" if self._theme == "light" else "light"
        self.apply_theme(next_theme, persist=True)

    def apply_theme(self, theme: str, *, persist: bool = False) -> None:
        """Apply ``theme`` (``'light'`` or ``'dark'``) to Studio chrome, panels, and modes."""
        norm = normalize_theme(theme)
        self._theme = norm
        app = QGuiApplication.instance()
        if app is not None:
            from PySide6.QtWidgets import QApplication

            if isinstance(app, QApplication):
                studio_theme.apply_studio_theme(app, norm)
        t = get_theme_tokens(norm)

        self.header.setStyleSheet(
            f"QFrame#studioHeader {{ background: {t.header_bg}; color: #f8fafc; border-bottom: 1px solid {t.header_border}; }}"
        )
        down_url = arrow_icon_url("down", "dark")
        down_rule = f"image: {down_url};" if down_url else ""
        self.mode_combo.setStyleSheet(
            f"QComboBox {{ background: {t.header_btn_bg}; color: #f8fafc; font-weight: 700; "
            f"padding: 5px 26px 5px 10px; border-radius: 8px; border: 1px solid {t.header_btn_border}; }}"
            f"QComboBox:hover {{ background: {t.header_btn_hover}; }}"
            "QComboBox::drop-down { subcontrol-origin: border; subcontrol-position: top right; width: 22px; border: none; background: transparent; }"
            f"QComboBox::down-arrow {{ {down_rule} width: 11px; height: 11px; }}"
            f"QComboBox QAbstractItemView {{ background-color: {t.header_btn_bg}; color: #f8fafc; border: 1px solid {t.header_btn_border}; "
            "selection-background-color: #2563eb; selection-color: #ffffff; outline: 0; }"
        )
        if self._gl_anchor is not None:
            self._gl_anchor.update()

        for w in (
            self.home_widget,
            self.handbook_viewer,
            self.kid_native,
            self.kid_scratch,
            self.kid_code,
            self.normal_ide,
            self.web_selector,
        ):
            if hasattr(w, "apply_theme"):
                w.apply_theme(norm)
        if self._handbook_window is not None:
            self._handbook_window.apply_theme(norm)
        self._sync_theme_btn()

        if persist:
            settings = load_studio_settings()
            if settings.theme != norm:
                settings.theme = norm
                save_studio_settings(settings)

    def _sync_theme_btn(self) -> None:
        """Update the theme toggle button text for the active theme and language."""
        key = "theme_to_light" if self._theme == "dark" else "theme_to_dark"
        self.theme_btn.setText(self.i18n.t(key))

    def _sync_handbook_btn(self) -> None:
        """Synchronize the Handbook toggle button checked state with embedded or pop-out visibility."""
        pop_vis = self._handbook_window is not None and self._handbook_window.isVisible()
        checked = self._active_mode_id == "handbook" or pop_vis
        self.handbook_btn.blockSignals(True)
        self.handbook_btn.setChecked(checked)
        self.handbook_btn.blockSignals(False)

    def _toggle_handbook(self, visible: bool) -> None:
        """Show or close the Handbook view when the header button is toggled."""
        if visible:
            self.open_handbook()
        else:
            if self._handbook_window is not None and self._handbook_window.isVisible():
                self._handbook_window.close()
            self.close_handbook()

    def open_handbook(self, url: Optional[QUrl] = None) -> None:
        """Display the embedded Handbook viewer, optionally navigating to ``url``.

        :param url: Optional target URL to load in the Handbook viewer.
        """
        if url is not None and url.isValid():
            self.handbook_viewer.load_url(url)
        else:
            self.handbook_viewer.ensure_loaded()
        self._active_mode_id = "handbook"
        self.stack.setCurrentWidget(self.handbook_viewer)
        self._sync_handbook_btn()

    def close_handbook(self) -> None:
        """Return from the embedded Handbook viewer to the previously active mode."""
        if self._active_mode_id == "handbook":
            self._active_mode_id = self._work_mode_id
            target_w = self._mode_widgets.get(self._work_mode_id)
            if target_w is not None:
                self.stack.setCurrentWidget(target_w)
                target_w.update()
                viewer = getattr(target_w, "viewer", None)
                if viewer is not None:
                    viewer.update()
                web_view = getattr(target_w, "web_view", None)
                if web_view is not None:
                    web_view.update()
        self._sync_handbook_btn()

    def _popout_handbook_window(self) -> None:
        """Detach the Handbook into a standalone window synced to the current URL."""
        if self._handbook_window is None:
            self._handbook_window = HandbookWindow(self.i18n, icon_path=self._icon_path)
            self._handbook_window.apply_theme(self._theme)
            self._handbook_window.embedRequested.connect(self._on_handbook_embed_requested)
            self._handbook_window.closed.connect(self._sync_handbook_btn)

        cur_url = self.handbook_viewer.current_url()
        if cur_url is not None and cur_url.isValid():
            self._handbook_window.viewer.load_url(cur_url)
        else:
            self._handbook_window.viewer.ensure_loaded()

        self.close_handbook()
        self._handbook_window.show()
        self._handbook_window.raise_()
        self._handbook_window.activateWindow()
        self._sync_handbook_btn()

    def _on_handbook_embed_requested(self, url: object) -> None:
        """Re-embed the Handbook into the main window at the requested URL."""
        qurl = url if isinstance(url, QUrl) else None
        self.open_handbook(url=qurl)

    def open_packages_dialog(self) -> None:
        """Open the Python Environment and Extra Packages configuration dialog."""
        dlg = PackagesDialog(self.i18n, self)
        dlg.exec()

    def _open_scratch_in_kid_code(self, code: str) -> None:
        """Load generated Scratch Python code into Kid Programming mode and switch to it."""
        self.kid_code.load_code_from_scratch(code)
        self.switch_mode("kid_code")

    def _open_workspace_folder(self) -> None:
        """Ensure the user workspace exists and open it in the system file manager."""
        ws = ensure_user_workspace()
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(ws)))

    def _on_combo_mode_selected(self, index: int) -> None:
        """Switch to the mode selected in the header combo box."""
        mode_id = self.mode_combo.itemData(index)
        if mode_id:
            self.switch_mode(str(mode_id))

    def switch_mode(self, mode_id: str) -> None:
        """Switch the active stacked view to ``mode_id`` and manage mode/worker lifecycle.

        :param mode_id: Target mode identifier from :data:`MODE_SPECS` or ``'home'``.
        """
        if mode_id not in self._mode_widgets:
            return
        if self._work_mode_id != mode_id:
            prev_w = self._mode_widgets.get(self._work_mode_id)
            if hasattr(prev_w, "deactivate_mode"):
                prev_w.deactivate_mode()

        self._work_mode_id = mode_id
        self._active_mode_id = mode_id
        target_w = self._mode_widgets[mode_id]
        self.stack.setCurrentWidget(target_w)
        self._sync_handbook_btn()

        if mode_id in ("home", "web"):
            self.supervisor.stop_worker()

        if hasattr(target_w, "activate_mode"):
            target_w.activate_mode()

        self.mode_combo.blockSignals(True)
        idx = self.mode_combo.findData(mode_id)
        if idx >= 0:
            self.mode_combo.setCurrentIndex(idx)
        self.mode_combo.blockSignals(False)

    def retranslate_ui(self) -> None:
        """Refresh window title, header controls, and mode selector labels for the active language."""
        self.setWindowTitle(self.i18n.t("app_title"))
        self.home_btn.setText(self.i18n.t("home_btn"))
        self.packages_btn.setText(self.i18n.t("packages_btn"))
        self.handbook_btn.setText(self.i18n.t("handbook"))
        self.workspace_btn.setText(self.i18n.t("workspace_btn"))
        self._sync_theme_btn()
        self.lang_btn.setText(self.i18n.t("lang_toggle"))

        self.mode_combo.blockSignals(True)
        self.mode_combo.clear()
        self.mode_combo.addItem(self.i18n.t("home_btn"), "home")
        for mode_id, title_key, _desc, _acc in MODE_SPECS:
            self.mode_combo.addItem(self.i18n.t(title_key), mode_id)
        idx = self.mode_combo.findData(self._work_mode_id)
        self.mode_combo.setCurrentIndex(max(0, idx))
        self.mode_combo.blockSignals(False)

    def closeEvent(self, event) -> None:
        """Close auxiliary windows and stop simulation/web workers before window teardown."""
        if self._handbook_window is not None:
            self._handbook_window.close()
        self.supervisor.stop_worker()
        self.web_selector.shutdown_all()
        super().closeEvent(event)
