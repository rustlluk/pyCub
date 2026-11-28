"""
Integrated Handbook / Handouts Viewer for pyCub Studio.

Displays the bundled offline Sphinx handbook (`docs/handbook/index.html` for English,
`docs/handbook/cs/index.html` for Czech) directly inside the application:
  - Full-width view inside `PyCubMainWindow`'s central `QStackedWidget`, OR
  - Freely-resizable standalone in-app window (`HandbookWindow`, default 1120×780, maximizable)
    for spacious side-by-side or multi-monitor reading without opening an external browser.
"""
from __future__ import annotations

import sys
from pathlib import Path
from typing import List, Optional, Tuple

from PySide6.QtCore import Qt, QUrl, Signal
from PySide6.QtGui import QColor, QDesktopServices, QIcon
from PySide6.QtWidgets import (
    QComboBox,
    QHBoxLayout,
    QPushButton,
    QTextBrowser,
    QVBoxLayout,
    QWidget,
)

from studio.widgets.flow_layout import FlowLayout
from studio.i18n import I18nManager
from studio.resource_paths import iter_studio_data_roots

try:
    from PySide6.QtWebEngineCore import QWebEnginePage
    from PySide6.QtWebEngineWidgets import QWebEngineView

    HAS_WEBENGINE = True
except Exception:  # pragma: no cover
    QWebEnginePage = None
    QWebEngineView = None
    HAS_WEBENGINE = False


_HANDBOOK_SECTIONS: List[Tuple[str, Tuple[str, str]]] = [
    ("index.html", ("🏠 Úvod", "🏠 Home")),
    ("studio.html", ("🖥️ Použití Studia", "🖥️ Using Studio")),
    ("programming.html", ("🐍 Programování robota", "🐍 Programming")),
    ("examples.html", ("📘 Ukázkové programy", "📘 Examples")),
    ("exercises.html", ("🧪 Kurzovní cvičení", "🧪 Exercises")),
    ("readme.html", ("📖 Přehled projektu (README)", "📖 Project README")),
    ("autoapi/index.html", ("📚 API reference", "📚 API Reference")),
]


def locate_handbook_dir() -> Optional[Path]:
    """Locate the bundled `docs/handbook` base directory in source, wheel, or frozen builds."""
    candidates: List[Path] = [
        Path(__file__).resolve().parents[2] / "docs" / "handbook",
    ]
    if getattr(sys, "frozen", False):
        exe_dir = Path(sys.executable).resolve().parent
        candidates.append(exe_dir.parent / "Resources" / "docs" / "handbook")
        candidates.append(exe_dir / "docs" / "handbook")
        candidates.append(exe_dir / "_internal" / "docs" / "handbook")
    meipass = getattr(sys, "_MEIPASS", None)
    if meipass:
        candidates.append(Path(meipass) / "docs" / "handbook")
    for data_root in iter_studio_data_roots():
        candidates.append(data_root / "handbook")

    for cand in candidates:
        if (cand / "index.html").is_file():
            return cand.resolve()
    return None


def locate_handbook_lang_dir(base_dir: Optional[Path], lang: str) -> Optional[Path]:
    """Return the language-specific handbook root (`docs/handbook/cs` for Czech if built)."""
    if base_dir is None:
        return None
    if lang == "cs":
        cs_dir = base_dir / "cs"
        if (cs_dir / "index.html").is_file():
            return cs_dir
    return base_dir


if HAS_WEBENGINE:

    class _HandbookWebPage(QWebEnginePage):
        """Keep local handbook navigation in-app; open external HTTP(S) links in system browser."""

        def acceptNavigationRequest(self, url: QUrl, nav_type, is_main_frame: bool) -> bool:
            """Open external HTTP(S) links in the system browser while keeping local navigation in-app."""
            if url.scheme() in ("http", "https"):
                QDesktopServices.openUrl(url)
                return False
            return super().acceptNavigationRequest(url, nav_type, is_main_frame)


class HandbookViewer(QWidget):
    """Full-featured in-app viewer widget for English & Czech offline handbooks."""

    backRequested = Signal()
    popoutRequested = Signal()

    def __init__(
        self,
        i18n: I18nManager,
        parent: Optional[QWidget] = None,
        *,
        is_popout: bool = False,
    ):
        """Initialize the handbook viewer widget for embedded or popout display."""
        super().__init__(parent)
        self.i18n = i18n
        self._is_popout = is_popout
        self._handbook_dir = locate_handbook_dir()
        self._loaded = False
        self._zoom = 1.0
        self._current_rel_path: str = "index.html"
        self._current_url: Optional[QUrl] = None

        self._build_ui()
        self.apply_theme()
        self.i18n.languageChanged.connect(self._on_language_changed)
        self.retranslate_ui()

    def _build_ui(self) -> None:
        """Construct the navigation toolbar and web or text browser widget."""
        lay = QVBoxLayout(self)
        lay.setContentsMargins(10, 8, 10, 10)
        lay.setSpacing(8)

        bar = FlowLayout(h_spacing=6, v_spacing=6)
        self.back_studio_btn = QPushButton()
        self.back_studio_btn.setStyleSheet(
            "QPushButton { background: #2563eb; color: #ffffff; font-weight: 800; "
            "padding: 5px 12px; border-radius: 6px; border: none; }"
            "QPushButton:hover { background: #1d4ed8; }"
        )
        self.back_studio_btn.clicked.connect(self.backRequested.emit)
        self.back_studio_btn.setVisible(not self._is_popout)
        bar.addWidget(self.back_studio_btn)

        self.back_btn = QPushButton("◀")
        self.back_btn.setFixedWidth(34)
        self.back_btn.clicked.connect(self._go_back)
        bar.addWidget(self.back_btn)

        self.fwd_btn = QPushButton("▶")
        self.fwd_btn.setFixedWidth(34)
        self.fwd_btn.clicked.connect(self._go_forward)
        bar.addWidget(self.fwd_btn)

        self.home_btn = QPushButton("🏠")
        self.home_btn.setFixedWidth(36)
        self.home_btn.clicked.connect(self.go_home)
        bar.addWidget(self.home_btn)

        self.section_combo = QComboBox()
        self.section_combo.setMinimumWidth(140)
        self.section_combo.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        self.section_combo.setMinimumContentsLength(12)
        self.section_combo.activated.connect(self._on_section_selected)
        bar.addWidget(self.section_combo)

        self.zoom_out_btn = QPushButton("−")
        self.zoom_out_btn.setFixedWidth(32)
        self.zoom_out_btn.clicked.connect(lambda: self._adjust_zoom(-0.1))
        bar.addWidget(self.zoom_out_btn)

        self.zoom_in_btn = QPushButton("+")
        self.zoom_in_btn.setFixedWidth(32)
        self.zoom_in_btn.clicked.connect(lambda: self._adjust_zoom(0.1))
        bar.addWidget(self.zoom_in_btn)

        self.lang_btn = QPushButton()
        self.lang_btn.setStyleSheet(
            "QPushButton { background: #2563eb; color: #ffffff; font-weight: 700; "
            "padding: 5px 10px; border-radius: 6px; border: none; }"
            "QPushButton:hover { background: #1d4ed8; }"
        )
        self.lang_btn.clicked.connect(self.i18n.toggle_language)
        self.lang_btn.setVisible(self._is_popout)
        bar.addWidget(self.lang_btn)

        self.popout_btn = QPushButton()
        self.popout_btn.clicked.connect(self.popoutRequested.emit)
        bar.addWidget(self.popout_btn)

        self.open_ext_btn = QPushButton("↗")
        self.open_ext_btn.setFixedWidth(34)
        self.open_ext_btn.clicked.connect(self._open_external)
        bar.addWidget(self.open_ext_btn)

        lay.addLayout(bar)

        if HAS_WEBENGINE:
            self.web_view = QWebEngineView(self)
            page = _HandbookWebPage(self.web_view)
            page.setBackgroundColor(QColor("#ffffff"))
            self.web_view.setPage(page)
            self.text_browser = None
            lay.addWidget(self.web_view, 1)
        else:
            self.web_view = None
            self.text_browser = QTextBrowser(self)
            self.text_browser.setOpenExternalLinks(False)
            lay.addWidget(self.text_browser, 1)

    def apply_theme(self, theme: Optional[str] = None) -> None:
        """Apply ``theme`` colors to handbook toolbar navigation buttons."""
        from studio.theme import get_active_theme, get_theme_tokens

        t = get_theme_tokens(theme if theme is not None else get_active_theme())
        btn_style = (
            f"QPushButton {{ background: {t.button_bg}; color: {t.text_primary}; font-weight: 700; "
            f"padding: 5px 10px; border-radius: 6px; border: 1px solid {t.button_border}; }}"
            f"QPushButton:hover {{ background: {t.button_hover}; }}"
        )
        for b in (
            self.back_btn,
            self.fwd_btn,
            self.home_btn,
            self.zoom_out_btn,
            self.zoom_in_btn,
            self.popout_btn,
            self.open_ext_btn,
        ):
            b.setStyleSheet(btn_style)

    def _active_handbook_dir(self) -> Optional[Path]:
        """Return the handbook directory matching the current UI language."""
        if self._handbook_dir is None:
            self._handbook_dir = locate_handbook_dir()
        return locate_handbook_lang_dir(self._handbook_dir, self.i18n.lang)

    def _relative_page_from_url(self, url: Optional[QUrl]) -> str:
        """Extract a relative handbook page path from a local ``QUrl``."""
        if url is None or not url.isValid() or not url.isLocalFile():
            return self._current_rel_path or "index.html"
        if self._handbook_dir is None:
            return self._current_rel_path or "index.html"
        try:
            p = Path(url.toLocalFile()).resolve()
            base = self._handbook_dir.resolve()
            cs_base = (base / "cs").resolve()
            if p.is_relative_to(cs_base):
                return p.relative_to(cs_base).as_posix()
            if p.is_relative_to(base):
                return p.relative_to(base).as_posix()
        except Exception:
            pass
        return self._current_rel_path or "index.html"

    def ensure_loaded(self) -> None:
        """Load the initial handbook page on first display if not yet loaded."""
        if self._loaded:
            return
        self._loaded = True
        self.go_home()

    def showEvent(self, event) -> None:
        """Trigger lazy initial page loading when the widget becomes visible."""
        self.ensure_loaded()
        super().showEvent(event)

    def current_url(self) -> Optional[QUrl]:
        """Return the currently displayed ``QUrl`` from the active backend."""
        if self.web_view is not None:
            u = self.web_view.url()
            if u.isValid() and not u.isEmpty():
                return u
        elif self.text_browser is not None:
            u = self.text_browser.source()
            if u.isValid() and not u.isEmpty():
                return u
        return self._current_url

    def load_url(self, url: QUrl) -> None:
        """
        Load the given ``QUrl`` into the active web or text browser backend.

        :param url: Target local file URL to display.
        """
        self._loaded = True
        self._current_url = url
        self._current_rel_path = self._relative_page_from_url(url)
        if self.web_view is not None:
            self.web_view.load(url)
            self.web_view.update()
        elif self.text_browser is not None:
            lang_dir = self._active_handbook_dir()
            if lang_dir is not None:
                self.text_browser.setSearchPaths([str(lang_dir), str(Path(url.toLocalFile()).parent)])
            self.text_browser.setSource(url)

    def navigate_to(self, rel_path: str = "index.html") -> None:
        """
        Navigate to a relative page inside the active language handbook directory.

        :param rel_path: Relative HTML path inside the handbook root.
        """
        self._loaded = True
        self._current_rel_path = rel_path
        lang_dir = self._active_handbook_dir()
        if lang_dir is None:
            msg = "<h3>Handbook not found</h3><p>Could not locate docs/handbook/index.html.</p>"
            if self.web_view is not None:
                self.web_view.setHtml(msg)
            elif self.text_browser is not None:
                self.text_browser.setHtml(msg)
            return

        target = lang_dir / rel_path
        if not target.is_file():
            target = lang_dir / "index.html"
            self._current_rel_path = "index.html"
        self.load_url(QUrl.fromLocalFile(str(target.resolve())))

    def go_home(self) -> None:
        """Navigate to the handbook home page and reset the section selector."""
        self.navigate_to("index.html")
        if self.section_combo.count() > 0:
            self.section_combo.setCurrentIndex(0)

    def _on_language_changed(self, _lang: str) -> None:
        """Retranslate UI controls and reload the current page in the new language."""
        self.retranslate_ui()
        if self._loaded:
            rel = self._relative_page_from_url(self.current_url())
            self.navigate_to(rel)

    def _on_section_selected(self, index: int) -> None:
        """Navigate to the handbook section chosen in the section combo box."""
        rel = self.section_combo.itemData(index)
        if rel:
            self.navigate_to(str(rel))

    def _go_back(self) -> None:
        """Navigate backward in the viewer's browsing history."""
        if self.web_view is not None:
            self.web_view.back()
        elif self.text_browser is not None:
            self.text_browser.backward()

    def _go_forward(self) -> None:
        """Navigate forward in the viewer's browsing history."""
        if self.web_view is not None:
            self.web_view.forward()
        elif self.text_browser is not None:
            self.text_browser.forward()

    def _adjust_zoom(self, delta: float) -> None:
        """Adjust the page zoom factor or text size by ``delta``."""
        self._zoom = max(0.6, min(2.0, self._zoom + delta))
        if self.web_view is not None:
            self.web_view.setZoomFactor(self._zoom)
        elif self.text_browser is not None:
            if delta > 0:
                self.text_browser.zoomIn(1)
            else:
                self.text_browser.zoomOut(1)

    def _open_external(self) -> None:
        """Open the current handbook page in the system's default web browser."""
        url = self.current_url()
        if url is not None and url.isValid():
            QDesktopServices.openUrl(url)
        else:
            lang_dir = self._active_handbook_dir()
            if lang_dir is not None:
                QDesktopServices.openUrl(QUrl.fromLocalFile(str(lang_dir / "index.html")))

    def retranslate_ui(self) -> None:
        """Update toolbar button labels, tooltips, and section titles for the active language."""
        self.back_studio_btn.setText(self.i18n.t("handbook_back"))
        self.lang_btn.setText(self.i18n.t("lang_toggle"))
        self.popout_btn.setText(
            self.i18n.t("handbook_dock_back") if self._is_popout else self.i18n.t("handbook_popout")
        )
        self.open_ext_btn.setToolTip(self.i18n.t("handbook_open_ext"))
        cur = self.section_combo.currentData()
        li = self.i18n.lang_idx
        self.section_combo.blockSignals(True)
        self.section_combo.clear()
        for rel, labels in _HANDBOOK_SECTIONS:
            self.section_combo.addItem(labels[li], rel)
        idx = self.section_combo.findData(cur)
        self.section_combo.setCurrentIndex(max(0, idx))
        self.section_combo.blockSignals(False)


class HandbookWindow(QWidget):
    """Freely-resizable, maximizable top-level in-app window for side-by-side Handbook reading."""

    embedRequested = Signal(object)
    closed = Signal()

    def __init__(self, i18n: I18nManager, icon_path: Optional[str] = None):
        """Initialize the standalone resizable handbook window."""
        super().__init__(None, Qt.WindowType.Window)
        self.i18n = i18n
        if icon_path:
            self.setWindowIcon(QIcon(icon_path))
        self.setMinimumSize(420, 320)
        self.resize(1120, 780)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        self.viewer = HandbookViewer(i18n, self, is_popout=True)
        self.viewer.popoutRequested.connect(self._embed_back)
        lay.addWidget(self.viewer)

        self.apply_theme()
        self.i18n.languageChanged.connect(lambda _l: self.retranslate_ui())
        self.retranslate_ui()

    def apply_theme(self, theme: Optional[str] = None) -> None:
        """Apply ``theme`` colors to the standalone handbook window and viewer."""
        self.viewer.apply_theme(theme)

    def _embed_back(self) -> None:
        """Hide the popout window and emit ``embedRequested`` with the current URL."""
        url = self.viewer.current_url()
        self.hide()
        self.embedRequested.emit(url)

    def retranslate_ui(self) -> None:
        """Update the window title for the active UI language."""
        self.setWindowTitle(self.i18n.t("handbook_title"))

    def closeEvent(self, event) -> None:
        """Emit ``closed`` when the handbook window is closed."""
        self.closed.emit()
        super().closeEvent(event)
