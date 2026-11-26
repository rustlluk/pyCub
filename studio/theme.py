"""
Consistent Light & Dark Studio Themes and Palettes for pyCub Standalone.

Forces Qt's cross-platform `Fusion` style with explicit light or dark `QPalette` tokens,
widget stylesheets, and bundled PNG arrow indicators for `QSpinBox` and `QComboBox` so
OS-level dark/light overrides never clash with Studio widgets.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from PySide6.QtGui import QColor, QPalette
from PySide6.QtWidgets import QApplication

from studio.resource_paths import locate_studio_asset


@dataclass(frozen=True)
class ThemeTokens:
    """Color and icon tokens shared across Studio's Qt chrome, cards, panels, and buttons."""

    name: str
    window_bg: str
    surface_bg: str
    surface_alt_bg: str
    surface_tint_bg: str
    text_primary: str
    text_secondary: str
    text_muted: str
    heading_accent: str
    section_accent: str
    indigo_accent: str
    purple_accent: str
    border: str
    border_subtle: str
    card_border: str
    viewer_border: str
    button_bg: str
    button_hover: str
    button_border: str
    button_disabled_bg: str
    button_disabled_fg: str
    spin_btn_bg: str
    spin_btn_hover: str
    tab_bg: str
    tab_fg: str
    tab_selected_bg: str
    tab_selected_fg: str
    selection_bg: str
    selection_fg: str
    highlight: str
    banner_warn_bg: str
    banner_warn_border: str
    banner_warn_fg: str
    speed_btn_bg: str
    speed_btn_fg: str
    speed_btn_border: str
    speed_checked_bg: str
    speed_checked_border: str
    palette_btn_bg: str
    palette_btn_fg: str
    palette_btn_border: str
    palette_btn_hover: str
    header_bg: str
    header_border: str
    header_btn_bg: str
    header_btn_hover: str
    header_btn_border: str
    console_bg: str
    console_fg: str
    console_border: str
    disabled_green_bg: str
    disabled_green_fg: str
    disabled_red_bg: str
    disabled_red_fg: str
    scrollbar_bg: str
    scrollbar_handle: str
    scrollbar_handle_hover: str
    focus_border: str
    tooltip_bg: str
    tooltip_fg: str
    arrow_variant: str


LIGHT_THEME = ThemeTokens(
    name="light",
    window_bg="#f1f5f9",
    surface_bg="#ffffff",
    surface_alt_bg="#f8fafc",
    surface_tint_bg="#eff6ff",
    text_primary="#0f172a",
    text_secondary="#475569",
    text_muted="#64748b",
    heading_accent="#1e3a8a",
    section_accent="#1d4ed8",
    indigo_accent="#4338ca",
    purple_accent="#4c1d95",
    border="#cbd5e1",
    border_subtle="#e2e8f0",
    card_border="#dbeafe",
    viewer_border="#bfdbfe",
    button_bg="#e2e8f0",
    button_hover="#cbd5e1",
    button_border="#cbd5e1",
    button_disabled_bg="#f1f5f9",
    button_disabled_fg="#94a3b8",
    spin_btn_bg="#f1f5f9",
    spin_btn_hover="#e2e8f0",
    tab_bg="#e2e8f0",
    tab_fg="#334155",
    tab_selected_bg="#ffffff",
    tab_selected_fg="#0f172a",
    selection_bg="#dbeafe",
    selection_fg="#1e3a8a",
    highlight="#2563eb",
    banner_warn_bg="#fef9c3",
    banner_warn_border="#facc15",
    banner_warn_fg="#713f12",
    speed_btn_bg="#ede9fe",
    speed_btn_fg="#4c1d95",
    speed_btn_border="#ddd6fe",
    speed_checked_bg="#6d28d9",
    speed_checked_border="#facc15",
    palette_btn_bg="#eff6ff",
    palette_btn_fg="#1e3a8a",
    palette_btn_border="#bfdbfe",
    palette_btn_hover="#dbeafe",
    header_bg="#0f172a",
    header_border="#1e293b",
    header_btn_bg="#1e293b",
    header_btn_hover="#334155",
    header_btn_border="#334155",
    console_bg="#0f172a",
    console_fg="#e2e8f0",
    console_border="#1e293b",
    disabled_green_bg="#bbf7d0",
    disabled_green_fg="#166534",
    disabled_red_bg="#fecaca",
    disabled_red_fg="#991b1b",
    scrollbar_bg="#f1f5f9",
    scrollbar_handle="#cbd5e1",
    scrollbar_handle_hover="#94a3b8",
    focus_border="#3b82f6",
    tooltip_bg="#0f172a",
    tooltip_fg="#f8fafc",
    arrow_variant="light",
)

DARK_THEME = ThemeTokens(
    name="dark",
    window_bg="#0f172a",
    surface_bg="#1e293b",
    surface_alt_bg="#172033",
    surface_tint_bg="#131c31",
    text_primary="#f1f5f9",
    text_secondary="#cbd5e1",
    text_muted="#94a3b8",
    heading_accent="#93c5fd",
    section_accent="#60a5fa",
    indigo_accent="#a5b4fc",
    purple_accent="#c4b5fd",
    border="#334155",
    border_subtle="#263244",
    card_border="#334155",
    viewer_border="#334155",
    button_bg="#334155",
    button_hover="#475569",
    button_border="#475569",
    button_disabled_bg="#1e293b",
    button_disabled_fg="#64748b",
    spin_btn_bg="#283548",
    spin_btn_hover="#3b4c66",
    tab_bg="#172033",
    tab_fg="#94a3b8",
    tab_selected_bg="#1e293b",
    tab_selected_fg="#f8fafc",
    selection_bg="#1d4ed8",
    selection_fg="#ffffff",
    highlight="#3b82f6",
    banner_warn_bg="#422006",
    banner_warn_border="#ca8a04",
    banner_warn_fg="#fef08a",
    speed_btn_bg="#2e1065",
    speed_btn_fg="#ddd6fe",
    speed_btn_border="#5b21b6",
    speed_checked_bg="#6d28d9",
    speed_checked_border="#facc15",
    palette_btn_bg="#17253b",
    palette_btn_fg="#bfdbfe",
    palette_btn_border="#2d4466",
    palette_btn_hover="#223554",
    header_bg="#090e1a",
    header_border="#1e293b",
    header_btn_bg="#1e293b",
    header_btn_hover="#334155",
    header_btn_border="#334155",
    console_bg="#090e1a",
    console_fg="#e2e8f0",
    console_border="#334155",
    disabled_green_bg="#14532d",
    disabled_green_fg="#86efac",
    disabled_red_bg="#7f1d1d",
    disabled_red_fg="#fca5a5",
    scrollbar_bg="#0f172a",
    scrollbar_handle="#475569",
    scrollbar_handle_hover="#64748b",
    focus_border="#60a5fa",
    tooltip_bg="#1e293b",
    tooltip_fg="#f8fafc",
    arrow_variant="dark",
)

_ACTIVE_THEME = "light"


def normalize_theme(theme: Optional[str]) -> str:
    """Normalize a theme identifier to ``'light'`` or ``'dark'``."""
    return "dark" if str(theme or "").strip().lower() == "dark" else "light"


def get_active_theme() -> str:
    """Return the currently active Studio theme identifier (``'light'`` or ``'dark'``)."""
    return _ACTIVE_THEME


def get_theme_tokens(theme: Optional[str] = None) -> ThemeTokens:
    """Return the :class:`ThemeTokens` palette for ``theme`` (or the active theme)."""
    norm = normalize_theme(theme if theme is not None else _ACTIVE_THEME)
    return DARK_THEME if norm == "dark" else LIGHT_THEME


def arrow_icon_url(direction: str, variant: str) -> str:
    """Return a QSS ``url(...)`` reference for ``spin_{direction}_{variant}.png``, or empty string."""
    d = "up" if direction == "up" else "down"
    v = "dark" if variant == "dark" else "light"
    p = locate_studio_asset("icons", f"spin_{d}_{v}.png")
    if p is None:
        return ""
    return f'url("{p.as_posix()}")'


def check_icon_url() -> str:
    """Return a QSS ``url(...)`` reference for ``check_white.png``, or empty string."""
    p = locate_studio_asset("icons", "check_white.png")
    if p is None:
        return ""
    return f'url("{p.as_posix()}")'


def build_base_qss(theme: Optional[str] = None) -> str:
    """Build the application-wide QSS stylesheet for ``theme``."""
    t = get_theme_tokens(theme)
    up_url = arrow_icon_url("up", t.arrow_variant)
    down_url = arrow_icon_url("down", t.arrow_variant)
    chk_url = check_icon_url()
    up_img_rule = f"image: {up_url};" if up_url else ""
    down_img_rule = f"image: {down_url};" if down_url else ""
    chk_img_rule = f"image: {chk_url};" if chk_url else ""

    return f"""
QMainWindow, QDialog {{
    background-color: {t.window_bg};
    color: {t.text_primary};
}}
QWidget {{
    color: {t.text_primary};
}}
QLabel {{
    background-color: transparent;
    color: {t.text_primary};
}}
QScrollArea {{
    background-color: transparent;
    border: none;
}}
QComboBox, QSpinBox, QLineEdit {{
    background-color: {t.surface_bg};
    color: {t.text_primary};
    border: 1px solid {t.border};
    border-radius: 6px;
    padding: 4px 8px;
    min-height: 22px;
}}
QComboBox {{
    padding-right: 26px;
}}
QSpinBox {{
    padding-right: 24px;
}}
QComboBox:focus, QSpinBox:focus, QLineEdit:focus {{
    border: 1px solid {t.focus_border};
}}
QComboBox::drop-down {{
    subcontrol-origin: border;
    subcontrol-position: top right;
    width: 22px;
    border-left: 1px solid {t.border};
    border-top-right-radius: 6px;
    border-bottom-right-radius: 6px;
    background-color: {t.spin_btn_bg};
}}
QComboBox::drop-down:hover {{
    background-color: {t.spin_btn_hover};
}}
QComboBox::down-arrow {{
    {down_img_rule}
    width: 11px;
    height: 11px;
}}
QSpinBox::up-button {{
    subcontrol-origin: border;
    subcontrol-position: top right;
    width: 20px;
    border-left: 1px solid {t.border};
    border-bottom: 1px solid {t.border};
    border-top-right-radius: 6px;
    background-color: {t.spin_btn_bg};
}}
QSpinBox::up-button:hover {{
    background-color: {t.spin_btn_hover};
}}
QSpinBox::down-button {{
    subcontrol-origin: border;
    subcontrol-position: bottom right;
    width: 20px;
    border-left: 1px solid {t.border};
    border-bottom-right-radius: 6px;
    background-color: {t.spin_btn_bg};
}}
QSpinBox::down-button:hover {{
    background-color: {t.spin_btn_hover};
}}
QSpinBox::up-arrow {{
    {up_img_rule}
    width: 10px;
    height: 10px;
}}
QSpinBox::down-arrow {{
    {down_img_rule}
    width: 10px;
    height: 10px;
}}
QComboBox QAbstractItemView {{
    background-color: {t.surface_bg};
    color: {t.text_primary};
    border: 1px solid {t.border};
    selection-background-color: {t.selection_bg};
    selection-color: {t.selection_fg};
}}
QCheckBox {{
    color: {t.text_primary};
    spacing: 6px;
}}
QCheckBox::indicator {{
    width: 16px;
    height: 16px;
    border-radius: 4px;
    border: 1px solid {t.border};
    background-color: {t.surface_alt_bg};
}}
QCheckBox::indicator:hover {{
    border-color: {t.focus_border};
}}
QCheckBox::indicator:checked {{
    background-color: {t.highlight};
    border-color: {t.highlight};
    {chk_img_rule}
}}
QPushButton {{
    background-color: {t.button_bg};
    color: {t.text_primary};
    border: 1px solid {t.button_border};
    border-radius: 7px;
    padding: 5px 10px;
    font-weight: 600;
}}
QPushButton:hover {{
    background-color: {t.button_hover};
}}
QPushButton:disabled {{
    background-color: {t.button_disabled_bg};
    color: {t.button_disabled_fg};
    border-color: {t.border_subtle};
}}
QTableWidget {{
    background-color: {t.surface_bg};
    alternate-background-color: {t.surface_alt_bg};
    color: {t.text_primary};
    gridline-color: {t.border_subtle};
    border: 1px solid {t.border};
    border-radius: 8px;
}}
QHeaderView::section {{
    background-color: {t.window_bg};
    color: {t.text_primary};
    font-weight: 700;
    padding: 6px 8px;
    border: none;
    border-bottom: 1px solid {t.border};
    border-right: 1px solid {t.border_subtle};
}}
QTabWidget::pane {{
    border: 1px solid {t.border};
    background-color: {t.surface_bg};
    border-radius: 6px;
}}
QTabBar::tab {{
    background-color: {t.tab_bg};
    color: {t.tab_fg};
    padding: 6px 12px;
    border-top-left-radius: 6px;
    border-top-right-radius: 6px;
    margin-right: 2px;
}}
QTabBar::tab:selected {{
    background-color: {t.tab_selected_bg};
    color: {t.tab_selected_fg};
    font-weight: 700;
}}
QListWidget, QTextBrowser, QPlainTextEdit {{
    background-color: {t.surface_bg};
    color: {t.text_primary};
    border: 1px solid {t.border};
    border-radius: 6px;
}}
QMenu {{
    background-color: {t.surface_bg};
    color: {t.text_primary};
    border: 1px solid {t.border};
    padding: 4px;
}}
QMenu::item {{
    padding: 5px 16px;
    border-radius: 4px;
}}
QMenu::item:selected {{
    background-color: {t.selection_bg};
    color: {t.selection_fg};
}}
QSplitter::handle {{
    background-color: {t.border};
}}
QScrollBar:vertical {{
    background: {t.scrollbar_bg};
    width: 10px;
    margin: 0px;
}}
QScrollBar::handle:vertical {{
    background: {t.scrollbar_handle};
    min-height: 24px;
    border-radius: 5px;
}}
QScrollBar::handle:vertical:hover {{
    background: {t.scrollbar_handle_hover};
}}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
    height: 0px;
}}
QScrollBar:horizontal {{
    background: {t.scrollbar_bg};
    height: 10px;
    margin: 0px;
}}
QScrollBar::handle:horizontal {{
    background: {t.scrollbar_handle};
    min-width: 24px;
    border-radius: 5px;
}}
QScrollBar::handle:horizontal:hover {{
    background: {t.scrollbar_handle_hover};
}}
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {{
    width: 0px;
}}
"""


def apply_studio_theme(app: QApplication, theme: str = "light") -> str:
    """Apply Fusion Light or Dark palette + crisp base widget stylesheet across all OS themes."""
    global _ACTIVE_THEME
    norm = normalize_theme(theme)
    _ACTIVE_THEME = norm
    t = get_theme_tokens(norm)

    app.setStyle("Fusion")

    pal = QPalette()
    pal.setColor(QPalette.ColorRole.Window, QColor(t.window_bg))
    pal.setColor(QPalette.ColorRole.WindowText, QColor(t.text_primary))
    pal.setColor(QPalette.ColorRole.Base, QColor(t.surface_bg))
    pal.setColor(QPalette.ColorRole.AlternateBase, QColor(t.surface_alt_bg))
    pal.setColor(QPalette.ColorRole.ToolTipBase, QColor(t.tooltip_bg))
    pal.setColor(QPalette.ColorRole.ToolTipText, QColor(t.tooltip_fg))
    pal.setColor(QPalette.ColorRole.Text, QColor(t.text_primary))
    pal.setColor(QPalette.ColorRole.Button, QColor(t.button_bg))
    pal.setColor(QPalette.ColorRole.ButtonText, QColor(t.text_primary))
    pal.setColor(QPalette.ColorRole.BrightText, QColor("#ffffff"))
    pal.setColor(QPalette.ColorRole.Highlight, QColor(t.highlight))
    pal.setColor(QPalette.ColorRole.HighlightedText, QColor("#ffffff"))
    pal.setColor(QPalette.ColorRole.PlaceholderText, QColor(t.text_muted))

    disabled_text = QColor(t.button_disabled_fg)
    pal.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.WindowText, disabled_text)
    pal.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Text, disabled_text)
    pal.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ButtonText, disabled_text)

    app.setPalette(pal)
    app.setStyleSheet(build_base_qss(norm))
    return norm
