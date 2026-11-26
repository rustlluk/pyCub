"""
Centralized Čeština / English (cs / en) localization manager for pyCub Standalone.
"""
from __future__ import annotations

from typing import Dict
from PySide6.QtCore import QObject, Signal


STRINGS: Dict[str, Dict[str, str]] = {
    "app_title": {"cs": "pyCub Studio – Simulátor humanoidního robota iCub", "en": "pyCub Studio – iCub Humanoid Simulator"},
    "home_btn": {"cs": "🏠 Domů", "en": "🏠 Home"},
    "handbook": {"cs": "📖 Příručka", "en": "📖 Handbook"},
    "handbook_title": {"cs": "Příručka a návody pyCub", "en": "pyCub Handbook & Handouts"},
    "handbook_back": {"cs": "← Zpět do Studia", "en": "← Back to Studio"},
    "handbook_popout": {"cs": "❐ Samostatné okno", "en": "❐ Separate Window"},
    "handbook_dock_back": {"cs": "↩ Do hlavního okna", "en": "↩ Embed in Studio"},
    "handbook_open_ext": {"cs": "Otevřít v prohlížeči", "en": "Open in browser"},
    "show_definition": {"cs": "Zobrazit definici", "en": "Show definition"},
    "show_parameters": {"cs": "Zobrazit parametry", "en": "Show parameters"},
    "no_definition": {"cs": "Definice není dostupná.", "en": "No definition available."},
    "workspace_btn": {"cs": "📂 Složka uživatele", "en": "📂 User Workspace"},
    "packages_btn": {"cs": "📦 Balíčky / Python", "en": "📦 Packages / Python"},
    "lang_toggle": {"cs": "🇬🇧 English", "en": "🇨🇿 Čeština"},
    "theme_to_dark": {"cs": "🌙 Tmavý", "en": "🌙 Dark"},
    "theme_to_light": {"cs": "☀️ Světlý", "en": "☀️ Light"},
    "launcher_heading": {"cs": "Vyber režim simulátoru pyCub", "en": "Choose a pyCub Simulator Mode"},
    "launcher_sub": {
        "cs": "Všechny desktopové režimy běží s rychlým vestavěným 3D zobrazením bez nutnosti prohlížeče.",
        "en": "All desktop modes run with fast native embedded 3D rendering—no browser required.",
    },
    # Mode titles & descriptions
    "mode_kid_native_title": {"cs": "🎮 Dětské ovládání (Tlačítka)", "en": "🎮 Kid GUI (Buttons)"},
    "mode_kid_native_desc": {
        "cs": "Velká barevná tlačítka pro okamžité ovládání rukou, hlavy a těla iCuba. Ideální pro nejmenší!",
        "en": "Large colorful buttons to move iCub's arms, head, and body right away. Great for beginners!",
    },
    "mode_kid_scratch_title": {"cs": "🧩 Dětské „Scratch“ (Bloky)", "en": "🧩 Kid \"Scratch\" (Blocks)"},
    "mode_kid_scratch_desc": {
        "cs": "Skládej barevné programovací bloky jako ve Scratchi – cykly, podmínky, senzory míčku a pohyby.",
        "en": "Snap visual blocks together like Scratch—loops, logic, ball sensors, and robot actions.",
    },
    "mode_kid_code_title": {"cs": "🐍 Dětské programování (Python)", "en": "🐍 Kid Programming (Python)"},
    "mode_kid_code_desc": {
        "cs": "Piš přehledný Python kód pomocí jednoduchých příkazů jako wave(), shoulder_up() a ball_touched().",
        "en": "Write clean beginner Python using simple commands like wave(), shoulder_up(), and ball_touched().",
    },
    "mode_normal_title": {"cs": "🛠️ Normální režim (Plné IDE & .py)", "en": "🛠️ Normal Mode (Full IDE & .py)"},
    "mode_normal_desc": {
        "cs": "Plný Python editor s pyCub API, kamerami očí (RGB/Depth), příklady, cvičeními a spouštěním vlastních .py souborů.",
        "en": "Full Python editor with pyCub API, eye cameras (RGB/Depth), examples, exercises, and external .py runner.",
    },
    "mode_web_title": {"cs": "🌐 Webový server pro třídu (Multi-instance)", "en": "🌐 Classroom Web Selector (Multi-Instance)"},
    "mode_web_desc": {
        "cs": "Spusť N nezávislých webových robotů a uvítací stránku s QR kódem pro tablety a počítače ve třídě.",
        "en": "Run N independent web instances plus an auto-connect landing page with QR code for classroom devices.",
    },
    "open_mode": {"cs": "Spustit režim →", "en": "Launch Mode →"},
    # Shared simulator controls
    "world_section": {"cs": "🌍 Svět", "en": "🌍 World"},
    "custom_world_btn": {"cs": "📄 Vlastní svět (.yaml)…", "en": "📄 Custom World (.yaml)…"},
    "restart_world": {"cs": "🔄 Restartovat svět", "en": "🔄 Restart World"},
    "stop_robot": {"cs": "🛑 Zastavit", "en": "🛑 Stop"},
    "reset_pose": {"cs": "🧍 Základní postoj", "en": "🧍 Reset Pose"},
    "reset_view": {"cs": "🎥 Vycentrovat kameru", "en": "🎥 Reset View"},
    "step_label": {"cs": "Krok (°):", "en": "Step (°):"},
    "speed_label": {"cs": "Rychlost:", "en": "Speed:"},
    "speed_slow": {"cs": "🐢 Pomalu", "en": "🐢 Slow"},
    "speed_medium": {"cs": "🚶 Středně", "en": "🚶 Medium"},
    "speed_fast": {"cs": "🐇 Rychle", "en": "🐇 Fast"},
    "status_loading": {"cs": "⏳ Načítám robota a svět…", "en": "⏳ Loading robot and world…"},
    "status_ready": {"cs": "✅ Připraveno", "en": "✅ Ready"},
    "status_busy": {"cs": "🤖 Robot se hýbe…", "en": "🤖 Robot is moving…"},
    "status_running_code": {"cs": "▶️ Program běží…", "en": "▶️ Program running…"},
    "status_stopped": {"cs": "⏹️ Zastaveno", "en": "⏹️ Stopped"},
    "ball_waiting": {
        "cs": "⚽ Míček čeká na stole – zkus do něj strčit!",
        "en": "⚽ Ball is on the table — try pushing it!",
    },
    "ball_stats": {
        "cs": "⚽ Zásahy: {hits}  |  Vzdálenost od stolu: {dist} cm  |  Po 3 s: {after} cm",
        "en": "⚽ Hits: {hits}  |  Distance from table: {dist} cm  |  After 3 s: {after} cm",
    },
    # Scratch & Kid Programming
    "run_program": {"cs": "▶️ Spustit program", "en": "▶️ Run Program"},
    "stop_program": {"cs": "⏹️ Zastavit", "en": "⏹️ Stop"},
    "save_blocks": {"cs": "💾 Uložit bloky", "en": "💾 Save Blocks"},
    "load_blocks": {"cs": "📂 Načíst bloky", "en": "📂 Load Blocks"},
    "clear_blocks": {"cs": "🗑️ Vyčistit", "en": "🗑️ Clear"},
    "to_kid_python": {"cs": "🐍 Otevřít jako Python v Dětském programování", "en": "🐍 Open as Python in Kid Programming"},
    "show_python_preview": {"cs": "👁️ Náhled Python kódu", "en": "👁️ Python Preview"},
    "open_file": {"cs": "📂 Otevřít .py", "en": "📂 Open .py"},
    "save_file": {"cs": "💾 Uložit .py", "en": "💾 Save .py"},
    "save_as_file": {"cs": "💾 Uložit jako…", "en": "💾 Save As…"},
    "new_file": {"cs": "📄 Nový", "en": "📄 New"},
    "command_palette": {"cs": "📚 Paleta příkazů (kliknutím vložíš)", "en": "📚 Command Palette (click to insert)"},
    "console_output": {"cs": "🖥️ Výstup a zprávy", "en": "🖥️ Output & Messages"},
    "clear_console": {"cs": "Vyčistit výstup", "en": "Clear Output"},
    # Normal IDE
    "run_script": {"cs": "▶️ Spustit skript (F5)", "en": "▶️ Run Script (F5)"},
    "run_external": {"cs": "🚀 Spustit externí .py…", "en": "🚀 Run External .py…"},
    "watch_external": {"cs": "Sledovat změny externího souboru", "en": "Auto-run external file on save"},
    "examples_menu": {"cs": "📘 Příklady && Cvičení", "en": "📘 Examples && Exercises"},
    "run_tester": {"cs": "🧪 Spustit tester cvičení", "en": "🧪 Run Exercise Tester"},
    "show_l_eye": {"cs": "👁️ Levé oko (l_eye)", "en": "👁️ Left Eye (l_eye)"},
    "show_r_eye": {"cs": "👁️ Pravé oko (r_eye)", "en": "👁️ Right Eye (r_eye)"},
    "show_axes": {"cs": "📐 Souřadnicové osy", "en": "📐 Coordinate Axes"},
    "save_rgb": {"cs": "📷 Uložit RGB", "en": "📷 Save RGB"},
    "save_depth": {"cs": "🌫️ Uložit hloubku", "en": "🌫️ Save Depth"},
    "idle_preview": {"cs": "🔄 Živý náhled robota", "en": "🔄 Idle Robot Preview"},
    # Web Selector Mode 5
    "web_cfg_heading": {"cs": "Nastavení třídního webového serveru", "en": "Classroom Web Server Configuration"},
    "web_instances_count": {"cs": "Počet instancí (N):", "en": "Number of instances (N):"},
    "web_landing_port": {"cs": "Port uvítací stránky:", "en": "Landing page port:"},
    "web_first_inst_port": {"cs": "První web port instance:", "en": "First instance web port:"},
    "web_first_stream_port": {"cs": "První video port (WebRTC):", "en": "First video port (WebRTC):"},
    "web_host_bind": {"cs": "Síťové rozhraní:", "en": "Network bind:"},
    "web_initial_world": {"cs": "Výchozí svět:", "en": "Initial world:"},
    "web_headless_chk": {
        "cs": "Bez obrazového streamu (--headless, pro rychlé testy)",
        "en": "Headless physics only (--headless, for fast testing)",
    },
    "web_start_server": {"cs": "🚀 Spustit server && instance", "en": "🚀 Start Server && Instances"},
    "web_stop_server": {"cs": "⏹️ Zastavit vše", "en": "⏹️ Stop All"},
    "web_open_browser": {"cs": "🌐 Otevřít uvítací stránku v prohlížeči", "en": "🌐 Open Landing Page in Browser"},
    "web_copy_url": {"cs": "📋 Kopírovat URL", "en": "📋 Copy URL"},
    # Packages / Python Environment Dialog
    "pkg_dialog_title": {"cs": "Python prostředí a vlastní balíčky", "en": "Python Environment & Extra Packages"},
    "pkg_runtime_lbl": {
        "cs": "Vestavěný Python: {ver}\nSložka extra balíčků (automaticky v sys.path): {extra_dir}",
        "en": "Bundled Python: {ver}\nExtra packages folder (always on sys.path): {extra_dir}",
    },
    "pkg_paths_heading": {
        "cs": "Další složky Python balíčků / site-packages z venv:",
        "en": "Extra Python package folders / venv site-packages:",
    },
    "pkg_add_folder": {"cs": "➕ Přidat složku…", "en": "➕ Add Folder…"},
    "pkg_add_venv": {"cs": "🐍 Přidat venv…", "en": "🐍 Add Venv…"},
    "pkg_remove": {"cs": "🗑️ Odebrat", "en": "🗑️ Remove"},
    "pkg_open_extra": {"cs": "📂 Otevřít extra_packages", "en": "📂 Open extra_packages"},
    "pkg_pip_heading": {
        "cs": "Nainstalovat balíček pomocí pip do složky extra_packages:",
        "en": "Install a PyPI package via pip into extra_packages:",
    },
    "pkg_pip_placeholder": {
        "cs": "např. scikit-learn pandas seaborn",
        "en": "e.g. scikit-learn pandas seaborn",
    },
    "pkg_pip_install": {"cs": "⬇️ Nainstalovat", "en": "⬇️ Install"},
    "pkg_interp_heading": {
        "cs": "Volitelný vlastní interpret Pythonu pro režim Normální (prázdné = vestavěný Python):",
        "en": "Optional custom Python executable for Normal Mode scripts (blank = Bundled Python):",
    },
    "pkg_browse_exe": {"cs": "Vybrat python…", "en": "Browse python…"},
    "pkg_use_bundled": {"cs": "Použít vestavěný", "en": "Use Bundled"},
    "pkg_save_close": {"cs": "Uložit a zavřít", "en": "Save & Close"},
}


class I18nManager(QObject):
    """Reactive language state ('cs' or 'en') shared across all modes."""

    languageChanged = Signal(str)

    def __init__(self, initial_lang: str = "en", parent: QObject | None = None):
        """Initialize the language manager with ``initial_lang`` ('cs' or 'en')."""
        super().__init__(parent)
        self._lang = initial_lang if initial_lang in ("cs", "en") else "en"

    @property
    def lang(self) -> str:
        """Return the currently active language code ('cs' or 'en')."""
        return self._lang

    @property
    def lang_idx(self) -> int:
        """Index into `(cs_label, en_label)` tuples used in `kid_ui.py`."""
        return 0 if self._lang == "cs" else 1

    def set_language(self, lang: str) -> None:
        """Switch the active language to ``lang`` ('cs' or 'en') and emit ``languageChanged`` if modified."""
        norm = "cs" if lang == "cs" else "en"
        if norm != self._lang:
            self._lang = norm
            self.languageChanged.emit(self._lang)

    def toggle_language(self) -> None:
        """Toggle the active language between English ('en') and Czech ('cs')."""
        self.set_language("cs" if self._lang == "en" else "en")

    def t(self, key: str, **kwargs: object) -> str:
        """Translate ``key`` into the active language, formatting with ``kwargs`` if provided.

        :param key: Lookup key in :data:`STRINGS`.
        :param kwargs: Optional format placeholders interpolated into the template.
        """
        entry = STRINGS.get(key)
        if not entry:
            return key
        template = entry.get(self._lang) or entry.get("en") or key
        return template.format(**kwargs) if kwargs else template
