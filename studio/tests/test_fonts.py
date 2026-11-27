"""Tests for bundled Studio fonts, glyph coverage, and Linux .deb package naming."""
from __future__ import annotations

import os
import struct
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

REPO_ROOT = Path(__file__).resolve().parents[2]


def _read_ttf_codepoints(font_path: Path) -> set[int]:
    """Parse TrueType/OpenType ``cmap`` format 4 and 12 tables into Unicode codepoints."""
    data = font_path.read_bytes()
    num_tables = struct.unpack(">H", data[4:6])[0]
    tables: dict[bytes, tuple[int, int]] = {}
    for i in range(num_tables):
        off = 12 + i * 16
        tag, _, toff, tlen = struct.unpack(">4sIII", data[off : off + 16])
        tables[tag] = (toff, tlen)
    if b"cmap" not in tables:
        return set()
    coff, _ = tables[b"cmap"]
    num_sub = struct.unpack(">H", data[coff + 2 : coff + 4])[0]
    cps: set[int] = set()
    for i in range(num_sub):
        _, _, sub_off = struct.unpack(">HHI", data[coff + 4 + i * 8 : coff + 12 + i * 8])
        soff = coff + sub_off
        fmt = struct.unpack(">H", data[soff : soff + 2])[0]
        if fmt == 4:
            seg_count = struct.unpack(">H", data[soff + 6 : soff + 8])[0] // 2
            end_codes = struct.unpack(f">{seg_count}H", data[soff + 14 : soff + 14 + 2 * seg_count])
            start_off = soff + 16 + 2 * seg_count
            start_codes = struct.unpack(f">{seg_count}H", data[start_off : start_off + 2 * seg_count])
            for start, end in zip(start_codes, end_codes):
                if start != 0xFFFF:
                    cps.update(range(start, end + 1))
        elif fmt == 12:
            num_groups = struct.unpack(">I", data[soff + 12 : soff + 16])[0]
            for g in range(num_groups):
                goff = soff + 16 + g * 12
                start, end, _ = struct.unpack(">III", data[goff : goff + 12])
                cps.update(range(start, end + 1))
    return cps


class BundledFontsTests(unittest.TestCase):
    """Verify bundled UI/emoji fonts, glyph coverage, Blockly CSS, and Linux packaging."""

    def test_bundled_fonts_exist_and_cover_all_studio_icons(self):
        """Ensure bundled fonts exist and cover every UI icon/emoji character used in Studio."""
        from studio.fonts import iter_bundled_font_files, locate_studio_fonts_dir

        fonts_dir = locate_studio_fonts_dir()
        self.assertIsNotNone(fonts_dir)
        font_names = {p.name for p in iter_bundled_font_files(fonts_dir)}
        self.assertTrue(
            {"NotoColorEmoji.ttf", "DejaVuSans.ttf", "DejaVuSans-Bold.ttf", "DejaVuSansMono.ttf"}.issubset(font_names)
        )

        covered = (
            _read_ttf_codepoints(fonts_dir / "NotoColorEmoji.ttf")
            | _read_ttf_codepoints(fonts_dir / "DejaVuSans.ttf")
        )

        targets = [
            REPO_ROOT / "studio" / "i18n.py",
            REPO_ROOT / "studio" / "kid_ui" / "kid_ui.py",
            REPO_ROOT / "studio" / "assets" / "blockly" / "pycub_blocks.js",
        ]
        missing: list[str] = []
        for path in targets:
            for ch in path.read_text(encoding="utf-8"):
                cp = ord(ch)
                if cp > 0x2000 and cp != 0xFE0F and cp not in covered:
                    missing.append(f"{path.name}: U+{cp:04X} ({ch})")
        self.assertEqual(missing, [], msg=f"Uncovered Studio characters: {missing}")

    def test_locate_studio_fonts_dir_supports_frozen_meipass_and_internal(self):
        """Verify ``locate_studio_fonts_dir`` resolves ``_MEIPASS`` and ``_internal`` layouts."""
        from studio import fonts

        with tempfile.TemporaryDirectory() as tmp_dir:
            tmp = Path(tmp_dir)
            fake_module = tmp / "site-packages" / "studio" / "fonts.py"
            fake_module.parent.mkdir(parents=True, exist_ok=True)
            fake_module.touch()

            meipass_fonts = tmp / "meipass" / "studio" / "assets" / "fonts"
            meipass_fonts.mkdir(parents=True, exist_ok=True)
            (meipass_fonts / "NotoColorEmoji.ttf").write_bytes(b"fake")

            with (
                patch.object(fonts, "__file__", str(fake_module)),
                patch.object(sys, "_MEIPASS", str(tmp / "meipass"), create=True),
            ):
                self.assertEqual(fonts.locate_studio_fonts_dir(), meipass_fonts)

            internal_fonts = tmp / "dist" / "_internal" / "studio" / "assets" / "fonts"
            internal_fonts.mkdir(parents=True, exist_ok=True)
            (internal_fonts / "DejaVuSans.ttf").write_bytes(b"fake")
            fake_exe = tmp / "dist" / "pyCub"
            fake_exe.touch()

            with (
                patch.object(fonts, "__file__", str(fake_module)),
                patch.object(sys, "_MEIPASS", None, create=True),
                patch.object(sys, "frozen", True, create=True),
                patch.object(sys, "executable", str(fake_exe)),
            ):
                self.assertEqual(fonts.locate_studio_fonts_dir(), internal_fonts)

    def test_register_fontconfig_app_fonts_on_linux(self):
        """Verify ``register_fontconfig_app_fonts`` invokes ``FcConfigAppFontAddDir`` on Linux."""
        from studio import fonts

        fake_fc = MagicMock()
        fake_fc.FcInit.return_value = 1
        fake_fc.FcConfigAppFontAddDir.return_value = 1

        with tempfile.TemporaryDirectory() as tmp_dir:
            fonts_dir = Path(tmp_dir)
            (fonts_dir / "NotoColorEmoji.ttf").write_bytes(b"fake")
            with (
                patch.object(fonts.sys, "platform", "linux"),
                patch.object(fonts, "_FONTCONFIG_REGISTERED_DIRS", set()),
                patch.object(fonts.ctypes.util, "find_library", return_value="libfontconfig.so.1"),
                patch.object(fonts.ctypes, "CDLL", return_value=fake_fc),
            ):
                ok = fonts.register_fontconfig_app_fonts(fonts_dir)
                self.assertTrue(ok)
                fake_fc.FcInit.assert_called_once()
                fake_fc.FcConfigAppFontAddDir.assert_called_once_with(
                    None, os.fsencode(os.path.abspath(str(fonts_dir)))
                )

    def test_configure_application_font_fallbacks_wires_qt_fallbacks_on_linux(self):
        """Verify ``configure_application_font_fallbacks`` registers bundled fonts and Qt fallbacks."""
        from studio import fonts

        class _FakeFont:
            def __init__(self):
                self._families = ["Ubuntu"]

            def family(self):
                return self._families[0]

            def families(self):
                return list(self._families)

            def setFamilies(self, fams):
                self._families = list(fams)

        fake_qfont = MagicMock()
        fake_qfont_db = MagicMock()
        family_by_name = {
            "DejaVuSans.ttf": ["DejaVu Sans"],
            "DejaVuSans-Bold.ttf": ["DejaVu Sans"],
            "DejaVuSansMono.ttf": ["DejaVu Sans Mono"],
            "NotoColorEmoji.ttf": ["Noto Color Emoji"],
        }
        added_ids: dict[int, list[str]] = {}

        def _add_font(path_str: str) -> int:
            idx = len(added_ids)
            added_ids[idx] = family_by_name.get(Path(path_str).name, ["Unknown"])
            return idx

        fake_qfont_db.addApplicationFont.side_effect = _add_font
        fake_qfont_db.applicationFontFamilies.side_effect = lambda idx: added_ids[idx]

        fake_app_font = _FakeFont()
        fake_app = MagicMock()
        fake_app.font.return_value = fake_app_font

        fake_qtgui = types.ModuleType("PySide6.QtGui")
        fake_qtgui.QFont = fake_qfont
        fake_qtgui.QFontDatabase = fake_qfont_db
        fake_qtwidgets = types.ModuleType("PySide6.QtWidgets")
        fake_qtwidgets.QApplication = MagicMock()
        fake_pyside = types.ModuleType("PySide6")
        fake_pyside.QtGui = fake_qtgui
        fake_pyside.QtWidgets = fake_qtwidgets

        with (
            patch.object(fonts.sys, "platform", "linux"),
            patch.object(fonts, "_QT_REGISTERED_FONTS", {}),
            patch.object(fonts, "register_fontconfig_app_fonts", return_value=True),
            patch.dict(
                sys.modules,
                {"PySide6": fake_pyside, "PySide6.QtGui": fake_qtgui, "PySide6.QtWidgets": fake_qtwidgets},
            ),
        ):
            loaded = fonts.configure_application_font_fallbacks(fake_app)

        self.assertIn("DejaVu Sans", loaded)
        self.assertIn("Noto Color Emoji", loaded)
        self.assertIn("DejaVu Sans Mono", loaded)
        self.assertEqual(fake_app_font.families()[:3], ["Ubuntu", "DejaVu Sans", "Noto Color Emoji"])
        fake_app.setFont.assert_called_once_with(fake_app_font)
        fake_qfont.insertSubstitutions.assert_any_call("Ubuntu", ["DejaVu Sans", "Noto Color Emoji", "DejaVu Sans Mono"])
        fake_qfont.insertSubstitutions.assert_any_call(
            "monospace", ["DejaVu Sans Mono", "Noto Color Emoji", "DejaVu Sans"]
        )

    def test_blockly_html_references_bundled_fonts(self):
        """Ensure Blockly workspace HTML embeds ``@font-face`` rules for bundled fonts."""
        html = (REPO_ROOT / "studio" / "assets" / "blockly" / "index.html").read_text(encoding="utf-8")
        self.assertIn("../fonts/NotoColorEmoji.ttf", html)
        self.assertIn("../fonts/DejaVuSans.ttf", html)
        self.assertIn("PyCub Bundled Emoji", html)

    def test_linux_deb_uses_pycub_studio_paths(self):
        """Ensure Linux .deb packaging installs to ``/opt/pycub-studio`` and ``/usr/bin/pycub-studio``."""
        script = (REPO_ROOT / "packaging" / "linux" / "build_linux.sh").read_text(encoding="utf-8")
        self.assertIn("/opt/pycub-studio", script)
        self.assertIn("/usr/bin/pycub-studio", script)
        self.assertIn("pycub-studio.desktop", script)
        self.assertIn("pycub-studio.png", script)
        self.assertNotIn('/opt/pycub"', script)
        self.assertNotIn('/usr/bin/pycub"', script)


if __name__ == "__main__":
    unittest.main()
