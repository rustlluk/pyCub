import sys
from pathlib import Path

DOCS_DIR = Path(__file__).resolve().parents[1]
if str(DOCS_DIR) not in sys.path:
    sys.path.insert(0, str(DOCS_DIR))

from readme_sync import prepare_readme_include

project = "pyCub Studio"
author = "Lukas Rustler"
copyright = "2026, Lukas Rustler"
language = "cs"
root = Path(__file__).resolve().parents[2]
extensions = ["myst_parser", "autoapi.extension", "sphinx.ext.imgconverter"]
myst_heading_anchors = 3
autoapi_dirs = [str(root / "icub_pybullet")]
autoapi_ignore = ["*/build/*", "*/setup.py", "*/iCub/*"]
autoapi_options = ["members", "undoc-members", "show-module-summary"]
html_theme = "sphinx_rtd_theme"
html_title = "pyCub Studio (CZ)"
html_theme_options = {"style_nav_header_background": "#163e48", "navigation_depth": 3}
html_static_path = ["../source/static"]
html_css_files = ["handbook.css"]
latex_logo = "../static/images/icub.png"
latex_documents = [("index", "pycub.tex", "Příručka pyCub Studia", author, "manual")]
latex_elements = {
    "babel": r"\usepackage[shorthands=off,main=czech]{babel}",
    "preamble": "\n".join([
        r"\ifdefined\DeclareUnicodeCharacter",
        r"  \DeclareUnicodeCharacter{2212}{-}",
        r"  \DeclareUnicodeCharacter{1F3E0}{}",
        r"  \DeclareUnicodeCharacter{1F4C2}{}",
        r"  \DeclareUnicodeCharacter{1F4E6}{}",
        r"\fi",
    ]),
}
html_copy_source = False
html_show_sourcelink = False


def _sync_readme(app):
    """Generate a builder-appropriate README include before Sphinx reads sources."""
    prepare_readme_include(
        root,
        DOCS_DIR / "_build" / "readme_body.md",
        builder_format=app.builder.format,
    )


def setup(app):
    """Register Sphinx build hooks for the Czech handbook."""
    app.connect("builder-inited", _sync_readme)
