"""Publish portable English and Czech Sphinx handbooks (HTML and PDF) for Studio, GitHub Pages, and Jekyll."""
import argparse
import re
import shutil
import subprocess
import sys
from pathlib import Path
from tempfile import TemporaryDirectory

from sphinx.cmd.build import main

DOCS = Path(__file__).resolve().parent
# Adapted from the recovered html_to_jekyll.py. Jekyll skips underscore paths.
ASSET_PATH = re.compile(r"(?<![\w-])_(static/|images/|downloads/|modules/|sources/|sphinx_javascript_frameworks_compat\.js\b)")


def publish(html: Path, destination: Path) -> None:
    """Copy Sphinx HTML output into ``destination`` while stripping leading underscores for Jekyll."""
    for source in html.rglob("*"):
        if not source.is_file() or source.name.startswith("."):
            continue
        relative = ASSET_PATH.sub(r"\1", source.relative_to(html).as_posix())
        target = destination / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        if source.suffix in {".html", ".css", ".js"}:
            content = source.read_text(encoding="utf-8")
            target.write_text(ASSET_PATH.sub(r"\1", content), encoding="utf-8")
        else:
            shutil.copyfile(source, target)


def _build_html_one(source_dir: Path, doctrees_dir: Path, out_dir: Path) -> int:
    """Build strict HTML Sphinx docs for ``source_dir`` and publish to ``out_dir``."""
    with TemporaryDirectory(dir=doctrees_dir.parent) as temp:
        html = Path(temp) / "html"
        status = main(["-b", "html", "-E", "-W", "--keep-going", "-d", str(doctrees_dir), str(source_dir), str(html)])
        if status == 0:
            publish(html, out_dir)
        return status


def _build_pdf_one(source_dir: Path, doctrees_dir: Path, out_dir: Path) -> int:
    """Build strict LaTeX Sphinx docs for ``source_dir`` and compile ``pycub.pdf`` into ``out_dir``."""
    with TemporaryDirectory(dir=doctrees_dir.parent) as temp:
        latex_dir = Path(temp) / "latex"
        status = main(["-b", "latex", "-E", "-W", "--keep-going", "-d", str(doctrees_dir), str(source_dir), str(latex_dir)])
        if status != 0:
            return status
        subprocess.run(
            ["make", "-C", str(latex_dir), "LATEXOPTS=-interaction=nonstopmode"],
            check=True,
        )
        pdf_src = latex_dir / "pycub.pdf"
        if not pdf_src.is_file():
            raise FileNotFoundError(f"Expected PDF not generated at {pdf_src}")
        out_dir.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(pdf_src, out_dir / "pycub.pdf")
        return 0


def build(build_pdf: bool = True) -> int:
    """Build English and Czech handbooks (HTML and optional PDF) and replace ``docs/handbook`` atomically."""
    cache = DOCS / "_build"
    cache.mkdir(exist_ok=True)
    has_latex = bool(shutil.which("latexmk") and shutil.which("pdflatex") and shutil.which("make"))

    with TemporaryDirectory(dir=cache) as temp:
        export = Path(temp) / "handbook"
        status_en = _build_html_one(DOCS / "source", cache / "doctrees", export)
        if status_en != 0:
            return status_en

        if build_pdf and has_latex:
            status_pdf_en = _build_pdf_one(DOCS / "source", cache / "doctrees_latex", export)
            if status_pdf_en != 0:
                return status_pdf_en
        elif build_pdf:
            print("Warning: latexmk/pdflatex/make not found; skipping PDF build.", file=sys.stderr)

        source_cs = DOCS / "source_cs"
        if source_cs.is_dir():
            status_cs = _build_html_one(source_cs, cache / "doctrees_cs", export / "cs")
            if status_cs != 0:
                return status_cs

            if build_pdf and has_latex:
                status_pdf_cs = _build_pdf_one(source_cs, cache / "doctrees_latex_cs", export / "cs")
                if status_pdf_cs != 0:
                    return status_pdf_cs

        # Replace the generated tree only after all renders and conversions succeed.
        destination = DOCS / "handbook"
        previous = Path(temp) / "previous"
        if destination.exists():
            destination.rename(previous)
        try:
            export.rename(destination)
        except OSError:
            if previous.exists():
                previous.rename(destination)
            raise
    return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Build English and Czech pyCub Studio handbooks.")
    parser.add_argument("--no-pdf", action="store_true", help="Skip compiling LaTeX PDF manuals.")
    args = parser.parse_args()
    sys.exit(build(build_pdf=not args.no_pdf))
