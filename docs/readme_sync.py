"""Prepare a Sphinx-compatible include of the repository ``README.md``."""
from __future__ import annotations

import re
from pathlib import Path

GITHUB_BLOB_BASE = "https://github.com/rustlluk/pyCub/blob/master"
PUBLIC_BASE = "https://rustlluk.github.io/pyCub"


def prepare_readme_include(
    repo_root: Path,
    output_path: Path,
    builder_format: str = "html",
) -> Path:
    """Transform ``README.md`` links, images, and badges for Sphinx HTML or LaTeX builds."""
    text = (repo_root / "README.md").read_text(encoding="utf-8")

    # Drop the top-level heading so each handbook page keeps its own H1 title.
    lines = text.splitlines()
    if lines and lines[0].startswith("# "):
        lines = lines[1:]
        while lines and not lines[0].strip():
            lines.pop(0)
    text = "\n".join(lines) + "\n"

    # Rewrite repo-relative paths so they resolve from docs/source/ and docs/source_cs/.
    text = text.replace("](docs/static/images/icub.png)", "](../static/images/icub.png)")
    text = re.sub(
        r"\[([^\]]+)\]\(docs/handbook/index\.html\)",
        r"{doc}`\1 <index>`",
        text,
    )
    text = text.replace(
        "](docs/handbook/cs/index.html)",
        f"]({PUBLIC_BASE}/handbook/cs/index.html)",
    )
    text = text.replace(
        "](.devcontainer.json)",
        f"]({GITHUB_BLOB_BASE}/.devcontainer.json)",
    )
    text = text.replace(
        "](.github/workflows/release-standalone.yml)",
        f"]({GITHUB_BLOB_BASE}/.github/workflows/release-standalone.yml)",
    )
    text = text.replace(
        "](icub_pybullet/kid_api.py)",
        f"]({GITHUB_BLOB_BASE}/icub_pybullet/kid_api.py)",
    )
    text = text.replace(
        "](LICENSE)",
        f"]({GITHUB_BLOB_BASE}/LICENSE)",
    )
    text = text.replace(
        "](THIRD_PARTY_LICENSES.md)",
        f"]({GITHUB_BLOB_BASE}/THIRD_PARTY_LICENSES.md)",
    )
    text = text.replace(
        "](docs/handbook/pycub.pdf)",
        f"]({PUBLIC_BASE}/handbook/pycub.pdf)",
    )
    text = text.replace(
        "](docs/handbook/cs/pycub.pdf)",
        f"]({PUBLIC_BASE}/handbook/cs/pycub.pdf)",
    )

    if builder_format == "latex":
        # Avoid fetching remote SVG/PNG shields during offline LaTeX PDF compilation.
        text = text.replace("[![CC BY 4.0][cc-by-shield]][cc-by]\n", "")
        text = text.replace("[![CC BY 4.0][cc-by-image]][cc-by]\n", "")
        text = re.sub(r"^\[cc-by-(?:image|shield)\]:.*\n?", "", text, flags=re.MULTILINE)
        text = text.replace(
            "[![Open in GitHub Codespaces](https://github.com/codespaces/badge.svg)](https://codespaces.new/rustlluk/pyCub?quickstart=1)",
            "[Open in GitHub Codespaces](https://codespaces.new/rustlluk/pyCub?quickstart=1)",
        )
        # Replace clickable remote wrapper on hero image with local image embed for LaTeX.
        text = text.replace(
            f"[![iCub Robot](../static/images/icub.png)]({PUBLIC_BASE})",
            "![iCub Robot](../static/images/icub.png)",
        )
        # Strip emojis not present in standard TeX fonts.
        text = text.replace("📦 ", "").replace("📦", "")
        text = text.replace("→", "->")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    if not output_path.exists() or output_path.read_text(encoding="utf-8") != text:
        output_path.write_text(text, encoding="utf-8")
    return output_path
