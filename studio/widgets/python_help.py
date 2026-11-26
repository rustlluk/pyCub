"""Static call and definition lookup for the Studio Python editors."""
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
from functools import lru_cache
from io import StringIO
from pathlib import Path
import re
from textwrap import dedent
import tokenize


# Jedi's parser caches are shared. Serialize work from all editor tabs.
analysis_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="Studio Python help")


@lru_cache(maxsize=1)
def kid_imports() -> str:
    """Return cached import header exposing ``kid_api`` symbols and Czech aliases."""
    from icub_pybullet.kid_api import CZECH_COMMANDS

    aliases = ", ".join(f"{en} as {cs}" for en, cs in CZECH_COMMANDS.items())
    return f"from icub_pybullet.kid_api import *\nfrom icub_pybullet.kid_api import {aliases}\n"


def parameter_comments(signature):
    """Read trailing comments on parameters declared on their own source line."""
    comments = []
    params = signature.params
    line_counts = Counter(param.line for param in params)
    for param in params:
        if not param.line or line_counts[param.line] != 1:
            continue
        source = param.get_line_code(after=1)
        lines = source.splitlines()
        if not lines or not re.match(rf"\s*\*{{0,2}}{re.escape(param.name)}\b", lines[0]):
            continue
        try:
            tokens = tokenize.generate_tokens(StringIO(lines[0]).readline)
            comment = next((tok for tok in tokens if tok.type == tokenize.COMMENT), None)
        except (tokenize.TokenError, IndentationError):
            continue
        if comment:
            before = lines[0][:comment.start[1]].rstrip()
            if before.endswith(",") or (len(lines) > 1 and lines[1].lstrip().startswith(")")):
                comments.append(f"{param.name} — {comment.string[1:].strip()}")
    return comments


def lookup(source, path, line, column, *, kid_mode, kind):
    """
    Look up hover help, symbol definition, or active call signature via Jedi.

    :param source: Python source text to analyze.
    :param path: File path associated with the buffer, or ``None``.
    :param line: 1-based cursor line number.
    :param column: 0-based cursor column offset in Unicode code points.
    :param kid_mode: Whether to prepend ``kid_api`` imports before analysis.
    :param kind: Lookup mode (``"hover"``, ``"definition"``, or ``"signature"``).
    """
    import jedi
    from jedi.api.environment import InterpreterEnvironment

    prefix = kid_imports() if kid_mode else ""
    root = Path(__file__).resolve().parents[2]
    added_sys_path = [str(root)]
    try:
        import icub_pybullet as _icub_pkg

        icub_parent = str(Path(_icub_pkg.__file__).resolve().parent.parent)
        if icub_parent not in added_sys_path:
            added_sys_path.append(icub_parent)
    except Exception:
        pass
    project = jedi.Project(path=Path(path).parent if path else root, added_sys_path=added_sys_path)
    # A frozen Studio executable cannot act as a separate Python interpreter.
    script = jedi.Script(prefix + source, path=path, project=project, environment=InterpreterEnvironment())
    line += prefix.count("\n")
    if kind == "hover":
        for symbol in script.infer(line, column):
            if symbol.type not in {"function", "class"}:
                continue
            signatures = symbol.get_signatures()
            if signatures:
                doc = symbol.docstring(raw=True)
                # Preserve free-form, Google and NumPy docs, and simplify Sphinx fields.
                doc = re.sub(r"^:param (?:[^:\n]+ )?([*\w]+):\s*", r"\1 — ", doc, flags=re.MULTILINE)
                notes = parameter_comments(signatures[0])
                return signatures[0].to_string(), "\n".join(filter(None, [doc, *notes]))
        return None
    if kind == "definition":
        for symbol in script.goto(line, column, follow_imports=True):
            start = symbol.get_definition_start_position()
            end = symbol.get_definition_end_position()
            if start and end and symbol.line:
                body = symbol.get_line_code(before=symbol.line - start[0], after=end[0] - symbol.line)
                if body.strip():
                    return symbol.full_name or symbol.name, dedent(body).rstrip()
            doc = symbol.docstring()
            if doc:
                return symbol.name, doc
        return None

    signatures = script.get_signatures(line, column)
    if signatures:
        signature = signatures[0]
        params = signature.params
        active = signature.index
        parameter = params[active].to_string() if active is not None and 0 <= active < len(params) else ""
        return signature.to_string(), parameter
    return None
