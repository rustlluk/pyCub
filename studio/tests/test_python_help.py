"""Unit tests for inline Python signature, definition, and hover help."""

import unittest

from studio.widgets.python_help import lookup


class PythonHelpTests(unittest.TestCase):
    """Test cases for :func:`studio.widgets.python_help.lookup`."""

    def test_incomplete_nested_keyword_call(self):
        """Verify signature lookup highlights the active keyword argument inside nested calls."""
        source = "def scale(x, *, factor=1): pass\nscale(\n    max(1, 2), factor="
        signature, active = lookup(source, None, 3, len(source.splitlines()[-1]),
                                   kid_mode=False, kind="signature")
        self.assertEqual(signature, "scale(x, *, factor=1)")
        self.assertEqual(active, "factor=1")

    def test_unsaved_definition_takes_precedence_over_kid_command(self):
        """Verify local user definitions shadow built-in kid API commands."""
        source = "def wave(times):\n    return times\nwave(3)"
        _, body = lookup(source, None, 3, 2, kid_mode=True, kind="definition")
        self.assertEqual(body, "def wave(times):\n    return times")

    def test_czech_command_in_template(self):
        """Verify Czech kid API aliases resolve to their underlying definitions."""
        call = 'zvedni_rameno("pravá", '
        source = "from icub_pybullet.kid_api import *\n" + call
        _, active = lookup(source, None, 2, len(call), kid_mode=True, kind="signature")
        self.assertTrue(active.startswith("degrees:"))
        _, body = lookup(source, None, 2, 2, kid_mode=True, kind="definition")
        self.assertTrue(body.startswith("def shoulder_up("))

    def test_hover_reads_unsaved_parameter_documentation(self):
        """Verify hover lookup extracts Sphinx ``:param:`` documentation from unsaved code."""
        source = 'def angle(degrees=15):\n    """Turn the head.\n\n    :param degrees: Rotation in degrees.\n    """\nangle(30)'
        signature, doc = lookup(source, None, 6, 2, kid_mode=False, kind="hover")
        self.assertEqual(signature, "angle(degrees=15)")
        self.assertIn("degrees — Rotation in degrees.", doc)

    def test_hover_ignores_plain_values(self):
        """Verify hover lookup returns ``None`` for plain variable identifiers."""
        self.assertIsNone(lookup("left = 5\nprint(left)", None, 2, 7, kid_mode=False, kind="hover"))

    def test_hover_reads_inline_parameter_comments(self):
        """Verify hover lookup falls back to inline trailing parameter comments."""
        source = 'def label(\n    text="#",  # label to display\n    count=2  # number of labels\n): pass\nlabel()'
        _, doc = lookup(source, None, 5, 2, kid_mode=False, kind="hover")
        self.assertEqual(doc, "text — label to display\ncount — number of labels")

    def test_hover_on_czech_kid_command(self):
        """Verify hover lookup translates signatures and docs for Czech kid API commands."""
        signature, doc = lookup("zvedni_rameno()", None, 1, 0, kid_mode=True, kind="hover")
        self.assertIn('side: str=', signature)
        self.assertIn("speed — Maximum joint speed in radians per second.", doc)
