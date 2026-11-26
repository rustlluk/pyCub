"""
Syntax-highlighted Python code editor widget with line numbers, smart indentation,
QCompleter symbol completion, and error-line highlighting (shared by Modes 3 & 4).
"""
from __future__ import annotations

import keyword
import logging
import re
from html import escape
from pathlib import Path
from typing import Iterable, List, Optional, Tuple

from PySide6.QtCore import QEvent, QRect, QSize, QStringListModel, Qt, QTimer
from PySide6.QtGui import (
    QColor,
    QCursor,
    QFont,
    QKeyEvent,
    QPainter,
    QSyntaxHighlighter,
    QTextCharFormat,
    QTextCursor,
    QTextDocument,
)
from PySide6.QtWidgets import QCompleter, QDialog, QPlainTextEdit, QTextEdit, QToolTip, QVBoxLayout, QWidget

from studio.widgets.python_help import analysis_pool, lookup


class PythonSyntaxHighlighter(QSyntaxHighlighter):
    """Lightweight Python + pyCub / kid_api syntax highlighter."""

    def __init__(self, document: QTextDocument, extra_builtins: Optional[Iterable[str]] = None):
        """Initialize syntax highlighting rules for Python keywords, builtins, and comments."""
        super().__init__(document)
        self._rules: List[Tuple[re.Pattern, QTextCharFormat]] = []

        kw_fmt = QTextCharFormat()
        kw_fmt.setForeground(QColor("#7c3aed"))
        kw_fmt.setFontWeight(QFont.Weight.Bold)
        for kw in keyword.kwlist:
            self._rules.append((re.compile(rf"\b{kw}\b"), kw_fmt))

        api_fmt = QTextCharFormat()
        api_fmt.setForeground(QColor("#0284c7"))
        api_fmt.setFontWeight(QFont.Weight.DemiBold)
        builtins_list = [
            "print", "range", "len", "int", "float", "str", "list", "dict", "abs", "min", "max", "round",
            "pyCub", "Pose", "Config", "URDF",
        ]
        if extra_builtins:
            builtins_list.extend(extra_builtins)
        for sym in set(builtins_list):
            self._rules.append((re.compile(rf"\b{re.escape(sym)}\b"), api_fmt))

        def_fmt = QTextCharFormat()
        def_fmt.setForeground(QColor("#1d4ed8"))
        def_fmt.setFontWeight(QFont.Weight.Bold)
        self._rules.append((re.compile(r"\bdef\s+([A-Za-z_][A-Za-z0-9_]*)"), def_fmt))
        self._rules.append((re.compile(r"\bclass\s+([A-Za-z_][A-Za-z0-9_]*)"), def_fmt))

        num_fmt = QTextCharFormat()
        num_fmt.setForeground(QColor("#d97706"))
        self._rules.append((re.compile(r"\b[0-9]+(\.[0-9]+)?([eE][+-]?[0-9]+)?\b"), num_fmt))

        str_fmt = QTextCharFormat()
        str_fmt.setForeground(QColor("#15803d"))
        self._rules.append((re.compile(r'"[^"\\]*(\\.[^"\\]*)*"'), str_fmt))
        self._rules.append((re.compile(r"'[^'\\]*(\\.[^'\\]*)*'"), str_fmt))

        self._comment_re = re.compile(r"#[^\n]*")
        self._comment_fmt = QTextCharFormat()
        self._comment_fmt.setForeground(QColor("#64748b"))
        self._comment_fmt.setFontItalic(True)

    def highlightBlock(self, text: str) -> None:
        """Apply syntax formatting rules to a single text block."""
        for pattern, fmt in self._rules:
            for m in pattern.finditer(text):
                if m.lastindex:
                    s, e = m.span(1)
                else:
                    s, e = m.span()
                self.setFormat(s, e - s, fmt)
        for m in self._comment_re.finditer(text):
            s, e = m.span()
            self.setFormat(s, e - s, self._comment_fmt)


class _LineNumberArea(QWidget):
    """Margin widget that paints line numbers alongside ``PythonCodeEditor``."""

    def __init__(self, editor: "PythonCodeEditor"):
        """Bind the line number gutter widget to ``editor``."""
        super().__init__(editor)
        self._editor = editor

    def sizeHint(self) -> QSize:
        """Return the preferred width for the line number gutter."""
        return QSize(self._editor.line_number_area_width(), 0)

    def paintEvent(self, event) -> None:
        """Delegate gutter painting to the parent editor."""
        self._editor.paint_line_numbers(event)


class PythonCodeEditor(QPlainTextEdit):
    """Python code editor with line numbers, autocomplete, 4-space Tab, and error-line highlight."""

    def __init__(
        self,
        completions: Optional[List[str]] = None,
        extra_highlight_symbols: Optional[Iterable[str]] = None,
        parent: Optional[QWidget] = None,
        *,
        kid_mode: bool = False,
        i18n=None,
    ):
        """Initialize the code editor with syntax highlighting, line numbers, and help timers."""
        super().__init__(parent)
        font = QFont("JetBrains Mono, Fira Code, Consolas, DejaVu Sans Mono, monospace")
        font.setStyleHint(QFont.StyleHint.Monospace)
        font.setPointSize(11)
        self.setFont(font)
        self.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        self.source_path = None
        self._kid_mode = kid_mode
        self._i18n = i18n
        self._help_future = None
        self._help_kind = None
        self._hover_rect = QRect()
        self._hint_visible = False
        self._hint_timer = QTimer(self, singleShot=True, interval=250)
        self._hint_timer.timeout.connect(lambda: self._request_help("signature"))
        self._result_timer = QTimer(self, interval=50)
        self._result_timer.timeout.connect(self._show_help_result)
        self.cursorPositionChanged.connect(self._schedule_hint)
        self.textChanged.connect(self._schedule_hint)
        self.setStyleSheet(
            "QPlainTextEdit { background: #ffffff; color: #0f172a; selection-background-color: #bfdbfe; "
            "border: 1px solid #cbd5e1; border-radius: 8px; }"
        )

        self._error_line: Optional[int] = None
        self._line_area = _LineNumberArea(self)
        self.highlighter = PythonSyntaxHighlighter(self.document(), extra_highlight_symbols)

        self.blockCountChanged.connect(self._update_line_area_width)
        self.updateRequest.connect(self._update_line_area)
        self.cursorPositionChanged.connect(self._highlight_lines)
        self._update_line_area_width(0)
        self._highlight_lines()

        self._completer: Optional[QCompleter] = None
        if completions:
            self.set_completions(completions)

    def set_completions(self, words: List[str]) -> None:
        """
        Configure or update the popup autocompletion word list.

        :param words: Candidate identifier strings offered by ``QCompleter``.
        """
        if self._completer is not None:
            self._completer.popup().hide()
            self._completer.model().setStringList(sorted(set(words)))
            return
        model = QStringListModel(sorted(set(words)), self)
        comp = QCompleter(model, self)
        comp.setWidget(self)
        comp.setCompletionMode(QCompleter.CompletionMode.PopupCompletion)
        comp.setCaseSensitivity(Qt.CaseSensitivity.CaseInsensitive)
        comp.activated.connect(self._insert_completion)
        self._completer = comp

    def _python_help_enabled(self):
        """Return whether Jedi-based Python help is active for the current buffer."""
        return not self.source_path or Path(self.source_path).suffix.lower() == ".py"

    def _schedule_hint(self):
        """Reset pending help lookups and schedule a signature hint if focused."""
        self._hint_timer.stop()
        self._result_timer.stop()
        if self._help_future is not None:
            self._help_future.cancel()
            self._help_future = None
        if self._hint_visible:
            QToolTip.hideText()
            self._hint_visible = False
        if self.hasFocus() and self._python_help_enabled():
            self._hint_timer.start()

    def viewportEvent(self, event):
        """Intercept tooltip events over identifiers to show hover documentation."""
        if event.type() == QEvent.Type.ToolTip and self._python_help_enabled():
            cursor = self.cursorForPosition(event.pos())
            cursor.select(QTextCursor.SelectionType.WordUnderCursor)
            if cursor.selectedText().isidentifier():
                end = self.cursorRect(cursor)
                cursor.setPosition(cursor.selectionStart())
                rect = self.cursorRect(cursor).united(end)
                if rect.contains(event.pos()):
                    if self._help_future is None or self._help_kind != "definition":
                        self._hover_rect = rect
                        self._hover_cursor = QTextCursor(cursor)
                        self._request_help("hover", cursor)
                    return True
            event.ignore()
            return True
        return super().viewportEvent(event)

    def _request_help(self, kind, cursor=None):
        """Submit an asynchronous Jedi lookup for hover, definition, or signature help."""
        if (kind != "hover" and not self.hasFocus()) or not self._python_help_enabled():
            return
        self._hint_timer.stop()
        if self._help_future is not None:
            self._help_future.cancel()
        if cursor is None:
            cursor = self.textCursor()
        # Qt measures columns in UTF-16 units. Jedi uses Unicode code points.
        line = cursor.blockNumber() + 1
        cursor.clearSelection()
        cursor.movePosition(QTextCursor.MoveOperation.StartOfBlock, QTextCursor.MoveMode.KeepAnchor)
        column = len(cursor.selectedText())
        self._help_kind = kind
        self._help_future = analysis_pool.submit(
            lookup, self.toPlainText(), self.source_path, line, column,
            kid_mode=self._kid_mode, kind=kind,
        )
        self._result_timer.start()

    def _show_help_result(self):
        """Display completed asynchronous Jedi help results as a tooltip or dialog."""
        future = self._help_future
        if future is None or not future.done():
            return
        self._result_timer.stop()
        self._help_future = None
        is_hover = self._help_kind == "hover"
        if is_hover:
            point = self.viewport().mapFromGlobal(QCursor.pos())
            if (not self.isVisible() or not self._hover_rect.contains(point)
                    or self.cursorRect(self._hover_cursor).topLeft() != self._hover_rect.topLeft()):
                return
        if (not is_hover and not self.hasFocus()) or not self._python_help_enabled():
            return
        try:
            result = future.result()
        except Exception:
            logging.getLogger(__name__).debug("Python help lookup failed", exc_info=True)
            result = None
        if is_hover:
            if result:
                signature, doc = result
                message = f"<p><b>{escape(signature)}</b></p>"
                if doc:
                    excerpt = doc[:1600].rstrip() + ("…" if len(doc) > 1600 else "")
                    message += escape(excerpt).replace("\n", "<br>")
                QToolTip.showText(QCursor.pos(), message, self.viewport(), self._hover_rect)
                self._hint_visible = True
            return
        if self._help_kind == "definition" and result:
            title, body = result
            dialog = QDialog(self)
            dialog.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose)
            dialog.setWindowTitle(title)
            view = QPlainTextEdit(body, dialog)
            view.setFont(self.font())
            view.setReadOnly(True)
            QVBoxLayout(dialog).addWidget(view)
            dialog.resize(760, 440)
            dialog.open()
            return
        if self._help_kind == "definition":
            message = self._i18n.t("no_definition") if self._i18n else "No definition available."
        elif result:
            signature, active = result
            message = escape(signature)
            if active:
                # Match the whole parameter (two names may share a prefix).
                pattern = r"(?<=[( ])" + re.escape(active) + r"(?=[,)])"
                match = re.search(pattern, signature)
                if match:
                    message = (escape(signature[:match.start()]) + "<b><u>" + escape(active)
                               + "</u></b>" + escape(signature[match.end():]))
            message = f"<p>{message}</p>"
        else:
            return
        QToolTip.showText(self.viewport().mapToGlobal(self.cursorRect().bottomLeft()), message, self)
        self._hint_visible = True

    def contextMenuEvent(self, event):
        """Extend the standard context menu with definition and parameter help actions."""
        menu = self.createStandardContextMenu()
        if self._python_help_enabled():
            menu.addSeparator()
            label = self._i18n.t("show_definition") if self._i18n else "Show definition"
            definition = menu.addAction(f"{label}\tF12")
            label = self._i18n.t("show_parameters") if self._i18n else "Show parameters"
            parameters = menu.addAction(f"{label}\tCtrl+Shift+Space")
            choice = menu.exec(event.globalPos())
            if choice in (definition, parameters):
                self.setFocus()
                self.setTextCursor(self.cursorForPosition(event.pos()))
                self._request_help("definition" if choice == definition else "signature")
        else:
            menu.exec(event.globalPos())
        menu.deleteLater()

    def set_error_line(self, lineno: Optional[int]) -> None:
        """
        Highlight a 1-based line number as an error and scroll it into view.

        :param lineno: 1-based error line number, or ``None`` to clear.
        """
        self._error_line = lineno
        self._highlight_lines()
        if lineno is not None and lineno >= 1:
            block = self.document().findBlockByNumber(lineno - 1)
            if block.isValid():
                cur = QTextCursor(block)
                self.setTextCursor(cur)
                self.centerCursor()

    def clear_error_line(self) -> None:
        """Clear any active error line highlight."""
        if self._error_line is not None:
            self._error_line = None
            self._highlight_lines()

    def insert_snippet(self, snippet: str) -> None:
        """
        Insert a code snippet on a new line at the current cursor and focus the editor.

        :param snippet: Code text to insert.
        """
        cur = self.textCursor()
        if cur.columnNumber() > 0:
            cur.insertText("\n")
        cur.insertText(snippet)
        if not snippet.endswith("\n"):
            cur.insertText("\n")
        self.setTextCursor(cur)
        self.setFocus()

    def line_number_area_width(self) -> int:
        """Calculate the pixel width required for the line number gutter."""
        digits = max(3, len(str(max(1, self.blockCount()))))
        return 14 + self.fontMetrics().horizontalAdvance("9") * digits

    def _update_line_area_width(self, _count: int) -> None:
        """Update the editor's left viewport margin to fit the line number gutter."""
        self.setViewportMargins(self.line_number_area_width(), 0, 0, 0)

    def _update_line_area(self, rect: QRect, dy: int) -> None:
        """Scroll or repaint the line number gutter when the viewport updates."""
        if dy:
            self._line_area.scroll(0, dy)
        else:
            self._line_area.update(0, rect.y(), self._line_area.width(), rect.height())
        if rect.contains(self.viewport().rect()):
            self._update_line_area_width(0)

    def resizeEvent(self, event) -> None:
        """Resize the line number gutter to match the editor's contents rect."""
        super().resizeEvent(event)
        cr = self.contentsRect()
        self._line_area.setGeometry(QRect(cr.left(), cr.top(), self.line_number_area_width(), cr.height()))

    def paint_line_numbers(self, event) -> None:
        """Paint visible line numbers and highlight the error line number if set."""
        painter = QPainter(self._line_area)
        painter.fillRect(event.rect(), QColor("#f1f5f9"))

        block = self.firstVisibleBlock()
        block_num = block.blockNumber()
        top = int(round(self.blockBoundingGeometry(block).translated(self.contentOffset()).top()))
        bottom = top + int(round(self.blockBoundingRect(block).height()))

        while block.isValid() and top <= event.rect().bottom():
            if block.isVisible() and bottom >= event.rect().top():
                num_str = str(block_num + 1)
                is_err = self._error_line == (block_num + 1)
                painter.setPen(QColor("#dc2626") if is_err else QColor("#64748b"))
                painter.drawText(
                    0,
                    top,
                    self._line_area.width() - 6,
                    self.fontMetrics().height(),
                    Qt.AlignmentFlag.AlignRight,
                    num_str,
                )
            block = block.next()
            top = bottom
            bottom = top + int(round(self.blockBoundingRect(block).height()))

    def _highlight_lines(self) -> None:
        """Apply full-width background highlights to the current line and error line."""
        selections: List[QTextEdit.ExtraSelection] = []
        if not self.isReadOnly():
            cur_sel = QTextEdit.ExtraSelection()
            cur_sel.format.setBackground(QColor("#f8fafc"))
            cur_sel.format.setProperty(QTextCharFormat.Property.FullWidthSelection, True)
            cur_sel.cursor = self.textCursor()
            cur_sel.cursor.clearSelection()
            selections.append(cur_sel)

        if self._error_line is not None and self._error_line >= 1:
            block = self.document().findBlockByNumber(self._error_line - 1)
            if block.isValid():
                err_sel = QTextEdit.ExtraSelection()
                err_sel.format.setBackground(QColor("#fee2e2"))
                err_sel.format.setProperty(QTextCharFormat.Property.FullWidthSelection, True)
                err_sel.cursor = QTextCursor(block)
                err_sel.cursor.clearSelection()
                selections.append(err_sel)

        self.setExtraSelections(selections)

    def _insert_completion(self, completion: str) -> None:
        """Insert the selected completion suffix at the current cursor position."""
        if self._completer is None:
            return
        tc = self.textCursor()
        extra = len(completion) - len(self._completer.completionPrefix())
        tc.movePosition(QTextCursor.MoveOperation.Left)
        tc.movePosition(QTextCursor.MoveOperation.EndOfWord)
        tc.insertText(completion[-extra:] if extra > 0 else "")
        self.setTextCursor(tc)

    def _word_under_cursor(self) -> str:
        """Return the word immediately under the text cursor."""
        tc = self.textCursor()
        tc.select(QTextCursor.SelectionType.WordUnderCursor)
        return tc.selectedText()

    def keyPressEvent(self, event: QKeyEvent) -> None:
        """Handle help shortcuts, completion popup keys, smart indentation, and auto-completion."""
        if (event.key() == Qt.Key.Key_Space
                and event.modifiers() == (Qt.KeyboardModifier.ControlModifier | Qt.KeyboardModifier.ShiftModifier)
                and self._python_help_enabled()):
            self._request_help("signature")
            return
        if event.key() == Qt.Key.Key_F12 and self._python_help_enabled():
            if self._completer:
                self._completer.popup().hide()
            self._request_help("definition")
            return
        if self._completer is not None and self._completer.popup().isVisible():
            if event.key() in (
                Qt.Key.Key_Enter,
                Qt.Key.Key_Return,
                Qt.Key.Key_Escape,
                Qt.Key.Key_Tab,
                Qt.Key.Key_Backtab,
            ):
                event.ignore()
                return

        if event.key() == Qt.Key.Key_Tab and not (event.modifiers() & Qt.KeyboardModifier.ControlModifier):
            self.insertPlainText("    ")
            return

        if event.key() == Qt.Key.Key_Backtab:
            tc = self.textCursor()
            tc.movePosition(QTextCursor.MoveOperation.StartOfLine, QTextCursor.MoveMode.MoveAnchor)
            line_text = tc.block().text()
            spaces = len(line_text) - len(line_text.lstrip(" "))
            remove_n = min(4, spaces)
            for _ in range(remove_n):
                tc.deleteChar()
            return

        if event.key() in (Qt.Key.Key_Return, Qt.Key.Key_Enter):
            tc = self.textCursor()
            line_text = tc.block().text()[: tc.positionInBlock()]
            indent = len(line_text) - len(line_text.lstrip(" "))
            if line_text.rstrip().endswith(":"):
                indent += 4
            super().keyPressEvent(event)
            self.insertPlainText(" " * indent)
            return

        super().keyPressEvent(event)

        if self._completer is None:
            return
        prefix = self._word_under_cursor()
        if len(prefix) < 2 or not (prefix[0].isalpha() or prefix[0] == "_"):
            self._completer.popup().hide()
            return
        if prefix != self._completer.completionPrefix():
            self._completer.setCompletionPrefix(prefix)
            self._completer.popup().setCurrentIndex(self._completer.completionModel().index(0, 0))
        cr = self.cursorRect()
        cr.setWidth(
            self._completer.popup().sizeHintForColumn(0)
            + self._completer.popup().verticalScrollBar().sizeHint().width()
            + 20
        )
        self._completer.complete(cr)
