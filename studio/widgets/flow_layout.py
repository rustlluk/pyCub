"""Wrapping horizontal flow layout for responsive Studio headers and toolbars."""
from __future__ import annotations

from typing import List, Optional

from PySide6.QtCore import QPoint, QRect, QSize, Qt
from PySide6.QtWidgets import QLayout, QLayoutItem, QSizePolicy, QSpacerItem, QStyle, QWidget


class _FlowStretchItem(QSpacerItem):
    """Zero-minimum horizontal spacer that expands only when all flow items fit on one line."""

    def __init__(self, stretch: int = 1):
        """Initialize a collapsible flow stretch marker."""
        super().__init__(0, 0, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)
        self.stretch = max(1, int(stretch))


class FlowLayout(QLayout):
    """Horizontal wrapping layout that keeps controls on one row when wide and wraps when narrow."""

    def __init__(
        self,
        parent: Optional[QWidget] = None,
        margin: int = -1,
        h_spacing: int = -1,
        v_spacing: int = -1,
    ):
        """Initialize the flow layout with optional margins and horizontal/vertical spacing."""
        super().__init__(parent)
        if margin >= 0:
            self.setContentsMargins(margin, margin, margin, margin)
        self._h_space = h_spacing
        self._v_space = v_spacing
        self._items: List[QLayoutItem] = []

    def __del__(self) -> None:
        """Delete any remaining layout items when the layout is destroyed."""
        while self._items:
            item = self.takeAt(0)
            if item is not None:
                del item

    def addItem(self, item: QLayoutItem) -> None:
        """Append a layout item to the flow sequence."""
        self._items.append(item)

    def addStretch(self, stretch: int = 1) -> None:
        """Append an inline stretch item active when all items fit on a single row."""
        self.addItem(_FlowStretchItem(stretch))

    def horizontalSpacing(self) -> int:
        """Return the horizontal gap in pixels between adjacent items."""
        if self._h_space >= 0:
            return self._h_space
        return self._smart_spacing(QStyle.PixelMetric.PM_LayoutHorizontalSpacing)

    def verticalSpacing(self) -> int:
        """Return the vertical gap in pixels between wrapped rows."""
        if self._v_space >= 0:
            return self._v_space
        return self._smart_spacing(QStyle.PixelMetric.PM_LayoutVerticalSpacing)

    def count(self) -> int:
        """Return the number of managed layout items."""
        return len(self._items)

    def itemAt(self, index: int) -> Optional[QLayoutItem]:
        """Return the item at ``index`` or ``None`` if out of range."""
        if 0 <= index < len(self._items):
            return self._items[index]
        return None

    def takeAt(self, index: int) -> Optional[QLayoutItem]:
        """Remove and return the item at ``index`` or ``None`` if out of range."""
        if 0 <= index < len(self._items):
            return self._items.pop(index)
        return None

    def expandingDirections(self) -> Qt.Orientation:
        """Report no automatic layout expansion beyond height-for-width wrapping."""
        return Qt.Orientation(0)

    def hasHeightForWidth(self) -> bool:
        """Return ``True`` because row wrapping depends on available width."""
        return True

    def heightForWidth(self, width: int) -> int:
        """Compute the height required to lay out all visible items within ``width``."""
        return self._do_layout(QRect(0, 0, width, 0), test_only=True)

    def setGeometry(self, rect: QRect) -> None:
        """Assign geometries to all child items within ``rect``."""
        super().setGeometry(rect)
        self._do_layout(rect, test_only=False)

    def sizeHint(self) -> QSize:
        """Return the preferred single-row size including margins."""
        left, top, right, bottom = self.getContentsMargins()
        h_space = self.horizontalSpacing()
        total_w = 0
        max_h = 0
        visible_count = 0
        for item in self._items:
            if isinstance(item, _FlowStretchItem):
                continue
            w_item = item.widget()
            if w_item is not None and w_item.isHidden():
                continue
            sh = item.sizeHint()
            if visible_count > 0:
                total_w += h_space
            total_w += sh.width()
            max_h = max(max_h, sh.height())
            visible_count += 1
        return QSize(total_w + left + right, max_h + top + bottom)

    def minimumSize(self) -> QSize:
        """Return the minimum size bounded by the widest individual item, not the row sum."""
        size = QSize(0, 0)
        for item in self._items:
            if isinstance(item, _FlowStretchItem):
                continue
            w_item = item.widget()
            if w_item is not None and w_item.isHidden():
                continue
            size = size.expandedTo(item.minimumSize())
        left, top, right, bottom = self.getContentsMargins()
        size += QSize(left + right, top + bottom)
        return size

    def _item_spacings(self, item: QLayoutItem) -> tuple[int, int]:
        """Compute effective horizontal and vertical spacing after ``item``."""
        wid = item.widget()
        space_x = self.horizontalSpacing()
        space_y = self.verticalSpacing()
        if wid is not None:
            if space_x < 0:
                space_x = wid.style().layoutSpacing(
                    QSizePolicy.ControlType.PushButton,
                    QSizePolicy.ControlType.PushButton,
                    Qt.Orientation.Horizontal,
                )
            if space_y < 0:
                space_y = wid.style().layoutSpacing(
                    QSizePolicy.ControlType.PushButton,
                    QSizePolicy.ControlType.PushButton,
                    Qt.Orientation.Vertical,
                )
        return max(0, space_x), max(0, space_y)

    def _do_layout(self, rect: QRect, test_only: bool) -> int:
        """Arrange visible items into wrapped rows and return total layout height."""
        left, top, right, bottom = self.getContentsMargins()
        effective = rect.adjusted(left, top, -right, -bottom)
        avail_w = max(0, effective.width())

        # Collect visible non-stretch items and determine whether all fit on a single row.
        visible_entries = []
        total_single_row_w = 0
        stretch_weight = 0
        prev_was_widget = False
        for item in self._items:
            if isinstance(item, _FlowStretchItem):
                stretch_weight += item.stretch
                visible_entries.append((item, True))
                continue
            w_item = item.widget()
            if w_item is not None and w_item.isHidden():
                continue
            sh = item.sizeHint()
            sx, _ = self._item_spacings(item)
            if prev_was_widget:
                total_single_row_w += sx
            total_single_row_w += sh.width()
            prev_was_widget = True
            visible_entries.append((item, False))

        single_row_extra = 0
        if stretch_weight > 0 and avail_w > total_single_row_w:
            single_row_extra = avail_w - total_single_row_w

        x = effective.x()
        y = effective.y()
        line_height = 0
        row_has_item = False

        for idx, (item, is_stretch) in enumerate(visible_entries):
            if is_stretch:
                if single_row_extra > 0 and not test_only:
                    assert isinstance(item, _FlowStretchItem)
                    extra_w = int(round(single_row_extra * (item.stretch / stretch_weight)))
                    item.setGeometry(QRect(x, y, extra_w, line_height))
                    x += extra_w
                elif single_row_extra > 0:
                    assert isinstance(item, _FlowStretchItem)
                    x += int(round(single_row_extra * (item.stretch / stretch_weight)))
                continue

            sh = item.sizeHint()
            space_x, space_y = self._item_spacings(item)
            item_w = min(sh.width(), avail_w) if avail_w > 0 else sh.width()
            next_x = x + item_w
            if row_has_item and next_x > effective.right() + 1:
                x = effective.x()
                y = y + line_height + space_y
                next_x = x + item_w
                line_height = 0
                row_has_item = False

            if not test_only:
                item.setGeometry(QRect(QPoint(x, y), QSize(item_w, sh.height())))

            x = next_x + space_x
            line_height = max(line_height, sh.height())
            row_has_item = True

            # Avoid adding trailing space if a stretch follows on the same single row.
            if single_row_extra > 0 and idx + 1 < len(visible_entries) and visible_entries[idx + 1][1]:
                x -= space_x

        return y + line_height - rect.y() + bottom

    def _smart_spacing(self, pm: QStyle.PixelMetric) -> int:
        """Query the parent widget or style for default layout spacing."""
        parent = self.parent()
        if parent is None:
            return -1
        if isinstance(parent, QWidget):
            return parent.style().pixelMetric(pm, None, parent)
        if isinstance(parent, QLayout):
            return parent.spacing()
        return -1
