"""
Transparent Qt particle confetti celebration overlay triggered when iCub hits the ball.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import List, Optional

from PySide6.QtCore import QRectF, Qt, QTimer
from PySide6.QtGui import QColor, QPainter
from PySide6.QtWidgets import QWidget


_PALETTE = [
    "#ef4444",
    "#f59e0b",
    "#10b981",
    "#3b82f6",
    "#8b5cf6",
    "#ec4899",
    "#eab308",
    "#06b6d4",
]


@dataclass
class _Particle:
    """Single confetti particle state with position, velocity, rotation, color, and lifetime."""

    x: float
    y: float
    vx: float
    vy: float
    size: float
    angle: float
    spin: float
    color: QColor
    life: float


class ConfettiOverlay(QWidget):
    """Click-through confetti particle overlay covering its parent widget."""

    def __init__(self, parent: Optional[QWidget] = None):
        """Initialize the transparent, click-through confetti overlay widget."""
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)
        self._particles: List[_Particle] = []
        self._timer = QTimer(self)
        self._timer.setInterval(16)
        self._timer.timeout.connect(self._step)
        self.hide()

    def burst(self, count: int = 75) -> None:
        """Spawn ``count`` confetti particles near the upper-center and start animating."""
        if self.parentWidget() is not None:
            self.setGeometry(self.parentWidget().rect())
        w = max(200, self.width())
        h = max(160, self.height())
        cx = w * 0.5
        cy = h * 0.28

        for _ in range(count):
            ang = random.uniform(-math.pi * 0.92, -math.pi * 0.08)
            spd = random.uniform(3.0, 11.5)
            self._particles.append(
                _Particle(
                    x=cx + random.uniform(-w * 0.18, w * 0.18),
                    y=cy + random.uniform(-20.0, 20.0),
                    vx=math.cos(ang) * spd,
                    vy=math.sin(ang) * spd - random.uniform(1.0, 4.0),
                    size=random.uniform(6.0, 12.0),
                    angle=random.uniform(0.0, 360.0),
                    spin=random.uniform(-11.0, 11.0),
                    color=QColor(random.choice(_PALETTE)),
                    life=random.uniform(0.85, 1.35),
                )
            )
        self.show()
        self.raise_()
        if not self._timer.isActive():
            self._timer.start()

    def _step(self) -> None:
        """Advance particle physics by one frame and stop when all particles expire."""
        if self.parentWidget() is not None and self.geometry() != self.parentWidget().rect():
            self.setGeometry(self.parentWidget().rect())

        alive: List[_Particle] = []
        max_y = self.height() + 40
        for p in self._particles:
            p.x += p.vx
            p.y += p.vy
            p.vy += 0.28  # gravity
            p.vx *= 0.992
            p.angle = (p.angle + p.spin) % 360.0
            p.life -= 0.016
            if p.life > 0.0 and p.y < max_y:
                alive.append(p)
        self._particles = alive
        if not self._particles:
            self._timer.stop()
            self.hide()
        else:
            self.update()

    def paintEvent(self, _event) -> None:
        """Render active confetti particles as rotated rounded rectangles."""
        if not self._particles:
            return
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.setPen(Qt.PenStyle.NoPen)
        for p in self._particles:
            col = QColor(p.color)
            col.setAlphaF(min(1.0, max(0.0, p.life * 1.4)))
            painter.save()
            painter.translate(p.x, p.y)
            painter.rotate(p.angle)
            painter.setBrush(col)
            painter.drawRoundedRect(QRectF(-p.size * 0.5, -p.size * 0.28, p.size, p.size * 0.56), 2.0, 2.0)
            painter.restore()
        painter.end()
