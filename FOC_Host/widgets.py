"""抗锯齿双指针仪表和实时趋势图；仅显示，不访问设备。"""
import math

from PySide6.QtCore import Qt, QPointF, QRectF, Signal
from PySide6.QtGui import QColor, QFont, QLinearGradient, QPainter, QPen, QPolygonF, QRadialGradient
from PySide6.QtWidgets import QWidget, QSizePolicy

BG = "#101419"
PANEL = "#191f27"
GRID = "#2a3441"
TEXT = "#edf2f8"
MUTED = "#8795a8"
CYAN = "#49c5ed"
AMBER = "#f4b65b"
GREEN = "#65d7a0"
RED = "#f47c8a"


class Gauge(QWidget):
    targetChanged = Signal(float)
    targetReleased = Signal(float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.low, self.high = -3000.0, 3000.0
        self.unit, self.caption = "rpm", "机械转速"
        self.actual, self.target = math.nan, 0.0
        self.dragging = False
        self.setMinimumSize(340, 330)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setCursor(Qt.CursorShape.OpenHandCursor)
        self.cx = self.cy = self.radius = 0.0

    def set_mode(self, position):
        self.low, self.high = (0.0, 360.0) if position else (-3000.0, 3000.0)
        self.unit, self.caption = ("°", "单圈位置 · 接口预留") if position else ("rpm", "机械转速")
        self.dragging = False
        self.update()

    def set_values(self, actual=None, target=None):
        if actual is not None:
            self.actual = actual
        if target is not None and not self.dragging:
            self.target = target
        self.update()

    def point(self, value, radius):
        fraction = (min(self.high, max(self.low, value))-self.low)/(self.high-self.low)
        angle = math.radians(135 + 270*fraction)
        return QPointF(self.cx+math.cos(angle)*radius, self.cy+math.sin(angle)*radius)

    def _text(self, painter, rect, text, size, color, bold=False):
        painter.setPen(QColor(color))
        font = QFont("Segoe UI", size)
        font.setBold(bold)
        painter.setFont(font)
        painter.drawText(rect, Qt.AlignmentFlag.AlignCenter, text)

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHints(QPainter.RenderHint.Antialiasing | QPainter.RenderHint.TextAntialiasing)
        w, h = self.width(), self.height()
        self.radius = r = max(50, min(w*.46, h*.47))
        self.cx, self.cy = w/2, h*.48
        cx, cy = self.cx, self.cy
        outer = QRectF(cx-r, cy-r, 2*r, 2*r)
        face = QRadialGradient(QPointF(cx, cy-r*.25), r*1.35)
        face.setColorAt(0, QColor("#242e3a"))
        face.setColorAt(.7, QColor("#151c24"))
        face.setColorAt(1, QColor("#10161e"))
        p.setBrush(face)
        p.setPen(QPen(QColor("#364352"), 1))
        p.drawEllipse(outer)
        p.setBrush(Qt.BrushStyle.NoBrush)
        for inset, color, width in ((4, "#0d1218", 4), (10, "#2c3948", 1), (19, "#334353", 8)):
            p.setPen(QPen(QColor(color), width, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
            p.drawArc(outer.adjusted(inset, inset, -inset, -inset), -45*16, 270*16)
        # 正负两个方向的主色弧，保持底部缺口和清楚的中性零点。
        p.setPen(QPen(QColor("#397991"), 3))
        p.drawArc(outer.adjusted(19, 19, -19, -19), -45*16, 270*16)
        major_count = 12 if self.low < 0 else 8
        subdivisions = major_count*5
        for i in range(subdivisions+1):
            value = self.low+(self.high-self.low)*i/subdivisions
            major = i % 5 == 0
            p.setPen(QPen(QColor("#b8c6d5" if major else "#4b5b6c"), 2 if major else 1))
            p.drawLine(self.point(value, r-29), self.point(value, r-(44 if major else 36)))
            if major:
                pos = self.point(value, r-64)
                self._text(p, QRectF(pos.x()-33, pos.y()-10, 66, 20), f"{value:g}", max(8, int(r/19)), MUTED)
        self._text(p, QRectF(cx-r*.5, cy-r*.52, r, 24), self.caption, max(10, int(r/15)), MUTED)
        if math.isfinite(self.actual):
            tip = self.point(self.actual, r-77)
            back = self.point(self.actual, -r*.085)
            dx, dy = tip.x()-cx, tip.y()-cy
            norm = max(1, math.hypot(dx, dy))
            perpendicular = QPointF(-dy/norm*r*.018, dx/norm*r*.018)
            needle = QPolygonF([tip, back+perpendicular, back-perpendicular])
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QColor(CYAN))
            p.drawPolygon(needle)
        tip = self.point(self.target, r-23)
        end = self.point(self.target, r*.18)
        p.setPen(QPen(QColor(AMBER), 1.8))
        p.drawLine(end, tip)
        p.setBrush(QColor(AMBER))
        p.setPen(QPen(QColor("#1b222b"), 2))
        p.drawEllipse(tip, 6, 6)
        hub = QLinearGradient(cx, cy-8, cx, cy+8)
        hub.setColorAt(0, QColor("#65778b"))
        hub.setColorAt(1, QColor("#18222e"))
        p.setBrush(hub)
        p.setPen(QPen(QColor("#8fa1b6"), 1))
        p.drawEllipse(QPointF(cx, cy), r*.045, r*.045)
        self._text(p, QRectF(cx-r*.65, cy+r*.22, r*1.3, r*.28),
                   f"{self.actual:,.1f}" if math.isfinite(self.actual) else "—", max(24, int(r/5.2)), TEXT, True)
        self._text(p, QRectF(cx-r*.5, cy+r*.49, r, 25), self.unit, max(10, int(r/16)), MUTED)
        self._text(p, QRectF(cx-r*.65, cy+r*.69, r*1.3, 26), f"目标  {self.target:g} {self.unit}",
                   max(10, int(r/16)), AMBER)
        p.end()

    def mousePressEvent(self, event):
        if event.button() != Qt.MouseButton.LeftButton:
            return
        distance = math.hypot(event.position().x()-self.cx, event.position().y()-self.cy)
        if 15 < distance < self.radius+8:
            self.dragging = True
            self.setCursor(Qt.CursorShape.ClosedHandCursor)
            self._drag(event.position())

    def _drag(self, point):
        angle = math.degrees(math.atan2(point.y()-self.cy, point.x()-self.cx)) % 360
        if angle < 135:
            angle += 360
        if angle > 405:
            angle = 405 if angle < 450 else 135
        self.target = round(self.low+(self.high-self.low)*(angle-135)/270, 1)
        self.targetChanged.emit(self.target)
        self.update()

    def mouseMoveEvent(self, event):
        if self.dragging:
            self._drag(event.position())

    def mouseReleaseEvent(self, event):
        if self.dragging and event.button() == Qt.MouseButton.LeftButton:
            self._drag(event.position())
            self.dragging = False
            self.setCursor(Qt.CursorShape.OpenHandCursor)
            self.targetReleased.emit(self.target)


class Trend(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.series = []
        self.setMinimumHeight(65)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

    def set_series(self, series):
        self.series = series
        self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        rect = QRectF(42, 8, max(1, w-52), max(1, h-20))
        finite = [abs(v) for data, _ in self.series for v in data if math.isfinite(v)]
        scale = max(100, max(finite, default=0)*1.15)
        p.setFont(QFont("Segoe UI", 8))
        for i in range(5):
            y = rect.top()+rect.height()*i/4
            p.setPen(QPen(QColor(GRID), 1, Qt.PenStyle.DotLine))
            p.drawLine(QPointF(rect.left(), y), QPointF(rect.right(), y))
            p.setPen(QColor(MUTED))
            p.drawText(QRectF(0, y-9, 35, 18), Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
                       f"{scale*(1-i/2):.0f}")
        for data, color in self.series:
            p.setPen(QPen(QColor(color), 1.5))
            points = []
            for i, value in enumerate(data):
                if math.isfinite(value):
                    points.append(QPointF(rect.left()+rect.width()*i/max(1, len(data)-1),
                                         rect.center().y()-value/scale*rect.height()/2))
                else:
                    if len(points) > 1:
                        p.drawPolyline(QPolygonF(points))
                    points = []
            if len(points) > 1:
                p.drawPolyline(QPolygonF(points))
        p.end()
