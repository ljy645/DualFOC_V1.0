"""VECTOR FOC — 单仪表双轴控制台。python main.py [--demo]。"""
from __future__ import annotations

import argparse
from collections import deque
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import queue
import sys
import time

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont, QTextCursor
from PySide6.QtWidgets import (QApplication, QButtonGroup, QCheckBox, QComboBox, QDoubleSpinBox,
    QFileDialog, QFrame, QGridLayout, QHBoxLayout, QHeaderView, QLabel, QLineEdit, QMainWindow,
    QPlainTextEdit, QProgressBar, QPushButton, QSizePolicy, QSplitter, QStackedWidget,
    QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget)

from link import Job, SerialLink, Telemetry, parameters, ports
from widgets import BG, PANEL, GRID, TEXT, MUTED, CYAN, AMBER, GREEN, RED, Gauge, Trend

ROOT = Path(__file__).resolve().parent
CSV_FIELDS = ["time", "axis", "enabled", "calibrated", "faults", "rpm", "target", "position", "bus", "id_a", "iq_a"]
MOTOR_PARAMS = [
    ("phase_resistance_ohm", "相电阻 · Ω", 1),
    ("ld_h", "Ld · µH", 1e-6),
    ("lq_h", "Lq · µH", 1e-6),
    ("flux_linkage_wb", "磁链 · mWb", .001),
    ("pole_pairs", "极对数", 1),
    ("current_limit_a", "电流上限 · A", 1),
]
TUNING_PARAMS = [
    ("speed_kp", "速度 Kp", 1),
    ("speed_ki", "速度 Ki", 1),
    ("cogging_gain", "齿槽补偿强度", 1),
]
PARAMS = MOTOR_PARAMS + TUNING_PARAMS


def text(value, role=None):
    widget = QLabel(value)
    if role:
        widget.setProperty("role", role)
    return widget


def button(caption, action, role=None):
    widget = QPushButton(caption)
    if role:
        widget.setProperty("role", role)
    widget.clicked.connect(action)
    widget.setCursor(Qt.CursorShape.PointingHandCursor)
    return widget


def card():
    frame = QFrame()
    frame.setObjectName("card")
    layout = QVBoxLayout(frame)
    layout.setContentsMargins(16, 12, 16, 12)
    layout.setSpacing(10)
    return frame, layout


def number(value, digits=1):
    return f"{value:,.{digits}f}" if math.isfinite(value) else "—"


class MotorPanel(QWidget):
    def __init__(self, app, axis):
        super().__init__()
        self.app, self.axis = app, axis
        self.latest = None
        self.readback = {}
        self.mode = "speed"
        self.target_edited = False
        self.targets = {"speed": 0.0, "position": 0.0}
        root = QHBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(14)
        left, right = QVBoxLayout(), QVBoxLayout()
        left.setSpacing(12)
        right.setSpacing(12)
        root.addLayout(left, 3)
        root.addLayout(right, 2)

        instrument, layout = card()
        head = QHBoxLayout()
        head.addWidget(text(f"MOTOR {axis}", "section"))
        head.addStretch()
        self.output_badge = text("●  输出未知", "muted")
        head.addWidget(self.output_badge)
        layout.addLayout(head)
        self.gauge = Gauge()
        self.gauge.targetChanged.connect(self.edit_target)
        self.gauge.targetReleased.connect(self.release_target)
        layout.addWidget(self.gauge, 1)
        legend = QHBoxLayout()
        legend.addStretch()
        actual_legend, target_legend = text("━ 实际反馈"), text("━ 目标 · 可拖动")
        actual_legend.setStyleSheet(f"color:{CYAN}")
        target_legend.setStyleSheet(f"color:{AMBER}")
        legend.addWidget(actual_legend)
        legend.addSpacing(24)
        legend.addWidget(target_legend)
        legend.addStretch()
        layout.addLayout(legend)
        metrics = QHBoxLayout()
        self.metrics = {}
        for key, title in (("bus", "母线电压"), ("iq", "Iq 电流"), ("cal", "校准状态")):
            cell = QVBoxLayout()
            cell.setSpacing(2)
            cell.addWidget(text(title, "muted"))
            value = text("—", "metric")
            cell.addWidget(value)
            metrics.addLayout(cell, 1)
            self.metrics[key] = value
        layout.addLayout(metrics)
        left.addWidget(instrument, 1)
        trace, trace_layout = card()
        trace_layout.setContentsMargins(12, 8, 12, 8)
        trace_layout.setSpacing(2)
        trace_layout.addWidget(text("转速趋势     蓝色 实际  /  橙色 目标", "muted"))
        self.trend = Trend()
        trace_layout.addWidget(self.trend)
        trace.setFixedHeight(105)
        left.addWidget(trace)

        controls, ctl = card()
        controls.setMinimumWidth(385)
        mode_row = QHBoxLayout()
        mode_row.addWidget(text("运动控制", "section"))
        mode_row.addStretch()
        self.mode_group = QButtonGroup(self)
        self.mode_buttons = {}
        for mode, caption in (("speed", "速度"), ("position", "位置 · 预留")):
            b = button(caption, lambda checked=False, m=mode: self.select_mode(m), "segment")
            b.setCheckable(True)
            b.setFixedHeight(30)
            self.mode_group.addButton(b)
            self.mode_buttons[mode] = b
            mode_row.addWidget(b)
        self.mode_buttons["speed"].setChecked(True)
        ctl.addLayout(mode_row)
        target_row = QHBoxLayout()
        self.target_input = QDoubleSpinBox()
        self.target_input.setRange(-3000, 3000)
        self.target_input.setDecimals(1)
        self.target_input.setSingleStep(10)
        self.target_input.setSuffix(" rpm")
        self.target_input.setMinimumHeight(36)
        self.target_input.valueChanged.connect(self.input_target)
        self.target_input.lineEdit().returnPressed.connect(self.send_target)
        target_row.addWidget(self.target_input, 1)
        self.send_button = button("发送速度", self.send_target, "primary")
        target_row.addWidget(self.send_button)
        ctl.addLayout(target_row)
        self.enable_button = button("等待设备状态", self.toggle_enable, "enable")
        self.enable_button.setMinimumHeight(35)
        ctl.addWidget(self.enable_button)
        self.notice = text("连接设备后读取实际状态", "muted")
        self.notice.setWordWrap(True)
        ctl.addWidget(self.notice)
        right.addWidget(controls)

        settings, cfg = card()
        cfg.setContentsMargins(12, 10, 12, 10)
        cfg.setSpacing(7)
        cfg_header = QHBoxLayout()
        cfg_header.addWidget(text("电机固有参数", "section"))
        cfg_header.addStretch()
        self.config_status = text("等待读取", "muted")
        cfg_header.addWidget(self.config_status)
        cfg.addLayout(cfg_header)
        grid = QGridLayout()
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(7)
        self.values = {}
        for i, (name, title, _) in enumerate(MOTOR_PARAMS):
            col, row = i % 3, i // 3
            cell = QVBoxLayout()
            cell.setSpacing(3)
            cell.addWidget(text(title, "muted"))
            entry = QLineEdit()
            entry.setMinimumHeight(29)
            entry.setPlaceholderText("—")
            entry.setMinimumWidth(70)
            cell.addWidget(entry)
            grid.addLayout(cell, row, col)
            grid.setColumnStretch(col, 1)
            self.values[name] = entry
        cfg.addLayout(grid)
        actions = QHBoxLayout()
        self.read_button = button("读取参数", self.read_params)
        self.save_button = button("保存电机参数", self.save_params, "primary")
        actions.addWidget(self.read_button)
        actions.addWidget(self.save_button)
        self.cal_button = button("完整校准", self.calibrate, "calibrate")
        actions.addWidget(self.cal_button)
        cfg.addLayout(actions)
        self.cog_status = text("补偿表未标定", "muted")
        cfg.addWidget(self.cog_status)
        right.addWidget(settings)
        tuning, tune_layout = card()
        tune_layout.setContentsMargins(12, 10, 12, 10)
        tune_layout.setSpacing(7)
        tune_header = QHBoxLayout()
        tune_header.addWidget(text("调试参数", "section"))
        tune_header.addStretch()
        self.tune_save_button = button("写入并保存", self.save_tuning, "primary")
        tune_header.addWidget(self.tune_save_button)
        tune_layout.addLayout(tune_header)
        tune_grid = QHBoxLayout()
        tune_grid.setSpacing(10)
        for name, title, _ in TUNING_PARAMS:
            cell = QVBoxLayout()
            cell.setSpacing(3)
            cell.addWidget(text(title, "muted"))
            entry = QLineEdit()
            entry.setPlaceholderText("—")
            entry.setMinimumWidth(60)
            entry.setMinimumHeight(29)
            cell.addWidget(entry)
            tune_grid.addLayout(cell, 1)
            self.values[name] = entry
        tune_layout.addLayout(tune_grid)
        self.tuning_status = text("可运行中写入，不清除校准；强度0关闭补偿。", "hint")
        self.tuning_status.setWordWrap(True)
        tune_layout.addWidget(self.tuning_status)
        right.addWidget(tuning)
        right.addStretch(1)
        self.refresh_controls()

    def select_mode(self, mode):
        if mode == self.mode:
            return
        self.targets[self.mode] = self.target_input.value()
        self.mode = mode
        position = mode == "position"
        self.gauge.set_mode(position)
        self.target_input.blockSignals(True)
        self.target_input.setRange(0, 360) if position else self.target_input.setRange(-3000, 3000)
        self.target_input.setSuffix(" °" if position else " rpm")
        self.target_input.setValue(self.targets[mode])
        self.target_input.blockSignals(False)
        self.gauge.set_values(actual=self.latest.position if position and self.latest else
                              self.latest.rpm if self.latest else math.nan, target=self.targets[mode])
        self.send_button.setText("接口预留" if position else "发送速度")
        self.refresh_controls()

    def input_target(self, value):
        self.targets[self.mode] = value
        if self.mode == "speed":
            self.target_edited = True
        self.gauge.set_values(target=value)

    def edit_target(self, value):
        self.targets[self.mode] = value
        self.target_input.blockSignals(True)
        self.target_input.setValue(value)
        self.target_input.blockSignals(False)
        if self.mode == "speed":
            self.target_edited = True

    def release_target(self, value):
        if self.mode == "position":
            self.app.message("角度已预览；位置控制接口尚未启用，未发送运动指令。")
        else:
            self.send_target()

    def send_target(self):
        if self.mode == "position":
            self.app.message("位置控制尚未实现，当前仅预览角度。")
            return
        value = self.target_input.value()
        if not self.app.fresh(self.axis) or not self.latest.enabled:
            self.app.message(f"M{self.axis} 未使能，目标保留在界面，未发送。", True)
            return
        self.app.submit(Job("设置转速", [f"speed {self.axis} {value:.9g}"], self.axis))

    def toggle_enable(self):
        if self.app.fresh(self.axis):
            want = 0 if self.latest.enabled else 1
            self.app.submit(Job("关闭输出" if not want else "使能输出", [f"enable {self.axis} {want}"], self.axis))

    def read_params(self):
        self.app.submit(Job("读取参数", [f"param show {self.axis}"], self.axis))

    def save_params(self):
        if not self.app.both_stopped():
            self.app.message("请先关闭两轴输出，等实际状态显示关闭后再保存。", True)
            return
        commands = []
        for name, title, scale in MOTOR_PARAMS:
            try:
                value = float(self.values[name].text())
                if not math.isfinite(value) or value < 0:
                    raise ValueError
                if name in ("phase_resistance_ohm", "ld_h", "lq_h", "current_limit_a", "pole_pairs") and value == 0:
                    raise ValueError
                if name == "pole_pairs" and (value != int(value) or value > 65535):
                    raise ValueError
                if name == "current_limit_a" and value > 10:
                    raise ValueError
            except ValueError:
                self.app.message(f"{title} 无效；电流须在 (0,10] A，极对数须为正整数。", True)
                self.values[name].setFocus()
                return
            commands.append(f"param set {self.axis} {name} {value*scale:.9g}")
        commands.extend((f"param save {self.axis}", f"param show {self.axis}"))
        self.app.submit(Job("发送并保存参数", commands, self.axis))

    def calibrate(self):
        if not self.app.both_stopped():
            self.app.message("校准要求两轴实际输出均关闭。", True)
            return
        if self.readback.get("dirty") == "1":
            self.app.message("设备有未保存参数，请先保存。", True)
            return
        self.app.submit(Job("完整校准", [f"calibrate {self.axis}", f"param show {self.axis}"], self.axis))

    def apply_params(self, values):
        self.readback = values
        for name, _, scale in PARAMS:
            if name in values:
                self.values[name].setText(f"{float(values[name])/scale:.7g}")
        saved, dirty = values.get("saved") == "1", values.get("dirty") == "1"
        self.config_status.setText("有未提交修改" if dirty else "已保存 EEPROM" if saved else "未保存 / 初值")
        self.config_status.setStyleSheet(f"color:{GREEN if saved and not dirty else AMBER}")
        self.cog_status.setText("补偿表已标定 · 1024 点" if values.get("cogging_valid") == "1" else "补偿表无效 / 未标定")
        self.tuning_status.setText(("调试参数已独立保存。" if values.get("tuning_saved") == "1" else "调试参数尚未独立保存。") +
                                  "可运行中写入，强度0关闭补偿。")

    def save_tuning(self):
        try:
            values = [float(self.values[name].text()) for name, _, _ in TUNING_PARAMS]
            if any(not math.isfinite(value) or value < 0 for value in values):
                raise ValueError
        except ValueError:
            self.app.message("速度 Kp、Ki 和补偿强度必须是非负有限数值。", True)
            return
        args = " ".join(f"{value:.9g}" for value in values)
        self.app.submit(Job("保存调试参数", [f"tune {self.axis} {args}", f"param show {self.axis}"], self.axis))

    def update_telemetry(self, sample):
        self.latest = sample
        if not self.target_edited and self.mode == "speed" and not self.target_input.hasFocus():
            self.target_input.blockSignals(True)
            self.target_input.setValue(sample.target)
            self.target_input.blockSignals(False)
            self.targets["speed"] = sample.target
        actual = sample.position if self.mode == "position" else sample.rpm
        self.gauge.set_values(actual=actual, target=self.targets[self.mode])
        self.output_badge.setText("●  输出已使能" if sample.enabled else "●  输出关闭")
        self.output_badge.setStyleSheet(f"color:{GREEN if sample.enabled else MUTED}")
        self.metrics["bus"].setText(number(sample.bus, 2)+" V")
        self.metrics["iq"].setText(number(sample.iq_a, 3)+" A")
        self.metrics["cal"].setText("已校准" if sample.calibrated else "未校准")
        self.metrics["cal"].setStyleSheet(f"color:{GREEN if sample.calibrated else AMBER}")

    def refresh_controls(self):
        fresh = self.app.fresh(self.axis)
        ready = self.app.ready and self.app.busy is None
        enabled = fresh and self.latest.enabled
        self.enable_button.setText("关闭输出" if enabled else "使能输出" if fresh else "等待设备状态")
        self.enable_button.setProperty("on", bool(enabled))
        self.enable_button.style().unpolish(self.enable_button)
        self.enable_button.style().polish(self.enable_button)
        self.enable_button.setEnabled(ready and fresh)
        self.send_button.setEnabled(ready and enabled and self.mode == "speed")
        self.read_button.setEnabled(ready)
        self.save_button.setEnabled(ready and self.app.both_stopped())
        self.cal_button.setEnabled(ready and self.app.both_stopped())
        self.cal_button.setToolTip("两轴都关闭输出后才能校准")
        self.tune_save_button.setEnabled(ready)
        if not fresh:
            self.gauge.set_values(actual=math.nan)
            self.output_badge.setText("●  输出未知")
            self.output_badge.setStyleSheet(f"color:{MUTED}")
            for metric in self.metrics.values():
                metric.setText("—")
            self.notice.setText("未连接或遥测已过期")
        elif self.mode == "position":
            self.notice.setText("位置模式为预留接口，仅预览，不发送角度控制。")
        elif self.latest.faults:
            self.notice.setText(f"反馈故障位 0x{self.latest.faults:08X}")
        else:
            self.notice.setText("拖动橙色指针，松开后发送；使能结果来自设备。")


class HostApp(QMainWindow):
    def __init__(self, demo=False):
        super().__init__()
        self.setWindowTitle("VECTOR FOC · 双轴电机控制台")
        self.resize(1280, 800)
        self.setMinimumSize(1060, 690)
        screen = QApplication.primaryScreen().availableGeometry()
        self.resize(min(1280, screen.width()-40), min(800, screen.height()-40))
        self.events = queue.Queue()
        self.link = None
        self.ready = False
        self.busy = None
        self.samples = [None, None]
        self.histories = [deque(maxlen=600), deque(maxlen=600)]
        self.rows = deque(maxlen=30000)
        self.session = self.writer = None
        self.last_flush = self.last_plot = 0
        self.last_table = [0, 0]
        try:
            self.preferences = json.loads((ROOT/"settings.json").read_text(encoding="utf-8"))
        except (OSError, ValueError):
            self.preferences = {}
        self._build(demo)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(40)

    def _build(self, demo):
        base = QWidget()
        self.setCentralWidget(base)
        root = QVBoxLayout(base)
        root.setContentsMargins(20, 14, 20, 10)
        root.setSpacing(12)
        header = QHBoxLayout()
        brand = text("VECTOR  /  FOC", "brand")
        header.addWidget(brand)
        header.addSpacing(16)
        header.addWidget(text("双轴电机控制台", "muted"))
        header.addStretch()
        self.status = text("●  未连接", "muted")
        header.addWidget(self.status)
        root.addLayout(header)
        nav = QHBoxLayout()
        nav.setSpacing(6)
        self.pages = QStackedWidget()
        self.nav_group = QButtonGroup(self)
        self.nav_buttons = []
        for i, caption in enumerate(("USB 连接", "MOTOR 0", "MOTOR 1")):
            b = button(caption, lambda checked=False, index=i: self.pages.setCurrentIndex(index), "navigation")
            b.setCheckable(True)
            b.setFixedSize(150, 38)
            self.nav_group.addButton(b)
            self.nav_buttons.append(b)
            nav.addWidget(b)
        nav.addStretch()
        nav.addWidget(text("USB CDC  ·  双轴独立控制", "hint"))
        self.nav_buttons[0].setChecked(True)
        root.addLayout(nav)
        root.addWidget(self.pages, 1)
        connection = QWidget()
        self.pages.addWidget(connection)
        self.motors = [MotorPanel(self, axis) for axis in (0, 1)]
        for panel in self.motors:
            self.pages.addWidget(panel)
        self._connection_ui(connection, demo)
        footer = QHBoxLayout()
        self.feedback = text("连接后只读取参数和状态，不自动使能。", "hint")
        self.feedback.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
        footer.addWidget(self.feedback, 1)
        self.progress = QProgressBar()
        self.progress.setFixedSize(100, 4)
        self.progress.setTextVisible(False)
        self.progress.hide()
        footer.addWidget(self.progress)
        root.addLayout(footer)

    def _connection_ui(self, page, demo):
        root = QVBoxLayout(page)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(12)
        settings, layout = card()
        row = QHBoxLayout()
        self.port, self.baud, self.bits, self.parity, self.stopbits, self.flow = [QComboBox() for _ in range(6)]
        self.baud.addItems(["115200", "230400", "460800", "921600"])
        self.baud.setCurrentText(str(self.preferences.get("baudrate", 115200)))
        self.bits.addItems(["8", "7"])
        self.parity.addItems(["N", "E", "O"])
        self.stopbits.addItems(["1", "1.5", "2"])
        self.flow.addItems(["无", "RTS/CTS", "XON/XOFF"])
        self.connection_inputs = [self.port, self.baud, self.bits, self.parity, self.stopbits, self.flow]
        for caption, widget, stretch in zip(("USB 串口", "波特率", "数据位", "校验", "停止位", "流控"), self.connection_inputs, (3, 2, 1, 1, 1, 2)):
            cell = QVBoxLayout()
            cell.setSpacing(4)
            cell.addWidget(text(caption, "muted"))
            cell.addWidget(widget)
            row.addLayout(cell, stretch)
        self.refresh_button = button("刷新", self.refresh_ports)
        self.connect_button = button("连接设备", self.connect, "primary")
        row.addWidget(self.refresh_button, 0, Qt.AlignmentFlag.AlignBottom)
        row.addWidget(self.connect_button, 0, Qt.AlignmentFlag.AlignBottom)
        layout.addLayout(row)
        sub = QHBoxLayout()
        self.demo_check = QCheckBox("离线演示 · 不连接硬件")
        self.demo_check.setChecked(demo)
        sub.addWidget(self.demo_check)
        sub.addStretch()
        sub.addWidget(text("断开连接不会改变电机输出；与 VOFA 轮流占用串口。", "hint"))
        layout.addLayout(sub)
        root.addWidget(settings)
        graph, graph_layout = card()
        graph_layout.setContentsMargins(12, 8, 12, 8)
        graph_layout.setSpacing(2)
        graph_layout.addWidget(text("双轴转速   /   蓝色 M0   ·   橙色 M1", "muted"))
        self.overview = Trend()
        graph_layout.addWidget(self.overview, 1)
        graph.setMaximumHeight(150)
        root.addWidget(graph, 1)
        toolbar = QHBoxLayout()
        toolbar.addWidget(text("历史采集与通信记录", "section"))
        toolbar.addStretch()
        toolbar.addWidget(button("打开历史 CSV", self.load_history))
        toolbar.addWidget(button("导出当前数据", self.export_history))
        root.addLayout(toolbar)
        split = QSplitter(Qt.Orientation.Horizontal)
        self.history = QTableWidget(0, 6)
        self.history.setHorizontalHeaderLabels(["时间", "轴", "实际 rpm", "目标 rpm", "角度 °", "输出"])
        self.history.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.history.verticalHeader().hide()
        self.history.verticalHeader().setDefaultSectionSize(26)
        self.history.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.history.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.history.setShowGrid(False)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.document().setMaximumBlockCount(500)
        self.log.setFont(QFont("Consolas", 9))
        split.addWidget(self.history)
        split.addWidget(self.log)
        split.setSizes([650, 450])
        root.addWidget(split, 3)
        manual = QHBoxLayout()
        self.manual = QLineEdit()
        self.manual.setPlaceholderText("手动指令，例如 enable 0 1 或 speed 0 100")
        self.manual.returnPressed.connect(self.send_manual)
        manual.addWidget(self.manual, 1)
        manual.addWidget(button("发送指令", self.send_manual))
        root.addLayout(manual)
        self.refresh_ports()
        for item in self.preferences.get("recent", [])[-3:]:
            self.log_line("INFO", "历史连接："+item)

    def refresh_ports(self):
        selected = self.port.currentText() or self.preferences.get("port", "")
        self.port.clear()
        for name, description in ports():
            self.port.addItem(name)
            self.port.setItemData(self.port.count()-1, description, Qt.ItemDataRole.ToolTipRole)
        index = self.port.findText(selected)
        if index >= 0:
            self.port.setCurrentIndex(index)

    def connect(self):
        if self.link is not None:
            self.link.stop()
            self.connect_button.setEnabled(False)
            return
        if not self.demo_check.isChecked() and not self.port.currentText():
            self.message("未发现串口，请连接设备后刷新。", True)
            return
        settings = dict(port=self.port.currentText(), baudrate=int(self.baud.currentText()),
            bytesize=int(self.bits.currentText()), parity=self.parity.currentText(), stopbits=float(self.stopbits.currentText()),
            rtscts=self.flow.currentText() == "RTS/CTS", xonxoff=self.flow.currentText() == "XON/XOFF")
        self.link = SerialLink(settings, self.events, self.demo_check.isChecked())
        self.link.start()
        self.connect_button.setText("断开连接")
        self.status.setText("●  正在握手")
        for widget in self.connection_inputs+[self.demo_check, self.refresh_button]:
            widget.setEnabled(False)

    def fresh(self, axis):
        sample = self.samples[axis]
        return self.ready and sample is not None and time.monotonic()-sample.received < 1.5

    def both_stopped(self):
        return all(self.fresh(axis) and not self.samples[axis].enabled for axis in (0, 1))

    def submit(self, job):
        if not self.ready or self.link is None:
            self.message("请先连接设备并完成 HOST 握手。", True)
            return False
        if self.busy is not None:
            self.message(f"正在{self.busy.label}，请等待应答。", True)
            return False
        self.busy = job
        self.progress.setRange(0, 0)
        self.progress.show()
        self.link.jobs.put(job)
        self.message(job.label+"…"+(" 校准会驱动电机定位和扫描。" if "校准" in job.label else ""))
        return True

    def send_manual(self):
        command = self.manual.text().strip()
        if not command:
            return
        if command.startswith(("host ", "config ")):
            self.message("连接期间保持 HOST 模式；使用 VOFA 请先断开本软件。", True)
            return
        if command.startswith("calibrate ") and not self.both_stopped():
            self.message("校准前须先关闭两轴输出。", True)
            return
        if command.startswith("cogging ") and not self.both_stopped():
            self.message("齿槽标定和补偿设置需要两轴关闭。", True)
            return
        if self.submit(Job("手动指令", [command])):
            self.manual.clear()

    def message(self, value, error=False):
        self.feedback.setText(value)
        self.feedback.setToolTip(value)
        self.feedback.setStyleSheet(f"color:{RED if error else MUTED}")
        if error:
            self.log_line("ERR", value)

    def log_line(self, kind, value):
        self.log.appendPlainText(f"{datetime.now():%H:%M:%S} {kind:4} {value}")
        self.log.moveCursor(QTextCursor.MoveOperation.End)

    def _connected(self, demo):
        self.ready = True
        self.status.setText("●  离线演示 · 非实机" if demo else f"●  {self.port.currentText()} 已连接")
        self.status.setStyleSheet(f"color:{AMBER if demo else GREEN}")
        self.rows.clear()
        for history in self.histories:
            history.clear()
        try:
            folder = ROOT/"history"
            folder.mkdir(exist_ok=True)
            self.session = (folder/f"{'DEMO_' if demo else ''}session_{datetime.now():%Y%m%d_%H%M%S_%f}.csv").open("w", newline="", encoding="utf-8-sig")
            self.writer = csv.DictWriter(self.session, CSV_FIELDS)
            self.writer.writeheader()
            recent = self.preferences.get("recent", [])
            recent.append(f"{datetime.now():%Y-%m-%d %H:%M} · {'DEMO' if demo else self.port.currentText()}")
            self.preferences.update(port=self.port.currentText(), baudrate=int(self.baud.currentText()), recent=recent[-10:])
            (ROOT/"settings.json").write_text(json.dumps(self.preferences, ensure_ascii=False, indent=2), encoding="utf-8")
        except OSError as error:
            self.message("无法保存本地历史："+str(error), True)
        self.submit(Job("读取两轴参数", ["param show 0", "param show 1"]))

    def _add_row(self, values):
        row = self.history.rowCount()
        self.history.insertRow(row)
        for col, value in enumerate(values):
            self.history.setItem(row, col, QTableWidgetItem(str(value)))
        if self.history.rowCount() > 250:
            self.history.removeRow(0)
        self.history.scrollToBottom()

    def _sample(self, sample):
        axis = sample.axis
        self.samples[axis] = sample
        self.motors[axis].latest = sample
        self.histories[axis].append(sample)
        row = {key: getattr(sample, key) for key in CSV_FIELDS if key != "time"}
        row["time"] = datetime.now().isoformat(timespec="milliseconds")
        self.rows.append(row)
        if self.writer:
            self.writer.writerow(row)
        if time.monotonic()-self.last_table[axis] > .2:
            self.last_table[axis] = time.monotonic()
            self._add_row((row["time"][11:], f"M{axis}", number(sample.rpm), number(sample.target),
                           number(sample.position), "ON" if sample.enabled else "OFF"))

    def _done(self, job, success, replies):
        self.busy = None
        self.progress.hide()
        for panel in self.motors:
            panel.cal_button.setText("完整校准")
        for command, lines in replies:
            if command.startswith("speed ") and lines and lines[0].startswith("OK"):
                args = command.split()
                self.motors[int(args[1]) if len(args) == 3 else 0].target_edited = False
            if command.startswith("param show ") and lines and lines[-1] == "END":
                values = parameters(lines)
                self.motors[int(values["axis"])].apply_params(values)
        if success:
            self.message(job.label+"完成 · 实际状态以设备反馈为准")
        else:
            error = replies[-1][1][0]
            descriptions = {"UNSAVED_CHANGES": "设备有未保存修改", "UNCALIBRATED": "请先执行校准",
                "FULL_CALIBRATION_UNSUPPORTED": "固件只完成了旧版单阶段校准，请烧录配套新固件",
                "UNSAVED": "请先保存参数", "EEPROM_IO": "EEPROM 通信失败", "STOP_BOTH_AXES": "请关闭两轴输出",
                "COGGING_CALIBRATION 1": "齿槽标定输出未启动", "COGGING_CALIBRATION 2": "齿槽标定反馈中断",
                "COGGING_CALIBRATION 3": "齿槽扫描超时", "COGGING_CALIBRATION 4": "齿槽采样未覆盖全表",
                "COGGING_CALIBRATION 5": "标定电流达到限幅", "COGGING_MAP_MISSING": "请先完成齿槽标定",
                "CALIBRATION 2": "校准要求两轴停止", "CALIBRATION 3": "请先保存参数或检查就绪状态"}
            explanation = next((value for key, value in descriptions.items() if key in error), "设备拒绝操作")
            if error.startswith("ERR CALIBRATION_ZERO"):
                explanation = "找零/方向阶段失败，未执行齿槽标定"
            elif error.startswith("ERR CALIBRATION_COGGING_STORAGE"):
                explanation = "齿槽阶段EEPROM保存失败，请读取参数确认状态"
            elif error.startswith("ERR CALIBRATION_COGGING"):
                explanation = "齿槽标定失败，完整校准未保存，请重新校准"
                if error == "ERR CALIBRATION_COGGING 3":
                    explanation = "齿槽等待转速稳定或三圈扫描超时，完整校准未保存"
            if job.axis is not None and job.commands[0].startswith(("cogging calibrate ", "calibrate ")):
                self.motors[job.axis].cog_status.setText("校准未全部完成 · 请读取参数确认状态")
            self.message(f"{job.label}失败：{explanation} [{error}]", True)

    def _disconnected(self):
        self.ready = False
        self.link = self.busy = None
        self.samples = [None, None]
        for panel in self.motors:
            panel.latest = None
            panel.cal_button.setText("完整校准")
        self.progress.hide()
        self.status.setText("●  未连接")
        self.status.setStyleSheet(f"color:{MUTED}")
        self.connect_button.setText("连接设备")
        self.connect_button.setEnabled(True)
        for widget in self.connection_inputs+[self.demo_check, self.refresh_button]:
            widget.setEnabled(True)
        if self.session:
            self.session.close()
            self.session = self.writer = None
        self.log_line("INFO", "连接关闭，未自动改变电机输出")

    def poll(self):
        try:
            while True:
                event = self.events.get_nowait()
                if event[0] == "connected":
                    self._connected(event[1])
                elif event[0] == "telemetry":
                    self._sample(event[1])
                elif event[0] == "log":
                    self.log_line(event[1], event[2])
                elif event[0] == "done":
                    self._done(*event[1:])
                elif event[0] == "phase" and self.busy is not None:
                    phase = "1/2 找零与方向校准" if event[1] == "ZERO" else "2/2 齿槽标定"
                    self.message(f"{phase}进行中 · 电机须空载、可自由转动")
                    if self.busy.axis is not None:
                        self.motors[self.busy.axis].cal_button.setText("1/2 找零中…" if event[1] == "ZERO" else "2/2 齿槽标定…")
                elif event[0] == "error":
                    self.message(event[1], True)
                elif event[0] == "disconnected":
                    self._disconnected()
        except queue.Empty:
            pass
        now = time.monotonic()
        if now-self.last_plot > .08:
            self.last_plot = now
            for axis, panel in enumerate(self.motors):
                if self.fresh(axis):
                    panel.update_telemetry(self.samples[axis])
                panel.refresh_controls()
                history = list(self.histories[axis])[-250:]
                panel.trend.set_series([([s.rpm for s in history], CYAN), ([s.target for s in history], AMBER)])
            self.overview.set_series([([s.rpm for s in list(h)[-250:]], color) for h, color in zip(self.histories, (CYAN, AMBER))])
        if self.session and now-self.last_flush > 1:
            self.session.flush()
            self.last_flush = now

    def export_history(self):
        path, _ = QFileDialog.getSaveFileName(self, "导出采集数据", str(ROOT/"history"/"export.csv"), "CSV (*.csv)")
        if path:
            try:
                with open(path, "w", newline="", encoding="utf-8-sig") as stream:
                    writer = csv.DictWriter(stream, CSV_FIELDS)
                    writer.writeheader()
                    writer.writerows(self.rows)
                self.message(f"已导出 {len(self.rows)} 条记录")
            except OSError as error:
                self.message(str(error), True)

    def load_history(self):
        path, _ = QFileDialog.getOpenFileName(self, "查看历史数据", str(ROOT/"history"), "CSV (*.csv)")
        if not path:
            return
        try:
            with open(path, encoding="utf-8-sig", newline="") as stream:
                reader = csv.DictReader(stream)
                if not set(CSV_FIELDS).issubset(reader.fieldnames or []):
                    raise ValueError("CSV 格式不匹配")
                records = list(deque(reader, maxlen=250))
            self.history.setRowCount(0)
            for row in records:
                self._add_row((row["time"], "M"+row["axis"], row["rpm"], row["target"], row["position"], row["enabled"]))
            self.message(f"已载入 {Path(path).name}，显示最后 {len(records)} 条记录")
        except (OSError, ValueError) as error:
            self.message(str(error), True)

    def closeEvent(self, event):
        self.timer.stop()
        if self.link:
            self.link.stop()
            self.link.join(timeout=.8)
        if self.session:
            self.session.close()
        event.accept()


STYLE = """
QWidget { background:#101419; color:#edf2f8; font-family:'Segoe UI','Microsoft YaHei UI'; font-size:12px; }
QFrame#card { background:#191f27; border:1px solid #2a3441; border-radius:10px; }
QFrame#card QLabel { background:transparent; }
QLabel[role='brand'] { font-size:22px; font-weight:700; letter-spacing:2px; }
QLabel[role='section'] { font-size:14px; font-weight:600; }
QLabel[role='metric'] { font-size:19px; font-weight:600; }
QLabel[role='muted'] { color:#8795a8; font-size:11px; }
QLabel[role='hint'] { color:#718095; font-size:10px; }
QPushButton { background:#293441; border:1px solid #364353; border-radius:5px; padding:7px 13px; font-weight:500; }
QPushButton:hover { background:#344453; border-color:#617b95; }
QPushButton:pressed { background:#1e2a35; }
QPushButton:disabled { background:#202730; color:#546173; border-color:#2a3441; }
QPushButton[role='primary'] { background:#358caa; color:white; border-color:#358caa; }
QPushButton[role='primary']:hover { background:#42a2c2; }
QPushButton[role='navigation'] { border:1px solid transparent; border-radius:6px; background:transparent; color:#8795a8; padding:0; font-weight:600; }
QPushButton[role='navigation']:hover { background:#202a35; }
QPushButton[role='navigation']:checked { background:#233c4b; color:#76d6f3; border:1px solid #3b6a80; }
QPushButton[role='segment'] { padding:4px 10px; background:#121920; color:#8795a8; }
QPushButton[role='segment']:checked { background:#2b4352; border-color:#47697e; color:#edf2f8; }
QPushButton[role='enable'] { background:#243d36; border-color:#3f6859; color:#8bd9b6; }
QPushButton[role='enable'][on='true'] { background:#4b2d37; border-color:#81505e; color:#ffb0b8; }
QPushButton[role='calibrate'] { background:#3d3426; border-color:#6c5838; color:#edc78d; }
QPushButton[role='primary']:disabled, QPushButton[role='enable']:disabled, QPushButton[role='calibrate']:disabled { background:#202730; border-color:#2a3441; color:#546173; }
QLineEdit, QDoubleSpinBox, QComboBox { background:#11181f; border:1px solid #354151; border-radius:5px; padding:5px 8px; min-height:18px; selection-background-color:#35566c; }
QLineEdit:focus, QDoubleSpinBox:focus { border-color:#49c5ed; }
QComboBox::drop-down { border:0; width:20px; }
QComboBox QAbstractItemView { background:#202a35; selection-background-color:#35566c; }
QTableWidget, QPlainTextEdit { background:#151c24; border:1px solid #2a3441; border-radius:7px; padding:3px; }
QHeaderView::section { background:#222d39; color:#8795a8; border:0; padding:7px; }
QTableWidget::item:selected { background:#284757; }
QScrollBar:vertical { width:7px; background:#151c24; }
QScrollBar::handle:vertical { background:#3b4b5e; border-radius:3px; min-height:20px; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height:0; }
QProgressBar { background:#233140; border:0; border-radius:2px; }
QProgressBar::chunk { background:#49c5ed; }
QToolTip { background:#263547; color:#edf2f8; border:1px solid #40546b; padding:5px; }
"""


def main():
    parser = argparse.ArgumentParser(description="VECTOR FOC 双轴电机控制台")
    parser.add_argument("--demo", action="store_true", help="离线演示，不打开实际串口")
    args = parser.parse_args()
    app = QApplication(sys.argv[:1])
    app.setStyle("Fusion")
    app.setFont(QFont("Microsoft YaHei UI", 9))
    app.setStyleSheet(STYLE)
    window = HostApp(demo=args.demo)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
