"""
Professional Mission Control - OpenMCT Style
Multi-view interface with sidebar navigation matching NASA/ESA mission control standards
"""

import logging
import time
import math
import shutil
from pathlib import Path
from typing import Dict, List, Optional
from dataclasses import dataclass

import numpy as np
import pyqtgraph as pg
from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QComboBox, QSplitter, QScrollArea,
    QFrame, QGroupBox, QFileDialog, QMessageBox, QTableWidget,
    QTableWidgetItem, QTextEdit, QCheckBox, QHeaderView, QStackedWidget,
    QListWidget, QListWidgetItem, QStatusBar, QMenuBar, QMenu,
    QSpinBox, QDoubleSpinBox, QSlider, QButtonGroup, QProgressBar, QSizePolicy,
    QStylePainter, QStyleOptionComboBox, QStyle,
)
from PyQt6.QtCore import Qt, QTimer, QTime, QSize
from PyQt6.QtGui import QFont, QColor, QIcon, QPainter, QPen, QLinearGradient, QGuiApplication

from config import APP_NAME, APP_VERSION, PLOT_UPDATE_INTERVAL
from core.telemetry import TelemetryDecoder, SignalMetadata, FrameInfo
from core.serial_handler import SerialHandler
from core.data_manager import DataManager
from core.stream_demux import SerialStreamDemux
from ui.responsive import ResponsiveScaling
from ui.themes.mission_control import Colors, MissionControlTheme

logger = logging.getLogger(__name__)


class ElidedComboBox(QComboBox):
    """ComboBox that elides long text (keeps right side, good for /dev/cu.* paths)."""

    def paintEvent(self, event):  # noqa: N802 (Qt override)
        painter = QStylePainter(self)
        option = QStyleOptionComboBox()
        self.initStyleOption(option)

        edit_rect = self.style().subControlRect(
            QStyle.ComplexControl.CC_ComboBox,
            option,
            QStyle.SubControl.SC_ComboBoxEditField,
            self,
        )
        fm = option.fontMetrics
        option.currentText = fm.elidedText(
            option.currentText,
            Qt.TextElideMode.ElideLeft,
            max(0, edit_rect.width()),
        )

        painter.drawComplexControl(QStyle.ComplexControl.CC_ComboBox, option)
        painter.drawControl(QStyle.ControlElement.CE_ComboBoxLabel, option)


@dataclass
class RfMetaFrame:
    """Decoded RF meta frame (0xD3 0xD4 ... format from firmware)"""
    rssi_pkt_dbm: int
    rssi_inst_dbm: int
    snr_db: float
    fei_hz: int
    offset_hz: int
    bw_bits: int
    sf: int
    cr: int
    rx_nb_bytes: int
    modem_stat: int
    irq_flags: int
    rx_header_cnt: int
    rx_packet_cnt: int
    link_score: int
    seq: int
    met: int
    flags: int
    payload_len: int
    crc_ok: bool


class RfMetaDecoder:
    """Parses RF meta frames appended on serial after each LoRa payload"""
    def __init__(self):
        self._buf = bytearray()
        self.marker_hits = 0
        self.frames_parsed = 0
        self.frames_crc_bad = 0

    def push_data(self, data: bytes) -> List[RfMetaFrame]:
        """Push new data and return list of parsed frames"""
        self._buf.extend(data)
        frames = []

        while len(self._buf) >= 28:
            # Look for sync marker 0xD3 0xD4
            if len(self._buf) < 2 or self._buf[0] != 0xD3 or self._buf[1] != 0xD4:
                self._buf.pop(0)
                continue

            self.marker_hits += 1

            if len(self._buf) < 28:
                break

            frame_data = self._buf[:28]

            # Parse frame
            ver = frame_data[2]
            total_len = frame_data[3]
            payload_len = frame_data[4]
            flags = frame_data[5]
            rssi_pkt = int.from_bytes([frame_data[6]], signed=True)
            rssi_inst = int.from_bytes([frame_data[7]], signed=True)
            snr_qdb = int.from_bytes([frame_data[8]], signed=True)
            snr_db = snr_qdb / 4.0
            fei_hz = int.from_bytes(frame_data[9:11], 'little', signed=True)
            offset_hz = int.from_bytes(frame_data[11:13], 'little', signed=True)
            bw_bits = frame_data[13]
            sf = frame_data[14]
            cr = frame_data[15]
            rx_nb_bytes = frame_data[16]
            modem_stat = frame_data[17]
            irq_flags = frame_data[18]
            rx_header_cnt = int.from_bytes(frame_data[19:21], 'little')
            rx_packet_cnt = int.from_bytes(frame_data[21:23], 'little')
            link_score = frame_data[23]
            seq = frame_data[24]
            met = int.from_bytes(frame_data[25:27], 'little')

            # CRC check
            crc_calculated = 0
            for b in frame_data[2:27]:
                crc_calculated ^= b
            crc_received = frame_data[27]
            crc_ok = (crc_calculated == crc_received)

            if not crc_ok:
                self.frames_crc_bad += 1

            self.frames_parsed += 1

            frames.append(RfMetaFrame(
                rssi_pkt_dbm=rssi_pkt,
                rssi_inst_dbm=rssi_inst,
                snr_db=snr_db,
                fei_hz=fei_hz,
                offset_hz=offset_hz,
                bw_bits=bw_bits,
                sf=sf,
                cr=cr,
                rx_nb_bytes=rx_nb_bytes,
                modem_stat=modem_stat,
                irq_flags=irq_flags,
                rx_header_cnt=rx_header_cnt,
                rx_packet_cnt=rx_packet_cnt,
                link_score=link_score,
                seq=seq,
                met=met,
                flags=flags,
                payload_len=payload_len,
                crc_ok=crc_ok,
            ))

            self._buf = self._buf[28:]

        return frames

    def reset(self):
        """Reset decoder state"""
        self._buf.clear()
        self.marker_hits = 0
        self.frames_parsed = 0
        self.frames_crc_bad = 0


class LiveBarWidget(QWidget):
    """
    Vertical bar with smoothing for slow-updating metrics (looks stable between meta frames).
    Supports zone-based coloring to match the gauge style.
    """
    def __init__(self, title: str, unit: str, min_val: float, max_val: float,
                 warn_threshold: float = None, crit_threshold: float = None,
                 lower_is_worse: bool = True, zones: List[tuple] | None = None):
        super().__init__()
        self.title = title
        self.unit = unit
        self.min_val = min_val
        self.max_val = max_val
        self.warn_threshold = warn_threshold
        self.crit_threshold = crit_threshold
        self.lower_is_worse = lower_is_worse
        self.zones = zones

        self.current_display = min_val
        self.target_value = min_val

        self._init_ui()

        self.smooth_timer = QTimer(self)
        self.smooth_timer.setInterval(50)  # 20 Hz smoothing
        self.smooth_timer.timeout.connect(self._tick)

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(4)

        title_label = QLabel(self.title)
        title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title_label.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        title_label.setStyleSheet(f"color: {Colors.TEXT_PRIMARY};")
        layout.addWidget(title_label)

        self.bar = QProgressBar()
        self.bar.setOrientation(Qt.Orientation.Vertical)
        self.bar.setRange(0, 1000)  # 0..100%
        self.bar.setValue(0)
        self.bar.setTextVisible(False)
        self.bar.setMinimumHeight(160)
        self.bar.setStyleSheet(f"""
            QProgressBar {{
                border: 1px solid #3A4652;
                border-radius: 4px;
                background: {Colors.BG_DARK};
            }}
            QProgressBar::chunk {{
                background: {Colors.STATUS_NOMINAL};
            }}
        """)
        layout.addWidget(self.bar, alignment=Qt.AlignmentFlag.AlignHCenter)

        self.value_label = QLabel(f"{self.current_display:.1f} {self.unit}")
        self.value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.value_label.setFont(QFont("Menlo", 12, QFont.Weight.Bold))
        self.value_label.setStyleSheet(f"color: {Colors.TEXT_PRIMARY};")
        layout.addWidget(self.value_label)

    def _norm(self, v: float) -> float:
        if self.max_val == self.min_val:
            return 0.0
        return max(0.0, min(1.0, (v - self.min_val) / (self.max_val - self.min_val)))

    def set_value(self, value: float):
        self.target_value = max(self.min_val, min(self.max_val, value))
        if not self.smooth_timer.isActive():
            self.smooth_timer.start()

    def _tick(self):
        # simple exponential smoothing toward target
        alpha = 0.2
        self.current_display += (self.target_value - self.current_display) * alpha

        # Stop timer when close enough
        if abs(self.target_value - self.current_display) < 0.01:
            self.current_display = self.target_value
            self.smooth_timer.stop()

        self._render()


class SignalBarWidget(QWidget):
    """
    Gauge-like vertical bar in a fixed black panel with zone coloring and a thin indicator stripe.
    """
    class Canvas(QWidget):
        def __init__(self, parent=None):
            super().__init__(parent)
            self.zones: List[tuple] = []
            self.norm_value = 0.0  # 0..1 (current)
            self.peak_norm_value = 0.0  # 0..1 (peak-hold display)
            self.stripe_color = "#ffffff"
            self.setMinimumSize(96, 110)

        def set_zones(self, zones: List[tuple]):
            self.zones = zones
            self.update()

        def set_norm_value(self, norm: float):
            self.norm_value = max(0.0, min(1.0, norm))
            self.update()

        def set_peak_norm_value(self, norm: float):
            self.peak_norm_value = max(0.0, min(1.0, norm))
            self.update()

        def set_stripe_color(self, color: str):
            self.stripe_color = color
            self.update()

        def paintEvent(self, event):
            painter = QPainter(self)
            try:
                painter.setRenderHint(QPainter.RenderHint.Antialiasing)
                w = self.width()
                h = self.height()
                # Background
                painter.fillRect(0, 0, w, h, QColor("#0d1016"))

                # Centered thick grey "track" (same visual weight as gauge grey arc),
                # square ends, no borders, no rounding.
                track_w = max(12, min(40, int(round(w * 0.18))))
                track_x = int(round((w - track_w) / 2))
                pad_y = max(6, int(round(h * 0.06)))
                track_top = pad_y
                track_bot = h - pad_y
                track_h = max(1, track_bot - track_top)
                painter.fillRect(track_x, track_top, track_w, track_h, QColor("#555555"))

                # BAR fill: single solid color based on the current value's zone.
                # As the value drops, the fill "moves down" and takes the zone color (green -> yellow -> red).
                fill_top = track_bot - int(round(track_h * self.norm_value))
                fill_top = max(track_top, min(track_bot, fill_top))
                if fill_top < track_bot:
                    painter.fillRect(track_x, fill_top, track_w, track_bot - fill_top, QColor(self.stripe_color))

                # Peak-hold marker: fixed high-contrast color independent of zones.
                peak_y = track_bot - int(round(track_h * self.peak_norm_value))
                peak_y = max(track_top, min(track_bot, peak_y))
                marker_h = 4
                marker_y0 = max(track_top, min(track_bot - marker_h, peak_y - marker_h // 2))
                painter.fillRect(track_x, marker_y0, track_w, marker_h, QColor("#f0f0f0"))
            finally:
                painter.end()

    def __init__(self, title: str, unit: str, min_val: float, max_val: float, zones: List[tuple]):
        super().__init__()
        self.title = title
        self.unit = unit
        self.min_val = min_val
        self.max_val = max_val
        self.zones_config = zones  # list of (start, end, color) in absolute units
        self.value = min_val
        self.current_display = min_val
        self.target_value = min_val
        self.peak_hold_value = min_val
        self.peak_hold_display = min_val
        self._peak_hold_until_s = 0.0

        self.smooth_timer = QTimer(self)
        self.smooth_timer.setInterval(50)  # 20 Hz
        self.smooth_timer.timeout.connect(self._tick)

        self.setFixedWidth(120)
        self.setFixedHeight(180)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)

        title_label = QLabel(self.title)
        title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title_label.setFont(QFont("Arial", 11, QFont.Weight.Bold))
        title_label.setStyleSheet("color: #FFFFFF; background: #000000; padding: 2px;")
        title_label.setFixedHeight(22)
        layout.addWidget(title_label)

        panel = QFrame()
        panel.setStyleSheet("background: #0d1016; border: none;")
        panel.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        panel_layout = QVBoxLayout(panel)
        panel_layout.setContentsMargins(0, 0, 0, 0)
        self.canvas = self.Canvas()
        self.canvas.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        panel_layout.addWidget(self.canvas)
        layout.addWidget(panel)

        self.status_label = QLabel("")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.status_label.setFont(QFont("Arial", 11, QFont.Weight.Bold))
        self.status_label.setStyleSheet("color: #FFFFFF; background: #000000; padding: 2px;")
        self.status_label.setFixedHeight(18)
        self.status_label.setVisible(False)
        layout.addWidget(self.status_label)

        self.value_label = QLabel(f"{self.value:.1f}")
        self.value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.value_label.setFont(QFont("Menlo", 12, QFont.Weight.Bold))
        self.value_label.setStyleSheet("color: #FFFFFF; background: #000000; padding: 2px;")
        self.value_label.setFixedHeight(24)
        layout.addWidget(self.value_label)

        self.unit_label = QLabel(self.unit)
        self.unit_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.unit_label.setFont(QFont("Arial", 9))
        self.unit_label.setStyleSheet("color: #BBBBBB; background: #000000; padding: 2px;")
        self.unit_label.setFixedHeight(18)
        layout.addWidget(self.unit_label)

        # prepare normalized zones
        self._norm_zones = []
        self._prepare_zones()
        self._render()

    def _tick(self):
        # Smooth current value
        alpha = 0.22
        self.current_display += (self.target_value - self.current_display) * alpha

        # Peak-hold behavior: hold the maximum, then decay toward current
        now_s = time.monotonic()
        if now_s < self._peak_hold_until_s:
            self.peak_hold_display = self.peak_hold_value
        else:
            # decay smoothly but never below current display
            decay_alpha = 0.12
            self.peak_hold_display += (self.current_display - self.peak_hold_display) * decay_alpha
            if self.peak_hold_display < self.current_display:
                self.peak_hold_display = self.current_display

        # Stop timer when stable and not in hold
        if (abs(self.target_value - self.current_display) < 0.01 and
                abs(self.peak_hold_display - self.current_display) < 0.01 and
                now_s >= self._peak_hold_until_s):
            self.current_display = self.target_value
            self.peak_hold_display = self.current_display
            self.smooth_timer.stop()

        self._render()

    def _prepare_zones(self):
        span = self.max_val - self.min_val if self.max_val != self.min_val else 1.0
        self._norm_zones = []
        for a, b, c in self.zones_config:
            n0 = (a - self.min_val) / span
            n1 = (b - self.min_val) / span
            self._norm_zones.append((n0, n1, c))
        self.canvas.set_zones(self._norm_zones)

    def set_value(self, value: float, status_text: str = ""):
        self.value = max(self.min_val, min(self.max_val, value))
        self.target_value = self.value

        # Peak-hold (keep the best value visible)
        if self.value >= self.peak_hold_value:
            self.peak_hold_value = self.value
            self.peak_hold_display = self.value
            self._peak_hold_until_s = time.monotonic() + 1.5

        self.status_label.setText(status_text)
        self.status_label.setVisible(bool(status_text))
        if not self.smooth_timer.isActive():
            self.smooth_timer.start()
        self._render()

    def _render(self):
        span = self.max_val - self.min_val if self.max_val != self.min_val else 1.0
        norm = self._norm(self.current_display, span)
        peak_norm = self._norm(self.peak_hold_display, span)
        self.canvas.set_norm_value(norm)
        self.canvas.set_peak_norm_value(peak_norm)
        self.value_label.setText(f"{self.current_display:.1f}")
        # pick zone color for value text/status
        color = "#FFFFFF"
        for n0, n1, c in self._norm_zones:
            if n0 <= norm <= n1:
                color = c
                break
        self.value_label.setStyleSheet(f"color: {color}; background: #000000; padding: 2px;")
        self.status_label.setStyleSheet(f"color: {color}; background: #000000; padding: 2px;")
        self.canvas.set_stripe_color(color)

    def _norm(self, v: float, span: float | None = None) -> float:
        if span is None:
            span = self.max_val - self.min_val if self.max_val != self.min_val else 1.0
        return max(0.0, min(1.0, (v - self.min_val) / span))


class LiveCenterBarWidget(QWidget):
    """
    Horizontal signed bar for centered metrics (e.g., FEI), shows zero marker explicitly.
    """
    def __init__(self, title: str, unit: str, min_val: float, max_val: float,
                 warn_threshold: float = None, crit_threshold: float = None,
                 lower_is_worse: bool = False):
        super().__init__()
        self.title = title
        self.unit = unit
        self.min_val = min_val
        self.max_val = max_val
        self.warn_threshold = warn_threshold
        self.crit_threshold = crit_threshold
        self.lower_is_worse = lower_is_worse
        self.current_display = 0.0

        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(2)

        title_label = QLabel(self.title)
        title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title_label.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        title_label.setStyleSheet(f"color: {Colors.TEXT_PRIMARY};")
        layout.addWidget(title_label)

        self.bar = QProgressBar()
        self.bar.setOrientation(Qt.Orientation.Horizontal)
        self.bar.setRange(int(self.min_val), int(self.max_val))
        self.bar.setValue(0)
        self.bar.setTextVisible(False)
        self.bar.setMinimumWidth(200)
        self.bar.setStyleSheet(f"""
            QProgressBar {{
                border: 1px solid #444;
                border-radius: 2px;
                background: {Colors.BG_PRIMARY};
            }}
            QProgressBar::chunk {{
                background: {Colors.STATUS_NOMINAL};
            }}
        """)
        layout.addWidget(self.bar)

        # Zero marker row
        zero_row = QHBoxLayout()
        zero_row.addStretch()
        zero_lbl = QLabel("0")
        zero_lbl.setStyleSheet(f"color: {Colors.TEXT_SECONDARY}; font-size: 8pt;")
        zero_row.addWidget(zero_lbl)
        zero_row.addStretch()
        layout.addLayout(zero_row)

        self.value_label = QLabel(f"{self.current_display:.1f} {self.unit}")
        self.value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.value_label.setFont(QFont("Menlo", 12, QFont.Weight.Bold))
        self.value_label.setStyleSheet(f"color: {Colors.TEXT_PRIMARY};")
        layout.addWidget(self.value_label)

    def set_value(self, value: float):
        v = max(self.min_val, min(self.max_val, value))
        self.current_display = v
        self.bar.setValue(int(v))
        self.value_label.setText(f"{v:.1f} {self.unit}")

        # Color thresholds
        is_warning = is_critical = False
        if self.lower_is_worse:
            if self.crit_threshold is not None and v <= self.crit_threshold:
                is_critical = True
            elif self.warn_threshold is not None and v <= self.warn_threshold:
                is_warning = True
        else:
            if self.crit_threshold is not None and v >= self.crit_threshold:
                is_critical = True
            elif self.warn_threshold is not None and v >= self.warn_threshold:
                is_warning = True

        color = Colors.STATUS_NOMINAL
        if is_critical:
            color = Colors.STATUS_CRITICAL
        elif is_warning:
            color = Colors.STATUS_WARNING

        self.bar.setStyleSheet(f"""
            QProgressBar {{
                border: 1px solid #444;
                border-radius: 2px;
                background: {Colors.BG_PRIMARY};
            }}
            QProgressBar::chunk {{
                background: {color};
            }}
        """)

class LiveGaugeWidget(QWidget):
    """Professional needle gauge with color zones (Open Mission Control style)"""

    def __init__(self, title: str, unit: str, min_val: float, max_val: float,
                 warn_threshold: float = None, crit_threshold: float = None,
                 lower_is_worse: bool = True,
                 zones: List[tuple] | None = None):
        super().__init__()
        self.title = title
        self.unit = unit
        self.min_val = min_val
        self.max_val = max_val
        self.warn_threshold = warn_threshold
        self.crit_threshold = crit_threshold
        self.lower_is_worse = lower_is_worse
        self.zones = zones  # optional list of (start, end, color)
        self.current_value = min_val
        self.target_value = min_val

        self.setMinimumSize(180, 200)
        self.setMaximumSize(220, 240)
        self._init_ui()

        # Smooth needle movement (similar to bar widget)
        self.smooth_timer = QTimer(self)
        self.smooth_timer.setInterval(50)  # 20 Hz
        self.smooth_timer.timeout.connect(self._tick)

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(2)

        # Title
        title_label = QLabel(self.title)
        title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title_label.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        title_label.setStyleSheet(f"color: {Colors.TEXT_PRIMARY};")
        layout.addWidget(title_label)

        # Gauge plot
        self.gauge_plot = pg.PlotWidget()
        self.gauge_plot.setAspectLocked(True)
        self.gauge_plot.hideAxis('left')
        self.gauge_plot.hideAxis('bottom')
        self.gauge_plot.setMouseEnabled(x=False, y=False)
        self.gauge_plot.setMenuEnabled(False)
        self.gauge_plot.setBackground(Colors.BG_PRIMARY)
        self.gauge_plot.setXRange(-1.4, 1.4)
        self.gauge_plot.setYRange(-0.15, 1.15)
        layout.addWidget(self.gauge_plot, 1)

        # Status text (zone-based) – above value
        self.warning_label = QLabel("")
        self.warning_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.warning_label.setFont(QFont("Arial", 12, QFont.Weight.Bold))
        self.warning_label.setVisible(False)
        self.warning_label.setFixedHeight(18)
        self.warning_label.setStyleSheet("color: #F7C948;")
        layout.addWidget(self.warning_label)

        # Value label (centered in gauge)
        self.value_label = QLabel(f"{self.current_value:.1f}")
        self.value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.value_label.setFont(QFont("Menlo", 18, QFont.Weight.Bold))
        self.value_label.setStyleSheet(f"color: {Colors.TEXT_PRIMARY}; background: #000000; padding: 2px;")
        layout.addWidget(self.value_label)

        # Unit label
        unit_label = QLabel(self.unit)
        unit_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        unit_label.setFont(QFont("Arial", 9))
        unit_label.setStyleSheet(f"color: {Colors.TEXT_SECONDARY};")
        layout.addWidget(unit_label)

        self._draw_gauge_background()
        # initial needle render
        self._render_value()

    def _draw_gauge_background(self):
        """Draw gauge with color zones and min/max labels"""
        # Arc parameters (180 degrees, from -180 to 0, displayed as semicircle)
        start_angle = -np.pi  # Left side
        end_angle = 0  # Right side
        radius = 1.0

        # Base grey arc so the gauge is always visible
        angles_base = np.linspace(start_angle, end_angle, 140)
        x_base = radius * np.cos(angles_base)
        y_base = radius * np.sin(angles_base)
        self.gauge_plot.plot(x_base, y_base, pen=pg.mkPen('#555555', width=12), antialias=True)

        # Custom zones override simple warn/crit logic
        if self.zones:
            for start_v, end_v, color in self.zones:
                a0 = (start_v - self.min_val) / (self.max_val - self.min_val)
                a1 = (end_v - self.min_val) / (self.max_val - self.min_val)
                a0 = max(0.0, min(1.0, a0))
                a1 = max(0.0, min(1.0, a1))
                if a1 <= a0:
                    continue
                ang0 = start_angle + a0 * (end_angle - start_angle)
                ang1 = start_angle + a1 * (end_angle - start_angle)
                angles = np.linspace(ang0, ang1, 120)
                x = radius * np.cos(angles)
                y = radius * np.sin(angles)
                # thicker pen and full opacity to make zones stand out
                self.gauge_plot.plot(x, y, pen=pg.mkPen(color, width=16), antialias=True)

        # Determine zone boundaries (legacy warn/crit)
        if (self.warn_threshold is not None and self.crit_threshold is not None) and not self.zones:
            def clamp01(v: float) -> float:
                return 0.0 if v < 0.0 else 1.0 if v > 1.0 else v

            def plot_zone(a0: float, a1: float, color: str) -> None:
                if a1 <= a0:
                    return
                ang0 = start_angle + a0 * (end_angle - start_angle)
                ang1 = start_angle + a1 * (end_angle - start_angle)
                angles = np.linspace(ang0, ang1, 100)
                x = radius * np.cos(angles)
                y = radius * np.sin(angles)
                self.gauge_plot.plot(x, y, pen=pg.mkPen(color, width=12), antialias=True)

            warn_norm = clamp01((self.warn_threshold - self.min_val) / (self.max_val - self.min_val))
            crit_norm = clamp01((self.crit_threshold - self.min_val) / (self.max_val - self.min_val))

            if self.lower_is_worse:
                # Critical (left) -> Warning -> Normal (right)
                a_crit = min(crit_norm, warn_norm)
                a_warn = max(crit_norm, warn_norm)
                plot_zone(0.0, a_crit, Colors.STATUS_CRITICAL)
                plot_zone(a_crit, a_warn, Colors.STATUS_WARNING)
                plot_zone(a_warn, 1.0, '#555555')
            else:
                # Normal (left) -> Warning -> Critical (right)
                a_warn = min(warn_norm, crit_norm)
                a_crit = max(warn_norm, crit_norm)
                plot_zone(0.0, a_warn, '#555555')
                plot_zone(a_warn, a_crit, Colors.STATUS_WARNING)
                plot_zone(a_crit, 1.0, Colors.STATUS_CRITICAL)
        else:
            # No thresholds - single grey arc
            angles = np.linspace(start_angle, end_angle, 200)
            x = radius * np.cos(angles)
            y = radius * np.sin(angles)
            self.gauge_plot.plot(x, y, pen=pg.mkPen('#555555', width=12), antialias=True)

        # Tick marks (6 major ticks for 180 degree arc)
        for i in range(6):
            angle = start_angle + i * (end_angle - start_angle) / 5
            x1 = 0.88 * np.cos(angle)
            y1 = 0.88 * np.sin(angle)
            x2 = 1.0 * np.cos(angle)
            y2 = 1.0 * np.sin(angle)
            self.gauge_plot.plot([x1, x2], [y1, y2], pen=pg.mkPen('#888888', width=2), antialias=True)

        # Min/max labels
        min_text = pg.TextItem(f"{self.min_val:.0f}", anchor=(0.5, 0.5), color='#AAAAAA')
        min_text.setFont(QFont("Arial", 9))
        min_text.setPos(-1.25, 0)
        self.gauge_plot.addItem(min_text)

        max_text = pg.TextItem(f"{self.max_val:.0f}", anchor=(0.5, 0.5), color='#AAAAAA')
        max_text.setFont(QFont("Arial", 9))
        max_text.setPos(1.25, 0)
        self.gauge_plot.addItem(max_text)

        # Center pivot point
        self.gauge_plot.plot([0], [0], pen=None, symbol='o', symbolSize=8, symbolBrush='#666666')
        self._auto_fit()

    def set_value(self, value: float):
        """Smoothly move needle toward target value"""
        self.target_value = max(self.min_val, min(self.max_val, value))
        if not self.smooth_timer.isActive():
            self.smooth_timer.start()
        self._render_value()

    def _tick(self):
        alpha = 0.2
        self.current_value += (self.target_value - self.current_value) * alpha
        if abs(self.target_value - self.current_value) < 0.05:
            self.current_value = self.target_value
            self.smooth_timer.stop()
        self._render_value()

    def _render_value(self):
        self.value_label.setText(f"{self.current_value:.1f}")

        # Determine color/text from zones if present
        zone_color = None
        zone_severity = None
        zone_text = ""
        if self.zones:
            for start_v, end_v, color in self.zones:
                if start_v <= self.current_value <= end_v:
                    zone_color = color
                    cl = color.lower()
                    critical_colors = {"#ff4b4b"}
                    warn_colors = {"#e4c748", "#f59e42", "#b58c5a"}
                    good_colors = {"#5ec27f", "#6fbf73", "#5cae64"}
                    if cl in critical_colors:
                        zone_severity = "critical"
                    elif cl in warn_colors:
                        zone_severity = "warning"
                    elif cl in good_colors:
                        zone_severity = "normal"
                    else:
                        zone_severity = None
                    # status labels per gauge type
                    if "Temp" in self.title or "emp" in self.title:
                        if end_v <= 0: zone_text = "COLD"
                        elif end_v <= 20: zone_text = "COOL"
                        elif end_v <= 30: zone_text = "OPTIMAL"
                        elif end_v <= 50: zone_text = "WARM"
                        elif end_v <= 70: zone_text = "HOT"
                        else: zone_text = "VERY HOT"
                    elif "Humidity" in self.title:
                        if end_v <= 30: zone_text = "DRY"
                        elif end_v <= 50: zone_text = "OK"
                        elif end_v <= 70: zone_text = "IDEAL"
                        elif end_v <= 85: zone_text = "MOIST"
                        else: zone_text = "WET"
                    elif "Pressure" in self.title:
                        if end_v <= 940: zone_text = "VERY LOW"
                        elif end_v <= 980: zone_text = "LOW"
                        elif end_v <= 1030: zone_text = "NORMAL"
                        elif end_v <= 1060: zone_text = "HIGH"
                        else: zone_text = "VERY HIGH"
                    elif "FEI" in self.title:
                        if end_v <= -200: zone_text = "NEG DRIFT"
                        elif end_v <= -80: zone_text = "NEG"
                        elif end_v <= 80: zone_text = "LOCK"
                        elif end_v <= 200: zone_text = "POS"
                        else: zone_text = "POS DRIFT"
                    break

        # Determine if in warning/critical range
        is_warning = False
        is_critical = False
        if self.lower_is_worse:
            if self.crit_threshold is not None and self.current_value <= self.crit_threshold:
                is_critical = True
            elif self.warn_threshold is not None and self.current_value <= self.warn_threshold:
                is_warning = True
        else:
            if self.crit_threshold is not None and self.current_value >= self.crit_threshold:
                is_critical = True
            elif self.warn_threshold is not None and self.current_value >= self.warn_threshold:
                is_warning = True

        # Status text (zones override warn/crit flags)
        show_warn = is_warning or is_critical
        blink = is_critical
        status_text = ""
        status_color = Colors.TEXT_SECONDARY
        if zone_text:
            status_text = zone_text
            status_color = zone_color or Colors.TEXT_SECONDARY
            show_warn = True
            blink = (zone_severity == "critical")
        elif show_warn:
            status_text = "WARN" if is_warning else "ALERT"
            status_color = Colors.STATUS_WARNING if is_warning else Colors.STATUS_CRITICAL

        if show_warn and status_text:
            if blink:
                blink_on = int(time.time() * 2) % 2 == 0  # ~2 Hz villogás
                on_color = status_color
                off_color = "#22262d"  # mély, sötét szürke, sejtelmes
                self.warning_label.setStyleSheet(f"color: {on_color if blink_on else off_color};")
                self.warning_label.setVisible(True)
                self.warning_label.setText(status_text)
            else:
                self.warning_label.setStyleSheet(f"color: {status_color};")
                self.warning_label.setVisible(True)
                self.warning_label.setText(status_text)
        else:
            self.warning_label.setVisible(False)
            self.warning_label.setText("")

        # Set value color
        if zone_color:
            self.value_label.setStyleSheet(f"color: {zone_color}; background: #000000; padding: 2px;")
        elif is_critical:
            self.value_label.setStyleSheet(f"color: {Colors.STATUS_CRITICAL}; background: #000000; padding: 2px;")
        elif is_warning:
            self.value_label.setStyleSheet(f"color: {Colors.STATUS_WARNING}; background: #000000; padding: 2px;")
        else:
            self.value_label.setStyleSheet(f"color: {Colors.STATUS_NOMINAL}; background: #000000; padding: 2px;")

        # Calculate needle angle
        normalized = (self.current_value - self.min_val) / (self.max_val - self.min_val)
        needle_angle = -np.pi + normalized * np.pi  # From -180° to 0°

        # Clear and redraw
        self.gauge_plot.clear()
        self._draw_gauge_background()

        # Draw needle (thin red line from center to arc)
        needle_length = 0.8
        needle_x = needle_length * np.cos(needle_angle)
        needle_y = needle_length * np.sin(needle_angle)
        self.gauge_plot.plot([0, needle_x], [0, needle_y],
                           pen=pg.mkPen('#FF3333', width=3),
                           antialias=True)

        # Keep the view aligned as if the user pressed the "A" autorange hotkey
        self._auto_fit()

    def _auto_fit(self):
        """Match the behavior of pyqtgraph 'A' autorange so gauges start centered."""
        try:
            vb = self.gauge_plot.getViewBox()
            vb.autoRange(padding=0.05)
            vb.disableAutoRange(pg.ViewBox.XYAxes)
        except Exception:
            pass


class FrameStripWidget(QWidget):
    """Compact strip that shows last N frames as colored blocks."""
    def __init__(self, max_items: int = 150, parent=None):
        super().__init__(parent)
        self.max_items = max_items
        self.items: List[Dict] = []
        self.setMinimumHeight(30)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.setStyleSheet("background: transparent;")
        self.type_colors = {
            # Open MCT‑szerű, “űr-küldetés” paletta
            "FULL": "#36E0FF",
            "DELTA": "#C08BFF",
            "META": "#F8C256",
            "UNKNOWN": "#6C7A89",
            "PRE": "#556070",
        }

    def add_frame(self, ftype: str, length: int, crc_ok: bool, fid: int | None = None):
        if length <= 0:
            length = 1
        self.items.append({
            "type": ftype.upper(),
            "len": length,
            "crc": crc_ok,
            "id": fid,
        })
        if len(self.items) > self.max_items:
            self.items = self.items[-self.max_items:]
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        try:
            painter.setRenderHint(QPainter.RenderHint.Antialiasing)
            w = self.width()
            h = self.height()

            # Border outline to show reserved area even when empty
            painter.setPen(QColor("#3A4652"))
            painter.drawRect(1, 1, w - 2, h - 2)

            pad_x = 6
            gap = 2
            block_w = 14
            top_margin = 6
            bottom_margin = 6
            block_h = h - (top_margin + bottom_margin)
            if block_h < 16:
                return

            avail = max(0, w - (pad_x * 2))
            slots = max(1, avail // (block_w + gap)) if avail > 0 else 1

            # Oldest on the left, newest on the right (left-to-right reading).
            tail = self.items[-slots:] if self.items else []

            # Draw slot outlines so the user sees exactly how many frames fit at this window size.
            x = pad_x
            for _ in range(slots):
                painter.setPen(QColor("#3A4652"))
                painter.drawRect(x, top_margin, block_w, block_h)
                x += block_w + gap

            if not tail:
                return

            # Split block area: top = letter, bottom = id band
            letter_h = max(8, int(block_h * 0.58))
            letter_h = min(letter_h, block_h)
            id_h = max(0, block_h - letter_h)

            x = pad_x
            for item in tail:
                color = QColor(self.type_colors.get(item["type"], "#888888"))
                if not item["crc"]:
                    color = QColor("#ff3b30")
                rect_y = top_margin

                painter.fillRect(x, rect_y, block_w, block_h, color)

                # Letter
                painter.setPen(QColor("#000000"))
                font = painter.font()
                font.setPointSize(9)
                painter.setFont(font)
                painter.drawText(x, rect_y, block_w, letter_h, int(Qt.AlignmentFlag.AlignCenter), item["type"][:1])

                # ID band (FULL/DELTA)
                fid = item.get("id")
                if fid is not None and item["type"] in ("FULL", "DELTA") and id_h > 0:
                    id_bg_y = rect_y + letter_h
                    painter.fillRect(x, id_bg_y, block_w, id_h, QColor("#000000"))
                    painter.setPen(QColor("#FFFFFF"))
                    font.setPointSize(8)
                    painter.setFont(font)
                    painter.drawText(x, id_bg_y, block_w, id_h, int(Qt.AlignmentFlag.AlignCenter), str(fid))

                x += block_w + gap
        finally:
            painter.end()


class ProfessionalMissionControl(QMainWindow):
    """
    Professional Mission Control with OpenMCT-style sidebar navigation
    """

    def __init__(self):
        super().__init__()

        # Core components
        self.telemetry_decoder = TelemetryDecoder()
        self.serial_handler = SerialHandler()
        self.data_manager = DataManager()
        self.stream_demux = SerialStreamDemux()
        self.rf_meta_decoder = RfMetaDecoder()

        # Data storage
        self.temperatures: List[float] = []
        self.humidities: List[float] = []
        self.pressures: List[float] = []
        self.met_ticks: List[int] = []
        self.rf_meta_history: List[RfMetaFrame] = []
        self.keyframe_indices: List[int] = []

        # Mission state
        self.frames_received = 0
        self.frames_dropped = 0
        self.meta_frames_parsed = 0
        self.meta_crc_bad = 0
        self.stream_desync_dropped = 0
        self.last_meta_ts: Optional[float] = None
        self.meta_avg_gap: Optional[float] = None
        self.meta_max_gap: float = 0.0
        self.meta_last_gap: float = 0.0
        self.meta_gap_samples: int = 0
        self.mission_start_time: Optional[float] = None
        self.is_recording = False
        self.last_meta_arrival_flash: float = 0.0
        self.meta_counter: int = 0
        self.strip_started: bool = False
        self.strip_frame_counter: int = 0
        self.current_mode = "LIVE"
        self.connection_timestamp: Optional[float] = None  # Track connection time for boot desync suppression
        self.last_strip_frame_id: int = -1
        self.last_strip_meta_idx: int = 0
        self.frame_table_limit = 1000
        self._pending_meta_frames: List[RfMetaFrame] = []
        self._pending_frame_numbers: List[int] = []
        self._frame_meta: List[Optional[RfMetaFrame]] = []
        self._frame_meta_dirty = False

        # Error tracking
        self._last_crc_error_count = 0
        self._last_desync_count = 0
        self._last_ref_error_count = 0

        # Sample buffer for 2Hz gauge updates (500ms per sample)
        self._sample_buffer_temp: List[float] = []
        self._sample_buffer_hum: List[float] = []
        self._sample_buffer_press: List[float] = []
        self._current_gauge_index = 0

        # Self-test / demo mode (no hardware required)
        self.self_test_active = False
        self._self_test_start_s: float | None = None
        self._self_test_timer = QTimer(self)
        self._self_test_timer.setInterval(100)  # 10 Hz
        self._self_test_timer.timeout.connect(self._self_test_tick)
        self._self_test_last_meta_emit_s: float = 0.0
        self._self_test_last_frame_emit_s: float = 0.0
        self._self_test_meta_period_s: float = 0.6
        self._self_test_frame_period_s: float = 0.25
        self._self_test_seq: int = 0
        self._self_test_last_full_seq: int | None = None
        self._self_test_frame_number: int = 0

        # Setup UI
        self.setWindowTitle(f"{APP_NAME} v{APP_VERSION} - Professional Mission Control")
        self.setMinimumSize(1280, 720)
        screen = QGuiApplication.primaryScreen()
        if screen is not None:
            avail = screen.availableGeometry()
            width = min(avail.width() - 40, 1900)
            height = min(avail.height() - 40, 1050)
            if width < 900:
                width = max(avail.width() - 20, 800)
            if height < 700:
                height = max(avail.height() - 20, 600)
            self.resize(width, height)

        self._setup_ui()
        self._setup_timers()
        self._setup_connections()

        logger.info("Professional Mission Control initialized")

    def _setup_ui(self):
        """Setup main UI with OpenMCT-style sidebar"""
        scaling = ResponsiveScaling()

        central = QWidget()
        self.setCentralWidget(central)

        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # === LEFT SIDEBAR (OpenMCT style) ===
        sidebar = self._create_sidebar()
        main_layout.addWidget(sidebar)

        # === RIGHT CONTENT AREA ===
        content_widget = QWidget()
        content_layout = QVBoxLayout(content_widget)
        content_layout.setContentsMargins(0, 0, 0, 0)
        content_layout.setSpacing(0)

        # Top toolbar
        toolbar = self._create_toolbar()
        content_layout.addWidget(toolbar, 0)  # No stretch

        # Spacer to separate toolbar from content (match toolbar background to avoid "black band")
        spacer = QWidget()
        spacer.setFixedHeight(scaling.scale_spacing(10))
        spacer.setStyleSheet(f"background-color: {Colors.BG_SECONDARY};")
        content_layout.addWidget(spacer, 0)

        # Stacked widget for different views
        self.view_stack = QStackedWidget()
        self.view_stack.setContentsMargins(0, 0, 0, 0)
        content_layout.addWidget(self.view_stack, 1)  # Stretch to fill

        # Create all views
        self.view_stack.addWidget(self._create_dashboard_view())      # Index 0
        self.view_stack.addWidget(self._create_frame_inspector_view())  # Index 1
        self.view_stack.addWidget(self._create_system_status_view())    # Index 2
        self.view_stack.addWidget(self._create_signal_analysis_view())  # Index 3

        main_layout.addWidget(content_widget, 1)

        # Status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Ready")

        # Menu bar
        self._create_menu_bar()

    def _create_sidebar(self) -> QWidget:
        """Create left sidebar with view navigation"""
        sidebar = QFrame()
        sidebar.setFrameShape(QFrame.Shape.StyledPanel)
        sidebar.setStyleSheet(f"""
            QFrame {{
                background-color: {Colors.BG_SECONDARY};
                border-right: 2px solid {Colors.BORDER};
            }}
        """)
        sidebar.setFixedWidth(220)

        layout = QVBoxLayout(sidebar)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        # Logo/Title
        title = QLabel("MISSION\nCONTROL")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setFont(QFont("Arial", 16, QFont.Weight.Bold))
        title.setStyleSheet(f"color: {Colors.STATUS_NOMINAL}; padding: 12px;")
        layout.addWidget(title)

        # View navigation list
        self.view_list = QListWidget()
        self.view_list.setStyleSheet(f"""
            QListWidget {{
                background-color: {Colors.BG_PRIMARY};
                border: 1px solid {Colors.BORDER};
                color: {Colors.TEXT_PRIMARY};
                font-size: 13px;
                font-weight: bold;
                outline: 0;
            }}
            QListWidget::item {{
                padding: 12px;
                margin: 4px 6px;
                border-radius: 6px;
            }}
            QListWidget::item:selected {{
                background-color: {Colors.BG_TERTIARY};
                color: {Colors.TEXT_PRIMARY};
                border: 1px solid {Colors.BORDER_EMPHASIS};
            }}
            QListWidget::item:hover {{
                background-color: {Colors.BG_HOVER};
            }}
        """)

        views = [
            "📊 Dashboard",
            "🔍 Frame Inspector",
            "⚙️ System Status",
            "📡 Signal Analysis",
        ]

        for view in views:
            item = QListWidgetItem(view)
            self.view_list.addItem(item)

        self.view_list.setCurrentRow(0)
        self.view_list.currentRowChanged.connect(self._on_view_changed)
        layout.addWidget(self.view_list)

        layout.addStretch()

        # Version info
        version_label = QLabel(f"v{APP_VERSION}")
        version_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        version_label.setStyleSheet(f"color: {Colors.TEXT_TERTIARY}; font-size: 10px;")
        layout.addWidget(version_label)

        return sidebar

    def _create_toolbar(self) -> QWidget:
        """Create top toolbar with connection controls"""
        scaling = ResponsiveScaling()
        radius_px = max(2, scaling.scale_spacing(2))

        toolbar = QFrame()
        toolbar.setFrameShape(QFrame.Shape.NoFrame)
        control_height = scaling.scale_button(30)
        margin_h = scaling.scale_spacing(8)
        margin_v = scaling.scale_spacing(6)
        margin_v_bottom = margin_v + scaling.scale_spacing(6)
        toolbar.setMinimumHeight(control_height + margin_v + margin_v_bottom)
        toolbar.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        toolbar.setStyleSheet(f"""
            QFrame {{
                background-color: {Colors.BG_SECONDARY};
                border-bottom: 1px solid {Colors.BORDER};
                padding: 0px;
            }}
        """)

        layout = QHBoxLayout(toolbar)
        layout.setContentsMargins(margin_h, margin_v, margin_h, margin_v_bottom)
        layout.setSpacing(scaling.scale_spacing(6))

        # Standard toolbar height
        TOOLBAR_HEIGHT = control_height

        # Port selection (narrower, styled)
        self.port_combo = ElidedComboBox()
        self.port_combo.setFixedHeight(TOOLBAR_HEIGHT)
        self.port_combo.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.port_combo.setMinimumWidth(scaling.scale(180))
        self.port_combo.setMaximumWidth(scaling.scale(360))
        self.port_combo.setStyleSheet(f"""
            QComboBox {{
                font-size: {scaling.scale_font(10)}pt;
                padding: 0px 22px 0px 8px; /* leave space for arrow */
                border: 1px solid {Colors.BORDER};
                border-radius: {radius_px}px;
                background-color: {Colors.BG_TERTIARY};
            }}
            QComboBox::drop-down {{
                border: none;
                width: 16px;
            }}
            QComboBox::down-arrow {{
                width: 10px;
                height: 10px;
            }}
        """)
        ports = self.serial_handler.list_ports()
        self.port_combo.addItems(ports)
        self._auto_select_port(ports)
        self.port_combo.setToolTip(self.port_combo.currentText())
        self.port_combo.currentTextChanged.connect(self.port_combo.setToolTip)
        layout.addWidget(self.port_combo)

        # Connect button (smaller, less padding)
        self.btn_connect = QPushButton("CONNECT")
        self.btn_connect.setCheckable(True)
        self.btn_connect.setFixedHeight(TOOLBAR_HEIGHT)
        self.btn_connect.setMinimumWidth(scaling.scale_button(92))
        self.btn_connect.setSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)
        self.btn_connect.setStyleSheet(f"""
            QPushButton {{
                background-color: {Colors.STATUS_NOMINAL};
                color: black;
                font-weight: normal;
                padding: 0px 8px;
                border: 1px solid {Colors.BORDER};
                border-radius: {radius_px}px;
                font-size: {scaling.scale_font(10)}pt;
            }}
            QPushButton:checked {{
                background-color: {Colors.STATUS_CRITICAL};
                color: white;
            }}
            QPushButton:hover {{
                border-color: {Colors.BORDER_ACTIVE};
            }}
        """)
        layout.addWidget(self.btn_connect)

        layout.addSpacing(scaling.scale_spacing(6))

        # Record button (compact, same style as others)
        self.btn_record = QPushButton("REC")
        self.btn_record.setCheckable(True)
        self.btn_record.setFixedHeight(TOOLBAR_HEIGHT)
        self.btn_record.setMinimumWidth(scaling.scale_button(56))
        self.btn_record.setSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)
        self.btn_record.setStyleSheet(f"""
            QPushButton {{
                background-color: {Colors.BG_TERTIARY};
                color: {Colors.TEXT_PRIMARY};
                padding: 0px 8px;
                border: 1px solid {Colors.BORDER};
                border-radius: {radius_px}px;
                font-size: {scaling.scale_font(10)}pt;
            }}
            QPushButton:checked {{
                background-color: {Colors.STATUS_CRITICAL};
                color: white;
                border-color: {Colors.STATUS_CRITICAL};
            }}
            QPushButton:hover {{
                border-color: {Colors.BORDER_ACTIVE};
                background-color: {Colors.BG_HOVER};
            }}
        """)
        layout.addWidget(self.btn_record)

        # Self-test button (compact, same style as others)
        self.btn_self_test = QPushButton("TEST")
        self.btn_self_test.setCheckable(True)
        self.btn_self_test.setFixedHeight(TOOLBAR_HEIGHT)
        self.btn_self_test.setMinimumWidth(scaling.scale_button(60))
        self.btn_self_test.setSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)
        self.btn_self_test.setToolTip("Self-test: animate all displays without hardware")
        self.btn_self_test.setStyleSheet(f"""
            QPushButton {{
                background-color: {Colors.BG_TERTIARY};
                color: {Colors.TEXT_PRIMARY};
                padding: 0px 8px;
                border: 1px solid {Colors.BORDER};
                border-radius: {radius_px}px;
                font-size: {scaling.scale_font(10)}pt;
            }}
            QPushButton:checked {{
                background-color: {Colors.STATUS_CAUTION};
                color: {Colors.TEXT_INVERSE};
                border-color: {Colors.STATUS_CAUTION};
            }}
            QPushButton:hover {{
                border-color: {Colors.BORDER_ACTIVE};
                background-color: {Colors.BG_HOVER};
            }}
        """)
        layout.addWidget(self.btn_self_test)

        layout.addSpacing(scaling.scale_spacing(6))

        # Fullscreen toggle button (brighter, more visible)
        self.btn_fullscreen = QPushButton("⛶")
        self.btn_fullscreen.setFixedSize(TOOLBAR_HEIGHT, TOOLBAR_HEIGHT)
        self.btn_fullscreen.setToolTip("Toggle Fullscreen (F11)")
        self.btn_fullscreen.setStyleSheet(f"""
            QPushButton {{
                background-color: {Colors.BG_SECONDARY};
                color: {Colors.TEXT_PRIMARY};
                font-size: {scaling.scale_font(12)}pt;
                border: 1px solid {Colors.BORDER};
                border-radius: {radius_px}px;
                padding: 0px;
            }}
            QPushButton:hover {{
                background-color: {Colors.BORDER};
            }}
        """)
        self.btn_fullscreen.clicked.connect(self.toggle_fullscreen)
        layout.addWidget(self.btn_fullscreen)

        # Spacer to push MET to center
        layout.addStretch(1)

        # MET display (PROMINENT, centered, large)
        self.met_label = QLabel("00:00:00.00")
        self.met_label.setMinimumHeight(TOOLBAR_HEIGHT)
        self.met_label.setFont(QFont("Monaco", scaling.scale_font(14), QFont.Weight.Bold))
        self.met_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.met_label.setStyleSheet(f"""
            color: {Colors.STATUS_NOMINAL};
            background-color: {Colors.BG_PRIMARY};
            padding: 0px 16px;
            font-size: {scaling.scale_font(14)}pt;
            border: 1px solid {Colors.BORDER};
            border-radius: 2px;
        """)
        layout.addWidget(self.met_label)

        # Spacer to balance layout
        layout.addStretch(1)

        # Frame counter (compact, right side)
        self.frame_count_label = QLabel("Frames: 0")
        self.frame_count_label.setMinimumHeight(TOOLBAR_HEIGHT)
        self.frame_count_label.setFont(QFont("Monaco", scaling.scale_font(10), QFont.Weight.Normal))
        self.frame_count_label.setStyleSheet(
            f"color: {Colors.TEXT_PRIMARY}; padding: 0px 8px; font-size: {scaling.scale_font(10)}pt;"
        )
        layout.addWidget(self.frame_count_label)

        return toolbar

    def _auto_select_port(self, ports: List[str]):
        """Pick a likely USB serial port automatically."""
        if not ports:
            return
        preferred = ["usbserial", "usbmodem", "ttyUSB", "ttyACM", "SLAB", "CP210"]
        chosen_idx = 0
        for idx, p in enumerate(ports):
            if any(token.lower() in p.lower() for token in preferred):
                chosen_idx = idx
                break
        self.port_combo.setCurrentIndex(chosen_idx)

    def _wrap_led_with_label(self, led: QLabel, text: str) -> QWidget:
        box = QWidget()
        box.setStyleSheet("background: transparent; border: none;")
        lay = QVBoxLayout(box)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        led.setStyleSheet(f"color: {Colors.STATUS_UNKNOWN}; font-size: 16pt; background: transparent; border: none;")
        led.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lay.addWidget(led, alignment=Qt.AlignmentFlag.AlignHCenter)
        lbl = QLabel(text)
        lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lbl.setStyleSheet(f"color: {Colors.TEXT_SECONDARY}; font-size: 8pt;")
        lay.addWidget(lbl)
        return box

    def _create_dashboard_view(self) -> QWidget:
        """Create main dashboard view with gauges and charts"""
        widget = QWidget()

        layout = QGridLayout(widget)
        layout.setContentsMargins(12, 8, 12, 12)  # Less top margin - toolbar handles spacing
        layout.setSpacing(12)

        # Row 0: Signal Quality Gauges
        signal_panel = self._create_panel_container("📡 SIGNAL QUALITY")
        signal_layout = QHBoxLayout()

        self.rssi_gauge = SignalBarWidget(
            "RSSI", "dBm", -164, 0,
            zones=[(-164, -120, "#ff4b4b"),
                   (-120, -100, "#e4c748"),
                   (-100, -80, "#5ec27f"),
                   (-80, 0, "#5cae64")]
        )
        self.snr_gauge = SignalBarWidget(
            "SNR", "dB", -10, 15,
            zones=[(-10, 0, "#ff4b4b"),
                   (0, 5, "#e4c748"),
                   (5, 10, "#5ec27f"),
                   (10, 15, "#5cae64")]
        )
        # FEI vissza mutatós gauge-ra (±500 Hz)
        self.fei_gauge = LiveGaugeWidget(
            "FEI", "Hz", -500, 500,
            lower_is_worse=False,
            zones=[
                (-500, -200, "#ff4b4b"),
                (-200, -80, "#e4c748"),
                (-80, 80, "#5ec27f"),
                (80, 200, "#e4c748"),
                (200, 500, "#ff4b4b"),
            ]
        )
        self.link_gauge = SignalBarWidget(
            "Link Score", "%", 0, 100,
            zones=[(0, 50, "#ff4b4b"),
                   (50, 70, "#e4c748"),
                   (70, 85, "#5ec27f"),
                   (85, 100, "#5cae64")]
        )

        bars_layout = QHBoxLayout()
        bars_layout.setSpacing(10)
        bars_layout.addWidget(self.rssi_gauge)
        bars_layout.addWidget(self.snr_gauge)
        bars_layout.addWidget(self.link_gauge)

        signal_layout.addLayout(bars_layout)

        # Wrap gauges + meta box horizontally so the info sits beside the bars
        signal_row = QHBoxLayout()
        signal_row.setSpacing(12)
        signal_row.addLayout(signal_layout)

        # RF stats (small framed block)
        meta_box = QFrame()
        meta_box.setFrameShape(QFrame.Shape.StyledPanel)
        meta_box.setStyleSheet(f"border: 1px solid {Colors.BORDER}; background: {Colors.BG_SECONDARY};")
        meta_box.setFixedHeight(150)
        meta_box.setMinimumWidth(190)
        meta_box.setSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)
        meta_layout = QVBoxLayout(meta_box)
        meta_layout.setContentsMargins(6, 6, 6, 6)
        meta_layout.setSpacing(2)

        meta_title = QLabel("RF STATS")
        meta_title.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
        meta_title.setStyleSheet(f"color: {Colors.TEXT_SECONDARY}; font-weight: bold; font-size: 9pt;")
        meta_layout.addWidget(meta_title)

        self.radio_meta_label = QLabel("No RF meta frames yet.")
        self.radio_meta_label.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop)
        self.radio_meta_label.setWordWrap(True)
        self.radio_meta_label.setStyleSheet(
            f"color: {Colors.TEXT_PRIMARY}; font-family: Menlo; font-size: 8pt;"
        )
        self.radio_meta_label.setMinimumWidth(0)
        self.radio_meta_label.setMaximumHeight(90)
        self.radio_meta_label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        meta_layout.addWidget(self.radio_meta_label)

        leds_row = QHBoxLayout()
        leds_row.setContentsMargins(0, 6, 0, 0)
        leds_row.setSpacing(18)
        # Meta arrival LED (blink on each meta)
        self.meta_arrival_led = QLabel("●")
        self.meta_arrival_led.setToolTip("META arrival pulse")
        leds_row.addWidget(self._wrap_led_with_label(self.meta_arrival_led, "META"), 0, Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
        # Link state LED (blue steady when within avg window, red blink on fault)
        self.meta_link_led = QLabel("●")
        self.meta_link_led.setToolTip("LOCK (avg gap based)")
        leds_row.addWidget(self._wrap_led_with_label(self.meta_link_led, "LOCK"), 0, Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
        leds_row.addStretch()
        meta_layout.addLayout(leds_row)

        signal_layout.addWidget(meta_box, 0)
        signal_layout.addStretch()

        signal_row.addWidget(self.fei_gauge)
        signal_row.addStretch()

        signal_panel.layout().addLayout(signal_row)
        layout.addWidget(signal_panel, 0, 0, 1, 2)

        # Row 0: Telemetry Gauges
        telem_panel = self._create_panel_container("🌡️ TELEMETRY")
        telem_layout = QHBoxLayout()

        self.temp_gauge = LiveGaugeWidget(
            "Temperature", "°C", -40, 85,
            lower_is_worse=False,
            zones=[
                (-40, 0, "#3da5ff"),   # cold
                (0, 20, "#6f9a6d"),    # cool
                (20, 30, "#5ec27f"),   # ideal green
                (30, 50, "#e4c748"),   # warm yellow
                (50, 70, "#f59e42"),   # hot orange
                (70, 85, "#ff4b4b"),   # very hot red
            ]
        )
        self.hum_gauge = LiveGaugeWidget(
            "Humidity", "%", 0, 100,
            lower_is_worse=False,
            zones=[
                (0, 30, "#b58c5a"),    # dry
                (30, 50, "#6fbf73"),   # ok
                (50, 70, "#5cae64"),   # ideal-greenish
                (70, 85, "#e4c748"),   # moist yellow
                (85, 100, "#ff4b4b"),  # too wet red
            ]
        )
        self.press_gauge = LiveGaugeWidget(
            "Pressure", "hPa", 822, 1077,
            lower_is_worse=False,
            zones=[
                (822, 940, "#ff4b4b"),   # very low
                (940, 980, "#e4c748"),   # low warning
                (980, 1030, "#5ec27f"),  # normal
                (1030, 1060, "#e4c748"), # high warning
                (1060, 1077, "#ff4b4b"), # very high
            ]
        )

        telem_layout.addWidget(self.temp_gauge)
        telem_layout.addWidget(self.hum_gauge)
        telem_layout.addWidget(self.press_gauge)
        telem_layout.addStretch()

        telem_panel.layout().addLayout(telem_layout)
        layout.addWidget(telem_panel, 0, 2, 1, 2)

        # Row 1: Frame Status strip
        frame_panel = self._create_panel_container("⚡ FRAME STATUS")
        self.frame_strip = FrameStripWidget(max_items=200)
        frame_panel.layout().addWidget(self.frame_strip)
        layout.addWidget(frame_panel, 1, 0, 1, 4)

        # Row 2: Telemetry Trends (3 plots)
        trends_panel = self._create_panel_container("📈 TELEMETRY TRENDS")
        trends_layout = QHBoxLayout()

        self.plot_temp = self._create_plot("Temperature (°C)", Colors.TELEM_TEMPERATURE)
        self.plot_hum = self._create_plot("Humidity (%)", Colors.TELEM_HUMIDITY)
        self.plot_press = self._create_plot("Pressure (hPa)", Colors.TELEM_PRESSURE)

        trends_layout.addWidget(self.plot_temp)
        trends_layout.addWidget(self.plot_hum)
        trends_layout.addWidget(self.plot_press)

        trends_panel.layout().addLayout(trends_layout)
        layout.addWidget(trends_panel, 2, 0, 1, 4)

        # Row 3: Signal History
        signal_hist_panel = self._create_panel_container("📊 SIGNAL HISTORY")
        signal_hist_layout = QHBoxLayout()

        self.plot_rssi = self._create_plot("RSSI (dBm)", Colors.STATUS_WARNING)
        self.plot_snr = self._create_plot("SNR (dB)", Colors.STATUS_NOMINAL)

        signal_hist_layout.addWidget(self.plot_rssi)
        signal_hist_layout.addWidget(self.plot_snr)

        signal_hist_panel.layout().addLayout(signal_hist_layout)
        layout.addWidget(signal_hist_panel, 3, 0, 1, 4)

        return widget

    def _create_frame_inspector_view(self) -> QWidget:
        """Create frame inspector view with detailed frame analysis"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)

        # Title
        title = QLabel("🔍 FRAME INSPECTOR - Protocol Analysis")
        title.setFont(QFont("Arial", 14, QFont.Weight.Bold))
        title.setStyleSheet(f"color: {Colors.STATUS_NOMINAL}; padding: 8px;")
        layout.addWidget(title)

        # Frame table
        self.frame_table = QTableWidget()
        self.frame_table.setColumnCount(9)
        self.frame_table.setHorizontalHeaderLabels([
            "#", "Type", "Seq", "Ref", "Status", "CRC calc", "CRC recv", "RSSI", "SNR"
        ])
        self.frame_table.setStyleSheet(f"""
            QTableWidget {{
                background-color: {Colors.BG_DARK};
                color: {Colors.TEXT_PRIMARY};
                gridline-color: {Colors.BORDER};
                font-family: Menlo;
                font-size: 10px;
            }}
            QHeaderView::section {{
                background-color: {Colors.BG_SECONDARY};
                color: {Colors.TEXT_PRIMARY};
                font-weight: bold;
                padding: 6px;
                border: 1px solid {Colors.BORDER};
            }}
        """)
        header = self.frame_table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        header.setStretchLastSection(True)
        layout.addWidget(self.frame_table, 1)

        # Frame detail view
        detail_label = QLabel("📄 Frame Detail (click row to inspect)")
        detail_label.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        detail_label.setStyleSheet(f"color: {Colors.TEXT_PRIMARY};")
        layout.addWidget(detail_label)

        self.frame_detail = QTextEdit()
        self.frame_detail.setReadOnly(True)
        self.frame_detail.setMaximumHeight(200)
        self.frame_detail.setFont(QFont("Menlo", 9))
        self.frame_detail.setTextInteractionFlags(
            Qt.TextInteractionFlag.TextSelectableByMouse |
            Qt.TextInteractionFlag.TextSelectableByKeyboard
        )
        self.frame_detail.setStyleSheet(f"""
            background-color: {Colors.BG_DARK};
            color: {Colors.TEXT_PRIMARY};
            border: 1px solid {Colors.BORDER};
        """)
        layout.addWidget(self.frame_detail)

        return widget

    def _create_system_status_view(self) -> QWidget:
        """Create system status view with logs and diagnostics"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)

        # Title
        title = QLabel("⚙️ SYSTEM STATUS - Diagnostics")
        title.setFont(QFont("Arial", 14, QFont.Weight.Bold))
        title.setStyleSheet(f"color: {Colors.STATUS_NOMINAL}; padding: 8px;")
        layout.addWidget(title)

        # Error indicators
        error_panel = self._create_panel_container("ERROR TRACKING")
        error_layout = QHBoxLayout()

        self.ref_err_indicator = self._create_status_box("REF ERR", "0", Colors.STATUS_UNKNOWN)
        self.trunc_indicator = self._create_status_box("TRUNC", "0", Colors.STATUS_UNKNOWN)
        self.desync_indicator = self._create_status_box("DESYNC", "0", Colors.STATUS_UNKNOWN)

        error_layout.addWidget(self.ref_err_indicator)
        error_layout.addWidget(self.trunc_indicator)
        error_layout.addWidget(self.desync_indicator)
        error_layout.addStretch()

        error_panel.layout().addLayout(error_layout)
        layout.addWidget(error_panel)

        # Frame log
        log_row = QHBoxLayout()
        log_row.addWidget(QLabel("FRAME LOG"))
        self.chk_autoscroll = QCheckBox("Auto-scroll")
        self.chk_autoscroll.setChecked(True)
        log_row.addStretch()
        log_row.addWidget(self.chk_autoscroll)
        layout.addLayout(log_row)

        self.frame_log = QTextEdit()
        self.frame_log.setReadOnly(True)
        self.frame_log.setMaximumHeight(300)
        self.frame_log.setLineWrapMode(QTextEdit.LineWrapMode.NoWrap)
        self.frame_log.setFont(QFont("Menlo", 9))
        self.frame_log.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOn)
        self.frame_log.setStyleSheet(f"""
            background-color: {Colors.BG_DARK};
            color: {Colors.TEXT_PRIMARY};
            border: 1px solid {Colors.BORDER};
        """)
        layout.addWidget(self.frame_log)

        # Event log
        event_row = QHBoxLayout()
        event_row.addWidget(QLabel("EVENT LOG"))
        self.chk_autoscroll_events = QCheckBox("Auto-scroll")
        self.chk_autoscroll_events.setChecked(True)
        event_row.addStretch()
        event_row.addWidget(self.chk_autoscroll_events)
        layout.addLayout(event_row)

        self.event_log = QTextEdit()
        self.event_log.setReadOnly(True)
        self.event_log.setMaximumHeight(300)
        self.event_log.setLineWrapMode(QTextEdit.LineWrapMode.NoWrap)
        self.event_log.setFont(QFont("Menlo", 9))
        self.event_log.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOn)
        self.event_log.setTextInteractionFlags(
            Qt.TextInteractionFlag.TextSelectableByMouse |
            Qt.TextInteractionFlag.TextSelectableByKeyboard
        )
        self.event_log.setStyleSheet(f"""
            background-color: {Colors.BG_DARK};
            color: {Colors.TEXT_PRIMARY};
            border: 1px solid {Colors.BORDER};
        """)
        layout.addWidget(self.event_log)

        layout.addStretch()

        return widget

    def _create_signal_analysis_view(self) -> QWidget:
        """Create signal analysis view with RF parameters"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)

        # Title
        title = QLabel("📡 SIGNAL ANALYSIS - RF Parameters")
        title.setFont(QFont("Arial", 14, QFont.Weight.Bold))
        title.setStyleSheet(f"color: {Colors.STATUS_NOMINAL}; padding: 8px;")
        layout.addWidget(title)

        # Radio parameters
        params_panel = self._create_panel_container("🔧 RADIO PARAMETERS")

        self.radio_params_text = QTextEdit()
        self.radio_params_text.setReadOnly(True)
        self.radio_params_text.setMaximumHeight(150)
        self.radio_params_text.setFont(QFont("Menlo", 11))
        self.radio_params_text.setTextInteractionFlags(
            Qt.TextInteractionFlag.TextSelectableByMouse |
            Qt.TextInteractionFlag.TextSelectableByKeyboard
        )
        self.radio_params_text.setStyleSheet(f"""
            background-color: {Colors.BG_DARK};
            color: {Colors.TEXT_PRIMARY};
            border: 1px solid {Colors.BORDER};
        """)
        params_panel.layout().addWidget(self.radio_params_text)
        layout.addWidget(params_panel)

        # Signal quality text
        quality_panel = self._create_panel_container("📊 SIGNAL QUALITY")

        self.signal_quality_text = QTextEdit()
        self.signal_quality_text.setReadOnly(True)
        self.signal_quality_text.setFont(QFont("Menlo", 11))
        self.signal_quality_text.setTextInteractionFlags(
            Qt.TextInteractionFlag.TextSelectableByMouse |
            Qt.TextInteractionFlag.TextSelectableByKeyboard
        )
        self.signal_quality_text.setStyleSheet(f"""
            background-color: {Colors.BG_DARK};
            color: {Colors.TEXT_PRIMARY};
            border: 1px solid {Colors.BORDER};
        """)
        quality_panel.layout().addWidget(self.signal_quality_text)
        layout.addWidget(quality_panel, 1)

        return widget

    def _create_panel_container(self, title: str) -> QGroupBox:
        """Create a styled panel container"""
        panel = QGroupBox(title)
        panel.setStyleSheet(f"""
            QGroupBox {{
                font-weight: bold;
                font-size: 12px;
                color: {Colors.STATUS_NOMINAL};
                border: 2px solid {Colors.BORDER};
                border-radius: 6px;
                margin-top: 12px;
                padding-top: 12px;
                background-color: {Colors.BG_SECONDARY};
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 6px;
            }}
        """)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(8, 8, 8, 8)
        return panel

    def _create_status_box(self, label: str, value: str, color: str) -> QWidget:
        """Create a status indicator box"""
        widget = QFrame()
        widget.setFrameShape(QFrame.Shape.StyledPanel)
        widget.setStyleSheet(f"""
            QFrame {{
                background-color: {Colors.BG_DARK};
                border: 2px solid {color};
                border-radius: 4px;
                padding: 8px;
            }}
        """)
        widget.setMinimumWidth(120)

        layout = QVBoxLayout(widget)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(4)

        label_widget = QLabel(label)
        label_widget.setAlignment(Qt.AlignmentFlag.AlignCenter)
        label_widget.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        label_widget.setStyleSheet(f"color: {Colors.TEXT_SECONDARY}; border: none;")
        layout.addWidget(label_widget)

        value_widget = QLabel(value)
        value_widget.setAlignment(Qt.AlignmentFlag.AlignCenter)
        value_widget.setFont(QFont("Menlo", 16, QFont.Weight.Bold))
        value_widget.setStyleSheet(f"color: {color}; border: none;")
        layout.addWidget(value_widget)

        # Store reference for updates
        widget.value_label = value_widget
        widget.border_color = color

        return widget

    def _create_plot(self, title: str, color: str) -> pg.PlotWidget:
        """Create a styled plot widget"""
        plot = pg.PlotWidget()
        plot.setBackground(Colors.BG_DARK)
        plot.setTitle(title, color=Colors.TEXT_PRIMARY, size='11pt')
        plot.showGrid(x=True, y=True, alpha=0.3)
        plot.setLabel('bottom', 'Time (s)', color=Colors.TEXT_SECONDARY)
        plot.getAxis('left').setTextPen(Colors.TEXT_SECONDARY)
        plot.getAxis('bottom').setTextPen(Colors.TEXT_SECONDARY)
        return plot

    def _create_menu_bar(self):
        """Create menu bar"""
        menubar = self.menuBar()

        # File menu
        file_menu = menubar.addMenu("File")
        file_menu.addAction("Open Recording...", self._on_open_recording)
        file_menu.addAction("Save Recording...", self._on_save_recording)
        file_menu.addSeparator()
        file_menu.addAction("Clear Data", self._on_clear_data)
        file_menu.addSeparator()
        file_menu.addAction("Exit", self.close)

        # Help menu
        help_menu = menubar.addMenu("Help")
        help_menu.addAction("About", self._on_about)

    def _setup_timers(self):
        """Setup update timers"""
        self.plot_timer = QTimer(self)
        self.plot_timer.timeout.connect(self._update_displays)
        self.plot_timer.start(PLOT_UPDATE_INTERVAL)

        self.met_timer = QTimer(self)
        self.met_timer.timeout.connect(self._update_met)
        self.met_timer.start(100)  # 100ms

        # Gauge update timer at 2Hz (500ms) for telemetry samples
        self.gauge_timer = QTimer(self)
        self.gauge_timer.timeout.connect(self._update_gauges_from_buffer)
        self.gauge_timer.start(500)  # 500ms = 2Hz

    def _setup_connections(self):
        """Setup signal connections"""
        self.btn_connect.clicked.connect(self._on_connect_clicked)
        self.btn_record.clicked.connect(self._on_record_clicked)
        self.btn_self_test.clicked.connect(self._on_self_test_clicked)
        self.frame_table.itemSelectionChanged.connect(self._on_frame_selected)

        # Connect to serial handler signals
        self.serial_handler.data_received.connect(self._read_serial_data)

    def _on_view_changed(self, index: int):
        """Handle view change from sidebar"""
        self.view_stack.setCurrentIndex(index)

    def _on_connect_clicked(self):
        """Handle connect button"""
        if self.btn_connect.isChecked():
            # Stop self-test if user connects
            if self.self_test_active:
                self._stop_self_test(reset=True)
                self.btn_self_test.setChecked(False)

            port = self.port_combo.currentText()
            if not port:
                ports = self.serial_handler.list_ports()
                self.port_combo.clear()
                self.port_combo.addItems(ports)
                self._auto_select_port(ports)
                port = self.port_combo.currentText()
            if self.serial_handler.connect(port):
                self.connection_timestamp = time.time()  # Start boot desync grace period
                self.btn_connect.setText("⚡ DISCONNECT")
                self.status_bar.showMessage(f"Connected to {port}")
                self._add_event(f"Connected to {port}", "INFO")
            else:
                self.btn_connect.setChecked(False)
                self.status_bar.showMessage("Connection failed")
                self._add_event("Connection failed", "CRITICAL")
        else:
            self.serial_handler.disconnect()
            self.connection_timestamp = None
            self.btn_connect.setText("🔌 CONNECT")
            self.status_bar.showMessage("Disconnected")
            self._add_event("Disconnected", "INFO")

    def _on_self_test_clicked(self):
        """Toggle demo/self-test mode (no hardware required)."""
        if self.btn_self_test.isChecked():
            if self.serial_handler.is_connected:
                QMessageBox.warning(self, "Self-test", "Disconnect from hardware before running self-test.")
                self.btn_self_test.setChecked(False)
                return
            self._start_self_test()
        else:
            self._stop_self_test(reset=True)

    @staticmethod
    def _pingpong(t: float, vmin: float, vmax: float, period_s: float) -> float:
        if period_s <= 0:
            return vmin
        span = vmax - vmin
        x = (t / period_s) % 2.0
        u = x if x <= 1.0 else (2.0 - x)
        return vmin + span * u

    def _start_self_test(self):
        self.self_test_active = True
        self._self_test_start_s = time.monotonic()
        self._self_test_last_meta_emit_s = 0.0
        self._self_test_last_frame_emit_s = 0.0
        self._self_test_seq = 0
        self._self_test_last_full_seq = None
        self._self_test_frame_number = 0

        # Clean slate so all panels/plots start updating immediately
        self._on_clear_data()
        self.frame_strip.items = []
        self.frame_strip.update()
        self.strip_started = False
        self.strip_frame_counter = 0
        self.last_strip_frame_id = -1
        self.last_strip_meta_idx = 0
        self.meta_counter = 0

        self._add_event("Self-test started", "INFO")
        self._self_test_timer.start()

    def _reset_all_after_self_test(self):
        # Reset all data/state and put UI back to an initial neutral state.
        self._on_clear_data()

        # Reset frame-strip tracking so real frames start from a clean slate
        self.strip_started = False
        self.strip_frame_counter = 0
        self.last_strip_frame_id = -1
        self.last_strip_meta_idx = 0
        self.meta_counter = 0

        # Reset demux buffer (avoid any leftover partial state)
        self.stream_demux.reset()

        # Reset RF gauges
        self.rssi_gauge.set_value(self.rssi_gauge.min_val)
        self.snr_gauge.set_value(self.snr_gauge.min_val)
        self.link_gauge.set_value(self.link_gauge.min_val)
        self.fei_gauge.set_value(self.fei_gauge.min_val)

        # Reset telemetry gauges
        self.temp_gauge.set_value(self.temp_gauge.min_val)
        self.hum_gauge.set_value(self.hum_gauge.min_val)
        self.press_gauge.set_value(self.press_gauge.min_val)

        # Reset meta panel and LEDs
        self.radio_meta_label.setText("No RF meta frames yet.")
        self.meta_arrival_led.setStyleSheet("color: #2A4A34; font-size: 16pt; background: transparent; border: none;")
        self.meta_link_led.setStyleSheet("color: #1F374C; font-size: 16pt; background: transparent; border: none;")

        # Clear frame strip and inspector
        self.frame_strip.items = []
        self.frame_strip.update()
        self.frame_table.setRowCount(0)
        self.frame_table.clearContents()
        self.frame_table.clearSelection()
        self.frame_detail.setPlainText("")
        self.frame_log.setHtml("")
        self.signal_quality_text.setPlainText("")

        self.frame_count_label.setText("Frames: 0")
        self._update_displays()

    def _stop_self_test(self, reset: bool = True):
        if not self.self_test_active:
            return
        self.self_test_active = False
        self._self_test_timer.stop()
        self._add_event("Self-test stopped", "INFO")
        if reset:
            self._reset_all_after_self_test()

    def _self_test_tick(self):
        if not self.self_test_active or self._self_test_start_s is None:
            return

        now_s = time.monotonic()
        t = now_s - self._self_test_start_s

        # --- RF values (pingpong across full configured ranges) ---
        rssi = self._pingpong(t, -164, 0, 6.0)
        snr = self._pingpong(t * 1.07, -10, 15, 5.0)
        link = self._pingpong(t * 0.85, 0, 100, 7.0)
        fei = self._pingpong(t * 1.2, -500, 500, 4.5)
        offset = self._pingpong(t * 0.6, -10000, 10000, 10.0)

        # Emit META frame periodically (so the LEDs + meta timing are tested too)
        if (now_s - self._self_test_last_meta_emit_s) >= self._self_test_meta_period_s:
            self._self_test_last_meta_emit_s = now_s
            now_ts = time.time()

            if self.last_meta_ts is not None:
                gap = now_ts - self.last_meta_ts
                self.meta_last_gap = gap
                self.meta_gap_samples += 1
                if gap > self.meta_max_gap:
                    self.meta_max_gap = gap
                if self.meta_avg_gap is None:
                    self.meta_avg_gap = gap
                else:
                    self.meta_avg_gap = 0.85 * self.meta_avg_gap + 0.15 * gap

            self.last_meta_ts = now_ts
            self.last_meta_arrival_flash = now_ts

            mf = RfMetaFrame(
                rssi_pkt_dbm=int(round(rssi)),
                rssi_inst_dbm=int(round(rssi - 70.0)),
                snr_db=float(snr),
                fei_hz=int(round(fei)),
                offset_hz=int(round(offset)),
                bw_bits=7,
                sf=12,
                cr=4,
                rx_nb_bytes=0,
                modem_stat=0x00,
                irq_flags=0x00,
                rx_header_cnt=self.frames_received,
                rx_packet_cnt=self.frames_received,
                link_score=int(round(link)),
                seq=self._self_test_seq & 0xFF,
                met=int(round(t * 2)),
                flags=0x03,
                payload_len=0,
                crc_ok=True,
            )
            self.rf_meta_history.append(mf)
            if len(self.rf_meta_history) > 2000:
                self.rf_meta_history = self.rf_meta_history[-2000:]
            self.meta_frames_parsed += 1

        # Emit telemetry frames periodically (feeds frame strip + frame inspector + stats)
        if (now_s - self._self_test_last_frame_emit_s) >= self._self_test_frame_period_s:
            self._self_test_last_frame_emit_s = now_s
            self._self_test_frame_number += 1

            is_full = (self._self_test_frame_number % 6) == 0
            frame_type = "FULL" if is_full else "DELTA"
            crc_ok = (self._self_test_frame_number % 17) != 0

            if frame_type == "FULL" and crc_ok:
                self._self_test_last_full_seq = self._self_test_seq & 0xFF

            if frame_type == "DELTA" and self._self_test_last_full_seq is None:
                status = "NO_REF"
                crc_valid = True
                error_reason = "No reference keyframe"
                ref_seq = None
            else:
                status = "ACCEPTED" if crc_ok else "CRC_ERROR"
                crc_valid = crc_ok
                error_reason = None if crc_ok else "CRC mismatch (self-test)"
                ref_seq = self._self_test_last_full_seq if frame_type == "DELTA" else None

            sync = 0xA5 if frame_type == "FULL" else 0xA4
            frame_len = 38 if frame_type == "FULL" else 23
            raw = bytearray([0] * frame_len)
            raw[0] = sync
            met_tick = int(round(t * 2)) & 0xFFFF
            raw[1] = met_tick & 0xFF
            raw[2] = (met_tick >> 8) & 0xFF
            raw[3] = 0
            raw[4] = self._self_test_seq & 0xFF
            if frame_type == "DELTA":
                raw[5] = (ref_seq or 0) & 0xFF

            crc_calc = 0xBEEF
            crc_recv = 0xBEEF if crc_ok else 0xDEAD

            self.telemetry_decoder.frames.append(
                FrameInfo(
                    frame_number=len(self.telemetry_decoder.frames),
                    frame_type=frame_type,
                    sequence=self._self_test_seq & 0xFF,
                    ref_sequence=ref_seq,
                    crc_valid=crc_valid,
                    error_reason=error_reason,
                    status=status,
                    raw_data=bytes(raw),
                    crc_calculated=crc_calc,
                    crc_received=crc_recv,
                )
            )
            self._self_test_seq += 1

            # Telemetry samples (8 per frame) for plots + gauges
            temp = self._pingpong(t * 0.9, -40, 85, 12.0)
            hum = self._pingpong(t * 1.1, 0, 100, 9.0)
            press = self._pingpong(t * 0.7, 822, 1077, 14.0)
            for i in range(8):
                jitter = (i - 3.5) * 0.15
                self.temperatures.append(float(temp + jitter))
                self.humidities.append(float(max(0.0, min(100.0, hum + jitter * 2.0))))
                self.pressures.append(float(press + jitter * 1.0))
            self._sample_buffer_temp.extend(self.temperatures[-8:])
            self._sample_buffer_hum.extend(self.humidities[-8:])
            self._sample_buffer_press.extend(self.pressures[-8:])
            self.frames_received += 1
            self.frame_count_label.setText(f"Frames: {self.frames_received}")

    def _read_serial_data(self, data: bytes):
        """Read and process serial data from serial handler signal"""
        if not data:
            return

        # Push to stream demux
        result = self.stream_demux.ingest(data)
        telem_chunks = result.telemetry_frames
        meta_chunks = result.meta_frames
        desync_dropped = result.dropped_bytes

        if desync_dropped > 0:
            self.stream_desync_dropped += desync_dropped

            # Suppress desync warnings during initial 3 seconds (ESP32 boot messages are normal)
            if self.connection_timestamp is None or (time.time() - self.connection_timestamp) > 3.0:
                self._add_event(f"Desync: {desync_dropped} bytes dropped", "WARN")

        # Process telemetry chunks
        for chunk in telem_chunks:
            prev_frames = len(self.telemetry_decoder.frames)
            temps, hums, press, keyframes = self.telemetry_decoder.decode_frames(chunk)
            new_frames = len(self.telemetry_decoder.frames) - prev_frames

            # Store all samples for historical data
            self.temperatures.extend(temps)
            self.humidities.extend(hums)
            self.pressures.extend(press)

            # Add to sample buffer for 2Hz gauge updates
            self._sample_buffer_temp.extend(temps)
            self._sample_buffer_hum.extend(hums)
            self._sample_buffer_press.extend(press)

            if new_frames > 0:
                self.frames_received += new_frames
                for i in range(new_frames):
                    fn = prev_frames + i
                    if self._pending_meta_frames:
                        self._frame_meta.append(self._pending_meta_frames.pop(0))
                    else:
                        self._frame_meta.append(None)
                        self._pending_frame_numbers.append(fn)

        # Process meta chunks
        for chunk in meta_chunks:
            meta_frames = self.rf_meta_decoder.push_data(chunk)
            for mf in meta_frames:
                self.rf_meta_history.append(mf)
                self.meta_frames_parsed += 1
                if not mf.crc_ok:
                    self.meta_crc_bad += 1
                if self._pending_frame_numbers:
                    fn = self._pending_frame_numbers.pop(0)
                    if fn < len(self._frame_meta):
                        self._frame_meta[fn] = mf
                        self._frame_meta_dirty = True
                else:
                    self._pending_meta_frames.append(mf)
                # Meta timing: measure interval between received META frames
                now_ts = time.time()
                if self.last_meta_ts is not None:
                    gap = now_ts - self.last_meta_ts
                    self.meta_last_gap = gap
                    self.meta_gap_samples += 1
                    if gap > self.meta_max_gap:
                        self.meta_max_gap = gap
                    if self.meta_avg_gap is None:
                        self.meta_avg_gap = gap
                    else:
                        # EMA to smooth jitter
                        self.meta_avg_gap = 0.85 * self.meta_avg_gap + 0.15 * gap
            self.last_meta_ts = now_ts
            self.last_meta_arrival_flash = now_ts
            # Temporary debug: log META values
            logger.info(f"META: RSSI_PKT={mf.rssi_pkt_dbm} RSSI_INST={mf.rssi_inst_dbm} SNR={mf.snr_db:.1f} CRC={'OK' if mf.crc_ok else 'BAD'}")

        # Update frame counter
        self.frame_count_label.setText(f"{self.frames_received}")

        # Update all displays (gauges, status indicators, plots)
        self._update_displays()

    def _on_record_clicked(self):
        """Handle record button"""
        if self.btn_record.isChecked():
            self.btn_record.setText("⏹ STOP")
            self.is_recording = True
            self._add_event("Recording started", "INFO")
        else:
            self.btn_record.setText("⚫ REC")
            self.is_recording = False
            self._add_event("Recording stopped", "INFO")

    def _on_frame_selected(self):
        """Handle frame table selection"""
        selected = self.frame_table.selectedItems()
        if not selected:
            return

        row = selected[0].row()
        frames = self.telemetry_decoder.get_recent_frames(self.frame_table_limit)
        if row >= len(frames):
            return

        frame = frames[row]

        # Display frame details
        crc_calc = f"{frame.crc_calculated:04X}" if frame.crc_calculated is not None else "N/A"
        crc_recv = f"{frame.crc_received:04X}" if frame.crc_received is not None else "N/A"

        detail = f"""
Frame #{frame.frame_number}
Type: {frame.frame_type}
Sequence: {frame.sequence}
Reference: {frame.ref_sequence if frame.ref_sequence else 'N/A'}
Status: {frame.status}
CRC Valid: {frame.crc_valid}
CRC Calculated: {crc_calc}
CRC Received: {crc_recv}
Error: {frame.error_reason if frame.error_reason else 'None'}

Raw Data ({len(frame.raw_data)} bytes):
{frame.raw_data.hex(' ').upper()}
"""
        # Only update if text changed AND no active selection/scroll
        # (prevents clearing selection and resetting scroll position)
        cursor = self.frame_detail.textCursor()
        scrollbar = self.frame_detail.verticalScrollBar()
        is_scrolled = scrollbar.value() != scrollbar.minimum()

        if self.frame_detail.toPlainText() != detail and not cursor.hasSelection() and not is_scrolled:
            self.frame_detail.setPlainText(detail)

    def _update_displays(self):
        """Update all displays (except gauges which update at 2Hz)"""
        # Update RF meta gauges (these update on new packets)
        if self.rf_meta_history:
            latest = self.rf_meta_history[-1]
            self.rssi_gauge.set_value(latest.rssi_pkt_dbm)
            self.snr_gauge.set_value(latest.snr_db)
            self.fei_gauge.set_value(latest.fei_hz)
            self.link_gauge.set_value(latest.link_score)

            # Élő késés a legutóbbi meta óta (nem írjuk felül a timestampet itt)
            age = 0.0 if self.last_meta_ts is None else (time.time() - self.last_meta_ts)
            lock_state = "LOCK" if age < 3.0 else "LOST"

            # LED1: meta arrival pulse (zöld pár tizedmásodpercig)
            now_t = time.time()
            if now_t - self.last_meta_arrival_flash < 0.6:
                self.meta_arrival_led.setStyleSheet("color: #32FF6A; font-size: 16pt; background: transparent; border: none;")
            else:
                self.meta_arrival_led.setStyleSheet("color: #2A4A34; font-size: 16pt; background: transparent; border: none;")

            # LED2: link state (AVG alapú küszöb: ha aktuális gap > (avg+1s) → fault)
            ref_gap = self.meta_avg_gap if self.meta_avg_gap is not None else (self.meta_max_gap if self.meta_max_gap > 0 else 2.0)
            threshold = ref_gap + 1.0
            gap_fault = (self.meta_last_gap > threshold) or (age > threshold)
            if not gap_fault:
                self.meta_link_led.setStyleSheet("color: #00A8FF; font-size: 16pt; background: transparent; border: none;")
            else:
                blink_on = int(time.time() * 2) % 2 == 0
                color = "#FF3B30" if blink_on else "#1F374C"
                self.meta_link_led.setStyleSheet(f"color: {color}; font-size: 16pt; background: transparent; border: none;")

            bw_map = {
                0: "7.8k", 1: "10.4k", 2: "15.6k", 3: "20.8k",
                4: "31.25k", 5: "41.7k", 6: "62.5k", 7: "125k",
                8: "250k", 9: "500k"
            }
            bw_txt = bw_map.get(latest.bw_bits, f"{latest.bw_bits}")
            tuned_freq_mhz = 433.200 + (latest.offset_hz / 1e6)
            meta_text = (
                f"LAST {self.meta_last_gap:0.1f}s | AVG {(self.meta_avg_gap or 0):0.1f}s | MAX {self.meta_max_gap:0.1f}s\n"
                f"LIVE {age:0.1f}s  TUNE {tuned_freq_mhz:0.3f}MHz ({latest.offset_hz:+.0f}Hz)\n"
                f"RSSI {latest.rssi_pkt_dbm: .1f}/{latest.rssi_inst_dbm: .1f} dBm  SNR {latest.snr_db: .1f} dB\n"
                f"FEI {latest.fei_hz: .0f}Hz  BW/SF/CR {bw_txt}/SF{latest.sf}/CR{latest.cr}\n"
                f"SEQ {latest.seq}  MET {latest.met}\n"
                f"RX {latest.rx_packet_cnt}p/{latest.rx_header_cnt}h  LINK {latest.link_score}%  CRC {latest.crc_ok}"
            )
            self.radio_meta_label.setText(meta_text)
        else:
            self.radio_meta_label.setText("No RF meta frames yet.")

        # Feed frame strip from decoder frames
        frames = self.telemetry_decoder.get_recent_frames(self.frame_table_limit)
        if frames:
            new_id = self.last_strip_frame_id
            for f in frames:
                if f.frame_number is not None and f.frame_number > self.last_strip_frame_id:
                    ftype = (f.frame_type or "UNKNOWN").upper()
                    flen = len(f.raw_data) if f.raw_data else 0
                    crc_ok = bool(f.crc_valid)
                    # Start counting only from first valid FULL frame
                    if not self.strip_started:
                        if ftype == "FULL" and crc_ok:
                            self.strip_started = True
                            self.strip_frame_counter = 0
                        else:
                            # pre-sync neutral block, no ID
                            self.frame_strip.add_frame("PRE", flen, crc_ok, fid=None)
                            new_id = max(new_id, f.frame_number)
                            continue
                    # After sync, increment counter in sync with displayed frames
                    self.strip_frame_counter += 1
                    self.frame_strip.add_frame(ftype, flen, crc_ok, fid=self.strip_frame_counter)
                    new_id = max(new_id, f.frame_number)
            if new_id > self.last_strip_frame_id:
                self.last_strip_frame_id = new_id

        # Add any new META frames to strip
        if self.last_strip_meta_idx < len(self.rf_meta_history):
            for mf in self.rf_meta_history[self.last_strip_meta_idx:]:
                self.meta_counter += 1
                # Only show META ids after sync start; before that neutral
                if self.strip_started:
                    # no numeric id for META to avoid keverés a számlálóval
                    self.frame_strip.add_frame("META", 28, mf.crc_ok, fid=None)
                else:
                    self.frame_strip.add_frame("PRE", 28, mf.crc_ok, fid=None)
            self.last_strip_meta_idx = len(self.rf_meta_history)

        # NOTE: Telemetry gauges (temp, hum, press) are updated by _update_gauges_from_buffer at 2Hz

        # Frame/error stats (for alerts and indicators below)
        stats = self.telemetry_decoder.get_frame_stats()
        crc_errors = stats.get('crc_errors', 0)
        # Update error indicators
        ref_errors = stats['ref_mismatch'] + stats['no_ref']
        self._update_status_box(self.ref_err_indicator, str(ref_errors),
                               Colors.STATUS_WARNING if ref_errors > 0 else Colors.STATUS_NOMINAL)
        self._update_status_box(self.trunc_indicator, str(stats['truncated']),
                               Colors.STATUS_WARNING if stats['truncated'] > 0 else Colors.STATUS_NOMINAL)
        self._update_status_box(self.desync_indicator, str(self.stream_desync_dropped),
                               Colors.STATUS_CRITICAL if self.stream_desync_dropped > 0 else Colors.STATUS_NOMINAL)

        # Trigger alerts for NEW errors (only after boot period)
        if crc_errors > self._last_crc_error_count:
            # Only alert if significant CRC errors (> 5)
            if crc_errors > 5:
                self._trigger_error_alert("CRC ERROR DETECTED!", Colors.STATUS_CRITICAL)
            self._last_crc_error_count = crc_errors

        if ref_errors > self._last_ref_error_count:
            # Only alert if significant ref errors (> 10)
            if ref_errors > 10:
                self._trigger_error_alert("REF ERROR DETECTED!", Colors.STATUS_WARNING)
            self._last_ref_error_count = ref_errors

        # Desync alert is already suppressed during boot in _read_serial_data
        self._last_desync_count = self.stream_desync_dropped

        # Update plots
        self._update_plots()

        # Update frame inspector table
        self._update_frame_table()

        # Update logs
        self._update_logs()

        # Update signal analysis
        self._update_signal_analysis()

    def _update_status_box(self, widget: QWidget, value: str, color: str):
        """Update status box value and color"""
        widget.value_label.setText(value)
        widget.value_label.setStyleSheet(f"color: {color}; border: none;")
        widget.setStyleSheet(f"""
            QFrame {{
                background-color: {Colors.BG_DARK};
                border: 2px solid {color};
                border-radius: 4px;
                padding: 8px;
            }}
        """)

    def _update_plots(self):
        """Update all plots"""
        if not self.temperatures:
            return

        # Get last 300 points
        n = len(self.temperatures)
        window = min(300, n)
        start_idx = max(0, n - window)

        # Create time axis (relative to first sample)
        rel_times = [(i - start_idx) * 4 for i in range(start_idx, n)]  # 4 seconds per frame

        # Temperature
        self.plot_temp.clear()
        self.plot_temp.plot(rel_times, self.temperatures[start_idx:n],
                           pen=pg.mkPen(Colors.TELEM_TEMPERATURE, width=2), antialias=True)

        # Humidity
        self.plot_hum.clear()
        self.plot_hum.plot(rel_times, self.humidities[start_idx:n],
                          pen=pg.mkPen(Colors.TELEM_HUMIDITY, width=2), antialias=True)

        # Pressure
        self.plot_press.clear()
        self.plot_press.plot(rel_times, self.pressures[start_idx:n],
                            pen=pg.mkPen(Colors.TELEM_PRESSURE, width=2), antialias=True)

        # Signal plots
        if self.rf_meta_history:
            m = len(self.rf_meta_history)
            sig_window = min(100, m)
            sig_start = max(0, m - sig_window)

            sig_times = list(range(sig_window))

            rssi_vals = [mf.rssi_pkt_dbm for mf in self.rf_meta_history[sig_start:m]]
            snr_vals = [mf.snr_db for mf in self.rf_meta_history[sig_start:m]]

            self.plot_rssi.clear()
            self.plot_rssi.plot(sig_times, rssi_vals,
                               pen=pg.mkPen(Colors.STATUS_WARNING, width=2), antialias=True)

            self.plot_snr.clear()
            self.plot_snr.plot(sig_times, snr_vals,
                              pen=pg.mkPen(Colors.STATUS_NOMINAL, width=2), antialias=True)

    def _update_frame_table(self):
        """Update frame inspector table"""
        frames = self.telemetry_decoder.get_recent_frames(self.frame_table_limit)
        current_rows = self.frame_table.rowCount()

        # Only update if there are NEW frames (don't re-render everything!)
        if len(frames) == current_rows and not self._frame_meta_dirty:
            return

        if len(frames) == current_rows and self._frame_meta_dirty:
            for row in range(current_rows):
                f = frames[row]
                if 0 <= f.frame_number < len(self._frame_meta):
                    mf = self._frame_meta[f.frame_number]
                else:
                    mf = None
                if mf is None or not mf.crc_ok:
                    continue
                rssi_text = f"{mf.rssi_pkt_dbm}"
                snr_text = f"{mf.snr_db:.1f}"
                rssi_item = self.frame_table.item(row, 7)
                snr_item = self.frame_table.item(row, 8)
                if rssi_item is not None:
                    rssi_item.setText(rssi_text)
                if snr_item is not None:
                    snr_item.setText(snr_text)
            self._frame_meta_dirty = False
            return

        # Block signals during bulk update to prevent lag
        self.frame_table.setUpdatesEnabled(False)

        # Only add NEW rows, don't touch existing ones
        start_row = current_rows
        self.frame_table.setRowCount(len(frames))

        for row in range(start_row, len(frames)):
            f = frames[row]
            crc_calc = f"{f.crc_calculated:04X}" if f.crc_calculated is not None else "-"
            crc_recv = f"{f.crc_received:04X}" if f.crc_received is not None else "-"

            # Get signal metadata if available
            rssi = "-"
            snr = "-"
            if 0 <= f.frame_number < len(self._frame_meta):
                mf = self._frame_meta[f.frame_number]
            else:
                mf = None
            if mf is not None and mf.crc_ok:
                rssi = f"{mf.rssi_pkt_dbm}"
                snr = f"{mf.snr_db:.1f}"

            values = [
                f"{f.frame_number}",
                f.frame_type,
                f"{f.sequence}",
                "-" if f.ref_sequence is None else f"{f.ref_sequence}",
                f.status,
                crc_calc,
                crc_recv,
                rssi,
                snr,
            ]

            # Color-code based on status
            if f.status == "CRC_ERROR":
                row_color = QColor(Colors.STATUS_CRITICAL)
                text_color = QColor("white")
            elif f.status in ("REF_MISMATCH", "NO_REF", "TRUNCATED"):
                row_color = QColor(Colors.STATUS_WARNING)
                text_color = QColor("black")
            else:
                row_color = QColor(Colors.BG_DARK)
                text_color = QColor(Colors.TEXT_PRIMARY)

            for col, val in enumerate(values):
                item = QTableWidgetItem(val)
                item.setFlags(item.flags() ^ Qt.ItemFlag.ItemIsEditable)
                item.setBackground(row_color)
                item.setForeground(text_color)
                self.frame_table.setItem(row, col, item)

        # Re-enable updates
        self.frame_table.setUpdatesEnabled(True)

        if self.chk_autoscroll.isChecked() and len(frames) > 0:
            self.frame_table.scrollToBottom()

    def _update_logs(self):
        """Update frame and event logs"""
        # Frame log
        frames = self.telemetry_decoder.get_recent_frames(50)
        lines = []

        color_map = {
            "ACCEPTED": Colors.TEXT_PRIMARY,
            "CRC_ERROR": Colors.STATUS_CRITICAL,
            "NO_REF": Colors.STATUS_WARNING,
            "REF_MISMATCH": Colors.STATUS_WARNING,
            "TRUNCATED": Colors.STATUS_WARNING,
        }

        for f in frames:
            color = color_map.get(f.status, Colors.TEXT_PRIMARY)
            crc_str = "OK" if f.crc_valid else f"FAIL ({f.crc_calculated:04X}/{f.crc_received:04X})"

            lines.append(
                f"<span style='color:{color}'>"
                f"#{f.frame_number:05d} {f.frame_type:<5} SEQ={f.sequence:03d} "
                f"REF={f.ref_sequence if f.ref_sequence is not None else '-':>3} "
                f"STATUS={f.status:<11} CRC={crc_str}"
                f"</span>"
            )

        scrollbar = self.frame_log.verticalScrollBar()
        prev_value = scrollbar.value()
        was_at_bottom = prev_value >= (scrollbar.maximum() - 2)
        self.frame_log.setHtml("<br>".join(lines))
        if self.chk_autoscroll.isChecked() or was_at_bottom:
            scrollbar.setValue(scrollbar.maximum())
        else:
            scrollbar.setValue(min(prev_value, scrollbar.maximum()))

        # Event log auto-scroll
        if self.chk_autoscroll_events.isChecked():
            scrollbar = self.event_log.verticalScrollBar()
            scrollbar.setValue(scrollbar.maximum())

    def _update_signal_analysis(self):
        """Update signal analysis view"""
        if not self.rf_meta_history:
            return

        latest = self.rf_meta_history[-1]

        # Radio parameters (bw_bits: 0-9, firmware uses 7=125kHz)
        bw_map = {
            0: "7.8kHz", 1: "10.4kHz", 2: "15.6kHz", 3: "20.8kHz",
            4: "31.25kHz", 5: "41.7kHz", 6: "62.5kHz",
            7: "125kHz", 8: "250kHz", 9: "500kHz"
        }
        bw_str = bw_map.get(latest.bw_bits, f"Unknown ({latest.bw_bits})")

        # Coding rate: cr value (1-4) from firmware -> display as 4/5, 4/6, 4/7, 4/8
        cr_map = {1: "4/5", 2: "4/6", 3: "4/7", 4: "4/8"}
        cr_str = cr_map.get(latest.cr, f"Unknown ({latest.cr})")

        params_text = f"""
Spreading Factor: {latest.sf}
Bandwidth: {bw_str}
Coding Rate: {cr_str}
Frequency Offset: {latest.offset_hz} Hz
Packets Received: {latest.rx_packet_cnt}
Headers Received: {latest.rx_header_cnt}
"""
        # Only update if text changed AND no active selection (prevents clearing selection)
        cursor = self.radio_params_text.textCursor()
        if self.radio_params_text.toPlainText() != params_text and not cursor.hasSelection():
            self.radio_params_text.setPlainText(params_text)

        # Signal quality
        # rssi_pkt_dbm = packet RSSI (signal strength)
        # rssi_inst_dbm = instant noise floor
        quality_text = f"""
RSSI (Signal): {latest.rssi_pkt_dbm} dBm
Noise Floor: {latest.rssi_inst_dbm} dBm
SNR: {latest.snr_db:.1f} dB
Frequency Error: {latest.fei_hz} Hz
Link Score: {latest.link_score}%

Modem Status: 0x{latest.modem_stat:02X}
IRQ Flags: 0x{latest.irq_flags:02X}
RX Bytes: {latest.rx_nb_bytes}
"""
        # Only update if text changed AND no active selection (prevents clearing selection)
        cursor = self.signal_quality_text.textCursor()
        if self.signal_quality_text.toPlainText() != quality_text and not cursor.hasSelection():
            self.signal_quality_text.setPlainText(quality_text)

    def _update_gauges_from_buffer(self):
        """Update telemetry gauges from sample buffer at 2Hz"""
        # If we have samples in buffer, show the next one
        if self._current_gauge_index < len(self._sample_buffer_temp):
            self.temp_gauge.set_value(self._sample_buffer_temp[self._current_gauge_index])
            self.hum_gauge.set_value(self._sample_buffer_hum[self._current_gauge_index])
            self.press_gauge.set_value(self._sample_buffer_press[self._current_gauge_index])
            self._current_gauge_index += 1

        # Clean up old samples from buffer once we've shown them
        if self._current_gauge_index > 100:  # Keep buffer from growing indefinitely
            self._sample_buffer_temp = self._sample_buffer_temp[self._current_gauge_index:]
            self._sample_buffer_hum = self._sample_buffer_hum[self._current_gauge_index:]
            self._sample_buffer_press = self._sample_buffer_press[self._current_gauge_index:]
            self._current_gauge_index = 0

    def _update_met(self):
        """Update MET display"""
        if self.temperatures:
            # Calculate MET based on number of samples (4 seconds per frame, 8 samples per frame)
            total_seconds = (len(self.temperatures) / 8) * 4
            hours = int(total_seconds // 3600)
            minutes = int((total_seconds % 3600) // 60)
            seconds = int(total_seconds % 60)
            centiseconds = int((total_seconds % 1) * 100)

            self.met_label.setText(f"{hours:02d}:{minutes:02d}:{seconds:02d}.{centiseconds:02d}")

    def _add_event(self, msg: str, severity: str = "INFO"):
        """Add event to event log"""
        color_map = {
            "INFO": Colors.STATUS_NOMINAL,
            "WARN": Colors.STATUS_WARNING,
            "CRITICAL": Colors.STATUS_CRITICAL,
        }
        color = color_map.get(severity, Colors.TEXT_PRIMARY)
        timestamp = QTime.currentTime().toString("hh:mm:ss.zzz")

        # Save cursor position and selection before appending
        cursor = self.event_log.textCursor()
        old_pos = cursor.position()
        had_selection = cursor.hasSelection()

        self.event_log.append(f"<font color='{color}'>[{timestamp}] {msg}</font>")

        # Restore cursor position if user had selection
        if had_selection:
            cursor.setPosition(old_pos)
            self.event_log.setTextCursor(cursor)

    def _trigger_error_alert(self, message: str, color: str):
        """Trigger visual and audio alert"""
        # Flash status bar
        self.status_bar.setStyleSheet(f"background-color: {color}; color: white; font-weight: bold;")
        self.status_bar.showMessage(f"⚠️ ALERT: {message}")

        # Reset after 2 seconds
        QTimer.singleShot(2000, lambda: self.status_bar.setStyleSheet(""))

        # Log event
        self._add_event(message, "CRITICAL")

        # System beep
        try:
            import sys
            if sys.platform == "darwin":
                import os
                os.system("afplay /System/Library/Sounds/Ping.aiff &")
            elif sys.platform == "win32":
                import winsound
                winsound.Beep(1000, 200)
            else:
                print('\a')
        except Exception:
            print('\a')

    def _on_open_recording(self):
        """Open recording file"""
        filename, _ = QFileDialog.getOpenFileName(
            self, "Open Telemetry Recording", "", "Binary Files (*.bin);;All Files (*)"
        )
        if filename:
            self._add_event(f"Opening {filename}", "INFO")

    def _on_save_recording(self):
        """Save recording file"""
        if not self.temperatures:
            QMessageBox.warning(self, "No Data", "No telemetry data to save")
            return

        filename, _ = QFileDialog.getSaveFileName(
            self, "Save Telemetry Recording", "", "Binary Files (*.bin);;All Files (*)"
        )
        if filename:
            self._add_event(f"Saving to {filename}", "INFO")

    def _on_clear_data(self):
        """Clear all data"""
        self.temperatures.clear()
        self.humidities.clear()
        self.pressures.clear()
        self.met_ticks.clear()
        self.rf_meta_history.clear()
        self.keyframe_indices.clear()
        self._pending_meta_frames.clear()
        self._pending_frame_numbers.clear()
        self._frame_meta.clear()
        self._frame_meta_dirty = False
        self._sample_buffer_temp.clear()
        self._sample_buffer_hum.clear()
        self._sample_buffer_press.clear()
        self._current_gauge_index = 0
        self.telemetry_decoder.reset()
        self.rf_meta_decoder.reset()
        self.frames_received = 0
        self.meta_frames_parsed = 0
        self.meta_crc_bad = 0
        self.stream_desync_dropped = 0
        self.last_meta_ts = None
        self.meta_avg_gap = None
        self.meta_max_gap = 0.0
        self.meta_last_gap = 0.0
        self.meta_gap_samples = 0
        self.last_meta_arrival_flash = 0.0
        self._last_crc_error_count = 0
        self._last_ref_error_count = 0
        self._last_desync_count = 0
        self._add_event("Data cleared", "INFO")

    def _on_about(self):
        """Show about dialog"""
        QMessageBox.about(
            self,
            "About Professional Mission Control",
            f"{APP_NAME} v{APP_VERSION}\n\n"
            "Professional telemetry monitoring for LoRa missions\n\n"
            "OpenMCT-style interface with real-time data visualization"
        )

    def toggle_fullscreen(self):
        """Toggle fullscreen mode"""
        if self.isFullScreen():
            self.showNormal()
            self.btn_fullscreen.setText("⛶")
        else:
            self.showFullScreen()
            self.btn_fullscreen.setText("⛶")

    def keyPressEvent(self, event):
        """Handle keyboard shortcuts"""
        if event.key() == Qt.Key.Key_F11:
            self.toggle_fullscreen()
        elif event.key() == Qt.Key.Key_Escape and self.isFullScreen():
            self.showNormal()
            self.btn_fullscreen.setText("⛶")
        else:
            super().keyPressEvent(event)
