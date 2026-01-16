"""
Professional Mission Control Dashboard
ESA/NASA/SpaceX-grade telemetry visualization with real-time monitoring
"""

import logging
from dataclasses import dataclass
from typing import List, Optional

import numpy as np
import pyqtgraph as pg
from PyQt6.QtWidgets import (
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QComboBox,
    QTabWidget,
    QFrame,
    QGridLayout,
    QProgressBar,
    QSplitter,
    QListWidget,
    QListWidgetItem,
    QScrollArea,
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QFont, QColor, QIcon

from config import APP_NAME, APP_VERSION, PLOT_UPDATE_INTERVAL
from core.telemetry import TelemetryDecoder
from core.serial_handler import SerialHandler
from core.data_manager import DataManager
from ui.themes.mission_control import Colors, MissionControlTheme

logger = logging.getLogger(__name__)


@dataclass
class SignalQuality:
    """Signal quality metrics"""

    rssi: float  # dBm (-140 to 0)
    snr: float  # dB
    packet_error_rate: float  # 0 to 1
    last_frame_age_ms: int  # milliseconds since last frame


class GaugeWidget(QWidget):
    """
    Professional real-time gauge display (circular indicator)
    Used for Temperature, Humidity, Pressure with needle animation
    """

    def __init__(
        self,
        name: str,
        unit: str,
        min_val: float,
        max_val: float,
        warn_threshold: float = None,
        crit_threshold: float = None,
        parent=None,
    ):
        super().__init__(parent)
        self.name = name
        self.unit = unit
        self.min_val = min_val
        self.max_val = max_val
        self.warn_threshold = warn_threshold
        self.crit_threshold = crit_threshold
        self.current_value = min_val

        self.setMinimumSize(180, 180)
        self._init_ui()

    def _init_ui(self):
        """Initialize gauge display"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(2)

        # Title
        title_font = QFont("Arial", 10, QFont.Weight.Bold)
        title = QLabel(self.name)
        title.setFont(title_font)
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(title)

        # Chart gauge (simple bar plot)
        self.plot = pg.PlotWidget()
        self.plot.setBackground(Colors.BG_DARK)
        self.plot.showGrid(False)
        self.plot.hideAxis("bottom")
        self.plot.hideAxis("left")
        self.plot.setLabel("left", self.unit, color=Colors.TEXT_PRIMARY)
        self.plot.setYRange(self.min_val, self.max_val)
        layout.addWidget(self.plot, 1)

        # Value display (large, centered)
        self.value_label = QLabel("--.-")
        self.value_label.setFont(QFont("Menlo", 16, QFont.Weight.Bold))
        self.value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.value_label.setStyleSheet(f"color: {Colors.TEXT_PRIMARY};")
        layout.addWidget(self.value_label)

        # Unit
        unit_label = QLabel(self.unit)
        unit_label.setFont(QFont("Arial", 9))
        unit_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        unit_label.setStyleSheet(f"color: {Colors.TEXT_SECONDARY};")
        layout.addWidget(unit_label)

    def set_value(self, value: float):
        """Update gauge with new value"""
        self.current_value = max(self.min_val, min(self.max_val, value))
        self.value_label.setText(f"{self.current_value:.1f}")

        # Determine color based on status
        if self.crit_threshold and self.current_value >= self.crit_threshold:
            color = Colors.STATUS_CRITICAL
        elif self.warn_threshold and self.current_value >= self.warn_threshold:
            color = Colors.STATUS_WARNING
        else:
            color = Colors.STATUS_NOMINAL

        self.value_label.setStyleSheet(f"color: {color};")

        # Draw gauge bar (simple horizontal bar)
        self.plot.clear()
        norm_value = (self.current_value - self.min_val) / (self.max_val - self.min_val)
        self.plot.plot([0, norm_value * 100], [0, 0], pen=pg.mkPen(color, width=3))


class SignalQualityPanel(QFrame):
    """
    Display signal quality metrics (RSSI, SNR, packet loss, etc.)
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFrameShape(QFrame.Shape.StyledPanel)
        self.setStyleSheet(f"background-color: {Colors.BG_PANEL};")
        self._init_ui()

    def _init_ui(self):
        """Initialize signal quality display"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)

        # Title
        title = QLabel("SIGNAL QUALITY")
        title.setFont(QFont("Arial", 11, QFont.Weight.Bold))
        title.setStyleSheet(f"color: {Colors.TEXT_PRIMARY};")
        layout.addWidget(title)

        # RSSI indicator
        rssi_layout = QHBoxLayout()
        rssi_label = QLabel("RSSI:")
        rssi_label.setMinimumWidth(60)
        rssi_label.setStyleSheet(f"color: {Colors.TEXT_SECONDARY};")
        rssi_layout.addWidget(rssi_label)

        self.rssi_progress = QProgressBar()
        self.rssi_progress.setMinimum(-140)
        self.rssi_progress.setMaximum(0)
        self.rssi_progress.setValue(-100)
        self.rssi_progress.setStyleSheet(
            f"""
            QProgressBar {{
                background-color: {Colors.BG_DARK};
                border: 1px solid {Colors.BORDER};
                border-radius: 3px;
            }}
            QProgressBar::chunk {{
                background-color: {Colors.STATUS_NOMINAL};
            }}
        """
        )
        rssi_layout.addWidget(self.rssi_progress)

        self.rssi_value = QLabel("-- dBm")
        self.rssi_value.setMinimumWidth(80)
        self.rssi_value.setStyleSheet(f"color: {Colors.TEXT_PRIMARY};")
        rssi_layout.addWidget(self.rssi_value)
        layout.addLayout(rssi_layout)

        # SNR indicator
        snr_layout = QHBoxLayout()
        snr_label = QLabel("SNR:")
        snr_label.setMinimumWidth(60)
        snr_label.setStyleSheet(f"color: {Colors.TEXT_SECONDARY};")
        snr_layout.addWidget(snr_label)

        self.snr_value = QLabel("-- dB")
        self.snr_value.setStyleSheet(f"color: {Colors.TEXT_PRIMARY};")
        snr_layout.addWidget(self.snr_value)
        snr_layout.addStretch()
        layout.addLayout(snr_layout)

        # Packet error rate
        per_layout = QHBoxLayout()
        per_label = QLabel("PKT ERR:")
        per_label.setMinimumWidth(60)
        per_label.setStyleSheet(f"color: {Colors.TEXT_SECONDARY};")
        per_layout.addWidget(per_label)

        self.per_value = QLabel("0.0%")
        self.per_value.setStyleSheet(f"color: {Colors.TEXT_PRIMARY};")
        per_layout.addWidget(self.per_value)
        per_layout.addStretch()
        layout.addLayout(per_layout)

        # Frame age
        age_layout = QHBoxLayout()
        age_label = QLabel("LAST RX:")
        age_label.setMinimumWidth(60)
        age_label.setStyleSheet(f"color: {Colors.TEXT_SECONDARY};")
        age_layout.addWidget(age_label)

        self.age_value = QLabel("-- ms")
        self.age_value.setStyleSheet(f"color: {Colors.TEXT_PRIMARY};")
        age_layout.addWidget(self.age_value)
        age_layout.addStretch()
        layout.addLayout(age_layout)

        layout.addStretch()

    def update_quality(self, quality: SignalQuality):
        """Update all signal quality indicators"""
        self.rssi_value.setText(f"{quality.rssi:.0f} dBm")
        self.rssi_progress.setValue(int(quality.rssi))

        self.snr_value.setText(f"{quality.snr:.1f} dB")
        self.per_value.setText(f"{quality.packet_error_rate*100:.1f}%")
        self.age_value.setText(f"{quality.last_frame_age_ms} ms")


class EventLogPanel(QFrame):
    """
    Real-time mission event log with color-coded severity
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFrameShape(QFrame.Shape.StyledPanel)
        self.setStyleSheet(f"background-color: {Colors.BG_PANEL};")
        self._init_ui()

    def _init_ui(self):
        """Initialize event log display"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)

        # Title
        title = QLabel("MISSION EVENTS")
        title.setFont(QFont("Arial", 11, QFont.Weight.Bold))
        title.setStyleSheet(f"color: {Colors.TEXT_PRIMARY};")
        layout.addWidget(title)

        # Event list
        self.event_list = QListWidget()
        self.event_list.setStyleSheet(
            f"""
            QListWidget {{
                background-color: {Colors.BG_DARK};
                border: 1px solid {Colors.BORDER};
                border-radius: 3px;
                color: {Colors.TEXT_PRIMARY};
                font-family: Menlo;
                font-size: 9px;
            }}
            QListWidget::item {{
                padding: 3px 5px;
            }}
        """
        )
        layout.addWidget(self.event_list)

    def add_event(self, timestamp: str, event: str, severity: str = "INFO"):
        """
        Add an event to the log

        Args:
            timestamp: Event timestamp (e.g., "12:34:56.789")
            event: Event description
            severity: "INFO", "WARN", "CRITICAL"
        """
        color_map = {
            "INFO": Colors.STATUS_NOMINAL,
            "WARN": Colors.STATUS_WARNING,
            "CRITICAL": Colors.STATUS_CRITICAL,
        }
        color = color_map.get(severity, Colors.TEXT_PRIMARY)

        item = QListWidgetItem(f"[{timestamp}] {event}")
        item.setForeground(QColor(color))
        self.event_list.insertItem(0, item)

        # Keep only last 100 events
        while self.event_list.count() > 100:
            self.event_list.takeItem(self.event_list.count() - 1)


class MissionDashboard(QMainWindow):
    """
    Professional Mission Control Dashboard
    Real-time telemetry visualization following ESA/NASA design standards
    """

    def __init__(self):
        super().__init__()

        # Core components
        self.telemetry_decoder = TelemetryDecoder()
        self.serial_handler = SerialHandler(self)
        self.data_manager = DataManager()

        # State
        self.current_mode = "OFFLINE"
        self.is_recording = False
        self.signal_quality = SignalQuality(
            rssi=-120, snr=0, packet_error_rate=0, last_frame_age_ms=9999
        )

        # Data buffers
        self.temperatures: List[float] = []
        self.humidities: List[float] = []
        self.pressures: List[float] = []
        self.frame_times: List[float] = []
        self.raw_buffer = bytearray()  # For raw stream hex dump display

        # Frame statistics
        self.frames_received = 0
        self.frames_dropped = 0
        self.last_frame_time = 0

        # Mission Elapsed Time (MET) tracking
        self.mission_start_time: Optional[float] = None
        self.met_seconds = 0.0

        # Setup UI
        self.setWindowTitle(f"{APP_NAME} ({APP_VERSION}) - Mission Dashboard")
        self.setMinimumSize(1600, 1000)

        # Apply theme
        theme = MissionControlTheme()
        self.setStyleSheet(theme.get_stylesheet())

        self._setup_ui()
        self._setup_connections()
        self._setup_timers()

        # Auto-detect Heltec module and initialize
        self._auto_initialize()

        logger.info("MissionDashboard initialized")

    def _auto_initialize(self):
        """Auto-detect Heltec module and initialize connection"""
        # Get available ports
        available_ports = self.serial_handler.list_ports()
        logger.info(f"Available ports: {available_ports}")

        # Look for Heltec board (typically "usbserial" on macOS, "COM" on Windows, "ttyUSB" on Linux)
        heltec_port = None
        for port in available_ports:
            port_lower = port.lower()
            if any(
                keyword in port_lower
                for keyword in ["usbserial", "ch340", "usb", "heltec", "lora"]
            ):
                heltec_port = port
                logger.info(f"Auto-detected Heltec module on port: {heltec_port}")
                break

        # Populate port combo
        if available_ports:
            self.port_combo.addItems(available_ports)

        # If Heltec found, switch to ONLINE and auto-connect
        if heltec_port:
            self.port_combo.setCurrentText(heltec_port)
            self.mode_combo.setCurrentText("ONLINE")
            self.current_mode = "ONLINE"
            self.port_combo.setEnabled(True)
            self.btn_connect.setEnabled(True)

            # Auto-connect after a short delay to allow UI to settle
            QTimer.singleShot(500, lambda: self._auto_connect(heltec_port))

            # Auto-start recording
            QTimer.singleShot(1000, self._auto_start_recording)

            self.event_log.add_event(
                "--:--:--.--", "Heltec module detected - AUTO CONNECT", "INFO"
            )
        else:
            # Stay in OFFLINE mode
            self.current_mode = "OFFLINE"
            self.port_combo.setEnabled(False)
            self.btn_connect.setEnabled(False)
            self.event_log.add_event(
                "--:--:--.--", "No Heltec module detected - OFFLINE MODE", "WARN"
            )

    def _auto_connect(self, port: str):
        """Auto-connect to the specified port"""
        logger.info(f"Auto-connecting to {port}")
        self.serial_handler.connect(port)
        self.btn_connect.setChecked(True)

    def _auto_start_recording(self):
        """Auto-start recording"""
        self.btn_record.setChecked(True)
        started = self.data_manager.start_recording()
        self.is_recording = started
        if started:
            self.recording_indicator.setText("🔴")
            self.recording_indicator.setStyleSheet(f"color: {Colors.STATUS_CRITICAL};")
            logger.info("Recording started automatically")
        else:
            self.btn_record.setChecked(False)
            self.is_recording = False
            logger.warning("AUTO recording failed to start")

    def _setup_ui(self):
        """Build the main dashboard layout"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(4, 4, 4, 4)
        main_layout.setSpacing(4)

        # Top status bar
        main_layout.addLayout(self._create_status_bar())

        # Main splitter: left (gauges+trends) | right (signal+events)
        main_splitter = QSplitter(Qt.Orientation.Horizontal)

        # LEFT: Gauges and Trends
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(4)

        # Telemetry title
        title_label = QLabel("TELEMETRY")
        title_label.setStyleSheet(
            f"color: {Colors.TEXT_PRIMARY}; font-weight: bold; font-size: 11px;"
        )
        left_layout.addWidget(title_label)

        # Gauge grid (3 columns)
        gauge_grid = QGridLayout()
        gauge_grid.setSpacing(4)
        gauge_grid.setContentsMargins(0, 0, 0, 0)

        self.gauge_temp = GaugeWidget(
            "Temperature", "°C", -50, 50, warn_threshold=40, crit_threshold=45
        )
        self.gauge_temp.setMinimumSize(200, 200)
        self.gauge_humidity = GaugeWidget(
            "Humidity", "%", 0, 100, warn_threshold=90, crit_threshold=95
        )
        self.gauge_humidity.setMinimumSize(200, 200)
        self.gauge_pressure = GaugeWidget(
            "Pressure", "hPa", 300, 1050, warn_threshold=1020, crit_threshold=1040
        )
        self.gauge_pressure.setMinimumSize(200, 200)

        gauge_grid.addWidget(self.gauge_temp, 0, 0)
        gauge_grid.addWidget(self.gauge_humidity, 0, 1)
        gauge_grid.addWidget(self.gauge_pressure, 0, 2)
        left_layout.addLayout(gauge_grid, 0)

        # Trend plots
        left_layout.addWidget(self._create_trend_plots(), 1)
        main_splitter.addWidget(left_widget)

        # RIGHT: Signal Quality + Mission Events
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(4)

        self.signal_panel = SignalQualityPanel()
        self.event_log = EventLogPanel()

        right_splitter = QSplitter(Qt.Orientation.Vertical)
        right_splitter.addWidget(self.signal_panel)
        right_splitter.addWidget(self.event_log)
        right_splitter.setSizes([300, 400])

        right_layout.addWidget(right_splitter)
        main_splitter.addWidget(right_widget)

        # Set splitter proportions
        main_splitter.setSizes([1200, 400])
        main_layout.addWidget(main_splitter, 1)

        # Bottom: Raw data viewer tabs
        main_layout.addWidget(self._create_bottom_panels(), 0)

    def _create_status_bar(self) -> QHBoxLayout:
        """Create top status bar with connection, mode, and vital statistics"""
        layout = QHBoxLayout()
        layout.setSpacing(12)

        # Mode selector
        layout.addWidget(QLabel("Mode:"))
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["OFFLINE", "ONLINE"])
        self.mode_combo.setFixedWidth(100)
        layout.addWidget(self.mode_combo)

        layout.addSpacing(12)

        # Port selector (only visible in ONLINE mode)
        layout.addWidget(QLabel("Port:"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(150)
        layout.addWidget(self.port_combo)

        # Connect button
        self.btn_connect = QPushButton("● CONNECT")
        self.btn_connect.setCheckable(True)
        self.btn_connect.setFixedSize(120, 36)
        self.btn_connect.setStyleSheet(
            f"""
            QPushButton {{
                background-color: {Colors.STATUS_NOMINAL};
                color: {Colors.BG_DARK};
                border-radius: 3px;
                font-weight: bold;
            }}
            QPushButton:checked {{
                background-color: {Colors.STATUS_WARNING};
            }}
        """
        )
        layout.addWidget(self.btn_connect)

        layout.addSpacing(20)

        # Mission Elapsed Time (MET)
        layout.addWidget(QLabel("MET:"))
        self.met_label = QLabel("00:00:00.0")
        self.met_label.setFont(QFont("Menlo", 12, QFont.Weight.Bold))
        self.met_label.setStyleSheet(f"color: {Colors.STATUS_NOMINAL};")
        layout.addWidget(self.met_label)

        layout.addSpacing(20)

        # Frame count
        layout.addWidget(QLabel("Frames:"))
        self.frame_count_label = QLabel("0")
        self.frame_count_label.setFont(QFont("Menlo", 10))
        layout.addWidget(self.frame_count_label)

        # Recording indicator
        self.recording_indicator = QLabel("⭕")
        self.recording_indicator.setStyleSheet(f"color: {Colors.TEXT_SECONDARY};")
        layout.addWidget(self.recording_indicator)

        # Record button
        self.btn_record = QPushButton("● REC")
        self.btn_record.setCheckable(True)
        self.btn_record.setFixedSize(80, 32)
        self.btn_record.setStyleSheet(
            f"""
            QPushButton {{
                background-color: {Colors.BG_PANEL};
                border: 1px solid {Colors.BORDER};
                border-radius: 3px;
            }}
            QPushButton:checked {{
                background-color: {Colors.STATUS_CRITICAL};
                color: white;
            }}
        """
        )
        layout.addWidget(self.btn_record)

        layout.addStretch()

        return layout

    def _create_trend_plots(self) -> QWidget:
        """Create mini trend plots for each telemetry parameter"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        # Title
        title_label = QLabel("TRENDS (5 min)")
        title_label.setStyleSheet(
            f"color: {Colors.TEXT_PRIMARY}; font-weight: bold; font-size: 11px;"
        )
        layout.addWidget(title_label)

        # Grid layout for plots
        grid = QGridLayout()
        grid.setSpacing(4)
        grid.setContentsMargins(0, 0, 0, 0)

        # Temperature trend
        self.plot_temp_trend = pg.PlotWidget(title="Temp (°C)")
        self.plot_temp_trend.setLabel("left", "°C")
        self.plot_temp_trend.setBackground(Colors.BG_DARK)
        self.plot_temp_trend.showGrid(True, True, alpha=0.2)
        self.plot_temp_trend.setMinimumHeight(150)
        grid.addWidget(self.plot_temp_trend, 0, 0)

        # Humidity trend
        self.plot_humidity_trend = pg.PlotWidget(title="Humidity (%)")
        self.plot_humidity_trend.setLabel("left", "%")
        self.plot_humidity_trend.setBackground(Colors.BG_DARK)
        self.plot_humidity_trend.showGrid(True, True, alpha=0.2)
        self.plot_humidity_trend.setMinimumHeight(150)
        grid.addWidget(self.plot_humidity_trend, 0, 1)

        # Pressure trend
        self.plot_pressure_trend = pg.PlotWidget(title="Pressure (hPa)")
        self.plot_pressure_trend.setLabel("left", "hPa")
        self.plot_pressure_trend.setBackground(Colors.BG_DARK)
        self.plot_pressure_trend.showGrid(True, True, alpha=0.2)
        self.plot_pressure_trend.setMinimumHeight(150)
        grid.addWidget(self.plot_pressure_trend, 0, 2)

        layout.addLayout(grid, 1)
        return widget

    def _create_bottom_panels(self) -> QWidget:
        """Create bottom panels with raw data, frame inspector, etc."""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)

        # Label
        label = QLabel("DATA STREAMS")
        label.setStyleSheet(
            f"color: {Colors.TEXT_PRIMARY}; font-weight: bold; font-size: 10px;"
        )
        layout.addWidget(label)

        tabs = QTabWidget()
        tabs.setStyleSheet(
            f"""
            QTabWidget {{
                background-color: {Colors.BG_PANEL};
            }}
            QTabBar::tab {{
                background-color: {Colors.BG_SECONDARY};
                color: {Colors.TEXT_PRIMARY};
                padding: 4px 12px;
                margin: 0px;
            }}
            QTabBar::tab:selected {{
                background-color: {Colors.BG_TERTIARY};
                border-bottom: 2px solid {Colors.STATUS_NOMINAL};
            }}
        """
        )

        # Raw stream tab
        raw_widget = QWidget()
        raw_layout = QVBoxLayout(raw_widget)
        raw_layout.setContentsMargins(4, 4, 4, 4)
        self.raw_text = QLabel("Waiting for data...")
        self.raw_text.setFont(QFont("Menlo", 8))
        self.raw_text.setWordWrap(True)
        self.raw_text.setAlignment(
            Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft
        )
        self.raw_text.setStyleSheet(
            f"color: {Colors.TEXT_PRIMARY}; background-color: {Colors.BG_DARK};"
        )
        scroll = QScrollArea()
        scroll.setWidget(self.raw_text)
        scroll.setStyleSheet(f"QScrollArea {{ border: none; }}")
        raw_layout.addWidget(scroll)
        tabs.addTab(raw_widget, "📡 Raw Stream")

        # Frame inspector tab
        inspector_widget = QWidget()
        inspector_layout = QVBoxLayout(inspector_widget)
        inspector_layout.setContentsMargins(4, 4, 4, 4)
        self.frame_inspector = QLabel("Frame details...")
        self.frame_inspector.setFont(QFont("Menlo", 8))
        self.frame_inspector.setWordWrap(True)
        self.frame_inspector.setAlignment(
            Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft
        )
        self.frame_inspector.setStyleSheet(
            f"color: {Colors.TEXT_PRIMARY}; background-color: {Colors.BG_DARK};"
        )
        scroll2 = QScrollArea()
        scroll2.setWidget(self.frame_inspector)
        scroll2.setStyleSheet(f"QScrollArea {{ border: none; }}")
        inspector_layout.addWidget(scroll2)
        tabs.addTab(inspector_widget, "🔍 Frame Inspector")

        tabs.setMaximumHeight(180)
        layout.addWidget(tabs)
        return widget

    def _setup_connections(self):
        """Connect signals and slots"""
        self.mode_combo.currentTextChanged.connect(self._on_mode_changed)
        self.btn_connect.clicked.connect(self._on_connect_clicked)
        self.btn_record.clicked.connect(self._on_record_clicked)

        # Serial signals
        self.serial_handler.data_received.connect(self._on_serial_data)
        self.serial_handler.connection_changed.connect(self._on_connection_changed)
        self.serial_handler.error_occurred.connect(self._on_serial_error)

    def _setup_timers(self):
        """Setup update timers"""
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update_displays)
        self.update_timer.start(PLOT_UPDATE_INTERVAL)

        self.met_timer = QTimer()
        self.met_timer.timeout.connect(self._update_met)
        self.met_timer.start(100)

    def _on_mode_changed(self, mode: str):
        """Handle mode change"""
        self.current_mode = mode
        self.port_combo.setEnabled(mode == "ONLINE")
        self.btn_connect.setEnabled(mode == "ONLINE")

    def _on_connect_clicked(self):
        """Handle connect/disconnect"""
        if self.btn_connect.isChecked():
            port = self.port_combo.currentText()
            if port:
                self.serial_handler.connect(port)
        else:
            self.serial_handler.disconnect()

    def _on_record_clicked(self):
        """Handle recording toggle"""
        if self.btn_record.isChecked():
            # Start recording
            self.is_recording = True
            self.data_manager.start_recording()
            self.recording_indicator.setText("🔴")
            self.recording_indicator.setStyleSheet(f"color: {Colors.STATUS_CRITICAL};")
            self.event_log.add_event(
                self.met_label.text(), "Recording started", severity="INFO"
            )
            logger.info("Recording started")
        else:
            # Stop recording and save file
            self.is_recording = False
            filename = self.data_manager.stop_recording()
            self.recording_indicator.setText("⭕")
            self.recording_indicator.setStyleSheet(f"color: {Colors.TEXT_SECONDARY};")
            self.event_log.add_event(
                self.met_label.text(),
                f"Recording saved: {filename}",
                severity="INFO",
            )
            logger.info(f"Recording stopped and saved to {filename}")

    def _on_serial_data(self, data: bytes):
        """Handle incoming serial data"""
        self.frames_received += 1
        self.frame_count_label.setText(str(self.frames_received))

        # Save to recording if active
        if self.is_recording:
            self.data_manager.append_recording(data)

        # Update raw stream hex dump (keep last 500 bytes)
        self.raw_buffer.extend(data)
        if len(self.raw_buffer) > 500:
            self.raw_buffer = self.raw_buffer[-500:]
        hex_dump = self._format_hex_dump(bytes(self.raw_buffer))
        self.raw_text.setText(hex_dump)

        # Update signal quality (mock realistic data)
        self.signal_quality.rssi = -100 + (self.frames_received % 40)
        self.signal_quality.snr = 5 + (self.frames_received % 10)
        self.signal_quality.packet_error_rate = min(
            0.05, self.frames_dropped / max(1, self.frames_received)
        )
        self.signal_quality.last_frame_age_ms = 50

        # Decode telemetry
        temps, humidities, pressures, keyframes = self.telemetry_decoder.decode_frames(
            data
        )

        if temps:
            self.temperatures.extend(temps)
            self.humidities.extend(humidities)
            self.pressures.extend(pressures)

            # Update latest value for gauges
            self.gauge_temp.set_value(temps[-1])
            self.gauge_humidity.set_value(humidities[-1])
            self.gauge_pressure.set_value(pressures[-1])

            # Add mission event
            self.event_log.add_event(
                self.met_label.text(),
                f"Received {len(temps)} samples ({len(data)} bytes)",
                severity="INFO",
            )

    def _on_connection_changed(self, connected: bool):
        """Handle connection state change"""
        if connected:
            self.btn_connect.setText("● DISCONNECT")
            self.btn_connect.setChecked(True)
            self.event_log.add_event(
                "--:--:--.--", "Serial connection established", severity="INFO"
            )
        else:
            self.btn_connect.setText("● CONNECT")
            self.btn_connect.setChecked(False)
            self.event_log.add_event(
                "--:--:--.--", "Serial connection closed", severity="WARN"
            )

    def _on_serial_error(self, error_msg: str):
        """Handle serial error"""
        self.event_log.add_event(
            "--:--:--.--", f"Serial error: {error_msg}", severity="CRITICAL"
        )

    def _update_displays(self):
        """Update all display elements"""
        # Update trend plots (clear and redraw with proper X-axis)
        if len(self.temperatures) > 0:
            self.plot_temp_trend.clear()
            self.plot_temp_trend.plot(
                list(range(len(self.temperatures[-300:]))),
                self.temperatures[-300:],
                pen=pg.mkPen(Colors.TELEM_TEMPERATURE, width=2),
            )
            self.plot_humidity_trend.clear()
            self.plot_humidity_trend.plot(
                list(range(len(self.humidities[-300:]))),
                self.humidities[-300:],
                pen=pg.mkPen(Colors.TELEM_HUMIDITY, width=2),
            )
            self.plot_pressure_trend.clear()
            self.plot_pressure_trend.plot(
                list(range(len(self.pressures[-300:]))),
                self.pressures[-300:],
                pen=pg.mkPen(Colors.TELEM_PRESSURE, width=2),
            )

        # Update signal quality panel
        self.signal_panel.update_quality(self.signal_quality)

    def _format_hex_dump(self, data: bytes, width: int = 16) -> str:
        """Format binary data as hex dump with ASCII representation"""
        if not data:
            return "No data"

        lines = []
        for i in range(0, len(data), width):
            chunk = data[i : i + width]
            hex_str = " ".join(f"{b:02X}" for b in chunk)
            ascii_str = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
            lines.append(f"{i:04X}  {hex_str:<{width*3}}  {ascii_str}")

        return "\n".join(lines)

    def _update_met(self):
        """Update Mission Elapsed Time"""
        import time

        if self.mission_start_time is None and self.frames_received > 0:
            # Start MET on first data reception
            self.mission_start_time = time.time()

        if self.mission_start_time is not None:
            elapsed = time.time() - self.mission_start_time
            hours = int(elapsed // 3600)
            minutes = int((elapsed % 3600) // 60)
            seconds = elapsed % 60
            self.met_label.setText(f"{hours:02d}:{minutes:02d}:{seconds:05.2f}")
            self.met_seconds = elapsed
