"""
Main Window - AstroLink Mission Control
Mission Control interface with professional ESA/NASA design
"""

import logging
from pathlib import Path
from typing import List

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
    QGroupBox,
    QTableWidget,
    QTableWidgetItem,
    QSplitter,
    QTabWidget,
    QTextEdit,
    QListWidget,
    QListWidgetItem,
    QMessageBox,
    QButtonGroup,
    QFileDialog,
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QShortcut, QKeySequence, QFont

from config import (
    APP_NAME,
    APP_VERSION,
    DEFAULT_WINDOW_WIDTH,
    DEFAULT_WINDOW_HEIGHT,
    PLOT_UPDATE_INTERVAL,
    USE_OPENGL,
)
from core.telemetry import TelemetryDecoder
from core.serial_handler import SerialHandler, SERIAL_AVAILABLE
from core.data_manager import DataManager
from ui.themes import Colors, MissionControlTheme

logger = logging.getLogger(__name__)


class MainWindow(QMainWindow):
    """
    Main application window
    Professional Mission Control interface following ESA/NASA design standards
    """

    def __init__(self):
        super().__init__()

        # Core components
        self.telemetry_decoder = TelemetryDecoder()
        self.serial_handler = SerialHandler(self)
        self.data_manager = DataManager()

        # Data buffers
        self.temperatures: List[float] = []
        self.humidities: List[float] = []
        self.pressures: List[float] = []
        self.keyframe_indices: List[int] = []
        self.raw_buffer = bytearray()

        # State
        self.current_mode = "OFFLINE"  # "OFFLINE" or "ONLINE"
        self.is_fullscreen = False

        # UI Setup
        self.setWindowTitle(f"{APP_NAME} ({APP_VERSION})")
        self.resize(DEFAULT_WINDOW_WIDTH, DEFAULT_WINDOW_HEIGHT)

        self._setup_ui()
        self._setup_plots()
        self._setup_connections()
        self._setup_keyboard_shortcuts()
        self._setup_timers()

        # Initial state
        self._update_mode("OFFLINE")
        self.refresh_file_list()
        self.refresh_port_list()

        logger.info("MainWindow initialized")

    def _setup_ui(self):
        """Create and layout all UI components"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(8)

        # Top control bar
        main_layout.addLayout(self._create_top_bar())

        # Main content area (splitter)
        self.main_splitter = QSplitter(Qt.Orientation.Vertical)
        main_layout.addWidget(self.main_splitter, 1)

        # File selection and controls
        self.main_splitter.addWidget(self._create_file_panel())

        # Tabs (plots, raw data, frame inspector)
        self.main_splitter.addWidget(self._create_tab_widget())

        # Set initial splitter sizes
        self.main_splitter.setSizes([180, 720])

    def _create_top_bar(self) -> QHBoxLayout:
        """Create top control bar with mode, serial, logging controls"""
        layout = QHBoxLayout()
        layout.setSpacing(8)

        # Mode selection
        layout.addWidget(QLabel("Mode:"))
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["OFFLINE", "ONLINE"])
        self.mode_combo.setFixedWidth(120)
        layout.addWidget(self.mode_combo)

        layout.addSpacing(16)

        # Serial port selection
        layout.addWidget(QLabel("Port:"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(200)
        layout.addWidget(self.port_combo)

        self.btn_refresh_ports = QPushButton("⟳")
        self.btn_refresh_ports.setFixedSize(42, 42)
        self.btn_refresh_ports.setToolTip("Refresh serial ports")
        layout.addWidget(self.btn_refresh_ports)

        # Connect/Disconnect buttons
        self.conn_btn_group = QButtonGroup(self)
        self.conn_btn_group.setExclusive(True)

        self.btn_connect = QPushButton("Connect")
        self.btn_connect.setCheckable(True)
        self.btn_connect.setFixedWidth(100)
        self.conn_btn_group.addButton(self.btn_connect)
        layout.addWidget(self.btn_connect)

        self.btn_disconnect = QPushButton("Disconnect")
        self.btn_disconnect.setCheckable(True)
        self.btn_disconnect.setChecked(True)
        self.btn_disconnect.setFixedWidth(100)
        self.conn_btn_group.addButton(self.btn_disconnect)
        layout.addWidget(self.btn_disconnect)

        layout.addSpacing(16)

        # Status LEDs
        layout.addWidget(QLabel("RX:"))
        self.led_rx = QLabel("●")
        self.led_rx.setProperty("ledStatus", True)
        self.led_rx.setFixedSize(36, 36)
        self.led_rx.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.led_rx.setStyleSheet("font-size: 20px;")
        self._set_led_status(self.led_rx, False)
        layout.addWidget(self.led_rx)

        layout.addWidget(QLabel("CONN:"))
        self.led_conn = QLabel("●")
        self.led_conn.setProperty("ledStatus", True)
        self.led_conn.setFixedSize(36, 36)
        self.led_conn.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.led_conn.setStyleSheet("font-size: 20px;")
        self._set_led_status(self.led_conn, False)
        layout.addWidget(self.led_conn)

        layout.addSpacing(16)

        # Bytes counter
        self.lbl_bytes = QLabel("Bytes: 0")
        self.lbl_bytes.setFixedWidth(120)
        self.lbl_bytes.setFrameStyle(3)  # Panel | Sunken
        self.lbl_bytes.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.lbl_bytes)

        # Status label
        self.lbl_status = QLabel("Status: idle")
        self.lbl_status.setMinimumWidth(300)
        self.lbl_status.setFrameStyle(3)  # Panel | Sunken
        layout.addWidget(self.lbl_status)

        # Log button
        self.btn_log = QPushButton("LOG")
        self.btn_log.setCheckable(True)
        self.btn_log.setEnabled(False)
        self.btn_log.setFixedWidth(60)
        self.btn_log.setToolTip("Record telemetry to file")
        layout.addWidget(self.btn_log)

        layout.addSpacing(16)

        # Theme toggle (for testing - can remove in production)
        layout.addWidget(QLabel("Theme:"))
        theme_combo = QComboBox()
        theme_combo.addItems(["Dark"])
        theme_combo.setEnabled(False)  # Only dark theme for now
        theme_combo.setFixedWidth(80)
        layout.addWidget(theme_combo)

        layout.addStretch(1)

        # Fullscreen button
        self.btn_fullscreen = QPushButton("⛶")
        self.btn_fullscreen.setFixedSize(42, 42)
        self.btn_fullscreen.setToolTip("Fullscreen (F11)")
        layout.addWidget(self.btn_fullscreen)

        return layout

    def _create_file_panel(self) -> QWidget:
        """Create file list and control panel"""
        panel = QWidget()
        layout = QHBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        # File list
        self.file_list = QListWidget()
        self.file_list.setMaximumHeight(120)
        self.file_list.setMinimumWidth(320)
        layout.addWidget(self.file_list, 1)

        # Right controls panel
        right_panel = QWidget()
        right_layout = QHBoxLayout(right_panel)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(10)

        # File actions group
        file_actions = QGroupBox("FILE ACTIONS")
        file_actions.setFixedWidth(180)
        file_actions_layout = QVBoxLayout(file_actions)
        file_actions_layout.setSpacing(8)

        self.btn_delete = QPushButton("Delete")
        self.btn_delete.setEnabled(False)
        file_actions_layout.addWidget(self.btn_delete)

        self.btn_refresh_files = QPushButton("Refresh")
        file_actions_layout.addWidget(self.btn_refresh_files)

        self.btn_export = QPushButton("Export CSV")
        self.btn_export.setEnabled(False)
        file_actions_layout.addWidget(self.btn_export)

        file_actions_layout.addStretch()
        right_layout.addWidget(file_actions)

        # Simulate button
        sim_group = QGroupBox("SIMULATION")
        sim_group.setFixedWidth(180)
        sim_layout = QVBoxLayout(sim_group)
        sim_layout.setSpacing(8)

        self.btn_simulate = QPushButton("Sim Flight")
        self.btn_simulate.setToolTip("Generate simulated telemetry data")
        sim_layout.addWidget(self.btn_simulate)

        sim_layout.addStretch()
        right_layout.addWidget(sim_group)

        # Playback controls
        playback_group = QGroupBox("PLAYBACK")
        playback_group.setFixedWidth(200)
        playback_layout = QVBoxLayout(playback_group)

        playback_buttons = QHBoxLayout()
        self.btn_play = QPushButton("▶")
        self.btn_play.setFixedSize(40, 32)
        self.btn_play.setEnabled(False)
        playback_buttons.addWidget(self.btn_play)

        self.btn_pause = QPushButton("⏸")
        self.btn_pause.setFixedSize(40, 32)
        self.btn_pause.setEnabled(False)
        playback_buttons.addWidget(self.btn_pause)

        self.btn_stop = QPushButton("⏹")
        self.btn_stop.setFixedSize(40, 32)
        self.btn_stop.setEnabled(False)
        playback_buttons.addWidget(self.btn_stop)

        playback_layout.addLayout(playback_buttons)
        playback_layout.addStretch()
        right_layout.addWidget(playback_group)

        # View controls
        view_group = QGroupBox("VIEW")
        view_group.setFixedWidth(200)
        view_layout = QVBoxLayout(view_group)

        # Plot selection buttons
        plot_select = QHBoxLayout()
        self.plot_btn_group = QButtonGroup(self)
        self.plot_btn_group.setExclusive(True)

        self.btn_view_t = QPushButton("T")
        self.btn_view_t.setCheckable(True)
        self.btn_view_t.setFixedSize(50, 32)
        self.plot_btn_group.addButton(self.btn_view_t)
        plot_select.addWidget(self.btn_view_t)

        self.btn_view_h = QPushButton("H")
        self.btn_view_h.setCheckable(True)
        self.btn_view_h.setFixedSize(50, 32)
        self.plot_btn_group.addButton(self.btn_view_h)
        plot_select.addWidget(self.btn_view_h)

        self.btn_view_p = QPushButton("P")
        self.btn_view_p.setCheckable(True)
        self.btn_view_p.setChecked(True)  # Default: Pressure
        self.btn_view_p.setFixedSize(50, 32)
        self.plot_btn_group.addButton(self.btn_view_p)
        plot_select.addWidget(self.btn_view_p)

        view_layout.addLayout(plot_select)

        # Layout toggle
        layout_row = QHBoxLayout()
        self.btn_single_plot = QPushButton("Single")
        self.btn_single_plot.setCheckable(True)
        self.btn_single_plot.setToolTip("Show only selected plot")
        layout_row.addWidget(self.btn_single_plot)

        self.btn_layout_toggle = QPushButton("H")
        self.btn_layout_toggle.setCheckable(True)
        self.btn_layout_toggle.setChecked(True)  # Horizontal default
        self.btn_layout_toggle.setFixedSize(40, 32)
        self.btn_layout_toggle.setToolTip("Toggle layout: H=Horizontal, V=Vertical")
        layout_row.addWidget(self.btn_layout_toggle)

        view_layout.addLayout(layout_row)
        view_layout.addStretch()
        right_layout.addWidget(view_group)

        right_layout.addStretch(1)
        layout.addWidget(right_panel, 0)

        return panel

    def _create_tab_widget(self) -> QTabWidget:
        """Create tabbed interface for plots, raw data, frame inspector"""
        tabs = QTabWidget()

        # Tab 1: Telemetry Plots
        plots_widget = QWidget()
        plots_layout = QVBoxLayout(plots_widget)
        plots_layout.setContentsMargins(0, 0, 0, 0)

        # Plots will be added in _setup_plots()
        self.plots_container = plots_layout

        tabs.addTab(plots_widget, "Telemetry Plots")

        # Tab 2: Decoded Samples Table
        self.sample_table = QTableWidget()
        self.sample_table.setColumnCount(4)
        self.sample_table.setHorizontalHeaderLabels(
            ["Index", "Temperature (°C)", "Humidity (%)", "Pressure (hPa)"]
        )
        tabs.addTab(self.sample_table, "Decoded Samples")

        # Tab 3: Raw Stream
        raw_splitter = QSplitter(Qt.Orientation.Horizontal)

        self.raw_human_view = QTextEdit()
        self.raw_human_view.setReadOnly(True)
        self.raw_human_view.setFont(MissionControlTheme.get_monospace_font(10))
        raw_splitter.addWidget(self.raw_human_view)

        self.raw_hex_view = QTextEdit()
        self.raw_hex_view.setReadOnly(True)
        self.raw_hex_view.setFont(MissionControlTheme.get_monospace_font(10))
        raw_splitter.addWidget(self.raw_hex_view)

        tabs.addTab(raw_splitter, "Raw Stream")

        # Tab 4: Frame Inspector
        self.frame_table = QTableWidget()
        self.frame_table.setColumnCount(7)
        self.frame_table.setHorizontalHeaderLabels(
            ["#", "Type", "SEQ", "REF", "CRC", "Status", "Details"]
        )
        tabs.addTab(self.frame_table, "Frame Inspector")

        return tabs

    def _setup_plots(self):
        """Setup pyqtgraph plots with Mission Control styling"""
        # Configure pyqtgraph
        pg.setConfigOptions(
            background=Colors.BG_PRIMARY,
            foreground=Colors.TEXT_PRIMARY,
            useOpenGL=USE_OPENGL,
            antialias=True,
        )

        # Create plot widget
        self.plot_widget = pg.GraphicsLayoutWidget()
        self.plots_container.addWidget(self.plot_widget)

        # Temperature plot
        self.plot_temp = self.plot_widget.addPlot(row=0, col=0)
        self.plot_temp.setLabel(
            "left",
            "Temperature",
            units="°C",
            color=Colors.TEXT_PRIMARY,
            **{"font-size": "14pt"},
        )
        self.plot_temp.setLabel(
            "bottom", "Sample Index", color=Colors.TEXT_PRIMARY, **{"font-size": "14pt"}
        )
        self.plot_temp.showGrid(x=True, y=True, alpha=0.3)
        self.plot_temp.getAxis("left").setStyle(tickFont=QFont("Arial", 12))
        self.plot_temp.getAxis("bottom").setStyle(tickFont=QFont("Arial", 12))
        self.curve_temp = self.plot_temp.plot(
            pen=pg.mkPen(color=Colors.TELEM_TEMPERATURE, width=3), name="Temperature"
        )

        # Humidity plot
        self.plot_hum = self.plot_widget.addPlot(row=1, col=0)
        self.plot_hum.setLabel(
            "left",
            "Humidity",
            units="%",
            color=Colors.TEXT_PRIMARY,
            **{"font-size": "14pt"},
        )
        self.plot_hum.setLabel(
            "bottom", "Sample Index", color=Colors.TEXT_PRIMARY, **{"font-size": "14pt"}
        )
        self.plot_hum.showGrid(x=True, y=True, alpha=0.3)
        self.plot_hum.getAxis("left").setStyle(tickFont=QFont("Arial", 12))
        self.plot_hum.getAxis("bottom").setStyle(tickFont=QFont("Arial", 12))
        self.curve_hum = self.plot_hum.plot(
            pen=pg.mkPen(color=Colors.TELEM_HUMIDITY, width=3), name="Humidity"
        )

        # Pressure plot
        self.plot_press = self.plot_widget.addPlot(row=2, col=0)
        self.plot_press.setLabel(
            "left",
            "Pressure",
            units="hPa",
            color=Colors.TEXT_PRIMARY,
            **{"font-size": "14pt"},
        )
        self.plot_press.setLabel(
            "bottom", "Sample Index", color=Colors.TEXT_PRIMARY, **{"font-size": "14pt"}
        )
        self.plot_press.showGrid(x=True, y=True, alpha=0.3)
        self.plot_press.getAxis("left").setStyle(tickFont=QFont("Arial", 12))
        self.plot_press.getAxis("bottom").setStyle(tickFont=QFont("Arial", 12))
        self.curve_press = self.plot_press.plot(
            pen=pg.mkPen(color=Colors.TELEM_PRESSURE, width=3), name="Pressure"
        )

        logger.info("Plots initialized with OpenGL acceleration")

    def _setup_connections(self):
        """Connect signals and slots"""
        # Mode change
        self.mode_combo.currentTextChanged.connect(self._on_mode_changed)

        # Serial port
        self.btn_refresh_ports.clicked.connect(self.refresh_port_list)
        self.btn_connect.clicked.connect(self._on_connect)
        self.btn_disconnect.clicked.connect(self._on_disconnect)

        # Serial handler signals
        self.serial_handler.data_received.connect(self._on_serial_data)
        self.serial_handler.connection_changed.connect(self._on_connection_changed)
        self.serial_handler.error_occurred.connect(self._on_serial_error)

        # File operations
        self.btn_refresh_files.clicked.connect(self.refresh_file_list)
        self.btn_delete.clicked.connect(self._on_delete_file)
        self.btn_export.clicked.connect(self._on_export_csv)
        self.file_list.currentItemChanged.connect(self._on_file_selected)
        self.file_list.itemDoubleClicked.connect(self._on_file_double_clicked)

        # Simulation
        self.btn_simulate.clicked.connect(self._on_simulate_flight)

        # Logging
        self.btn_log.clicked.connect(self._on_log_toggle)

        # Fullscreen
        self.btn_fullscreen.clicked.connect(self.toggle_fullscreen)

        # View controls
        self.btn_view_t.clicked.connect(self._update_plot_visibility)
        self.btn_view_h.clicked.connect(self._update_plot_visibility)
        self.btn_view_p.clicked.connect(self._update_plot_visibility)
        self.btn_single_plot.clicked.connect(self._update_plot_visibility)
        self.btn_layout_toggle.clicked.connect(self._update_plot_layout)

    def _setup_keyboard_shortcuts(self):
        """Setup keyboard shortcuts"""
        # F11: Fullscreen
        shortcut_fullscreen = QShortcut(QKeySequence("F11"), self)
        shortcut_fullscreen.activated.connect(self.toggle_fullscreen)

        # F5: Refresh files
        shortcut_refresh = QShortcut(QKeySequence("F5"), self)
        shortcut_refresh.activated.connect(self.refresh_file_list)

        # Ctrl+Q: Quit
        shortcut_quit = QShortcut(QKeySequence("Ctrl+Q"), self)
        shortcut_quit.activated.connect(self.close)

        logger.info("Keyboard shortcuts configured")

    def _setup_timers(self):
        """Setup update timers"""
        # Plot update timer
        self.plot_timer = QTimer(self)
        self.plot_timer.timeout.connect(self._update_plots)
        self.plot_timer.setInterval(PLOT_UPDATE_INTERVAL)

        # RX LED blink timer
        self.rx_blink_timer = QTimer(self)
        self.rx_blink_timer.timeout.connect(
            lambda: self._set_led_status(self.led_rx, False)
        )
        self.rx_blink_timer.setSingleShot(True)

    # ========================================================================
    # Mode and Connection Management
    # ========================================================================

    def _update_mode(self, mode: str):
        """Update application mode (OFFLINE/ONLINE)"""
        self.current_mode = mode
        logger.info(f"Mode changed to: {mode}")

        if mode == "OFFLINE":
            # Disconnect serial if connected
            if self.serial_handler.is_connected:
                self.serial_handler.disconnect()

            # Enable file list
            self.file_list.setEnabled(True)
            self.btn_refresh_files.setEnabled(True)
            self.btn_simulate.setEnabled(True)

            # Disable serial controls
            self.port_combo.setEnabled(False)
            self.btn_refresh_ports.setEnabled(False)
            self.btn_connect.setEnabled(False)

            self.lbl_status.setText("Status: OFFLINE mode")

        else:  # ONLINE
            # Disable file list
            self.file_list.setEnabled(False)
            self.btn_refresh_files.setEnabled(False)
            self.btn_simulate.setEnabled(False)

            # Enable serial controls
            self.port_combo.setEnabled(True)
            self.btn_refresh_ports.setEnabled(True)
            self.btn_connect.setEnabled(True)

            # Clear data
            self._clear_data()

            self.lbl_status.setText("Status: ONLINE mode - select port and connect")

    def _on_mode_changed(self, mode: str):
        """Handle mode combo box change"""
        self._update_mode(mode)

    def refresh_port_list(self):
        """Refresh list of available serial ports"""
        if not SERIAL_AVAILABLE:
            self.port_combo.clear()
            self.port_combo.addItem("pyserial not installed")
            return

        self.port_combo.clear()
        ports = SerialHandler.list_ports()

        if not ports:
            self.port_combo.addItem("No ports found")
            self.btn_connect.setEnabled(False)
            return

        self.port_combo.addItems(ports)

        # Try to auto-select Heltec board
        heltec_port = SerialHandler.find_heltec_port()
        if heltec_port and heltec_port in ports:
            self.port_combo.setCurrentText(heltec_port)
            logger.info(f"Auto-selected Heltec port: {heltec_port}")

        self.btn_connect.setEnabled(True)
        logger.info(f"Found {len(ports)} serial ports")

    def _on_connect(self):
        """Connect to selected serial port"""
        port = self.port_combo.currentText()

        if not port or port == "No ports found":
            self._show_error("No Port", "Please select a valid serial port")
            return

        success = self.serial_handler.connect(port)

        if success:
            self.lbl_status.setText(f"Status: Connected to {port}")
            self.btn_log.setEnabled(True)
            self._clear_data()
            self.plot_timer.start()
        else:
            self.btn_disconnect.setChecked(True)

    def _on_disconnect(self):
        """Disconnect from serial port"""
        self.serial_handler.disconnect()
        self.plot_timer.stop()
        self.btn_log.setEnabled(False)
        self.lbl_status.setText("Status: Disconnected")

    def _on_connection_changed(self, connected: bool):
        """Handle connection state change"""
        self._set_led_status(self.led_conn, connected)

        if not connected:
            self.btn_disconnect.setChecked(True)

    def _on_serial_data(self, data: bytes):
        """Handle incoming serial data"""
        # Blink RX LED
        self._set_led_status(self.led_rx, True)
        self.rx_blink_timer.start(100)

        # Append to buffer
        self.raw_buffer.extend(data)

        # Update byte counter
        self.lbl_bytes.setText(f"Bytes: {len(self.raw_buffer)}")

        # Decode telemetry
        self._decode_telemetry()

        # Record if logging enabled
        if self.btn_log.isChecked():
            self.data_manager.append_recording(data)

    def _on_serial_error(self, error: str):
        """Handle serial port error"""
        logger.error(f"Serial error: {error}")
        self.lbl_status.setText(f"Status: Error - {error}")

    # ========================================================================
    # File Management
    # ========================================================================

    def refresh_file_list(self):
        """Refresh list of .bin files"""
        self.file_list.clear()

        files = self.data_manager.list_telemetry_files()

        for filename in files:
            item = QListWidgetItem(filename)
            item.setData(Qt.ItemDataRole.UserRole, filename)
            self.file_list.addItem(item)

        self._update_file_actions()
        logger.info(f"File list refreshed: {len(files)} files")

    def _on_file_selected(self):
        """Handle file selection change"""
        self._update_file_actions()

    def _on_file_double_clicked(self, item: QListWidgetItem):
        """Load file on double-click"""
        filename = item.data(Qt.ItemDataRole.UserRole)
        self._load_telemetry_file(filename)

    def _load_telemetry_file(self, filename: str):
        """Load and decode telemetry file"""
        data = self.data_manager.load_file(filename)

        if data is None:
            self._show_error("Load Error", f"Failed to load file: {filename}")
            return

        self._clear_data()
        self.raw_buffer = bytearray(data)

        # Decode
        self._decode_telemetry()

        # Update UI
        self.lbl_bytes.setText(f"Bytes: {len(data)}")
        self.lbl_status.setText(f"Status: Loaded {filename}")
        self.btn_export.setEnabled(True)

        logger.info(
            f"Loaded {filename}: {len(data)} bytes, {len(self.temperatures)} samples"
        )

    def _on_delete_file(self):
        """Delete selected file"""
        item = self.file_list.currentItem()
        if not item:
            return

        filename = item.data(Qt.ItemDataRole.UserRole)

        reply = QMessageBox.question(
            self,
            "Delete File",
            f"Delete file?\n\n{filename}",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )

        if reply == QMessageBox.StandardButton.Yes:
            success = self.data_manager.delete_file(filename)

            if success:
                self.refresh_file_list()
                self.lbl_status.setText(f"Status: Deleted {filename}")
            else:
                self._show_error("Delete Error", f"Failed to delete {filename}")

    def _update_file_actions(self):
        """Update file action button states"""
        has_selection = self.file_list.currentItem() is not None
        is_offline = self.current_mode == "OFFLINE"

        self.btn_delete.setEnabled(is_offline and has_selection)

    # ========================================================================
    # Telemetry Decoding and Display
    # ========================================================================

    def _decode_telemetry(self):
        """Decode raw telemetry buffer"""
        if not self.raw_buffer:
            return

        # Reset decoder
        self.telemetry_decoder.reset()

        # Decode
        temps, hums, press, keyframes = self.telemetry_decoder.decode_frames(
            bytes(self.raw_buffer)
        )

        # Update data
        self.temperatures = temps
        self.humidities = hums
        self.pressures = press
        self.keyframe_indices = keyframes

        # Update stats
        stats = self.telemetry_decoder.get_frame_stats()
        logger.debug(f"Decoded: {stats}")

        # Update UI
        self._update_sample_table()
        self._update_raw_views()
        self._update_frame_table()
        self._update_plots()

    def _update_sample_table(self):
        """Update decoded samples table"""
        self.sample_table.setRowCount(len(self.temperatures))

        for i, (temp, hum, press) in enumerate(
            zip(self.temperatures, self.humidities, self.pressures)
        ):
            self.sample_table.setItem(i, 0, QTableWidgetItem(str(i)))
            self.sample_table.setItem(i, 1, QTableWidgetItem(f"{temp:.1f}"))
            self.sample_table.setItem(i, 2, QTableWidgetItem(f"{hum:.1f}"))
            self.sample_table.setItem(i, 3, QTableWidgetItem(f"{press:.1f}"))

    def _update_raw_views(self):
        """Update raw stream views (human-readable and hex)"""
        if not self.raw_buffer:
            self.raw_human_view.clear()
            self.raw_hex_view.clear()
            return

        # Human-readable view
        human_text = f"Buffer Size: {len(self.raw_buffer)} bytes\n"
        human_text += f"Decoded Samples: {len(self.temperatures)}\n"
        human_text += f"FULL Frames: {sum(1 for f in self.telemetry_decoder.frames if f.frame_type == 'FULL')}\n"
        human_text += f"DELTA Frames: {sum(1 for f in self.telemetry_decoder.frames if f.frame_type == 'DELTA')}\n"
        human_text += "\n[Raw Bytes (last 256)]:\n"

        tail = self.raw_buffer[-256:] if len(self.raw_buffer) > 256 else self.raw_buffer
        human_text += " ".join(f"{b:02X}" for b in tail)

        self.raw_human_view.setText(human_text)

        # Hex dump view
        hex_text = ""
        for i in range(0, len(self.raw_buffer), 16):
            chunk = self.raw_buffer[i : i + 16]
            offset = f"{i:04X}"
            hex_bytes = " ".join(f"{b:02X}" for b in chunk)
            ascii_repr = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
            hex_text += f"{offset}: {hex_bytes:<48} | {ascii_repr}\n"

        self.raw_hex_view.setText(hex_text)

    def _update_frame_table(self):
        """Update frame inspector table"""
        frames = self.telemetry_decoder.frames
        self.frame_table.setRowCount(len(frames))

        for i, frame in enumerate(frames):
            self.frame_table.setItem(i, 0, QTableWidgetItem(str(frame.frame_number)))
            self.frame_table.setItem(i, 1, QTableWidgetItem(frame.frame_type))
            self.frame_table.setItem(i, 2, QTableWidgetItem(str(frame.sequence)))
            self.frame_table.setItem(
                i,
                3,
                QTableWidgetItem(
                    str(frame.ref_sequence) if frame.ref_sequence else "-"
                ),
            )
            self.frame_table.setItem(
                i, 4, QTableWidgetItem("✓" if frame.crc_valid else "✗")
            )
            self.frame_table.setItem(
                i, 5, QTableWidgetItem("OK" if frame.crc_valid else "ERROR")
            )
            self.frame_table.setItem(i, 6, QTableWidgetItem(frame.error_reason or ""))

    def _update_plots(self):
        """Update telemetry plots"""
        if not self.temperatures:
            return

        # Update curves
        x = np.arange(len(self.temperatures))

        self.curve_temp.setData(x, np.array(self.temperatures))
        self.curve_hum.setData(x, np.array(self.humidities))
        self.curve_press.setData(x, np.array(self.pressures))

    def _update_plot_visibility(self):
        """Update plot visibility based on view controls"""
        # Determine which plot to show
        if self.btn_view_t.isChecked():
            selected = "T"
        elif self.btn_view_h.isChecked():
            selected = "H"
        else:
            selected = "P"

        single_mode = self.btn_single_plot.isChecked()

        # Update plot visibility
        self.plot_temp.setVisible(selected == "T" or not single_mode)
        self.plot_hum.setVisible(selected == "H" or not single_mode)
        self.plot_press.setVisible(selected == "P" or not single_mode)

        # Auto-resize plot layout
        self.plot_widget.ci.layout.update()

    def _update_plot_layout(self):
        """Toggle plot layout (horizontal/vertical)"""
        is_horizontal = self.btn_layout_toggle.isChecked()
        layout_text = "H" if is_horizontal else "V"
        self.btn_layout_toggle.setText(layout_text)

        # Recreate plot layout
        self.plot_widget.clear()
        self.plots_container.removeWidget(self.plot_widget)

        # Create new plots with new layout
        self._setup_plots()

        # If vertical and single plot mode, show only selected
        if not is_horizontal and self.btn_single_plot.isChecked():
            self._update_plot_visibility()

    # ========================================================================
    # Logging and Export
    # ========================================================================

    def _on_log_toggle(self, checked: bool):
        """Toggle telemetry logging"""
        if checked:
            # Start recording
            success = self.data_manager.start_recording()

            if success:
                self.lbl_status.setText("Status: Recording telemetry...")
                logger.info("Started recording")
            else:
                self.btn_log.setChecked(False)
                self._show_error("Recording Error", "Failed to start recording")

        else:
            # Stop recording
            filename = self.data_manager.stop_recording()

            if filename:
                self.lbl_status.setText(f"Status: Saved {filename}")
                self.refresh_file_list()
                logger.info(f"Stopped recording: {filename}")

    def _on_export_csv(self):
        """Export current data to CSV"""
        if not self.temperatures:
            self._show_error("Export Error", "No data to export")
            return

        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Export to CSV",
            str(self.data_manager.working_dir / "telemetry_export.csv"),
            "CSV Files (*.csv)",
        )

        if filename:
            success = self.data_manager.export_to_csv(
                Path(filename).name, self.temperatures, self.humidities, self.pressures
            )

            if success:
                self.lbl_status.setText(f"Status: Exported to {Path(filename).name}")
            else:
                self._show_error("Export Error", "Failed to export CSV")

    # ========================================================================
    # Simulation
    # ========================================================================

    def _on_simulate_flight(self):
        """Generate simulated flight data"""
        self._clear_data()

        # Simulated flight profile (realistic atmospheric)
        logger.info("Generating simulated flight data...")

        # Ground (stable, 25°C, 50% RH, 1013 hPa)
        for _ in range(30):
            self.temperatures.append(25.0)
            self.humidities.append(50.0)
            self.pressures.append(1013.25)

        # Ascent (pressure drops, temp drops, humidity rises)
        for i in range(100):
            progress = i / 100.0
            temp = 25.0 - progress * 30
            humidity = 50.0 + progress * 30
            pressure = 1013.25 - progress * 300
            self.temperatures.append(temp)
            self.humidities.append(humidity)
            self.pressures.append(pressure)

        # Apogee (high altitude plateau, -30°C, 80% RH, 300 hPa)
        for _ in range(20):
            self.temperatures.append(-5.0)
            self.humidities.append(80.0)
            self.pressures.append(300.0)

        # Descent (mirror ascent)
        for i in range(100):
            progress = i / 100.0
            temp = -5.0 + progress * 30
            humidity = 80.0 - progress * 30
            pressure = 300.0 + progress * 700
            self.temperatures.append(temp)
            self.humidities.append(humidity)
            self.pressures.append(pressure)

        # Landing (stable again)
        for _ in range(30):
            self.temperatures.append(25.0)
            self.humidities.append(50.0)
            self.pressures.append(1013.25)

        # Update UI
        self.lbl_bytes.setText(f"Bytes: {len(self.temperatures) * 4}")
        self._update_sample_table()
        self._update_raw_views()
        self._update_plots()
        self.lbl_status.setText(
            f"Status: Generated {len(self.temperatures)} simulated samples"
        )

        logger.info(f"Generated {len(self.temperatures)} simulated telemetry samples")

    # ========================================================================
    # UI Helpers
    # ========================================================================

    def _set_led_status(self, led: QLabel, active: bool):
        """Set LED indicator status"""
        if active:
            led.setStyleSheet(
                f"""
                QLabel {{
                    background-color: {Colors.STATUS_NOMINAL};
                    color: {Colors.TEXT_INVERSE};
                    border-radius: 12px;
                    font-weight: bold;
                }}
            """
            )
        else:
            led.setStyleSheet(
                f"""
                QLabel {{
                    background-color: {Colors.BG_TERTIARY};
                    color: {Colors.TEXT_TERTIARY};
                    border-radius: 12px;
                }}
            """
            )

    def _clear_data(self):
        """Clear all telemetry data"""
        self.temperatures.clear()
        self.humidities.clear()
        self.pressures.clear()
        self.keyframe_indices.clear()
        self.raw_buffer.clear()
        self.telemetry_decoder.reset()

        self.lbl_bytes.setText("Bytes: 0")
        self._update_plots()
        self.sample_table.setRowCount(0)

    def _show_error(self, title: str, message: str):
        """Show error message dialog"""
        QMessageBox.critical(self, title, message)

    def _show_info(self, title: str, message: str):
        """Show info message dialog"""
        QMessageBox.information(self, title, message)

    # ========================================================================
    # Window Management
    # ========================================================================

    def toggle_fullscreen(self):
        """Toggle fullscreen mode"""
        if self.isFullScreen():
            self.showNormal()
            self.is_fullscreen = False
            logger.info("Exited fullscreen")
        else:
            self.showFullScreen()
            self.is_fullscreen = True
            logger.info("Entered fullscreen")

    def closeEvent(self, event):
        """Handle window close event"""
        # Disconnect serial if connected
        if self.serial_handler.is_connected:
            self.serial_handler.disconnect()

        # Stop recording if active
        if self.data_manager.is_recording():
            self.data_manager.stop_recording()

        logger.info("MainWindow closed")
        event.accept()
