"""
ESA/NASA Mission Control Theme
Professional aerospace color scheme and styling
"""

from PyQt6.QtGui import QColor, QPalette, QFont

# ============================================================================
# COLOR PALETTE - Aerospace Standards
# ============================================================================


class Colors:
    """Mission Control color constants following ESA/NASA guidelines"""

    # Background hierarchy (Deep Space theme)
    BG_PRIMARY = "#0B0C10"  # Deep space black
    BG_SECONDARY = "#1F2833"  # Panel backgrounds
    BG_TERTIARY = "#2D333B"  # Elevated elements (dropdowns, modals)
    BG_HOVER = "#363B44"  # Hover states
    BG_PANEL = "#1F2833"  # Panel backgrounds (alias for consistency)
    BG_DARK = "#0B0C10"  # Dark background (alias)

    # Text hierarchy
    TEXT_PRIMARY = "#F0F0F0"  # Primary text (off-white, eye-friendly)
    TEXT_SECONDARY = "#C5C6C7"  # Secondary labels
    TEXT_TERTIARY = "#8B949E"  # Disabled/inactive text
    TEXT_INVERSE = "#0B0C10"  # Text on light backgrounds

    # Status colors (Industry standard)
    STATUS_NOMINAL = "#00C853"  # Green - All systems GO
    STATUS_CAUTION = "#FFB300"  # Amber - Warning condition
    STATUS_CRITICAL = "#F44336"  # Red - Alert/critical
    STATUS_UNKNOWN = "#757575"  # Gray - No data/indeterminate
    STATUS_INFO = "#2196F3"  # Blue - Informational
    STATUS_WARNING = "#FFB300"  # Amber - Warning condition (alias)

    # Telemetry accent colors
    TELEM_TEMPERATURE = "#00D9FF"  # Cyan
    TELEM_HUMIDITY = "#9D4EDD"  # Purple/Magenta
    TELEM_PRESSURE = "#20B2AA"  # Teal

    # Additional accent colors
    ACCENT_CYAN = "#00D9FF"
    ACCENT_PURPLE = "#9D4EDD"
    ACCENT_TEAL = "#20B2AA"
    ACCENT_ORANGE = "#FF9500"
    ACCENT_PINK = "#FF2D55"

    # UI element colors
    BORDER_SUBTLE = "rgba(255, 255, 255, 0.1)"
    BORDER_EMPHASIS = "rgba(255, 255, 255, 0.2)"
    BORDER_ACTIVE = "#2196F3"
    BORDER = "rgba(255, 255, 255, 0.2)"  # Default border color

    # Frame type colors (for frame inspector)
    FRAME_FULL = "#2ecc71"  # Green - FULL/keyframe
    FRAME_DELTA = "#3498db"  # Blue - DELTA frame
    FRAME_NOISE = "#e74c3c"  # Red - Noise/error
    FRAME_META = "#aaaaaa"  # Gray - Metadata


class MissionControlTheme:
    """
    Mission Control theme manager
    Provides QPalette and stylesheet for consistent ESA/NASA look
    """

    @staticmethod
    def get_palette() -> QPalette:
        """
        Returns QPalette configured for Mission Control dark theme

        Returns:
            QPalette: Configured palette
        """
        palette = QPalette()

        # Window colors
        palette.setColor(QPalette.ColorRole.Window, QColor(Colors.BG_SECONDARY))
        palette.setColor(QPalette.ColorRole.WindowText, QColor(Colors.TEXT_PRIMARY))

        # Base widget colors
        palette.setColor(QPalette.ColorRole.Base, QColor(Colors.BG_PRIMARY))
        palette.setColor(QPalette.ColorRole.AlternateBase, QColor(Colors.BG_TERTIARY))

        # Text colors
        palette.setColor(QPalette.ColorRole.Text, QColor(Colors.TEXT_PRIMARY))
        palette.setColor(QPalette.ColorRole.BrightText, QColor(Colors.STATUS_CRITICAL))
        palette.setColor(
            QPalette.ColorRole.PlaceholderText, QColor(Colors.TEXT_TERTIARY)
        )

        # Button colors
        palette.setColor(QPalette.ColorRole.Button, QColor("#2D3236"))
        palette.setColor(QPalette.ColorRole.ButtonText, QColor(Colors.TEXT_PRIMARY))

        # Tooltips
        palette.setColor(QPalette.ColorRole.ToolTipBase, QColor(Colors.BG_PRIMARY))
        palette.setColor(QPalette.ColorRole.ToolTipText, QColor(Colors.TEXT_PRIMARY))

        # Selection (use transparent, rely on stylesheet for custom outline)
        palette.setColor(QPalette.ColorRole.Highlight, QColor(0, 0, 0, 0))
        palette.setColor(
            QPalette.ColorRole.HighlightedText, QColor(Colors.TEXT_PRIMARY)
        )

        # Links
        palette.setColor(QPalette.ColorRole.Link, QColor(Colors.ACCENT_CYAN))
        palette.setColor(QPalette.ColorRole.LinkVisited, QColor(Colors.ACCENT_PURPLE))

        return palette

    @staticmethod
    def get_stylesheet() -> str:
        """
        Returns complete QSS stylesheet for Mission Control theme

        Returns:
            str: Qt Stylesheet String
        """
        return f"""
        /* ===================================================================
           MISSION CONTROL STYLESHEET - ESA/NASA Standards
           =================================================================== */

        /* Main Window */
        QMainWindow {{
            background-color: {Colors.BG_PRIMARY};
            color: {Colors.TEXT_PRIMARY};
        }}

        /* Group Boxes */
        QGroupBox {{
            border: 1px solid {Colors.BORDER_EMPHASIS};
            border-radius: 6px;
            margin-top: 12px;
            padding: 16px;
            font-weight: 500;
            background-color: {Colors.BG_SECONDARY};
        }}

        QGroupBox::title {{
            subcontrol-origin: margin;
            left: 12px;
            padding: 0 8px;
            color: {Colors.TEXT_SECONDARY};
            font-size: 11px;
            text-transform: uppercase;
            letter-spacing: 0.5px;
        }}

        /* Push Buttons */
        QPushButton {{
            background-color: {Colors.BG_TERTIARY};
            border: 1px solid {Colors.BORDER_EMPHASIS};
            border-radius: 6px;
            padding: 10px 20px;
            color: {Colors.TEXT_PRIMARY};
            font-weight: 500;
            font-size: 13px;
            min-height: 38px;
        }}

        QPushButton:hover {{
            border-color: {Colors.BORDER_ACTIVE};
            background-color: rgba(33, 150, 243, 0.15);
        }}

        QPushButton:pressed {{
            background-color: rgba(33, 150, 243, 0.25);
            border-color: {Colors.BORDER_ACTIVE};
        }}

        QPushButton:checked {{
            background-color: {Colors.ACCENT_CYAN};
            border-color: {Colors.ACCENT_CYAN};
            color: {Colors.TEXT_INVERSE};
        }}

        QPushButton:disabled {{
            color: {Colors.TEXT_TERTIARY};
            border-color: {Colors.BORDER_SUBTLE};
            background-color: {Colors.BG_SECONDARY};
        }}

        /* Combo Boxes */
        QComboBox {{
            background-color: {Colors.BG_TERTIARY};
            border: 1px solid {Colors.BORDER_EMPHASIS};
            border-radius: 4px;
            padding: 8px 16px;
            color: {Colors.TEXT_PRIMARY};
            font-size: 13px;
            min-height: 38px;
        }}

        QComboBox:hover {{
            border-color: {Colors.BORDER_ACTIVE};
        }}

        QComboBox:disabled {{
            color: {Colors.TEXT_TERTIARY};
            border-color: {Colors.BORDER_SUBTLE};
            background-color: {Colors.BG_SECONDARY};
        }}

        QComboBox::drop-down {{
            border: none;
            width: 20px;
        }}

        QComboBox::down-arrow {{
            image: none;
            border-left: 4px solid transparent;
            border-right: 4px solid transparent;
            border-top: 5px solid {Colors.TEXT_SECONDARY};
            margin-right: 8px;
        }}

        QComboBox QAbstractItemView {{
            background-color: {Colors.BG_TERTIARY};
            border: 1px solid {Colors.BORDER_EMPHASIS};
            selection-background-color: {Colors.ACCENT_CYAN};
            selection-color: {Colors.TEXT_INVERSE};
            color: {Colors.TEXT_PRIMARY};
            outline: none;
        }}

        /* Labels */
        QLabel {{
            color: {Colors.TEXT_PRIMARY};
            background-color: transparent;
            font-size: 13px;
        }}

        QLabel:disabled {{
            color: {Colors.TEXT_TERTIARY};
        }}

        /* Status Labels (LED-style indicators) */
        QLabel[ledStatus="true"] {{
            border-radius: 4px;
            padding: 4px 12px;
            font-weight: bold;
            font-size: 10px;
        }}

        /* Tables */
        QTableWidget {{
            background-color: {Colors.BG_PRIMARY};
            alternate-background-color: {Colors.BG_SECONDARY};
            gridline-color: {Colors.BORDER_SUBTLE};
            border: 1px solid {Colors.BORDER_SUBTLE};
            color: {Colors.TEXT_PRIMARY};
            selection-background-color: transparent;
            selection-color: {Colors.TEXT_PRIMARY};
        }}

        QTableWidget::item {{
            padding: 6px;
            border: none;
        }}

        QTableWidget::item:selected {{
            background-color: transparent;
            border: 2px solid {Colors.STATUS_CRITICAL};
        }}

        QHeaderView::section {{
            background-color: {Colors.BG_TERTIARY};
            color: {Colors.TEXT_SECONDARY};
            padding: 8px;
            border: none;
            border-right: 1px solid {Colors.BORDER_SUBTLE};
            border-bottom: 1px solid {Colors.BORDER_EMPHASIS};
            font-weight: 600;
            text-transform: uppercase;
            font-size: 10px;
            letter-spacing: 0.5px;
        }}

        /* List Widgets */
        QListWidget {{
            background-color: {Colors.BG_PRIMARY};
            border: 1px solid {Colors.BORDER_SUBTLE};
            color: {Colors.TEXT_PRIMARY};
            outline: none;
        }}

        QListWidget::item {{
            padding: 8px;
            border-radius: 4px;
        }}

        QListWidget::item:hover {{
            background-color: {Colors.BG_HOVER};
        }}

        QListWidget::item:selected {{
            background-color: transparent;
            border: 2px solid {Colors.STATUS_CRITICAL};
        }}

        /* Text Edit / Plain Text Edit */
        QTextEdit, QPlainTextEdit {{
            background-color: {Colors.BG_PRIMARY};
            border: 1px solid {Colors.BORDER_SUBTLE};
            color: {Colors.TEXT_PRIMARY};
            selection-background-color: {Colors.ACCENT_CYAN};
            selection-color: {Colors.TEXT_INVERSE};
        }}

        /* Scroll Bars */
        QScrollBar:vertical {{
            background-color: {Colors.BG_SECONDARY};
            width: 12px;
            border: none;
        }}

        QScrollBar::handle:vertical {{
            background-color: {Colors.BG_TERTIARY};
            min-height: 30px;
            border-radius: 6px;
        }}

        QScrollBar::handle:vertical:hover {{
            background-color: {Colors.BG_HOVER};
        }}

        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
            height: 0px;
        }}

        QScrollBar:horizontal {{
            background-color: {Colors.BG_SECONDARY};
            height: 12px;
            border: none;
        }}

        QScrollBar::handle:horizontal {{
            background-color: {Colors.BG_TERTIARY};
            min-width: 30px;
            border-radius: 6px;
        }}

        QScrollBar::handle:horizontal:hover {{
            background-color: {Colors.BG_HOVER};
        }}

        QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {{
            width: 0px;
        }}

        /* Sliders */
        QSlider::groove:horizontal {{
            height: 8px;
            border-radius: 4px;
            background: {Colors.BG_TERTIARY};
        }}

        QSlider::sub-page:horizontal {{
            background: {Colors.ACCENT_CYAN};
            border-radius: 4px;
        }}

        QSlider::add-page:horizontal {{
            background: {Colors.BG_SECONDARY};
            border-radius: 4px;
        }}

        QSlider::handle:horizontal {{
            width: 16px;
            height: 16px;
            margin: -5px 0;
            border-radius: 8px;
            background: {Colors.TEXT_PRIMARY};
            border: 2px solid {Colors.BG_TERTIARY};
        }}

        QSlider::handle:horizontal:hover {{
            background: white;
            border-color: {Colors.ACCENT_CYAN};
        }}

        /* Tab Widget */
        QTabWidget::pane {{
            border: 1px solid {Colors.BORDER_SUBTLE};
            background-color: {Colors.BG_PRIMARY};
            border-radius: 4px;
        }}

        QTabBar::tab {{
            background-color: {Colors.BG_SECONDARY};
            color: {Colors.TEXT_SECONDARY};
            padding: 10px 20px;
            margin-right: 2px;
            border-top-left-radius: 4px;
            border-top-right-radius: 4px;
            font-weight: 500;
        }}

        QTabBar::tab:hover {{
            background-color: {Colors.BG_TERTIARY};
            color: {Colors.TEXT_PRIMARY};
        }}

        QTabBar::tab:selected {{
            background-color: {Colors.BG_PRIMARY};
            color: {Colors.ACCENT_CYAN};
            border-bottom: 2px solid {Colors.ACCENT_CYAN};
        }}

        /* Tooltips */
        QToolTip {{
            background-color: {Colors.BG_PRIMARY};
            color: {Colors.TEXT_PRIMARY};
            border: 1px solid {Colors.BORDER_EMPHASIS};
            padding: 6px 10px;
            border-radius: 4px;
            font-size: 11px;
        }}

        /* Splitter */
        QSplitter::handle {{
            background-color: {Colors.BG_SECONDARY};
        }}

        QSplitter::handle:horizontal {{
            width: 2px;
        }}

        QSplitter::handle:vertical {{
            height: 2px;
        }}

        QSplitter::handle:hover {{
            background-color: {Colors.ACCENT_CYAN};
        }}

        /* Progress Bar */
        QProgressBar {{
            border: 1px solid {Colors.BORDER_SUBTLE};
            border-radius: 4px;
            text-align: center;
            background-color: {Colors.BG_SECONDARY};
            color: {Colors.TEXT_PRIMARY};
        }}

        QProgressBar::chunk {{
            background-color: {Colors.ACCENT_CYAN};
            border-radius: 3px;
        }}

        /* Menu Bar */
        QMenuBar {{
            background-color: {Colors.BG_SECONDARY};
            color: {Colors.TEXT_PRIMARY};
            border-bottom: 1px solid {Colors.BORDER_SUBTLE};
        }}

        QMenuBar::item:selected {{
            background-color: {Colors.BG_TERTIARY};
        }}

        QMenu {{
            background-color: {Colors.BG_TERTIARY};
            border: 1px solid {Colors.BORDER_EMPHASIS};
            color: {Colors.TEXT_PRIMARY};
        }}

        QMenu::item:selected {{
            background-color: {Colors.ACCENT_CYAN};
            color: {Colors.TEXT_INVERSE};
        }}

        /* Status Bar */
        QStatusBar {{
            background-color: {Colors.BG_SECONDARY};
            color: {Colors.TEXT_SECONDARY};
            border-top: 1px solid {Colors.BORDER_SUBTLE};
        }}
        """

    @staticmethod
    def get_font(size: int = 11, bold: bool = False) -> QFont:
        """
        Returns standard font for Mission Control interface

        Args:
            size: Font size in points
            bold: Whether font should be bold

        Returns:
            QFont: Configured font
        """
        font = QFont()
        font.setFamily("Inter, Segoe UI, -apple-system, system-ui, sans-serif")
        font.setPointSize(size)
        if bold:
            font.setWeight(QFont.Weight.Bold)
        return font

    @staticmethod
    def get_monospace_font(size: int = 11) -> QFont:
        """
        Returns monospace font for telemetry data display

        Args:
            size: Font size in points

        Returns:
            QFont: Configured monospace font
        """
        font = QFont()
        font.setStyleHint(QFont.StyleHint.Monospace)
        font.setFamily("Roboto Mono, Consolas, Monaco, monospace")
        font.setPointSize(size)
        return font
