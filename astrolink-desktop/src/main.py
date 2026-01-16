#!/usr/bin/env python3
"""
AstroLink Mission Control - Desktop Edition
Professional Telemetry Ground Station Application

Entry point for the application
"""

import sys
import logging
from pathlib import Path

from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import Qt

# Add src directory to Python path
sys.path.insert(0, str(Path(__file__).parent))

from config import (
    APP_NAME,
    APP_VERSION,
    LOG_LEVEL,
    LOG_FORMAT,
    LOG_DATE_FORMAT,
    LOGS_DIR,
)
from ui.themes import MissionControlTheme
from ui.professional_mission_control import ProfessionalMissionControl


def setup_logging():
    """Configure application logging"""
    # Create logs directory if it doesn't exist (best-effort)
    try:
        LOGS_DIR.mkdir(parents=True, exist_ok=True)
    except PermissionError:
        pass

    handlers: list[logging.Handler] = [logging.StreamHandler(sys.stdout)]
    try:
        handlers.insert(0, logging.FileHandler(LOGS_DIR / "astrolink.log"))
    except (PermissionError, OSError):
        pass

    # Configure logging
    logging.basicConfig(
        level=getattr(logging, LOG_LEVEL),
        format=LOG_FORMAT,
        datefmt=LOG_DATE_FORMAT,
        handlers=handlers,
    )

    logger = logging.getLogger(__name__)
    logger.info(f"Starting {APP_NAME} v{APP_VERSION}")
    logger.info(f"Python {sys.version}")
    logger.info(f"PyQt6 {sys.modules['PyQt6.QtCore'].qVersion()}")


def main():
    """Main application entry point"""
    # Setup logging
    setup_logging()
    logger = logging.getLogger(__name__)

    # Enable High DPI support
    QApplication.setHighDpiScaleFactorRoundingPolicy(
        Qt.HighDpiScaleFactorRoundingPolicy.PassThrough
    )

    # Create application
    app = QApplication(sys.argv)
    app.setApplicationName(APP_NAME)
    app.setApplicationVersion(APP_VERSION)
    app.setOrganizationName("AstroLink")

    # Apply Mission Control theme
    theme = MissionControlTheme()
    app.setStyleSheet(theme.get_stylesheet())

    logger.info("Mission Control theme applied")

    # Create and show Professional Mission Control window
    window = ProfessionalMissionControl()
    window.show()

    logger.info("Professional Mission Control window created and displayed")

    # Run application event loop
    exit_code = app.exec()

    logger.info(f"Application exiting with code {exit_code}")
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
