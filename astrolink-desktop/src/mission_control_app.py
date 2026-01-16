#!/usr/bin/env python3
"""
Mission Control Application Entry Point
Launches the professional Mission Dashboard
"""

import sys
import logging

from PyQt6.QtWidgets import QApplication

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)

# Import the dashboard
from ui.mission_dashboard import MissionDashboard

if __name__ == "__main__":
    app = QApplication(sys.argv)

    # Create and show dashboard
    dashboard = MissionDashboard()
    dashboard.show()

    sys.exit(app.exec())
