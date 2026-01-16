"""
Application Configuration and Constants
"""

import os
from datetime import datetime
from pathlib import Path

# ============================================================================
# APPLICATION METADATA
# ============================================================================

APP_NAME = "AstroLink Mission Control"
APP_VERSION = os.environ.get("ASTROLINK_VERSION", "1.0.4")
APP_BUILD_DATE = os.environ.get(
    "ASTROLINK_BUILD_DATE", datetime.utcnow().strftime("%Y-%m-%d")
)
APP_ORGANIZATION = "AstroLink"
APP_DOMAIN = "astrolink.space"

# ============================================================================
# TELEMETRY PROTOCOL CONSTANTS (must match firmware)
# ============================================================================

# Frame synchronization bytes
SYNC_FULL = 0xA5  # FULL/keyframe marker
SYNC_DELTA = 0xA4  # DELTA frame marker

# Frame lengths (bytes)
FULL_FRAME_LEN = 38  # Full telemetry frame
DELTA_FRAME_LEN = 23  # Delta-encoded frame

# Serial communication
DEFAULT_BAUD_RATE = 115200
SERIAL_TIMEOUT = 0.1  # seconds

# ============================================================================
# UI CONSTANTS
# ============================================================================

# Window dimensions (BASE VALUES - will be scaled by responsive.py)
DEFAULT_WINDOW_WIDTH = 1440
DEFAULT_WINDOW_HEIGHT = 900
FULLSCREEN_WIDTH = 1920
FULLSCREEN_HEIGHT = 1080

# Plot settings (BASE VALUES - will be adjusted by responsive.py)
PLOT_UPDATE_INTERVAL = 50  # milliseconds (20 Hz on medium screens)
PLOT_HISTORY_SECONDS = 300  # 5 minutes default
MAX_PLOT_POINTS = 10000  # Maximum points to render (medium screens)

# File list refresh
FILE_REFRESH_INTERVAL = 2000  # milliseconds

# ============================================================================
# DATA PATHS
# ============================================================================

def _safe_mkdir(path: Path) -> bool:
    try:
        path.mkdir(parents=True, exist_ok=True)
        return True
    except PermissionError:
        return False


def _resolve_app_data_dir() -> Path:
    env = os.environ.get("ASTROLINK_DATA_DIR")
    if env:
        return Path(env).expanduser().resolve()

    # Default: user home
    candidate = Path.home() / ".astrolink"
    if _safe_mkdir(candidate):
        return candidate

    # Fallback: keep everything inside the project if home is not writable
    project_dir = Path(__file__).resolve().parents[1]  # astrolink-desktop/
    fallback = project_dir / ".astrolink"
    _safe_mkdir(fallback)
    return fallback


# Application data directory (safe even in sandboxed environments)
APP_DATA_DIR = _resolve_app_data_dir()
LOGS_DIR = APP_DATA_DIR / "logs"
SETTINGS_FILE = APP_DATA_DIR / "settings.json"

# Best-effort directory creation (no crash on restricted filesystems)
_safe_mkdir(APP_DATA_DIR)
_safe_mkdir(LOGS_DIR)

# Working directory (for .bin files)
WORKING_DIR = Path.cwd()

# ============================================================================
# TELEMETRY DECODING CONSTANTS
# ============================================================================

# Temperature decoding
TEMP_OFFSET = -40.0
TEMP_SCALE = 0.1
TEMP_BITS = 11

# Humidity decoding
HUMIDITY_BITS = 7

# Pressure decoding
PRESSURE_OFFSET = 822.0
PRESSURE_SCALE = 0.1
PRESSURE_INT_BITS = 8
PRESSURE_FRAC_BITS = 4

# Combined sample size
SAMPLE_BITS = 30  # T(11) + H(7) + P(12) = 30 bits

# ============================================================================
# FILE FORMATS
# ============================================================================

TELEMETRY_FILE_EXTENSION = ".bin"
EXPORT_CSV_EXTENSION = ".csv"
EXPORT_JSON_EXTENSION = ".json"

# Timestamp format for filenames
TIMESTAMP_FORMAT = "%Y%m%d_%H%M%S"
TIMESTAMP_FORMAT_DISPLAY = "%Y-%m-%d %H:%M:%S"

# ============================================================================
# LOGGING
# ============================================================================

LOG_LEVEL = "INFO"  # DEBUG, INFO, WARNING, ERROR, CRITICAL
LOG_FORMAT = "%(asctime)s [%(levelname)s] %(name)s: %(message)s"
LOG_DATE_FORMAT = "%Y-%m-%d %H:%M:%S"

# ============================================================================
# SERIAL PORT DETECTION
# ============================================================================

# Keywords for auto-detecting Heltec LoRa boards
HELTEC_KEYWORDS = [
    "CP210",  # CP2102 USB-UART chip
    "CH340",  # CH340 USB-UART chip
    "USB-SERIAL",
    "UART",
    "Heltec",
]

# ============================================================================
# PERFORMANCE SETTINGS
# ============================================================================

# Enable OpenGL acceleration for plots
USE_OPENGL = True

# Downsampling threshold (auto-downsample if more points)
DOWNSAMPLE_THRESHOLD = 5000

# Thread pool size for background tasks
THREAD_POOL_SIZE = 4

# ============================================================================
# FEATURE FLAGS
# ============================================================================

ENABLE_SIMULATED_FLIGHT = True  # Show "Simulate Flight" button
ENABLE_EXPORT = True  # Enable data export features
ENABLE_LOGGING = True  # Enable telemetry logging to disk
ENABLE_FULLSCREEN = True  # Enable fullscreen mode
ENABLE_KEYBOARD_SHORTCUTS = True  # Enable F1-F12 shortcuts

# ============================================================================
# DEFAULT SETTINGS (can be overridden by user preferences)
# ============================================================================

DEFAULT_SETTINGS = {
    "theme": "dark",
    "plot_history_seconds": PLOT_HISTORY_SECONDS,
    "auto_connect": False,
    "last_serial_port": None,
    "window_geometry": None,
    "opengl_enabled": USE_OPENGL,
    "log_telemetry": False,
}
