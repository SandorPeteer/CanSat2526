# AstroLink Mission Control - Desktop Edition

**Professional Telemetry Ground Station Application**

![Version](https://img.shields.io/badge/version-1.0.0-blue)
![Platform](https://img.shields.io/badge/platform-Windows%20%7C%20macOS%20%7C%20Linux-lightgrey)
![Python](https://img.shields.io/badge/python-3.10%2B-green)
![License](https://img.shields.io/badge/license-MIT-yellow)

## Overview

AstroLink Mission Control is a professional-grade telemetry visualization and ground station application designed following ESA/NASA mission control design principles. Built with PyQt6 and optimized for real-time telemetry monitoring from LoRa-based satellite systems.

## Features

### Real-Time Telemetry
- 60 FPS real-time plotting with hardware acceleration (pyqtgraph + OpenGL)
- Multi-parameter monitoring (Temperature, Humidity, Pressure)
- Auto-scaling axes and time-series visualization
- Frame-by-frame telemetry inspection

### Mission Control Interface
- ESA/NASA-inspired dark theme with aerospace color standards
- 1920x1080 optimized layout (scales to smaller resolutions)
- Fullscreen mode (F11) for professional monitoring
- Dual-monitor support

### Connectivity
- **OFFLINE Mode:** Load and replay .bin telemetry files
- **ONLINE Mode:** Live serial connection to LoRa receivers (Heltec WiFi32)
- Auto-detection of serial ports
- Automatic reconnection on disconnect

### Data Management
- Record telemetry sessions to binary files
- Export data to CSV, JSON formats
- Historical data playback with time controls
- Frame validation and CRC checking

### Professional Features
- Keyboard shortcuts (F1-F12 function keys)
- Status indicators (connection, data rate, frame health)
- Alert system for out-of-range telemetry
- Comprehensive logging system

## System Requirements

### Minimum
- **OS:** Windows 10, macOS 10.15, Linux (Ubuntu 20.04+)
- **CPU:** Intel Core i3 / AMD Ryzen 3 or equivalent
- **RAM:** 4 GB
- **Storage:** 500 MB free space
- **Display:** 1366x768 (recommended: 1920x1080)
- **USB:** 1x port for LoRa module

### Recommended
- **OS:** Windows 11, macOS 12+, Linux (latest)
- **CPU:** Intel Core i5 / AMD Ryzen 5 or better
- **RAM:** 8 GB
- **Storage:** SSD with 2 GB free space
- **Display:** 1920x1080 or higher, dual monitor setup
- **GPU:** Dedicated graphics (for OpenGL acceleration)

## Installation

### Option 1: Binary Installer (Recommended)

#### Windows
```bash
# Download from releases
astrolink-desktop-1.0.0-windows-x64.exe

# Run installer
# Application installed to: C:\Program Files\AstroLink
```

#### macOS
```bash
# Download from releases
astrolink-desktop-1.0.0-macos.dmg

# Drag to Applications folder
```

#### Linux
```bash
# Download from releases
astrolink-desktop-1.0.0-linux-x64.AppImage

# Make executable and run
chmod +x astrolink-desktop-1.0.0-linux-x64.AppImage
./astrolink-desktop-1.0.0-linux-x64.AppImage
```

### Option 2: From Source

#### Prerequisites
```bash
# Python 3.10 or higher
python --version

# Install dependencies
pip install -r requirements.txt
```

#### Run from source
```bash
cd astrolink-desktop
python src/main.py
```

## Quick Start

### 1. Connect Hardware
- Connect Heltec LoRa WiFi32 board via USB
- Board should be running compatible firmware (115200 baud)

### 2. Launch Application
- Double-click AstroLink icon (installed version)
- Or run: `python src/main.py` (source version)

### 3. Select Mode
- **OFFLINE:** Load existing .bin files from file list
- **ONLINE:** Select serial port, click "Connect"

### 4. Monitor Telemetry
- Live plots update in real-time
- View decoded samples in table
- Inspect raw frames in "Raw Stream" tab

### 5. Record Session
- Click "LOG" button to start recording
- Telemetry saved to `raw_YYYYMMDD_HHMMSS.bin`

## Architecture

```
astrolink-desktop/
├── src/
│   ├── main.py              # Application entry point
│   ├── ui/
│   │   ├── main_window.py   # Main window UI
│   │   ├── widgets/         # Custom widgets
│   │   └── themes/          # ESA/NASA theme definitions
│   ├── core/
│   │   ├── telemetry.py     # Telemetry decoder
│   │   ├── serial_handler.py # Serial port communication
│   │   └── data_manager.py  # Data persistence
│   ├── utils/
│   │   ├── crc.py           # CRC16 implementation
│   │   └── helpers.py       # Utility functions
│   └── resources/
│       ├── icons/           # Application icons
│       └── fonts/           # Custom fonts
├── tests/                   # Unit tests
├── docs/                    # Documentation
├── build/                   # Build scripts
├── requirements.txt         # Python dependencies
└── README.md
```

## Technology Stack

- **Framework:** PyQt6
- **Plotting:** pyqtgraph (OpenGL-accelerated)
- **Serial:** pyserial
- **Data Processing:** NumPy, pandas
- **Build:** PyInstaller
- **Standards:** CCSDS telemetry formats

## Keyboard Shortcuts

| Shortcut | Action |
|----------|--------|
| `F11` | Toggle fullscreen |
| `F1` | Show help |
| `F5` | Refresh file list |
| `Ctrl+O` | Open file |
| `Ctrl+S` | Save session |
| `Ctrl+E` | Export data |
| `Ctrl+Q` | Quit application |
| `Space` | Pause/Resume playback |

## Color Scheme (ESA/NASA Standard)

### Status Colors
- **Nominal (Green):** `#00C853` - All systems operating normally
- **Caution (Amber):** `#FFB300` - Warning, approaching limits
- **Critical (Red):** `#F44336` - Out of limits, immediate attention
- **Unknown (Gray):** `#757575` - No data or indeterminate state

### Telemetry Colors
- **Temperature:** `#00D9FF` (Cyan)
- **Humidity:** `#9D4EDD` (Purple)
- **Pressure:** `#20B2AA` (Teal)

### UI Theme
- **Background Primary:** `#0B0C10` (Deep space)
- **Background Secondary:** `#1F2833` (Panel backgrounds)
- **Text Primary:** `#F0F0F0` (Off-white)
- **Text Secondary:** `#C5C6C7` (Gray labels)

## Troubleshooting

### Serial Port Not Detected
1. Check USB cable connection
2. Install CH340/CP2102 drivers for Heltec board
3. Verify board is powered on
4. Check device manager (Windows) or `ls /dev/tty*` (Linux/macOS)

### Slow Performance
1. Enable hardware acceleration in settings
2. Reduce plot history buffer size
3. Close other resource-intensive applications
4. Update graphics drivers

### Application Won't Start
1. Verify Python 3.10+ is installed
2. Check all dependencies: `pip install -r requirements.txt`
3. Run with verbose logging: `python src/main.py --verbose`
4. Check logs in `~/.astrolink/logs/`

## Development

### Setup Development Environment
```bash
# Clone repository
git clone https://github.com/yourusername/astrolink-desktop
cd astrolink-desktop

# Create virtual environment
python -m venv venv
source venv/bin/activate  # Linux/macOS
# or
venv\Scripts\activate     # Windows

# Install development dependencies
pip install -r requirements-dev.txt

# Run tests
pytest tests/

# Run with debug mode
python src/main.py --debug
```

### Building Installers
```bash
# Windows
python build/build_windows.py

# macOS
python build/build_macos.py

# Linux
python build/build_linux.py
```

## Contributing

We welcome contributions! Please see [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

## License

This project is licensed under the MIT License - see [LICENSE](LICENSE) file for details.

## Acknowledgments

- Design inspired by NASA's OpenMCT and ESA mission control interfaces
- Color palette based on aerospace human interface standards
- Built with support from the open-source community

## Support

- **Documentation:** [docs/](docs/)
- **Issues:** [GitHub Issues](https://github.com/yourusername/astrolink-desktop/issues)
- **Email:** support@astrolink.space

## Roadmap

### Version 1.0 (Current)
- ✅ Core telemetry visualization
- ✅ OFFLINE/ONLINE modes
- ✅ ESA/NASA theme
- ✅ Cross-platform support

### Version 1.1 (Planned)
- 🔲 Multi-satellite support
- 🔲 Custom alert rules
- 🔲 Plugin system
- 🔲 Database integration

### Version 2.0 (Future)
- 🔲 Web-based companion app
- 🔲 Cloud telemetry storage
- 🔲 AI-powered anomaly detection
- 🔲 3D satellite visualization

---

**Made with ❤️ for the space community**
