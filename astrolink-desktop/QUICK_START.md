# AstroLink Mission Control - Quick Start Guide

## Installation & Running

### macOS / Linux

```bash
cd astrolink-desktop
./run.sh
```

### Windows

```cmd
cd astrolink-desktop
run.bat
```

### Manual Run (if scripts don't work)

```bash
# Create virtual environment (first time only)
python3 -m venv venv

# Activate virtual environment
source venv/bin/activate    # macOS/Linux
venv\Scripts\activate       # Windows

# Install dependencies (first time only)
pip install -r requirements.txt

# Run application
python src/main.py
```

## First Time Setup

1. **Connect Heltec LoRa Board** via USB
2. **Select Mode:**
   - **OFFLINE**: Load existing `.bin` files
   - **ONLINE**: Live serial connection
3. **For ONLINE mode:**
   - Click "Refresh" to detect ports
   - Select your port (auto-detected if Heltec board)
   - Click "Connect"
4. **View telemetry** in real-time plots

## Features

- **Real-time plotting** (Temperature, Humidity, Pressure)
- **OFFLINE mode**: Load and replay `.bin` files
- **ONLINE mode**: Live LoRa telemetry streaming
- **Frame inspector**: Analyze FULL/DELTA frames
- **Export**: Save data to CSV/JSON
- **Record**: LOG button to save telemetry
- **Fullscreen**: Press F11

## Keyboard Shortcuts

- `F11` - Toggle fullscreen
- `F5` - Refresh file list
- `Ctrl+Q` - Quit application

## Troubleshooting

### "pyserial not installed"
```bash
pip install pyserial
```

### "No ports found"
- Check USB cable connection
- Install CH340/CP2102 drivers for Heltec board
- Try clicking "Refresh" button

### Application won't start
```bash
# Check Python version (need 3.10+)
python3 --version

# Reinstall dependencies
pip install --force-reinstall -r requirements.txt

# Run with verbose logging
python src/main.py --verbose
```

## File Structure

```
astrolink-desktop/
├── src/
│   ├── main.py              # Entry point
│   ├── config.py            # Configuration
│   ├── ui/
│   │   ├── main_window.py   # Main UI
│   │   └── themes/          # ESA/NASA theme
│   ├── core/
│   │   ├── telemetry.py     # Decoder
│   │   ├── serial_handler.py # Serial comm
│   │   └── data_manager.py  # File I/O
│   └── utils/               # Utilities
├── requirements.txt         # Python dependencies
├── run.sh                   # macOS/Linux launcher
└── run.bat                  # Windows launcher
```

## Next Steps

1. **Try OFFLINE mode** - Load a `.bin` file to see decoded telemetry
2. **Connect LoRa** - Switch to ONLINE mode and stream live data
3. **Record session** - Click LOG to save telemetry to disk
4. **Export data** - Save CSV for analysis in Excel/Python

For more details, see [README.md](README.md)
