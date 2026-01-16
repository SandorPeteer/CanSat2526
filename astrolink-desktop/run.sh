#!/bin/bash
# AstroLink Mission Control - Startup Script
# Professional Telemetry Ground Station v1.0.4

set -e  # Exit on error

cd "$(dirname "$0")"

echo "=========================================="
echo "AstroLink Mission Control - Desktop"
echo "Professional Telemetry Ground Station v1.0.4"
echo "=========================================="
echo ""

# Check Python
if ! command -v python3 &> /dev/null; then
    echo "ERROR: Python 3 not found!"
    echo "Please install Python 3.10 or higher"
    exit 1
fi

PYTHON_VERSION=$(python3 --version 2>&1 | awk '{print $2}')
echo "✓ Python: $PYTHON_VERSION"

# Check if virtual environment exists
if [ ! -d "venv" ]; then
    echo ""
    echo "📦 Creating virtual environment..."
    python3 -m venv venv

    echo "📦 Installing dependencies..."
    source venv/bin/activate
    pip install --upgrade pip > /dev/null 2>&1
    if [ -f "requirements.txt" ]; then
        pip install -r requirements.txt > /dev/null 2>&1
    else
        pip install PyQt6 pyqtgraph numpy pandas scipy pyserial > /dev/null 2>&1
    fi
    echo "✓ Dependencies installed"
else
    source venv/bin/activate
fi

echo ""
echo "🚀 Launching Professional Mission Control..."
echo ""

# Headless/sandbox mode support (useful for CI/Codex runs)
# - Does NOT affect normal desktop usage unless explicitly enabled.
if [ "${ASTROLINK_HEADLESS:-0}" = "1" ] && [ -z "${QT_QPA_PLATFORM:-}" ]; then
    export QT_QPA_PLATFORM=offscreen
fi

# Run application using Python module
cd src
python3 -m main "$@"

# Cleanup
deactivate || true
