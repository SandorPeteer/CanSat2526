#!/bin/bash
# Quick launcher for AstroLink Mission Control
# Checks responsive UI and launches application

set -e
cd "$(dirname "$0")"

echo "=========================================="
echo "🚀 AstroLink Mission Control Launcher"
echo "=========================================="
echo ""

# Activate virtual environment
if [ -d "venv" ]; then
    echo "✓ Activating virtual environment..."
    source venv/bin/activate
else
    echo "⚠️  Virtual environment not found!"
    echo "Run ./run.sh first to set up dependencies"
    exit 1
fi

# Test responsive UI module
echo ""
echo "🧪 Testing Responsive UI System..."

# Clear Python cache INSIDE src directory to ensure latest code
rm -rf src/ui/__pycache__/responsive*.pyc 2>/dev/null || true
rm -rf src/__pycache__ 2>/dev/null || true
rm -rf src/ui/__pycache__ 2>/dev/null || true

cd src
python3 -c "
import sys

# CRITICAL: Create QApplication BEFORE importing responsive module!
# The responsive module needs QApplication to detect screen size
from PyQt6.QtWidgets import QApplication
app = QApplication(sys.argv)

# Now import responsive (will correctly detect screen size)
from ui.responsive import get_responsive_scaling
rs = get_responsive_scaling()
print(f'  📊 Screen Profile: {rs.profile.name}')
print(f'  📏 Scale Factor: {rs.profile.scale_factor}×')
print(f'  🖱️  Button Scale: {rs.profile.button_scale}×')
print(f'  📈 Max Plot Points: {rs.get_recommended_plot_points():,}')
print(f'  ⏱️  Update Interval: {rs.get_plot_update_interval()}ms')
print(f'  🎮 OpenGL: {\"Enabled\" if rs.should_use_opengl() else \"Disabled\"}')
width, height = rs.get_window_size(1440, 900)
print(f'  🖥️  Window Size: {width}×{height}')
"

echo ""
echo "✅ Responsive UI test passed!"
echo ""
echo "🚀 Launching Mission Control..."
echo ""

# Launch application
python3 -m main "$@"

echo ""
echo "✅ Mission Control closed"
deactivate || true
