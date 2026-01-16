@echo off
REM AstroLink Mission Control - Startup Script (Windows)

cd /d "%~dp0"

echo ==========================================
echo AstroLink Mission Control Desktop v1.0.4
echo ==========================================
echo.

REM Check Python
python --version >nul 2>&1
if errorlevel 1 (
    echo ERROR: Python not found!
    echo Please install Python 3.10 or higher from python.org
    pause
    exit /b 1
)

python --version

REM Check if virtual environment exists
if not exist "venv\" (
    echo.
    echo Creating virtual environment...
    python -m venv venv

    echo Installing dependencies...
    call venv\Scripts\activate.bat
    pip install --upgrade pip
    pip install -r requirements.txt
) else (
    call venv\Scripts\activate.bat
)

echo.
echo Starting AstroLink Mission Control...
echo.

REM Run application
python src\main.py

REM Deactivate venv on exit
call venv\Scripts\deactivate.bat
pause
