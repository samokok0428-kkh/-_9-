@echo off
cd /d "%~dp0"
set "ROAD_PYTHON=%LOCALAPPDATA%\Python\pythoncore-3.14-64\python.exe"
if not exist "%ROAD_PYTHON%" (
    echo Python runtime not found: %ROAD_PYTHON%
    pause
    exit /b 1
)
"%ROAD_PYTHON%" -X utf8 view_road.py --sadang-manual
if errorlevel 1 pause
