@echo off
setlocal
cd /d "%~dp0"
where py >nul 2>nul
if not errorlevel 1 (
  set "FOC_PYTHON=py -3"
) else (
  where python >nul 2>nul
  if errorlevel 1 (
    echo Python 3.10 or newer is required.
    pause
    exit /b 1
  )
  set "FOC_PYTHON=python"
)
%FOC_PYTHON% -c "import sys; assert sys.version_info >= (3,10)" >nul 2>nul
if errorlevel 1 (
  echo Python 3.10 or newer is required.
  pause
  exit /b 1
)
%FOC_PYTHON% -c "import serial; from PySide6 import QtWidgets" >nul 2>nul
if errorlevel 1 (
  echo Installing pySerial and Qt Widgets...
  %FOC_PYTHON% -m pip install -r "%~dp0requirements.txt"
  if errorlevel 1 (
    echo Dependency installation failed. Check your network and Python environment.
    pause
    exit /b 1
  )
)
%FOC_PYTHON% "%~dp0main.py" %*
if errorlevel 1 pause
