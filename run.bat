@echo off
setlocal enabledelayedexpansion
cd /d "%~dp0"

REM Motor GUI Smart Launcher for Windows

set "VENV_DIR=venv"
set "VENV_PY=%VENV_DIR%\Scripts\python.exe"
set "REQUIRED_PKGS=PyQt5 pyqtgraph numpy pyserial PyOpenGL"
set "OPTIONAL_PKGS=python-can trimesh"
REM "package:import" pairs used to verify the venv is usable
set "CHECK_LIST=PyQt5:PyQt5 pyqtgraph:pyqtgraph numpy:numpy pyserial:serial PyOpenGL:OpenGL"

echo ========================================
echo   Motor GUI - Smart Launcher (Windows)
echo ========================================
echo.

set "PYTHON="
where py >nul 2>&1 && set "PYTHON=py"
if not defined PYTHON (
    where python >nul 2>&1 && set "PYTHON=python"
)
if not defined PYTHON (
    echo [ERROR] Python 3 was not found in PATH.
    echo         Install it from https://www.python.org/downloads/ and make sure
    echo         "Add python.exe to PATH" is checked, then run this file again.
    echo.
    pause
    exit /b 1
)
echo Using Python launcher: %PYTHON%
echo.

set "NEED_SETUP=0"

if exist "%VENV_PY%" (
    echo Virtual environment found. Checking required packages...
    set "MISSING=0"
    for %%E in (%CHECK_LIST%) do (
        for /f "tokens=1,2 delims=:" %%A in ("%%E") do (
            "%VENV_PY%" -c "import %%B" >nul 2>&1
            if errorlevel 1 (
                echo   - Missing: %%A
                set "MISSING=1"
            )
        )
    )
    if "!MISSING!"=="1" (
        echo [INFO] Some required packages are missing. Rebuilding the environment...
        set "NEED_SETUP=1"
    ) else (
        echo All required packages are present.
    )
) else (
    echo Virtual environment not found.
    set "NEED_SETUP=1"
)

if "!NEED_SETUP!"=="1" (
    echo.
    echo Setting up environment...

    if exist "%VENV_DIR%" (
        echo Removing old venv...
        rmdir /s /q "%VENV_DIR%"
    )

    echo Creating new venv...
    %PYTHON% -m venv "%VENV_DIR%"
    if errorlevel 1 (
        echo [ERROR] Failed to create venv.
        echo.
        pause
        exit /b 1
    )

    echo Upgrading pip...
    "%VENV_PY%" -m pip install --upgrade pip

    echo Installing required packages...
    "%VENV_PY%" -m pip install %REQUIRED_PKGS%
    if errorlevel 1 (
        echo [ERROR] Failed to install required packages.
        echo.
        pause
        exit /b 1
    )

    echo Installing optional packages ^(CAN, 3D models^)...
    "%VENV_PY%" -m pip install %OPTIONAL_PKGS%
    if errorlevel 1 (
        echo [INFO] Optional packages not installed ^(CAN/3D features disabled^).
    )
    echo.
)

echo Starting Motor GUI...
"%VENV_PY%" main.py
if errorlevel 1 (
    echo.
    echo [ERROR] Motor GUI exited with an error. See the messages above.
    echo.
    pause
    exit /b 1
)

endlocal