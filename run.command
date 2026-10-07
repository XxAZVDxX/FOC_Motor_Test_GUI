#!/bin/bash

# Motor GUI - Setup and Run (macOS .command)
# Double-click this file in Finder to launch, or run from terminal.

# cd to the directory where this script resides
cd "$(dirname "$0")" || exit 1

echo "========================================"
echo "  Motor GUI - Setup and Run (macOS)"
echo "========================================"
echo

# ── Helper: create venv and install all dependencies ──
setup_venv() {
    if [ -d "venv" ]; then
        echo "Removing old venv..."
        rm -rf venv
    fi
    echo "Creating virtual environment..."
    python3 -m venv venv || { echo "[ERROR] Failed to create venv."; exit 1; }
    source venv/bin/activate || { echo "[ERROR] Failed to activate venv."; exit 1; }
    pip install --upgrade pip
    echo "Installing required packages..."
    pip install PyQt5 pyqtgraph numpy pyserial PyOpenGL PyOpenGL_accelerate
    echo "Installing optional packages (CAN, 3D models)..."
    pip install python-can trimesh 2>/dev/null || echo "[Warning] Optional packages not installed."
}

# ── Helper: activate existing venv ──
activate_venv() {
    source venv/bin/activate || { echo "[ERROR] Failed to activate venv."; exit 1; }
}

# ── Main logic ──
RUN_OK=0

if [ -d "venv" ]; then
    # Venv exists – just activate and try to run
    echo "Using existing virtual environment..."
    activate_venv
    echo "Starting Motor GUI..."
    python main.py && RUN_OK=1

    if [ $RUN_OK -eq 0 ]; then
        echo
        echo "[ERROR] Failed to start Motor GUI."
        read -p "Recreate virtual environment and retry? (y/n) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            setup_venv
            echo
            echo "Starting Motor GUI..."
            python main.py && RUN_OK=1
        fi
    fi
else
    # No venv – create fresh, install, run
    echo "No virtual environment found."
    setup_venv
    echo
    echo "Starting Motor GUI..."
    python main.py && RUN_OK=1
fi

if [ $RUN_OK -eq 0 ]; then
    echo
    echo "Motor GUI did not start successfully."
    echo "Press any key to close this window..."
    read -n 1 -s
fi
