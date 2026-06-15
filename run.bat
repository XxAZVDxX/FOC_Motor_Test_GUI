#!/bin/bash

# Motor GUI Smart Launcher for Linux/macOS

set -e

VENV_DIR="venv"
REQUIRED_PKGS="PyQt5 pyqtgraph numpy pyserial PyOpenGL"
OPTIONAL_PKGS="python-can trimesh"

echo "========================================"
echo "  Motor GUI - Smart Launcher (Unix)"
echo "========================================"
echo

# 检查 Python
if ! command -v python3 &> /dev/null; then
    echo "[ERROR] Python3 is not installed or not in PATH."
    exit 1
fi

NEED_SETUP=0

# 检查虚拟环境
if [ -d "$VENV_DIR/bin" ]; then
    echo "Virtual environment found. Activating..."
    source "$VENV_DIR/bin/activate"
    if [ $? -ne 0 ]; then
        echo "[WARN] Failed to activate venv. Will recreate."
        NEED_SETUP=1
    else
        # 检查必需库
        echo "Checking required packages..."
        MISSING=0
        for pkg in $REQUIRED_PKGS; do
            python3 -c "import $pkg" 2>/dev/null
            if [ $? -ne 0 ]; then
                echo "  - Missing: $pkg"
                MISSING=1
            fi
        done
        if [ $MISSING -eq 1 ]; then
            echo "[INFO] Some required packages are missing. Installing..."
            NEED_SETUP=1
        else
            echo "All required packages are present."
            NEED_SETUP=0
        fi
    fi
else
    echo "Virtual environment not found."
    NEED_SETUP=1
fi

# 设置环境
if [ $NEED_SETUP -eq 1 ]; then
    echo
    echo "Setting up environment..."

    # 删除旧 venv（如果存在）
    if [ -d "$VENV_DIR" ]; then
        echo "Removing old venv..."
        rm -rf "$VENV_DIR"
    fi

    echo "Creating new venv..."
    python3 -m venv "$VENV_DIR"
    if [ $? -ne 0 ]; then
        echo "[ERROR] Failed to create venv."
        exit 1
    fi

    echo "Activating venv..."
    source "$VENV_DIR/bin/activate"
    if [ $? -ne 0 ]; then
        echo "[ERROR] Failed to activate venv."
        exit 1
    fi

    echo "Upgrading pip..."
    pip install --upgrade pip

    echo "Installing required packages..."
    pip install $REQUIRED_PKGS PyOpenGL_accelerate
    if [ $? -ne 0 ]; then
        echo "[WARN] PyOpenGL_accelerate failed, installing PyOpenGL only..."
        pip install PyOpenGL
    fi

    echo "Installing optional packages (CAN, 3D models)..."
    pip install $OPTIONAL_PKGS 2>/dev/null
    if [ $? -ne 0 ]; then
        echo "[INFO] Optional packages not installed (CAN/3D features disabled)."
    fi
    echo
fi

# 确保虚拟环境已激活
if [ -z "$VIRTUAL_ENV" ]; then
    source "$VENV_DIR/bin/activate"
fi

# 运行主程序
echo "Starting Motor GUI..."
python3 motor_gui.py