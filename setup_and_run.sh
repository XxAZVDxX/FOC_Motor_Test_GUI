#!/bin/bash

echo "========================================"
echo "  Motor GUI - Setup and Run (Unix)"
echo "========================================"
echo

# 检查虚拟环境
if [ -d "venv" ]; then
    read -p "Virtual environment 'venv' exists. Delete and recreate? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "Deleting old venv..."
        rm -rf venv
        echo "Creating new venv..."
        python3 -m venv venv
    fi
else
    echo "Creating virtual environment..."
    python3 -m venv venv
fi

# 激活虚拟环境
source venv/bin/activate
if [ $? -ne 0 ]; then
    echo "Failed to activate venv."
    exit 1
fi

# 升级 pip
pip install --upgrade pip

# 安装必需库
echo "Installing required packages..."
pip install PyQt5 pyqtgraph numpy pyserial PyOpenGL PyOpenGL_accelerate

# 安装可选扩展
echo "Installing optional packages (CAN, 3D models)..."
pip install python-can trimesh 2>/dev/null
if [ $? -ne 0 ]; then
    echo "[Warning] Optional packages not installed. CAN and 3D model loading will be disabled."
fi

# 运行 GUI
echo
echo "Starting Motor GUI..."
python motor_gui.py