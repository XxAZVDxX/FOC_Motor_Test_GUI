#!/bin/bash

# 启用严格模式：遇到错误立即退出（可选，但建议）
# set -e

echo "========================================"
echo "  Motor GUI - Setup and Run (Unix)"
echo "========================================"
echo

# ============================================
# 第一步：安装系统级 PyQt5 及相关依赖
#   避免通过 pip 编译，省时省力
# ============================================
echo "Installing system dependencies (PyQt5, pyqtgraph, numpy, etc.)..."
sudo apt update
sudo apt install -y \
    python3-pyqt5 \
    python3-pyqt5.qtsvg \
    python3-pyqt5.qtmultimedia \
    python3-pyqtgraph \
    python3-numpy \
    python3-serial \
    python3-opengl

# ============================================
# 第二步：处理 Python 虚拟环境
#   使用 --system-site-packages 让 venv 能共用系统包
# ============================================
if [ -d "venv" ]; then
    read -p "Virtual environment 'venv' exists. Delete and recreate? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "Deleting old venv..."
        rm -rf venv
        echo "Creating new venv (with system site packages)..."
        python3 -m venv venv --system-site-packages
    fi
else
    echo "Creating virtual environment (with system site packages)..."
    python3 -m venv venv --system-site-packages
fi

# ============================================
# 第三步：激活虚拟环境
# ============================================
source venv/bin/activate
if [ $? -ne 0 ]; then
    echo "Failed to activate venv."
    exit 1
fi

# ============================================
# 第四步：升级 pip（可选）
# ============================================
pip install --upgrade pip

# ============================================
# 第五步：安装额外需要 pip 的包
#   这些包不依赖 Qt 编译，pip 安装快速
#   (python-can 和 trimesh 用于 CAN 通信和 3D 模型)
# ============================================
echo "Installing optional packages (CAN, 3D models)..."
pip install python-can trimesh

# 如果上面安装失败，给出提示但不中断执行
if [ $? -ne 0 ]; then
    echo "[Warning] Optional packages not installed. CAN and 3D model loading will be disabled."
fi

# ============================================
# 第六步：运行主程序
# ============================================
echo
echo "Starting Motor GUI..."
python main.py 