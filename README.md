# Motor Test GUI

Motor GUI for motor control, monitoring, tuning, and IMU visualization.

<img width="1512" height="1391" alt="Screenshot 2026-06-15 at 11 42 44 PM" src="https://github.com/user-attachments/assets/0f119523-51f3-4d45-bfb0-8cf0e5f72782" />

## Features

- Serial UART,RS485 and CAN support
- Motor ID detection
- Motor mode switching:
  - Stop
  - Self-test
  - Calibration
  - Open-loop
  - Current loop
  - Speed loop
  - Position loop
- Target control for:
  - Iq
  - Id
  - Speed
  - Position
  - Uq / Ud
- PID tuning interface
- Limits configuration
- Real-time data monitoring
- Communication log
- Manual hex command sending
- IMU 3D visualization
- Optional custom 3D model loading

## Project Files

- `main.py` — main GUI application (entry point)
- `run.bat` — launcher for Windows
- `run.sh` — launcher for Linux
- `run.command` — launcher for macOS (double-click in Finder)


## Installation

Each launcher creates a `venv`, installs the required packages
(`PyQt5`, `pyqtgraph`, `numpy`, `pyserial`, `PyOpenGL`) plus the optional
`python-can` / `trimesh`, and then starts the GUI. Re-running a launcher is
fast: the existing environment is reused when all required packages are present.

### Option 1: Linux/macOS

```bash
sh ./run.sh
```

On macOS you can also double-click `run.command`.

### Option 2: Windows

Double-click `run.bat`, or run it from a terminal:

```bat
run.bat
```

> Requires Python 3 on `PATH` (the `py` launcher also works). If Python is
> missing, install it from <https://www.python.org/downloads/> with
> **Add python.exe to PATH** enabled.

> `run.bat` is a Windows batch script. Do **not** run it with `bash`.

## Main Interface

The application contains these tabs:

- Connection
- Motor Control
- PID Tuning
- Real-time Data
- Limits
- Communication Log
- Manual Command
- IMU 3D Display

## Usage

### 1. Connect to device

- Select communication interface:
  - UART
  - RS485
  - CAN
- For serial:
  - Choose serial port
  - Choose baudrate
- For CAN:
  - Set CAN channel
  - Set bustype
  - Set bitrate
- Click **Connect**

### 2. Detect motor

- Click **Detect Motor ID**
- Select the detected motor ID in the motor control tab

### 3. Set operating mode

Available modes:

- Stop
- Self-test
- Calibration
- Open-loop
- Current loop
- Speed loop
- Position loop

### 4. Configure targets

Depending on the control mode, set:

- Iq target
- Id target
- Speed target
- Position target
- Uq / Ud target

Then apply the values through the GUI.

### 5. Tune parameters

Use the PID tuning tab to adjust controller parameters.

### 6. Monitor data

Use the real-time data and IMU tabs to observe:

- Currents
- Speed
- Position
- IMU orientation

### 7. Send manual commands

Use the manual command tab to send raw hexadecimal commands and inspect received data.


## Troubleshooting

### No serial port shown

- Check whether the device is connected
- Verify system permission for serial devices
- Refresh the port list

### CAN not available

- Install `python-can`
- Verify CAN adapter, channel, and driver configuration


### GUI does not start

- Confirm Python 3 is installed
- Confirm all required packages are installed
- Re-run the .bat or .sh
- Activate the virtual environment before running
