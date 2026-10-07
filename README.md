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
- Stillness-gated gyro calibration and complementary filter attitude estimation
- Light and dark themes
- UI languages: English, 简体中文, 繁體中文

## Project Files

- `main.py` — main GUI application (entry point)
- `run.bat` — launcher for Windows
- `run.sh` — launcher for Linux
- `run.command` — launcher for macOS (double-click in Finder)
- `theme.py` — light/dark palettes and the global stylesheet
- `i18n.py` — translation catalog and the `tr()` lookup helper
- `settings.py` — loads/saves the selected theme and language
- `assets/` — icons used by the stylesheet


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

## Appearance and Language

Both settings live in the **View** menu and apply immediately — no restart needed.

### Theme

**View → Theme → Light / Dark**

The theme restyles the whole window: widget palette, tab bar, status bar,
buttons, the phase-current labels, the 2D motor preview and the IMU 3D
background.

### Language

**View → Language → English / 简体中文 / 繁體中文**

Switching a language retranslates every label, button, tab title, group box and
status message in place. Values already shown on screen (angles, current
readings, plot labels) are not re-rendered until the next update.

### Persistence

The chosen theme and language are written to `config/settings.json` and restored
on the next launch:

```json
{
  "theme": "light",
  "language": "en"
}
```

`config/settings.json` is a settings file, not a motor profile, so it is hidden
from the configuration dropdown in the Motor Control tab. Delete it to fall back
to the defaults (`light` + `en`).

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

### Window is taller than the screen

The Motor Control and PID Tuning tabs are dense. When a tab does not fit
vertically it is placed inside a scroll area, and the window is clamped to the
available screen area on startup. Maximise the window, or scroll the affected
tab.

### IMU 3D model does not move

The 3D model follows the gyroscope, but the gyro zero bias is measured on
startup and it must be measured while the device is completely still.

- Press **Start Polling** and leave the device still for the first 2-5 seconds
  (the status bar shows `Calibrating IMU... n/30`).
- Samples taken while the device is moving are discarded, so calibration simply
  takes longer instead of learning a wrong bias.
- Rotation is always applied to the model, even while calibration is still
  running, so the model responds from the first packet.
- If the model drifts or does not move, stop polling and start it again with the
  device at rest.

### IMU axes do not match the physical board

`decode_imu_packet()` in `gui_tabs/main_window.py` maps the sensor frame
straight through. If roll/pitch/yaw appear swapped or inverted, uncomment the
alternative mapping block there and adjust the signs for your mounting.

### Wrong theme or language after launch

The last selection is remembered in `config/settings.json`. Delete that file to
reset both back to the defaults (light theme, English).
