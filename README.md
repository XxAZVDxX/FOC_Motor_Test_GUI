# Motor Test GUI

Motor GUI for motor control, monitoring, tuning, and IMU visualization.

<img width="1512" height="1391" alt="Screenshot 2026-06-15 at 11 42 44 PM" src="https://github.com/user-attachments/assets/0f119523-51f3-4d45-bfb0-8cf0e5f72782" />

## Features

- Serial UART,RS485 and CAN support
- Motor ID detection, run automatically on connect with a 3 s timeout
- Auto-refreshing serial port list (new ports appear at the top)
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
- Only the visible tab polls the device, so background tabs cost no bandwidth

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
- Manual
- IMU 3D Display

### IMU 3D Display layout

The 3D view takes the left side of the tab and the readings take the right side,
separated by a draggable splitter (default 3:2). Everything that used to be
stacked above and below the 3D view is now packed into two compact rows
(polling controls, then the model selector) and a tabbed side panel, so the 3D
model gets nearly the full tab height instead of a narrow strip.

- **Top rows** — enable polling and set the polling interval on the first row;
  choose the 3D model on the second. Two rows are used instead of one because
  eight controls in a single row needed 1363 px and stretched the whole window
  to a 1399 px minimum width.
- **Left pane** — the 3D view. Drag the splitter handle to give it more or less
  room, or maximise the window for the largest view.
- **Right pane** — IMU readings in a compact grid, then a nested tab bar with
  **IMU Debug Data** (raw values plus *Copy Current IMU Data*) and
  **IMU Data Log** (rolling log plus *Clear Log* / *Save Log to CSV* / *Auto Log*).

### Motor Control, PID Tuning, Limits and Manual layouts

These four tabs were re-laid out so that controls that used to be stacked
vertically now sit side by side, which cuts each tab's minimum width and leaves
far less empty space on a wide screen.

- **Motor Control** — eight group boxes in a two-column grid. Config spans the
  full width on top; **Operating Mode** / **Motor Parameters** sit in the left
  column and **Motor Selection** / **Target Values** in the right; **Phase
  Currents** / **Auto Refresh** share the next row. The six *Target Values*
  spin boxes are paired two per row instead of six rows. The motor preview spans
  the bottom with its readings in a two-column grid underneath the dial.
- **PID Tuning** — the four PID groups form a 2×2 grid instead of a tall column.
- **Limits** — each limit gets a `label | max | min | label` row, turning eight
  rows into four.
- **Manual** — the command and response boxes share a draggable vertical
  splitter, so the response area grows with the window.
- **Connection** — the serial port picker is a **list** rather than a dropdown,
  so every available port is visible at once.

### Serial port list and automatic motor ID detection

The serial port list refreshes itself every 1.5 s while the Connection tab is
open and the interface is not connected.

- Newly plugged-in ports are inserted at the **top** of the list, newest first.
- Unplugged ports disappear, and already-listed ports never move, so the entry
  you selected stays selected while the list is live.
- Untick **Auto-refresh ports** to freeze the list, or press **Refresh** to
  update it once.
- Auto-refresh pauses as soon as you connect and resumes when you disconnect.

Clicking **Connect** now also broadcasts a motor-ID detection automatically, so
you do not have to press **Detect Motor ID** after every connection:

- While the broadcast is in flight the status bar shows
  *Searching for motor ID…*.
- If no motor answers within 3 s, a warning dialog appears, the status bar shows
  *Motor ID not detected*, and you can retry with **Detect Motor ID**.
- If a motor answers, the detected IDs are shown as before and the status bar
  returns to *Connected*.

The **Detect Motor ID** button is kept for manual re-detection; it is disabled
while a detection is already pending.

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

## Polling and Bandwidth

The GUI only requests data for the tab you are currently looking at. Each kind of
polling belongs to one tab:

| Polling | Tab | Default interval |
| --- | --- | --- |
| Curve data (`Ia/Ib/Ic`, `Iq/Id`, speed, position) | Real-time Data | 50 ms |
| Motor preview + phase currents | Motor Control | 50 ms / 300 ms |
| Parameter auto refresh | Motor Control | 1000 ms |
| IMU attitude packets | IMU 3D Display | 100 ms |
| Serial port list refresh | Connection | 1500 ms |

When you switch to another tab, the timers owned by the tab you left are stopped
and no further requests are sent on the bus. Switching back resumes them
automatically, so a checkbox you ticked stays ticked and its polling restarts.

Notes:

- This is unconditional — there is no option to keep a background tab polling.
  Keep the owning tab open if you need its data to keep flowing.
- The serial port list is the one exception: it only refreshes while the
  Connection tab is visible, nothing is connected, and **Auto-refresh ports** is
  ticked. It never touches the bus, so it is stopped while connected to avoid
  pointless work.
- IMU gyro calibration is **preserved** across tab switches. Leaving the IMU tab
  pauses the stream only; the measured gyro bias and the current attitude are
  kept, and the timestamp is re-stamped on resume so the paused time is not
  integrated as motion.
- Disconnecting stops every poller regardless of the visible tab.

## Usage

### 1. Connect to device

- Select communication interface:
  - UART
  - RS485
  - CAN
- For serial:
  - Pick the serial port from the list
  - Choose baudrate
- For CAN:
  - Set CAN channel
  - Set bustype
  - Set bitrate
- Click **Connect**. The motor ID broadcast runs automatically; if the device
  does not answer within 3 seconds a warning is shown.

### 2. Detect motor

Motor ID detection runs automatically right after **Connect**. To run it again,
click **Detect Motor ID** and select the detected motor ID in the motor control
tab.

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
