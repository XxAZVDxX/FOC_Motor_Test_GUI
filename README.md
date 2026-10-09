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
- Per-value **Set** buttons: each sends only its own target, and Speed/Position also switch the motor to the matching closed loop
- The target that is active for the current mode is highlighted in bold
- **Stop** button and a mode indicator that follow the real operating mode
- PID tuning interface
- Limits configuration
- Real-time data monitoring
- Communication log
- Manual hex command sending
- IMU 3D visualization
- Optional custom 3D model loading
- Median-based gyro bias calibration, spike rejection and complementary filter attitude estimation
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
- `settings.py` — loads/saves the selected theme, language and zoom level
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
  The exported CSV starts with a `timestamp, ax(g), ... yaw(deg)` header line so
  the columns are self-describing; the gyro columns are the raw, pre-bias values,
  which makes a calibration problem visible in the export.

### Motor Control, PID Tuning, Limits, Manual and Connection layouts

These tabs were re-laid out so that controls that used to be stacked
vertically now sit side by side, which cuts each tab's minimum width and leaves
far less empty space on a wide screen.

- **Motor Control** — eight group boxes in a two-column grid. Config spans the
  full width on top; **Operating Mode** / **Motor Parameters** sit in the left
  column and **Motor Selection** / **Target Values** in the right; **Phase
  Currents** / **Auto Refresh** share the next row. The six *Target Values*
  entries each occupy a `label | spin box | Set` triplet, paired two per row, so
  every value can be sent on its own without touching the other five. The speed
  read-back is scaled by a factor the GUI detects automatically at runtime (rpm
  on the patched firmware — see *Firmware speed read-back scaling*) and never
  overwrites a value you are still typing. The motor
  preview spans the bottom with its readings in a two-column grid underneath the
  dial.
- **PID Tuning** — the four PID groups form a 2×2 grid instead of a tall column.
- **Limits** — each limit gets a `label | max | min | label` row, turning eight
  rows into four.
- **Manual** — the command and response boxes share a draggable vertical
  splitter, so the response area grows with the window.
- **Connection** — the interface picker moved to its own row at the top, and the
  Serial and CAN settings now live in separate group boxes so only the group for
  the selected interface is shown (Serial **or** CAN). The *Connect* /
  *Detect Motor ID* buttons and the detected-ID label sit in one action bar
  underneath, and the serial port list was shortened from 96–140 px to
  72–96 px. The serial port picker is a **list** rather than a dropdown, so
  every available port is still visible at once.

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

## Appearance, Zoom and Language

These settings live in the **View** menu and apply immediately — no restart
needed.

### Theme

**View → Theme → Light / Dark**

The theme restyles the whole window: widget palette, tab bar, status bar,
buttons, the phase-current labels, the 2D motor preview and the IMU 3D
background.

### Zoom

**View → Zoom In / Zoom Out / Reset Zoom** or the percentage dropdown at the
right end of the status bar.

| Action | Shortcut |
| --- | --- |
| Zoom in (+10 %) | `Ctrl` / `⌘` + `+` (or `=`) |
| Zoom out (−10 %) | `Ctrl` / `⌘` + `-` (or `_`) |
| Reset to 100 % | `Ctrl` / `⌘` + `0` |

Zoom ranges from 50 % to 250 % in 10 % steps and scales **everything**: fonts,
padding, button and control sizes, the plot axes/labels/legend and the motor
preview. Tabs that no longer fit are put inside a scroll area, so every control
stays reachable on small screens. On macOS `Ctrl` is mapped to `⌘`
automatically.

### Language

**View → Language → English / 简体中文 / 繁體中文**

Switching a language retranslates every label, button, tab title, group box and
status message in place. Values already shown on screen (angles, current
readings, plot labels) are not re-rendered until the next update.

### Persistence

The chosen theme, language, zoom level and the last motor commands are written to
`config/settings.json` and restored on the next launch:

```json
{
  "theme": "light",
  "language": "en",
  "zoom": 100,
  "speed_cmd": 1200.0,
  "position_cmd": 90.0
}
```

`config/settings.json` is a settings file, not a motor profile, so it is hidden
from the configuration dropdown in the Motor Control tab. Delete it to fall back
to the defaults (`light` + `en` + `100` + `0` + `0`). A `zoom` value outside 50–250,
or one that is not a number, is ignored; `speed_cmd` and `position_cmd` are clamped
to ±100000.

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
  pauses the stream only; the measured gyro bias, the median filter window and
  the current attitude are kept, and the timestamp is re-stamped on resume so the
  paused time is not integrated as motion.
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

The target value that is actually in effect for the selected mode is drawn in
**bold** in the Target Values group, so you can see at a glance which numbers the
controller is currently using. Selecting **Stop** clears the emphasis.

### 4. Configure targets

The Target Values group shows six entries, each with its own **Set** button:

| Entry | Command sent by its Set button |
| --- | --- |
| Iq | `0x20` set-current |
| Id | `0x21` set-current |
| Speed (rpm) | `0x22` set-speed |
| Position (deg) | `0x23` set-position |
| Uq | `0x24` set-voltage (both words) |
| Ud | `0x24` set-voltage (both words) |

Each **Set** button transmits **only that single value**, so typing a speed no
longer pushes a stale position along with it. Uq and Ud share one command, so
either button sends both words together.

**Speed and Position also switch the mode.** Pressing **Set** next to *Speed (rpm)*
first sends the Speed-loop mode command and then the speed value, and *Position (deg)*
does the same for the Position loop. The Operating Mode drop-down updates
immediately, and the bold emphasis moves to the value you just sent. The other four
entries never change the mode.

The mode command and the target value are deliberately **not** written back to back.
Switching into a closed loop makes the firmware initialise the current loop and reset
the speed/position reference, and a target value that arrives during that window is
treated as a disturbance, which shows up as the motor shaking left and right. So when
the mode actually has to change, the GUI sends the mode command, waits
`MODE_SWITCH_SETTLE_MS` (120 ms), and only then sends the target. When the Operating
Mode drop-down already shows the loop you are targeting, no mode command is sent at
all and the value goes out immediately.

**Once you press Set, the box belongs to you.** The 1 s automatic refresh reads the
targets back from the controller, but it is **not allowed to change a Speed or Position
box that you have typed into or sent**. Only **Get All** hands the boxes back to the
controller. Without this rule the refresh silently replaced the value you had just
commanded with whatever the controller reported, and the next Set sent *that* — which is
how a correct 100 rpm turned into a shrinking setpoint and a motor that only shook. See
*Firmware speed read-back scaling* below for the full chain.

The command line follows the same order:

1. Enter the value in the spin box (Speed and Position accept ±100000 with three
   decimals, so you can type an exact target instead of stepping to it).
2. Press the **Set** button to the right of that entry.
3. Confirm the Operating Mode drop-down shows the loop you expect.

#### Firmware speed read-back scaling

`Get All` and the automatic 1 s refresh both read the current target values back from
the controller. The controller's `motor_aim_speed_param()` stores the speed you send in
two places:

```c
motorA.aim.aim_speed      = package.data1.f / motorA.as5047p.speed_rpm_param; /* rpm -> rad/s */
motorA.aim.aim_speeed_rpm = package.data1.f;                                 /* the rpm you typed  */
```

The speed loop uses `aim_speed`, which is correct — the internal reference really is in
rad/s and `as5047p.speed` is in rad/s too. The **read-back branch, however, returns
`aim_speed` instead of `aim_speeed_rpm`**, so a `Get` returns your rpm divided by
`speed_rpm_param` (`9.5238095`). A Set of 100 rpm reads back as `10.5`.

Because the automatic refresh ran `get_targets()` every second and wrote that value
straight into the spin box, the number in the box used to shrink on its own:
`100 → 10.5 → 1.10 → 0.116 → … → 0`. Every further Set then commanded a smaller
setpoint until the speed loop was left hunting around zero — the motor shook and could
not turn.

The GUI handles this in three ways:

- **The polling read-back can no longer overwrite a command.** A spin box is *owned by
  the user* as soon as you type into it **or press Set**, and the 1 s auto-refresh skips
  any owned box. Only an explicit **Get All** hands the boxes back to the controller.
  This is the important part: earlier the Set handlers *cleared* the flag right after
  sending, so the very next poll overwrote what you had just commanded — and the
  following Set sent that overwritten value. That feedback loop was what turned the
  controller's read-back bug into a shrinking setpoint:

  | cycle | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
  | --- | --- | --- | --- | --- | --- | --- | --- |
  | old code | 100 | 10.5 | 1.103 | 0.116 | 0.012 | 0.001 | 0 |
  | current code | 100 | 100 | 100 | 100 | 100 | 100 | 100 |

- **The speed read-back scale is auto-detected**, so the displayed number is right on
  both firmware revisions. `_detect_speed_scale()` compares the value it reads back with
  the value it last sent: a ratio of `1.0` means the controller returns rpm (patched
  firmware), a ratio of `1/9.5238095` means it returns internal rad/s (unpatched
  firmware). `firmware_speed_scale` is then set to match. `FIRMWARE_SPEED_READBACK_SCALE`
  (`9.5238095`) is only the starting guess used before the first Set of a session.
- **Position, Iq, Id, Uq and Ud are written back unchanged** — the controller stores
  those verbatim. A position that the controller clamped is only visible after
  **Get All**, which is deliberate: the box keeps showing the value you commanded.

> **The firmware has also been fixed.** `Nebula_st_mdk/FOC/AuroFOCCOMM.c`,
> `motor_aim_speed_param()`, now ends its read branch with
> `package.data1.f = motorA.aim.aim_speeed_rpm;`, so a `Get` returns rpm directly, and
> the scale above detects that automatically. No constant needs editing either way.
>
> `FOC/AuroFOCCOMM copy.c` still contains the old bug, but it is **not listed in
> `MDK-ARM/Nebula_st_mdk.uvprojx`**, so Keil never compiles it. It is dead code.

See also *Firmware fixes in `Nebula_st_mdk`* below for the other two controller-side
bugs fixed in the same pass.

Additional buttons in the same group:

- **Set All** – sends all six values at once, in the order Iq, Id, Speed, Position, Uq/Ud.
- **Get All** – reads back the six targets currently stored in the controller.
- **Get Speed** – reads back the measured speed.
- **Stop** – switches the motor to Stop mode. A Stop also cancels a target value that
  is still waiting for its mode switch to settle, so nothing is sent after you stop.

The last Speed and Position values you sent are remembered in `settings.json`
(keys `speed_cmd` and `position_cmd`) and restored into the spin boxes on the next
launch, so a repeated command needs no retyping.

If the motor still shakes at a steady speed, work through the controller side first —
see **Firmware fixes in `Nebula_st_mdk`** below. Both the speed read-back bug (which
made the setpoint shrink towards zero, leaving the speed loop hunting) and the
un-reset PID integrators (which carried a saturated integral across mode changes and
Stop) are fixed there. Confirm the speed box still shows the value you typed after the
auto-refresh has run a few times — with the current build it always will, because the
polling read-back is locked out of a box that holds a command you sent.

If the setpoint is correct and the motor still oscillates, check the **PID Tuning** tab
next: the Speed-loop gains are not read from the controller on connect, so the spin
boxes show `0.000000` until you set them, and a zero-gain speed loop will oscillate. The
**Limits** tab behaves the same way — its boxes also start at `0` and are only sent when
you press **Set Limits**. Note that the controller's own defaults are not zero
(`speed_pid = 0.0985, 0.00185`, `max_speed = ±1000 rpm`); the zeros are only what the
GUI displays before it has been told anything.

### Firmware fixes in `Nebula_st_mdk`

Three controller-side bugs behind the "motor shakes instead of turning" symptom were
fixed in `FOC/AuroPID.c`, `FOC/AuroPID.h`, `FOC/AuroFOC.c` and `FOC/AuroFOCCOMM.c`.

**1. Speed read-back returned the wrong variable** (`AuroFOCCOMM.c`,
`motor_aim_speed_param`). The read branch returned `aim_speed` (internal rad/s) instead
of `aim_speeed_rpm` (the rpm you sent), so a `Get` returned your value divided by
`9.5238095`. Combined with the GUI's 1 s auto-refresh this made the setpoint ratchet
towards zero — `100 → 10.5 → 1.10 → … → 0` — and the speed loop was left hunting.
Now returns `aim_speeed_rpm`.

**2. PID integrators were never reset** (`AuroPID.c`). `acc_integral` was only cleared
by `PID_Init`, which `APP/MotorEvent.c` calls once at boot. Nothing reset it on a mode
change, on Stop, or on a gain change, so a saturated integral was carried into the next
loop. With the defaults (`speed_pid.I = 0.00185`, `I_integral_maxlimit = 400`) a
saturated integrator alone commands `0.00185 × 400 ≈ 0.74 A` of Iq the instant a loop is
entered. A new public helper was added:

```c
void PID_Reset_Integral( AuroPID *pid ){
	pid->acc_integral = 0;
	pid->last_error   = 0;
	pid->error        = 0;
}
```

It is called from `motor_work_mode_param` (all four loops, via the file-local
`_foc_reset_all_integrals`), from `_foc_stop_loop`, and from all four PID parameter
handlers, so a gain change no longer keeps an old integral scaled by the new `I`.

**3. Position limits were never enforced** (`AuroFOCCOMM.c` +
`FOC/AuroFOC.c`). `max_position` / `min_position` were stored and read back but used
nowhere. `motor_aim_position_param` now clamps the incoming target with
`_foc_value_limit`, and `_foc_position_loop` clamps again before the position PID.

Also in `motor_work_mode_param` / `_foc_stop_loop`: entering position mode now clears
`aim.aim_position` along with `as5047p.postion`, and Stop zeroes `aim_iq`, `aim_speed`
and `aim_speeed_rpm`. Without that, entering position mode reset the *measured* position
to 0 while leaving the old *target*, so the position PID saw the whole target angle as
an error and the motor lurched.

> **You must re-flash the board.** The firmware has no `Makefile` or `CMakeLists.txt` —
> the only build path is the Keil MDK-5 project `MDK-ARM/Nebula_st_mdk.uvprojx`, so it
> must be rebuilt on Windows. The sources pass a syntax check with an ARM-targeted
> `clang -fsyntax-only` with zero errors and no new warnings, but that is not a real
> build.

> The `_foc_get_angle` offset/wrap handling was investigated as a fourth suspect and
> **cleared**: over 3600 samples its output is exactly `(raw − mechanical_offset) mod 360`
> with a maximum step of 0.01° per 0.01° of raw input. The apparent jump at raw ≈ 301.46
> is a correct wrap through 0°. It was left untouched.

A later audit of the same control code cleared the transform chain and found two further
defects — one in `FOC/AuroFOCCOMM.c` and one in the encoder driver
`BSP/as5047p_bsp_drv.c`:

**4. The encoder parity counter was never reset on retry** (`BSP/as5047p_bsp_drv.c`,
`AS5047P_bsp_read_angle`). `num` was initialised once at the top of the function, but the
parity check is retried with `goto AS5047_RE_READ`. On every retry `num` kept accumulating
on top of the previous total, so the second attempt produced a value in `[0, 30]` and the
parity test became meaningless — either looping forever or **accepting a corrupt frame**.
A corrupt frame makes `as5047p->speed` jump to an impossible value for one refresh; the
speed loop sees that as a huge error and slams Iq to its limit, which is exactly "shakes
violently but will not turn". `num = 0;` is now the first statement after the label.

**5. `motor_rotate_direct` was never returned to the GUI** (`AuroFOCCOMM.c`,
`motor_pole_pair_param`). `data1` and `data2` were filled in but `data3` was left as the
echoed request word (always `0`), so the Motor Parameters panel's *Encoder Direction* read
`0` no matter how the board was calibrated. It now returns `motorA.motor_rotate_direct`.

**Cleared by the same audit — do not change these.** The complete
Clarke → Park → iPark → SVPWM → duty chain was re-implemented in Python and swept over a
full 360° with `Uq = 1.0, Ud = 0`: the reconstructed αβ vector matches the commanded vector
at **every** angle, worst-case angle error `0.0000°`, amplitude ratio exactly `1.0`. The
apparent sign asymmetry between `_foc_park` and `_foc_ipark` is not a bug.

**Open issue: the calibration result is never saved.** `motorA.Pole_Pair`,
`motorA.motor_rotate_direct` and `motorA.mechanical_offset` are written in exactly two
places — the hard-coded defaults in `APP/MotorEvent.c` (`7`, `1`, `301.464`) and the
runtime self-commissioning routine `_foc_calibration_angle_pole_pair_mode` (mode 2).
There is no flash, EEPROM or backup-register write anywhere in the project, and
`motor_pole_pair_param` has no write branch, so the controller cannot accept a
calibrated offset back over the wire either. **Every power cycle therefore reverts the
board to `7 / ENABLE / 301.464°`.** Those are demo values for a different motor; if they
do not match the hardware, the current vector is commutated at the wrong rotor angle and
the motor buzzes instead of turning. To keep a calibration, run mode 2, read the three
values back with **Get Parameters**, and paste them into `APP/MotorEvent.c`.

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

The Motor Control and PID Tuning tabs are dense. When a tab does not fit inside
the window (too tall *or* too wide, which happens at high zoom levels) it is
placed inside a scroll area, and the window is clamped to the available screen
area on startup and after every zoom change. Maximise the window, lower the
zoom, or scroll the affected tab.

### IMU 3D model does not move, or the cube keeps spinning

The 3D model follows the gyroscope, and the gyro zero bias is measured while
polling starts. Three things keep the cube still when the board is not moving.

- **The bias is learned from the median of the still samples, not from their
  magnitude.** MEMS gyros commonly report a large constant offset at rest (the
  reference log in this repo reads about `+14 / -35 / -0.5 dps` on a board that
  is sitting perfectly flat), so the calibration cannot use "is the reading
  large?" to decide whether the device is moving. It instead discards samples
  that deviate from the running median by more than `calib_motion_dps`. Movement
  therefore no longer prevents the bias from converging - the offset can be
  arbitrarily large and still be measured correctly.
- Press **Start Polling** and leave the device still for the first 2-5 seconds
  (the status bar shows `Calibrating IMU... n/30`). Before the bias is ready the
  running median is subtracted from every sample, so the model does not spin up
  during calibration either.
- **Corrupt frames are filtered out.** A single bad gyro frame (the reference
  log contains one that briefly reads `416 dps` on Z between neighbours of
  `-0.5 dps`) would integrate into tens of degrees of phantom yaw, so the newest
  five samples per axis are median-filtered. This removes bursts of up to two
  bad frames with no effect on real rotation - sustained motion passes through
  unchanged, delayed by two frames (40 ms).
- Rotation is always applied to the model, even while calibration is still
  running, so the model responds from the first packet.
- **Frames that no sensor can explain are discarded.** Some corrupt frames keep
  a perfectly plausible accelerometer but carry an impossible gyro reading. The
  reference log contains 14 such frames that all repeat the constant
  `gy = 527.90 dps` (an I16 saturation value), and 16 frames whose raw rate
  exceeds `300 dps` even though the accelerometer-derived tilt never leaves
  +/-11 degrees. Because the blend is gated by `accel_gate_dps` and a corrupt
  high-rate frame closes that gate by itself, the one frame that needs the
  accelerometer correction is the one denied it - so a single frame could
  integrate up to `56` degrees (the per-frame `dt` is clamped to `0.1 s`) and
  the next few seconds slowly pulled it back. That is the "jolt, then return"
  symptom. The filter now checks the kinematics instead: the change in the
  accelerometer-derived gravity direction must agree with what the gyro
  predicts (`residual = |d(g_hat)/dt - omega x g_hat|`). Measured separation is
  wide - legitimate rotation stays below `91 dps` even at `400 dps`, while all
  known corrupt frames exceed `450 dps`. Frames above `consist_tol_deg` (60)
  skip the roll/pitch integration but still receive the accelerometer blend, so
  the attitude is corrected immediately rather than integrated away.
  `consist_min_dps` (40) suppresses the check at low rates, where dividing by
  `dt` would amplify accelerometer noise into false positives.
- **Frames whose accelerometer is implausible also skip integration.** If
  `|a|` leaves `[accel_norm_min, accel_norm_max]` (`0.75`-`1.25 g`) the frame
  cannot be cross-checked at all, yet its gyro is usually corrupt too, so the
  attitude is frozen for that frame. Sustained bursts (verified up to 60
  consecutive bad frames) no longer accumulate any drift. Yaw is never frozen,
  because a pure yaw rotation does not change the gravity direction and so
  cannot be validated.
- Do **not** raise `accel_gate_dps` to hide these frames. A constant rotation
  above the gate never blends in the accelerometer, and the attitude runs away
  completely (verified: a `60 dps` rotation reads `103` degrees of error per
  frame once the gate is raised to 80).

With all of the above, the reference "jolt" log improves from `4.83` to `1.30`
degrees mean tilt error, `25.83` to `4.55` at the 95th percentile, `62.05` to
`14.45` worst case, and steady pitch wander from `59.66` to `16.79` degrees,
with zero change on a clean log and identical tracking at 10-300 dps.

If the model still drifts slowly with the board at rest, the gyro is picking up
real noise above the `gyro_deadband_dps` floor (0.5 dps); raise that value in
`gui_tabs/main_window.py`. If the model spins *fast* while the board is still,
the reported zero-rate offset is larger than usual and the board's configured
gyro full-scale range is worth checking against `GYRO_SCALE` (0.0305 dps/LSB
assumes +/-1000 dps) - a mismatched range adds a scale error on top of the bias.

The frames themselves are a link-layer problem: `comm_backend.py` scans for the
`0xDE`/`0xED` head and tail bytes and `CommandPacket.parse()` validates the
length and those two bytes only. There is **no checksum**, so a corrupted
24-byte window that happens to start and end correctly is accepted silently.
Adding a CRC to the protocol, or reducing the firmware's IMU send rate (the
reference log shows `dt` jitter from 2 ms to 351 ms around a 90 ms mean), would
remove the bad frames at the source instead of filtering them here.

### IMU axes do not match the physical board

`decode_imu_packet()` in `gui_tabs/main_window.py` maps the sensor frame
straight through. If roll/pitch/yaw appear swapped or inverted, uncomment the
alternative mapping block there and adjust the signs for your mounting.

### Wrong theme, language or zoom after launch

The last selection is remembered in `config/settings.json`. Delete that file to
reset all three back to the defaults (light theme, English, 100 %).
