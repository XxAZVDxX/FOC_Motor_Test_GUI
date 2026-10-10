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

- **Motor Control** — nine group boxes in a two-column grid. Config spans the
  full width on top; **Operating Mode** / **Motor Parameters** /
  **Set Motor Parameters** sit in the left column and **Motor Selection** /
  **Target Values** in the right; **Phase Currents** / **Auto Refresh** share the
  next row. The six *Target Values*
  entries each occupy a `label | spin box | Set` triplet, paired two per row, so
  every value can be sent on its own without touching the other five. The speed
  read-back is scaled by a factor the GUI detects automatically at runtime (rpm
  on the patched firmware — see *Firmware speed read-back scaling*) and never
  overwrites a value you are still typing. **Set Motor Parameters** writes the
  pole pair, zero offset and encoder direction back to the controller, which
  stores them in flash; it is the manual override for a bad auto-calibration,
  and **Get Parameters** pre-fills its three inputs. **Clear Calibration**
  erases the stored record and restores the `7 / ENABLE / 301.464°` compile-time
  defaults, which is the way back if a calibration left the board unusable. The motor
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

**6. The pole-pair calibration arithmetic was wrong** (`FOC/AuroFOC.c`,
`_foc_calibration_angle_pole_pair_mode`). This is the reason **only Calibration mode turned
smoothly** and every other mode shook or stood still.

The routine drives the current vector through `360 × 6 = 2160` electrical degrees and then
estimates the pole-pair count from a *single* encoder sample:

```c
/* old, wrong */
if( fabsf( calibration_end - calibration_start ) < 200 )
    calibration_Pole_Pair = (360 * 6) / fabsf( calibration_end - calibration_start );
else
    calibration_Pole_Pair = (360 * 6) / (360 - fabsf( calibration_end - calibration_start ));
```

`calibration_start` and `calibration_end` are absolute encoder readings in `[0, 360)`, so
their difference is **wrapped**: it can only describe a mechanical travel below ~200°. That
holds only for `2160 / PP < 200`, i.e. **`PP ≥ ~11`**. Below that the measured difference is
an alias of the true travel and the result is wrong by an integer factor — or is a division
by zero. Measured against a synthetic rotor for `PP = 2…25`:

| `PP` true | 2 | 3 | 4 | 5 | 6 | **7** | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 20 | 21 | 25 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| old formula | ∞ | ∞ | 12 | 30 | ∞ | **42** | 24 | 18 | 15 | 13.2 | 12 | 11.1 | 14 | 20 | ∞ | 25? |
| new formula | 2 | 3 | 4 | 5 | 6 | **7** | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 20 | 21 | 25 |

The old formula is wrong for **13 of 24** values. For the default motor it returned `42`
where the truth is `7` — **6× too large**.

Why that produces exactly the reported symptom: in every mode except calibration,
`_foc_cal_sincos` computes the electrical angle as `as5047p.angle * Pole_Pair`. With
`Pole_Pair = 42` the electrical angle advances 6× too fast, so the current vector is never
near the rotor flux axis, the average torque is ~zero and only the torque ripple remains —
the motor **buzzes and shakes but never turns**, and at some setpoints it cannot start at
all. **Calibration mode is immune** because it overrides both inputs: it forces
`foc->Pole_Pair = 1` and drives `as5047p.angle` from its own ramp, never calling
`_foc_get_angle`. That is precisely why it was the one smooth mode.

The fix accumulates the **unwrapped** encoder travel tick by tick during the ramp and then

```c
float travel_per_elec = travel / (CALIBRATION_ELEC_TURNS * 360.0f);
if( travel_per_elec > 0.0001f )
    calibration_Pole_Pair = (uint16_t)( 1.0f / travel_per_elec + 0.5f );
```

`PP = electrical_travel / mechanical_travel`, exact for every pole-pair count and free of
the divide-by-zero. Verified exact for `PP = 2…25`.

**7. Calibration results are now persisted to flash** (`APP/MotorEvent.c`). The three
commissioning values are written to the last flash page (`0x0801F800`, page 63) as a
24-byte record with a magic word and a checksum, and reloaded on boot. `APP/User_APP.c`
services the write in the **1 ms** `HAL_TIM_PeriodElapsedCallback` rather than the 40 kHz
ADC ISR, because erasing a flash page takes tens of milliseconds. The board has no DBANK
option byte set, so the single-bank 128 KB layout makes page 63 valid.

**8. `motor_pole_pair_param` gained a write branch** (`AuroFOCCOMM.c`). `0x02` with
`func3 = 0x01` now sets `Pole_Pair` (clamped to `[1, 64]`), `mechanical_offset`
(clamped to `[0, 360)`) and `motor_rotate_direct`, arms the flash save, and echoes the
values actually applied. The **Set Motor Parameters** group in the Motor Control tab uses
this: if auto-calibration ever mis-measures, the correct pole pair, zero offset and
encoder direction can be written from the GUI without rebuilding the firmware. Every
**Get Parameters** pre-fills those boxes.

**9. Mode 1 (Self-test) now sets its own `Uq`/`Ud`** (`AuroFOC.c`, `_foc_selftest_loop`).
It previously inherited whatever the previous mode had left in `foc->Uq`/`foc->Ud`, so the
self-test either did nothing or ran at an arbitrary amplitude. It now forces
`Uq = 1.5, Ud = 0`. The self-test still synthesises its angle and does not read the
encoder — that is by design, it is a power-stage test, not a motion test.

**10. `motor_pole_pair_param` gained a clear branch** (`AuroFOCCOMM.c`). `0x02` with
`func3 = 0x02` erases the stored calibration record and resets `Pole_Pair`,
`mechanical_offset` and `motor_rotate_direct` to the compile-time defaults. The **Clear
Calibration** button uses it, so a calibration that produced a nonsensical pole pair can
be undone from the GUI instead of needing another full re-calibration.

**11. `sin()`/`cos()` were double-precision inside the 20 kHz ISR** (`FOC/AuroFOC.c`,
`_foc_cal_sincos`). This was the remaining "little fast shake" in Speed-loop and
Position-loop mode. The STM32G4 toolchain here targets a **single-precision** FPU
(`fpu: sp`, `-mfpu=fpv4-sp-d16`), so `sin()` and `cos()` — which take and return
`double` — compile to the software double-precision library. Two of those calls per
50 µs ADC interrupt cost hundreds to thousands of cycles each and overran the ISR
budget, so samples were dropped and the torque command jittered.

Why only those two modes: pure Current-loop has no outer loop and just fits the 50 µs
budget. Speed-loop adds one PID, Position-loop adds two. The overflow is marginal, which
is exactly why the symptom was a small high-frequency tremble rather than a failure.

The `while( e_angle > 360.0f ) e_angle -= 360.0f` wrap was a second problem: it is an
unbounded loop on a variable. At `Pole_Pair = 14` the maximum is `360×14 + 360`, i.e. up
to 14 iterations.

Both were replaced by a single-precision 5th-order polynomial, folded into `[0°, 90°]`,
and `fmodf()` for the wrap:

```c
static inline float _foc_sin_poly( float deg ){
	float x = deg;
	float sign = 1.0f;
	if( x >= 360.0f ) x -= 360.0f;
	if( x >= 180.0f ){ x -= 180.0f; sign = -1.0f; }
	if( x >= 90.0f )  x = 180.0f - x;
	x *= 0.017453292f;
	{
		float x2 = x * x;
		return sign * x * ( 1.0f - x2 * ( 0.16666667f - x2 * ( 0.008333333f - x2 * 0.0001984127f )));
	}
}
```

`cosVal = _foc_sin_poly( e_angle + 90.0f )` reuses the same helper; the fold handles it
because the input range is designed as `[0, 450)`. Swept over 0…360° in 0.01° steps the
worst error is **0.000157** against the true `sin`/`cos` — two orders of magnitude inside
the 0.2 % budget, and it is a handful of single-precision multiply-adds.

**A lookup table was considered and rejected.** It would have to live in flash page 63
(`0x0801F800`) — the same page as the calibration record — and fine-grained flash reads
contend for, and can stall, the AHB bus at 20 kHz. A smaller table with linear
interpolation was also rejected because a 260-iteration loop in an unoptimised Debug
build cannot run 20 kHz inside a 50 µs ISR.

**12. Calibration results persist across power cycles** (`APP/MotorEvent.c`). See item 7
— the record is written to flash page 63, so the values in item 11's screenshot survive a
reboot.

**13. The SVPWM duty limiter was asymmetric, capping the output voltage at 4.16 V**
(`FOC/AuroFOC.c`, `_foc_update_pwm`). The old code clamped each leg into `[0, 0.8]`:

```c
tu = (tu > 0.8f) ? 0.8f : (tu < 0.0f) ? 0.0f : tu;   /* ×3 legs */
```

This is wrong in two independent ways:

* **The ceiling is 0.8, not 1.0.** The highest leg of an SVPWM frame is `0.5 + span/2`, so
  a 0.8 ceiling caps `span` at 0.6 — and `span = |V|·√3/Udc` — giving an effective linear
  ceiling of `0.8·Udc/√3 = 0.8×12/1.7320508 = 4.157 V`, not the theoretical `Udc/√3 = 6.928 V`.
* **It only clamps the high side.** The lowest leg is `0.5 − span/2`, which does not reach
  0 until `|V| > 5.54 V`. So between 4.16 V and 5.54 V *only the top leg flattens*, with no
  compensation at the mid-point. The duty triple no longer represents the commanded αβ
  vector — it acquires a **DC offset and a flat-top distortion** that rotates with the
  electrical angle. Swept over a full electrical revolution: 0 % of angles clipped at
  `|V| ≤ 4.0 V`, **75 % at 4.5 V**, and **100 % from 5.0 V up**, with the peak vector error
  growing monotonically (0.20 V at 4.5 V, 0.49 at 5.0, 1.06 at 6.0, 1.60 at 6.93).

Because `max_uq = max_ud = 6 V` (`APP/MotorEvent.c`), the commanded voltage legitimately
exceeds 4.157 V whenever the loop demands high torque — which is exactly and only at high
speed. The distortion is a **torque ripple** at the electrical frequency and its harmonics.
It is far too fast to see on the shaft, but it is directly palpable by hand, which matches
the report of a high-speed-only vibration that is invisible in the telemetry (the
`0x30`–`0x33` frames carry no `Uq`/`Ud`/duty values at all).

The fix replaces the two-sided clamp with a **symmetric mid-point limiter**: find the frame's
`max`/`min`, scale the triple about `(max+min)/2` by a single factor `k`, and only reduce `k`
when the span exceeds the available range.

```c
#define PWM_DUTY_MAX      0.95f
#define PWM_DUTY_MIN      0.05f
#define PWM_PERIOD_COUNT  4250.0f

mx = tu; if( tv > mx ) mx = tv; if( tw > mx ) mx = tw;
mn = tu; if( tv < mn ) mn = tv; if( tw < mn ) mn = tw;
mid  = 0.5f * ( mx + mn );
half = 0.5f * ( mx - mn );
lim  = 0.5f * ( PWM_DUTY_MAX - PWM_DUTY_MIN );
k    = ( half > lim ) ? ( lim / half ) : 1.0f;
tu = 0.5f + ( tu - mid ) * k;   /* ×3 legs */
```

Because the Clarke transform is common-mode invariant, subtracting the mid-point and
scaling by a single `k` **preserves the commanded angle exactly** and only attenuates the
magnitude — which is what removes the ripple. Swept in bit-exact `float32` over the whole
`0…8 V` disc (`k` and the `0.95` span were chosen so nothing clips until `|V| > 5.5 V`):

| \|V\| | max vector error, old | max vector error, new | angle error |
|---|---|---|---|
| ≤ 4.0 V | 0.0000 | 7.6e-7 | 0.0000° |
| 4.5 V | 0.1981 | 7.5e-7 | 0.0000° |
| 5.0 V | 0.4868 | 7.6e-7 | 0.0000° |
| 5.5 V | 0.7754 | 8.0e-7 | 0.0000° |
| 6.0 V | 1.0641 | 7.7e-7 | 0.0000° |
| 6.928 V | 1.5999 | 0.6930 (graceful) | 0.0000° |

The residual is `float32` round-off (~1e-6 V). Every duty stays inside `[0.05, 0.95]` and
every `CCR` inside `[0, 4249]`. The 0.05 floor is 212 counts, comfortably clear of the
50-count (≈294 ns) dead time configured in `Core/Src/tim.c`. The `+ 0.5f` before the `uint16_t` cast also fixes a separate
small loss: the old code truncated, throwing away up to 1 LSB (0.235 ‰ of full scale).

**14. The DC bus voltage is now measured on PA7, and `Udc` can be corrected from the GUI**
(`FOC/AuroFOCCOMM.c`, `FOC/AuroFOCCOMM.h`, `APP/MotorEvent.c` comment only; `COMM/protocol.h`
was deliberately left untouched).
The board's 电源采样 divider was previously **dead hardware**: `MX_ADC2_Init()` scans two
channels — `ADC2_IN3` (PA6, the NTC) at `ADC_REGULAR_RANK_1` and `ADC2_IN4` (PA7, VBAT) at
`ADC_REGULAR_RANK_2` — but `motor_temp_param()` polled the ADC **once** and therefore only
ever read rank 1. PA7's result was never fetched.

### The divider

`VBAT --18 kΩ--> PA7 --1 kΩ--> GND`, plus 100 nF from PA7 to GND. The ratio is therefore

$$V_{BUS} = \text{adc} \cdot \frac{V_{REF}}{4095} \cdot \frac{18\,\text{k}+1\,\text{k}}{1\,\text{k}} = \text{adc} \cdot \frac{3.3}{4095} \cdot 19 = \text{adc} \cdot 0.0153114\ \text{V}$$

| Quantity | Value |
|---|---|
| Divider gain | 19 (ratio 1/19 = 0.0526316) |
| Full scale | **62.7 V** |
| 1 LSB | **15.3 mV** (0.064 % of 24 V) |
| Filter τ | 947 Ω · 100 nF ≈ **95 µs** (≈ 12× attenuation of the 20 kHz PWM ripple) |
| 12 V ⇒ adc | 784 |
| 24 V ⇒ adc | 1567 |
| 48 V ⇒ adc | 3135 |

The one-sided RC is why an averaged single read is good enough here — the sampled node is
already smoothed 12× below the switching frequency.

### Why two polls are required

With `ScanConvMode = ENABLE`, `NbrOfConversion = 2`, `EOCSelection = ADC_EOC_SINGLE_CONV`
and `ContinuousConvMode = DISABLE`, one `HAL_ADC_Start()` runs rank 1 → rank 2 and then
stops, raising **exactly two** `EOC` flags. `HAL_ADC_PollForConversion()` waits for one
`EOC` per call, so the standard idiom applies:

```c
HAL_ADC_Start( &hadc2 );
HAL_ADC_PollForConversion( &hadc2 , 100 ); *ntc  = HAL_ADC_GetValue( &hadc2 ); /* rank 1: PA6 */
HAL_ADC_PollForConversion( &hadc2 , 100 ); *vbus = HAL_ADC_GetValue( &hadc2 ); /* rank 2: PA7 */
HAL_ADC_Stop( &hadc2 );
```

Reading only one rank leaves the sequence mid-flight, so the *next* call returns a stale
sample from the other channel. `_bus_adc_read()` wires this up once and both
`motor_temp_param()` and the new bus-voltage handler go through it. At a 42.5 MHz ADC clock
(170 MHz / `ADC_CLOCK_SYNC_PCLK_DIV4`) each 12.5-cycle conversion is ≈ 294 ns, so the pair
costs ≈ 1.2 µs once the ADC is running.

> **Superseded.** The `HAL_ADC_Start()` / `HAL_ADC_PollForConversion()` / `HAL_ADC_Stop()`
> idiom shown above was correct in principle but unsafe in practice: this function runs in
> **interrupt context**, where those calls' `HAL_GetTick()` timeouts can never fire. It also
> assumed the ADC was already enabled, which `HAL_ADC_Init()` does not do. `_bus_adc_read()`
> is now register level and tick-free — see **item 16**. The 1.2 µs figure also applies only
> to the steady state; the very first call additionally has to enable the ADC and wait for
> `ADRDY`.

### New commands

| `func2` | `func3` | Meaning |
|---|---|---|
| `0x39` | `0x00` | Read bus voltage. `data1` = volts, `data2` = active `Udc`, `data3` = active `K`, `data4` = raw ADC code |
| `0x3B` | `0x00` | Read back `Udc` (`data1`) and `K` (`data2`) |
| `0x3B` | `0x01` | Write `Udc` (`data1`); firmware clamps to `[6, 60] V` and recomputes `K = √3·Ts/Udc` |

`data4` on `0x39` returns the raw ADC code on purpose: it is the quickest way to confirm the
divider on the bench. Power the board from a known 12 V supply and check that the code
reads ≈ 784 and the reported voltage ≈ 12.0 V. **If the voltage reads about 19× too high,
the bottom 1 kΩ leg is not fitted** — stop, because 24 V on a 3.3 V pin would damage PA7.

The GUI side adds a **Bus Voltage (PA7)** group to the Motor Control tab with a
**Read Bus Voltage** button and an **Apply as Udc** button. Applying is deliberately a
**manual, two-step** action: the reading is copied into the `Set Udc (V)` box and a separate
**Set** click sends `0x3B/0x01`. Nothing in the poll path ever writes `Udc`. `Udc` feeds
`K = √3·Ts/Udc`, so letting a noisy ADC reading update it automatically would modulate the
SVPWM gain every second.

### Why this matters for the Round-12 fix

`APP/MotorEvent.c` still ships `motorA.Udc = 12` (line 128) with `max_uq = max_ud = 6 V`.
That is self-consistent **only if the bench supply really is 12 V**:

| `Udc` | `K = √3·Ts/Udc` | SVPWM linear ceiling `Udc/√3` | Speed ceiling at 32 rpm/V |
|---|---|---|---|
| 12 V | 0.144338 | 6.93 V | ≈ 222 rpm |
| 24 V | 0.072169 | 13.86 V | ≈ 443 rpm |

If the supply turns out to be 24 V while `Udc` stays at 12, `K` is **twice** too large: the
firmware believes it is applying half the voltage it actually is, the current loop gain is
doubled, and the `6 V` limit corresponds to 12 V of real output. **Read PA7 first, then set
`Udc` to the measured value.** (A 12 V bus is also what makes the Round-12 distortion
threshold coherent: the old `[0, 0.8]` clamp only produced its asymmetric flat-top above
`0.8·Udc/√3 = 5.54 V`, which `Uq = 6 V` can reach on a 12 V bus but not on 24 V.)

### Limits that are too permissive for this motor

The datasheet photo (HT4310) gives **24 V nominal, 14 pole pairs, 690 rpm max no-load**, and
32 rpm/V. Two compiled-in limits disagree with it:

* `max_speed = 1000` — the motor's no-load maximum is **690 rpm**, and the SVPWM ceiling
  above caps it near 222 rpm (12 V bus) anyway.
* `max_iq = max_id = 4 A` — **stall current is 1.8 A** and nominal current is 0.97 A, so the
  limit permits more than twice the stall current.

The pole pair is the useful confirmation: the datasheet's **14** matches the field
calibration exactly (Pole Pairs 14, Zero Offset 55.261°, Encoder Direction 0), which
independently validates the Round-10 pole-pair arithmetic fix.

**The layout fix that came with it.** The Motor Control tab had a real grid bug: both
`set_param_group` (Set Motor Parameters) and `current_group` (Phase Currents) were added to
cell `(3, 0)`, so their group-box titles were painted on top of each other and the tab
showed an unreadable *"S̶e̶t̶ ̶M̶o̶t̶o̶r̶ ̶P̶a̶r̶a̶m̶e̶t̶e̶r̶s̶ Phase Currents"* header.
`current_group` moved to the free `(4, 0)`, `Auto Refresh` now spans rows 3–4, the preview
moved to row 5 and the new bus group sits at row 6.

**Cleared by the same audit — do not change these.** The complete
Clarke → Park → iPark → SVPWM → duty chain was re-implemented in Python and swept over a
full 360° with `Uq = 1.0, Ud = 0`: the reconstructed αβ vector matches the commanded vector
at **every** angle, worst-case angle error `0.0000°`, amplitude ratio exactly `1.0`. The
apparent sign asymmetry between `_foc_park` and `_foc_ipark` is not a bug.

**15. RS485 transmission blocked the 1 ms task, and one overrun killed the receive path
permanently** (`COMM/protocol.c`, `APP/User_APP.c`, `Core/Src/usart.c`, `Core/Inc/usart.h`).
Symptom: after flashing a board, the GUI could **detect** the motor over USB but could not
control it at all over RS485 — `rs485_rx_bytes` climbed to a value and then froze forever
while `rs485_tx_bytes` kept rising, and `rs485_data` stayed `0`.

> **Scope note.** This is a real defect in the RS485 code path, verified line by line in the
> HAL source, and it is worth keeping. It is **not** the cause of the "detected but
> uncontrollable" symptom on the bench, because the bench board is driven over **USB CDC
> (Type-C)**, not RS485. The USB path is covered by item 16.

### The frame does not fit in the poll period

`package_send()` sent every RS485 frame with a **blocking** transmit:

```c
HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_SET);
HAL_UART_Transmit(&huart3, data, sizeof(data), 0xFFFF);
for (volatile int i = 0; i < 200; i++);   /* guess-the-timing busy-wait */
HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET);
```

`package_send()` is reached from `package_poll_send()` → `motor_poll_send()` inside the
**1 ms `TIM6` interrupt**. At 8N1 a 24-byte frame costs `24 · 10 / baud`:

| Port | Baud | 24-byte frame | TIM6 period | Verdict |
|---|---|---|---|---|
| `huart1` (USB) | 2 000 000 | 0.1200 ms | 1.0000 ms | fits — harmless |
| `huart3` (RS485) | 115 200 | **2.0833 ms** | 1.0000 ms | **208 % of the period** |

This asymmetry is the whole reason "detect works but RS485 does not": the identical code is
fine on the 2 Mbaud USB link and fatal on the 115 kbaud RS485 link. Three consequences
follow, in order of severity:

1. **The 1 ms task is saturated.** Each frame consumes more than two full periods, and
   `TICK_INT_PRIORITY` equals TIM6's pre-emption priority, so the HAL's `HAL_GetTick()`
   timeout cannot even advance while the transmit runs.
2. **The receiver is muted for the duration.** `RS485_EN` is driven high for the whole
   ~2.1 ms, so the transceiver cannot hear anything the GUI sends in that window.
3. **A single lost byte kills the receive path forever.** See below.

### Why one overrun is permanent

This is the exact chain, verified against the vendored HAL source rather than inferred.
In `Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart.c`:

1. `HAL_UART_IRQHandler()` (**line 2209**) computes `errorflags`; the ORE branch
   (**line 2270**) clears `OREF` and sets `huart->ErrorCode |= HAL_UART_ERROR_ORE`.
2. Because `errorcode & (RTO | ORE)` is non-zero, the "blocking error" test
   (**line 2303**) calls **`UART_EndRxTransfer(huart)`** (**line 2308**).
3. **`UART_EndRxTransfer()` (line 3577) disables `RXNEIE` and `PEIE` in `CR1`, disables
   `EIE` and `RXFTIE` in `CR3`, sets `RxState = HAL_UART_STATE_READY`, and sets
   `RxISR = NULL`.**
4. It then calls `HAL_UART_ErrorCallback(huart)` — which was the **`__weak` empty stub at
   line 2599**, because `USE_HAL_UART_REGISTER_CALLBACKS` is `0` (**`stm32g4xx_hal_conf.h:107**)
   and the project had **no override anywhere** in `APP/`, `Core/Src/`, `COMM/` or `FOC/`.

So `HAL_UART_RxISR_8BIT` stops being reached on USART3, `HAL_UART_RxCpltCallback` never
fires again, and the 1-byte re-arm inside it never runs. **RX stays dead until the next
reset** — exactly the observed freeze. There is no other re-arm point in the project, and
`HAL_UART_Receive_IT()` alone cannot recover it because it returns `HAL_BUSY` *without
touching any register* whenever `RxState != HAL_UART_STATE_READY`.

Note that the 1-byte registration in `RS485_Start_Receive()` (`Core/Src/usart.c`) and the
immediate re-arm in `HAL_UART_RxCpltCallback` were **already correct**. The defect was never
in the registration; it is that the callback stops being reached.

### The fix

* **Non-blocking transmit** (`COMM/protocol.c`). `HAL_UART_Transmit_IT()` replaces the
  blocking call, so the 1 ms task is released immediately. Because the transmit is now
  asynchronous and `package_send()` rewrites the shared `data[24]` on every call, the frame
  is copied into a dedicated `static uint8_t rs485_tx_data[24]` staging buffer first —
  without that copy the next frame would overwrite a frame still in flight and the wire
  would carry spliced garbage. A `huart3.gState == HAL_UART_STATE_READY` guard drops a frame
  when the port is busy, which is safe for a periodic-report protocol and far better than
  blocking. `RS485_EN` is raised before the call and lowered only if the call fails.
* **`HAL_UART_TxCpltCallback`** (`APP/User_APP.c`). `UART_EndTransmit_IT()` (**line 4132**)
  invokes it after the final stop bit, so `RS485_EN` is released precisely instead of after
  the old guess-the-timing busy-wait. `HAL_UART_Transmit_IT` does **not** set `TCIE` itself —
  `UART_TxISR_8BIT` enables it when the last byte is queued — so the callback fires exactly once.
* **`HAL_UART_ErrorCallback`** (`APP/User_APP.c`). Clears PE/FE/NE/ORE, resets `ErrorCode`,
  and re-arms both UARTs, with a `RxState = HAL_UART_STATE_READY` force-reset fallback. This
  is the actual cure: it turns permanent RX death into a recoverable dropped frame. It also
  releases `RS485_EN` for `huart3` unless a transmit is genuinely in flight, so a mid-frame
  fault cannot leave the transceiver muted forever.
* **Shared receive buffer** (`Core/Src/usart.c`, `Core/Inc/usart.h`). `rs485_rx_data` was a
  function-local `static` inside `RS485_Start_Receive()`, i.e. invisible to the error
  callback. It is now file-scope with an `extern` declaration, so the error callback and the
  RX-complete callback arm the **same** buffer instead of two different ones.

### What was ruled out

* A stale `RS485_Rx_Init()` / `RS485_Scan()` / `stm32_main.c` BoardComm layer was suspected
  of holding `gState` busy with a one-shot 64-byte `HAL_UART_Receive_IT`. **Those symbols do
  not exist anywhere in `FOC_Motor_Project`.**
* The RX registration being wrong. It was already exactly 1 byte, already re-armed on every
  completion. **No change was needed here.**
* `ADC2` hanging the 1 ms task. The log's injected-ADC intervals were Doppler-shifted to
  50.02 µs, proving the timebase never stalled. `ADC2` is instantaneous.

### Still worth doing on the bench

* **Confirm the fix** by re-flashing and watching `rs485_rx_bytes` climb again and
  `rs485_data` become non-zero while the GUI sends commands. The root cause is proven from
  the HAL source, but the fix has not yet been scope-verified.
* **Check that the RS485 adapter really runs at 115200.** A GUI/firmware baud mismatch is a
  simpler and equally plausible cause of zero received bytes and has not been excluded.
* **The poll period is still too fast for the bus.** At 2.0833 ms per frame, a 1 ms poll can
  use the link at most ~48 % of the time even with a non-blocking transmit. A dedicated
  slower RS485 poll (≥ 3 ms), or sending only when the port is idle, would remove the
  remaining frame drops. Not required to stop the overrun death.

**16. The bus-voltage poll ran a HAL ADC conversion inside the USB interrupt, and that
interrupt can never time out** (`FOC/AuroFOCCOMM.c`, `COMM/protocol.c`; GUI
`gui_tabs/main_window.py`). Symptom: after flashing a board and connecting over **Type-C**,
the GUI detects the motor ID but **nothing else responds** — mode read-back, targets, PID
values and the live plots all stay frozen.

This is a **pre-existing vendor defect** that was dormant until the PA7 bus-voltage feature
(item 14) gave it a caller that runs automatically on every connect.

### Why the detection reply still gets through

`package_analysis()` runs in **interrupt context**. It is reached from four callers, and the
one that matters on the bench is the USB one:

| Caller | File |
|---|---|
| `CDC_Receive_FS` (the live Type-C path) | `USB_Device/App/usbd_cdc_if.c:269` |
| `HAL_FDCAN_RxFifo0Callback` | `Core/Src/fdcan.c:105` |
| `HAL_UART_RxCpltCallback` (USART1 / USART3) | `APP/User_APP.c:57`, `:61` |

The motor-ID reply is sent **inline, from inside that same interrupt**
(`motor_detect_param()` in `FOC/AuroFOCCOMM.c` calls `package_send()` directly and never
checks the target ID), so the GUI receives it and the ID appears in the dropdown. Everything
after that is queued behind the interrupt — which is where the board stops answering.

### The priority inversion

| Interrupt | Pre-empt priority | Where |
|---|---|---|
| `ADC1_2_IRQn` | 0 | `Core/Src/adc.c:226`, `:264` |
| `USB_LP_IRQn` | **1** | `USB_Device/Target/usbd_conf.c:95` |
| `USART1_IRQn` / `USART3_IRQn` | **1** | `Core/Src/usart.c:174`, `:214` |
| `TIM1_UP_TIM16_IRQn` | 0 | `Core/Src/tim.c:262` |
| `TIM6_DAC_IRQn` (the 1 ms poll) | 2 | `Core/Src/tim.c:277` |
| **SysTick (`TICK_INT_PRIORITY`)** | **2** | `Core/Inc/stm32g4xx_hal_conf.h:184` |

`HAL_GetTick()` only advances from SysTick. At pre-empt priority **1**, SysTick (priority 2)
is **masked**, so `HAL_GetTick()` returns a frozen value. Every tick-based HAL ADC timeout
therefore compares `0 > timeout`, which is never true:

| Function | Tick variable | Timeout test |
|---|---|---|
| `HAL_ADC_Start` → `ADC_Enable` | `tickstart` 3483 | `> ADC_ENABLE_TIMEOUT` 3500 |
| `HAL_ADC_PollForConversion` | `tickstart` 1480 | `> Timeout` 1488 |
| `HAL_ADC_Stop` → `ADC_Disable` | `tickstart` 3560 | `> ADC_DISABLE_TIMEOUT` 3564 |

`HAL_ADC_Init()` is *not* the problem: it never calls `ADC_Enable`, it only starts the
internal regulator, so **ADC2 is left disabled (`ADEN == 0`)** after boot. The first
`HAL_ADC_Start()` therefore has to run the full `ADC_Enable` path — including the `ADRDY`
wait at 3485 — and if `ADRDY` does not come up, that wait **never ends**. The core sits in
the USB interrupt forever: the device still enumerates, the detect reply already went out,
and the firmware never returns to `main()` again.

### The trigger chain

The wedge is armed automatically, every time the GUI connects:

1. `connect_device()` → `detect_motor_id(auto=True)` → sends `0x00`.
2. Firmware answers inline → `handle_detect_response()`.
3. `motor_id_combo.clear()` + `addItem(...)` (`main_window.py:1561`, `:1565`).
4. That repopulation fires `currentIndexChanged`, which is **connected at line 757** →
   `on_motor_id_changed()` → `_apply_polling_gates()` → `_sync_auto_refresh_timer(True)`.
5. `refresh_all_except_mode()` ends with `read_bus_voltage(quiet=True)` (line 2302) →
   `send_command(0x39, 0x00)` (line 1637).
6. Firmware dispatches `0x39` → `motor_bus_voltage_param()` → `_bus_adc_read()` → wedge.

The GUI has no lock-out of its own: `is_busy` only gates `request_next_preview()` and
`request_currents()`, never a manual send. The board simply never answers again.

### The fix

`_bus_adc_read()` in `FOC/AuroFOCCOMM.c` is now **register level and completely tick-free**:

* Two bounded spin helpers, `_bus_adc_wait_clear()` and `_bus_adc_wait_isr()`, each capped at
  `BUS_ADC_SPIN_LIMIT` (100 000) iterations, replace every tick-based wait. A missing flag
  now costs a few milliseconds and returns `0` instead of hanging.
* Correct register order, which the previous code got partly wrong:
  `ADSTP` → wait for it to clear → (`ADEN`? `ADDIS` → wait) → `LL_ADC_Enable()` → wait
  `ADRDY` → clear `ADRDY|OVR` → `LL_ADC_REG_StartConversion()` → wait `EOC` → read `DR`
  (rank 1 = PA6/NTC) → wait `EOC` → read `DR` (rank 2 = PA7/VBUS).
* ADC2 is **scan mode** (`NbrOfConversion = 2`, `EOC_SINGLE_CONV`, software start,
  non-continuous), so one start produces exactly two EOCs. The old code polled once and only
  ever consumed rank 1 — PA7 was effectively dead even when it did not hang.

Because `LL_ADC_REG_StartConversion()` is used directly, the fixed conversion sequence is
written straight into `CR`. This is required: a plain read-modify-write of `CR` would
mis-handle the hardware-read-sensitive bits masked by `ADC_CR_BITS_PROPERTY_RS`.

### Three more defects fixed at the same time

| # | Defect | Effect | Fix |
|---|---|---|---|
| 1 | `motor_poll_send()` (`FOC/AuroFOCCOMM.c`) `return`ed silently when the requested ID did not match the local one, and only the **Position** branch ever filled `package.motor_id` | `0x30`–`0x33` replies either never arrived or carried a garbage ID byte — "detected, but every reading empty" | The function now **always** replies, zero-filling the payload on mismatch, and sets `motor_id` once right after `head` so **all five** branches are correct |
| 2 | `poll_type` was **never cleared** — only written by the five poll setters and read by `motor_poll_send()`, which runs every 1 ms from TIM6 | One read request turned into a **1000 Hz reply flood** that starved the GUI's one-byte-at-a-time reader | `protocol_ctrl.poll_type = POLL_PACKAGE_TYPE_NONE;` right after `package_send(...)` — one request, one reply. The global is also initialised to `POLL_PACKAGE_TYPE_NONE` in its definition |
| 3 | `package_send()`'s `usb_cdc_type` branch called `CDC_Transmit_FS(data, ...)` and **discarded the return value**, passing the shared `data[24]` global | `CDC_Transmit_FS()` silently drops the frame when USB is busy (`TxState != 0`), and `USBD_CDC_SetTxBuffer()` only stores the pointer — the USB interrupt later copies a buffer that has already been overwritten, splicing two frames together | A private `usb_cdc_tx_data[24]` staging buffer plus `memcpy()` before the call, so the frame stays intact until the USB interrupt has copied it |

A duplicate `else if( send_type == can_type )` inside the `nrf24l01_type` branch of
`package_send()` was also corrected — it repeated the CAN condition verbatim, so the branch
could never be entered even if `SUPPORT_NRF24L01` were enabled.

Separately, `gui_tabs/main_window.py` no longer does `ids = list(set(ids))` on the detected
IDs. Set ordering put the `0xFFFF` broadcast sentinel **first** for every ID ≡ 7 (mod 8), so
`self.motor_id` could be set to the sentinel and every subsequent command would be addressed
to broadcast. It is now an order-preserving de-duplication with the sentinel filtered out.

### What was ruled out

* **Stack overflow.** `MDK-ARM/out/.../Nebula_st_mdk.axf.map` shows `Stack_Mem 0x20004d40`
  with `5376` bytes reserved, and the build report records `Stack Usage = 1064 bytes` —
  about 20 % used.
* **A PA7 pin conflict.** PA6 and PA7 are configured once, as `GPIO_MODE_ANALOG` with no
  pull, in `Core/Src/adc.c:258`.
* **Baud-rate mismatch** between GUI and firmware.
* **`package_init()`'s status gate** and the motor-ID byte order.
* **The poll setters never being reached** — they are reached, and they do set `poll_type`.
* **A GUI lock-out.** `setEnabled` is never called on the control widgets after connect.
* **A missing `package_send()` in the bus-voltage and temperature handlers** — both always
  reply for `func3 == 0x00`.

### Still worth doing on the bench

* **Rebuild before flashing.** The last build artefacts in
  `MDK-ARM/out/Nebula_st_mdk/Nebula_st_mdk/` are timestamped 05:14, while the edited sources
  are 18:11–18:18 — the existing `Nebula_st_mdk.bin` does **not** contain these fixes, and it
  does not contain the RS485 fix from item 15 either.
* **Check what the erase mode did to the calibration page.** `LR_IROM1` is
  `0x08000000 0x00020000` (128 KB) and the calibration record lives on the last page at
  `0x0801F800`. A **full-chip erase** wipes it, so `Pole_Pair` silently reverts from the
  measured 14 to the default 7 and the zero offset from 55.261° to 301.464°. Prefer
  *Erase Sectors*.
* **Watch the raw PA7 code.** `motor_bus_voltage_param` (`0x39`) returns the raw 12-bit code
  in `data4`, which is the quickest way to confirm the divider without a meter: at a 24 V bus
  that word should read about 1570, and at 12 V about 785.
* **Confirm the divider assumes the 1 kΩ is the bottom leg** (18 kΩ top, 1 kΩ bottom, gain
  19). At a 12 V bus, PA7 should sit at ≈ 0.63 V. **If it measures near 12 V, stop and
  disconnect immediately** — the pin would be over-volted.
* **Re-check `Udc`.** The firmware default is 12 V but this motor (HT4310) is rated 24 V.
* **`0x3A` and `0x3C` are dispatched by neither side** — no firmware handler is reachable and
  no GUI control sends them.
* **`NTC_GetTemperature()` contradicts itself:** the comment says `R33 = 2800 Ω` /
  `R34 = 2500 Ω`, the code says `10000.0f` / `3.3f`. Left alone; there is no GUI path to
  `0x3C` anyway.
* **The motion limits are over-permissive for this motor:** `max_speed = 1000` against a
  690 rpm maximum no-load speed, and `max_iq`/`max_id = 4 A` against a 1.8 A stall current.

### 5. Calibrate the board (do this once after flashing)

Flash the firmware, open the Motor Control tab, and switch the mode to **Calibration**.
The board turns the rotor through six electrical revolutions in each direction and then
writes the measured pole pair, zero offset and encoder direction to flash automatically.
Open-loop, current, speed and position modes will then commutate correctly — provided the
proper bus voltage and current limits are set. Confirm with **Get Parameters**; all three
readings must be plausible (a pole pair in the single digits or low tens, an offset inside
`[0, 360)`). If a reading looks wrong, type the correct value into **Set Motor
Parameters** and press **Set** — it is saved immediately and survives a power cycle. If
even that does not help, press **Clear Calibration** to erase the stored record and go back
to the `7 / ENABLE / 301.464°` defaults, then calibrate once more.

> **Note for the Keil flash settings.** In *Options for Target → Utilities → Settings*
> use **Erase Sectors** (not *Erase Full Chip*) so that re-flashing the firmware does not
> wipe the calibration page. *Erase Full Chip* erases everything and the board will fall
> back to the `7 / ENABLE / 301.464°` defaults until you calibrate again. If the board
> still behaves oddly after a *Download*, that is why.

### 6. Tune parameters

Use the PID tuning tab to adjust controller parameters.

### 7. Monitor data

Use the real-time data and IMU tabs to observe:

- Currents
- Speed
- Position
- IMU orientation

### 8. Send manual commands

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
