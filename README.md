# FluidDial TabUI — JC2432W328C CNC Pendant

A CNC pendant firmware for the **JC2432W328C** (320×240 capacitive touchscreen, ESP32, ILI9341).
Designed for use with [FluidNC](https://github.com/bdring/FluidNC).

## Hardware

- **Display:** JC2432W328C — 320×240 ILI9341 + CST816S capacitive touch
- **MCU:** ESP32 (no PSRAM)
- **MPG Handwheel:** Quadrature encoder on GPIO16 (A) / GPIO4 (B)
- **Switch inputs:** PCF8574 I2C expander on GPIO21 (SDA) / GPIO22 (SCL)
- **E-stop:** GPIO17 (active-LOW, internal pullup)

## PCF8574 Pin Map

| Pin | Function |
|-----|----------|
| P0  | Axis X   |
| P1  | Axis Y   |
| P2  | Axis Z   |
| P3  | Axis A   |
| P4  | Step 1mm |
| P5  | Step 10mm |

All active-LOW — switch common to GND. PCF8574 address: 0x20 (A0=A1=A2=GND).
PCF8574 is optional — UI works normally without it connected.

## Building

Uses PlatformIO. Build the `tabui` environment:

```
pio run -e tabui
pio run -e tabui -t buildfs
pio run -e tabui -t build_merged
```

Flash `merged-flash.bin` at address `0x0` using esptool or the GitHub Actions workflow.

## UI Tabs

| Tab | Function |
|-----|----------|
| DRO | Live axis positions, feed override, G-code visualizer |
| Home | Homing, endstop status, zero work coordinates |
| Files | SD card file browser and job runner |
| Term | Serial terminal with command shortcuts |
| Macros | 12 configurable macro buttons |

---

## Settings Menu

Access by holding the **EN button during boot** (within the first 1 second).

| Setting | Description |
|---------|-------------|
| SIM | Sim mode on/off — skips FluidNC connection for UI testing |
| THEME | Dark / Neutral / Light colour scheme |
| P6 BTN | EN button behaviour: Gate All / Touch / Jog / Macro / Off |
| X mm | Work area width (mm) |
| Y mm | Work area height (mm) |
| HOME | Machine home corner (Bot-L / Bot-R / Top-L / Top-R) |
| BRIGHT | Backlight brightness slider (saved to NVS) |
| VOL | Speaker volume 0–9 (0 = mute, saved to NVS) |

Scroll the menu with finger drag or the MPG encoder. **Save & Boot** restarts with new settings.

---

## Job Execution & Safe Pre-run Sequence

When **Run** is confirmed in the Files tab, the pendant automatically:

1. Sets absolute mode (`G90`)
2. Retracts Z to machine home (`G53 G0 Z0`) — safe height, cannot overshoot endstop
3. Moves XY to WCS zero (`G0 X0 Y0`) — Z is already safe before XY moves
4. Starts the file (`$Localfs/Run=...`)

A **Preparing Job** overlay shows in the viz panel during this sequence. The left DRO gauges (X/Y/Z/A positions) remain visible throughout.

> **Requirement:** machine must be homed (`$H`) before running a job so that `G53` machine coordinates are valid.

---

## Z Height Nudge (Mid-Job)

While a job is in **Hold** state, tap the toolpath viz area to open the Z height adjustment panel:

- **MPG encoder** adjusts Z by the current step increment
- **Step rotary switch** selects increment: x1 = 0.01mm · x10 = 0.1mm · x100 = 1mm
- The panel shows **Current Z**, **New Z**, and accumulated **Offset**
- **Cancel** — closes panel, resets offset, sends `G49`
- **Apply + Resume** — applies offset and sends `~` to resume the job

---

## E-Stop Recovery

When the e-stop is pressed:
- Machine receives Ctrl-X soft reset → Alarm state
- Alarm siren beeps 3× (two-tone, full volume)

When e-stop is released, a recovery menu appears with 4 options:

| Button | Action |
|--------|--------|
| **Resume if safe (~)** | `$X` → wait for Idle → send `~` to resume |
| **Rehome** | `$X` → Idle → `$HZ` → Idle → `$HX` + `$HY` |
| **Rehome + Resume job** | Same as Rehome, then re-runs the last job file |
| **Cancel Job ($X)** | `$X`, clears job state, clears viz, deselects file |

All actions use an async queue — no blocking delays on the UI thread.

---

## Job Resurrection System

The pendant continuously saves a checkpoint to LittleFS flash every 20 executed G-code lines. On next power-up, if an interrupted job is detected, a wizard guides the operator through safe recovery.

### What is saved (every 20 lines)

- Job file path on SD card
- Last confirmed executed line number
- Machine position X/Y/Z
- Active WCS (G54–G59)
- Units (G20/G21), distance mode (G90/G91)
- Feed rate, spindle speed, tool number, coolant state

The checkpoint has a `dirty` flag — set on save, cleared only on clean job completion.

### On boot — if dirty checkpoint found

A full-screen prompt appears: **"Interrupted job found — Resume?"**

### Recovery wizard flow

**Step 1 — Tool break or E-stop involved?**

| Answer | Path |
|--------|------|
| NO (clean power loss / pause) | → Standard resume (automatic preamble) |
| YES (tool break / E-stop / crash) | → Manual safe recovery flow |

**Standard resume sequence (NO)**
1. Home machine to re-establish reference
2. Restore modal state (WCS, units, spindle, coolant, tool)
3. Rapid to XY of last position at safe Z height
4. Lower Z to cutting depth at feed rate
5. Re-run job file from ~5 lines before checkpoint

**Manual safe recovery (YES — tool break / E-stop)**
1. **Show last 10 lines** — operator selects resume line (default: checkpoint −5)
2. **Manual jog screen** — use MPG wheel to jog Z up clear, then XY clear of workpiece. No automatic movement — path may be obstructed
3. **Tool change screen** — change tool and run Z probe (Home tab → Probe)
4. **Confirm screen** — shows full preamble sequence before any motion
5. Operator gives final confirmation → machine executes preamble and resumes

### Critical safety rules
- **Never** auto-return to job position after a suspected tool break
- **Always** retract Z before any XY movement during return
- **Never** auto-resume without explicit operator confirmation
- **Always** re-home after power loss — stepper position is unknown
- **Always** back up 3–5 lines from checkpoint — last move may have been incomplete

---

## Home Tab — Reference Operations

Scrollable list sections (scroll with finger or MPG encoder):

| Section | Buttons | Action |
|---------|---------|--------|
| Homing | Home All | `$HZ` → Idle → `$HX` + `$HY` (Z first for safety) |
| Homing | Probe | Opens probe operations overlay |
| Home Axis | X / Y / Z | Individual axis homing |
| Zero WCS | X=0 / Y=0 / Z=0 / All=0 | `G10 L20 P1 ...` |
| Go to 0 | →X / →Y / →Z / →All | Rapid to WCS zero (Z lifts first for XY moves) |
| Endstops | X / Y / Z | Live endstop status indicators |

---

## Speaker Tones

| Event | Frequency | Duration |
|-------|-----------|----------|
| Touch click | 600 Hz | 8 ms |
| EN pressed | 1100 Hz | 20 ms |
| EN released | 700 Hz | 15 ms |
| Axis selected | 1200 Hz | 30 ms |
| Axis deselected | 800 Hz | 30 ms |
| Step changed | 1800 Hz | 20 ms |
| Alarm | 880+1400 Hz × 4 | 60 ms each |
| E-stop | 880+1400 Hz × 3 | 80 ms each (full volume) |

Volume 0 = muted. E-stop beep always plays at full volume regardless of volume setting.

---

## VIZ Upload Tool

`viz_upload.html` — standalone browser tool for pre-generating `.viz` toolpath sidecar files.

1. Open in Firefox (or run `python -m http.server 8765` for Chrome)
2. Enter FluidNC IP address
3. Drop G-code file → toolpath preview renders instantly
4. Click Upload → uploads both `.nc` and `.viz` to SD card
5. On pendant: select file → toolpath loads instantly (no parsing needed)

---

## Colour Themes

Select in Settings menu (hold EN on boot):

| Theme | Background | Best for |
|-------|-----------|---------|
| Dark | Deep blue-grey | Low-light workshop, night use |
| Neutral | Medium grey | Mixed lighting |
| Light | Near-white | Bright environment, direct sunlight |

All UI elements adapt to the selected theme. Light theme uses black text and darker accent colours for maximum contrast.
