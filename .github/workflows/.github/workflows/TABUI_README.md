# FluidDial Tab UI — Integration Guide

## Overview

`TabScene.cpp` implements a completely new 5-tab landscape UI for the
**JC2432W328C 320×240** display.  It replaces the radial pie-menu with a
persistent bottom navigation bar and five full-screen tabs:

| Tab | Purpose |
|-----|---------|
| **DRO** | Live axis readouts (X/Y/Z/A), G-code visualizer, feed-rate control |
| **Homing** | Home All, per-axis home, endstop status, Zero WCS, Probe overlay |
| **Files** | SD card browser — navigate folders, select and run files |
| **Terminal** | Serial log output with quick-command buttons |
| **Macros** | 12 configurable one-tap macro buttons |

---

## Files added

```
src/TabScene.cpp     — Complete tab UI implementation
src/TabScene.h       — Header exposing getTabScene()
src/ardmain_tab.cpp  — Drop-in ardmain that boots into TabScene
```

---

## Integration steps

### Option A — Replace ardmain (simplest)

1. **Back up** the original `src/ardmain.cpp`.
2. **Delete** `src/ardmain.cpp` and **rename** `src/ardmain_tab.cpp` →
   `src/ardmain.cpp`.
3. Build normally.

### Option B — Compile-time switch (keep both UIs)

1. Add to `src/Config.h`:
   ```cpp
   #define USE_TAB_UI
   ```
2. Wrap `src/ardmain.cpp` with a guard at the top and bottom:
   ```cpp
   #ifndef USE_TAB_UI
   // ... entire original file ...
   #endif
   ```
3. Wrap `src/ardmain_tab.cpp` with:
   ```cpp
   #ifdef USE_TAB_UI
   // ... entire file ...
   #endif
   ```
4. Build normally.  Define or undefine `USE_TAB_UI` to switch.

---

## Layout constants (top of TabScene.cpp)

```cpp
static const int W       = 320;   // display width
static const int H       = 240;   // display height
static const int TOP     = 20;    // header height
static const int BOT     = 34;    // nav bar height
static const int DROW    = 90;    // DRO column width
static const int FEED_H  = 34;    // feed bar height
static const int TAB_W   = 22;    // visualizer tab-strip width
```

Adjust these if you need a different panel split.

---

## Colour palette

All colours are defined as `COL_*` constants near the top of `TabScene.cpp`
as RGB565 values.  Change them freely to match your preference.

---

## Macros

The 12 macro buttons are defined in the `MACROS[]` table:

```cpp
static const MacroBtn MACROS[] = {
    { "Tool Change", COL_ORANGE,  "M6"                     },
    { "Park",        COL_CYAN,    "G53 G0 Z0"              },
    { "Coolant On",  COL_BLUE,    "M8"                     },
    { "Coolant Off", COL_BLUE,    "M9"                     },
    { "Probe Z",     COL_YELLOW,  "G38.2 Z-50 F100"        },
    { "Zero X",      COL_AX_X,   "G10 L20 P1 X0"          },
    { "Zero Y",      COL_AX_Y,   "G10 L20 P1 Y0"          },
    { "Zero Z",      COL_AX_Z,   "G10 L20 P1 Z0"          },
    { "Zero All",    COL_ORANGE,  "G10 L20 P1 X0 Y0 Z0"   },
    { "Custom 1",    COL_DIM2,    ""                        },
    { "Custom 2",    COL_DIM2,    ""                        },
    { "Custom 3",    COL_DIM2,    ""                        },
};
```

Edit names, colours, and GCode/`$` commands as needed.  Empty `cmd` strings
are ignored (button does nothing).

---

## Probe options

Six probe operations are defined in `PROBE_OPTS[]`.  Each sends a raw GCode
command.  Adjust the commands to match your probe configuration and seek
distances.

---

## Terminal

The terminal auto-appends `[MSG:…]` messages received from FluidNC via
`onMessage()`.  Quick-command buttons send `$H`, `$?`, `!` (feed hold),
`~` (cycle start), and `$X` (unlock alarm) as real-time commands.

---

## DRO

Axis positions are read from `myAxes[]` (the existing FluidNC model).
The visualizer panel is left as a placeholder — to add G-code path rendering,
implement `drawVizContent()` using the existing `FileParser` data.

---

## Alarm overlay

When FluidNC enters Alarm state, a full-screen red overlay appears on *every
tab* with an **UNLOCK ($X)** button.  All other touches are blocked until the
alarm is cleared.

---

## Extending

To add a sixth tab, increase `N_TABS` to 6, add an entry to `TAB_LABELS[]`,
and add a `case 5:` branch in `reDisplay()` and `onTouchClick()`.
