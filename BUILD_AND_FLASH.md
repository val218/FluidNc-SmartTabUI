# FluidDial Tab UI — Build & Flash Guide

## What you need

- A computer with Python 3.8+ installed
- USB cable connected to your JC2432W328C board
- The extracted project folder (this zip)

---

## Step 1 — Install PlatformIO

```bash
pip install platformio
```

Verify it worked:
```bash
pio --version
```

---

## Step 2 — Build the firmware

Open a terminal in the extracted project folder (where `platformio.ini` is), then run:

```bash
pio run -e cyddial
```

This compiles the firmware. First run downloads ~500 MB of ESP32 toolchain automatically.

---

## Step 3 — Build the filesystem image

```bash
pio run -e cyddial -t buildfs
```

This packages the `/data` folder (icons, fonts) into a LittleFS image.

---

## Step 4 — Merge into one flash binary

```bash
pio run -e cyddial -t build_merged
```

Output file:
```
.pio/build/cyddial/merged-flash.bin
```

This single file contains bootloader + firmware + filesystem — flash it at address `0x0`.

---

## Step 5 — Flash via web installer

### Option A — FluidDial web installer (easiest)
1. Go to: **https://installer.fluidnc.com** (or the FluidDial-specific installer URL)
2. Click **Custom Image**
3. Select `.pio/build/cyddial/merged-flash.bin`
4. Connect your board via USB and click **Flash**

### Option B — esptool.py (command line)
```bash
pip install esptool
esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 921600 \
  write_flash 0x0 .pio/build/cyddial/merged-flash.bin
```
Replace `/dev/ttyUSB0` with your port (`COM3` on Windows, `/dev/cu.usbserial-*` on Mac).

### Option C — ESP32 Flash Tool (Windows GUI)
1. Download from: https://www.espressif.com/en/support/download/other-tools
2. Select chip: **ESP32**
3. Add the merged binary at address **0x0**
4. Click **START**

---

## Board variants

| Board | Environment flag | Notes |
|---|---|---|
| JC2432W328C (resistive or capacitive) | `cyddial` | **Use this — auto-detects** |
| JC2432W328C resistive only | `cyd_resistive` | |
| JC2432W328C capacitive only | `cyd_nodebug` | |
| M5Dial | `m5dial` | Different hardware |

To change environment, replace `cyddial` with the appropriate flag in all commands above.

---

## Troubleshooting

**`pio: command not found`** — Close and reopen terminal after installing PlatformIO.

**Upload fails / port not found** — Hold the BOOT button on the ESP32 while clicking Flash, release after upload starts.

**Screen blank after flash** — The display init sometimes needs a power cycle (unplug USB, plug back in).

**Build error about missing `GrblParserC.h`** — PlatformIO will download it automatically on first build. Ensure internet access during build.
