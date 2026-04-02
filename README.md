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
