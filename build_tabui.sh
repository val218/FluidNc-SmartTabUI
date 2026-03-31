#!/usr/bin/env bash
# build_tabui.sh
# Builds the FluidDial Tab UI firmware and produces the merged flash image
# ready for the FluidDial web installer.
#
# Requirements:
#   - Python 3.8+
#   - PlatformIO CLI  (pip install platformio)
#
# Usage:
#   chmod +x build_tabui.sh
#   ./build_tabui.sh
#
# Output:
#   .pio/build/cyddial/merged-flash.bin   ← flash this via the web installer

set -e

ENV="cyddial"   # JC2432W328C — change to cyd_resistive or cyd_nodebug if needed

echo "============================================================"
echo "  FluidDial Tab UI — Build Script"
echo "  Target environment: $ENV"
echo "============================================================"

# 1. Compile firmware
echo ""
echo ">>> Step 1/3  Building firmware..."
pio run -e $ENV

# 2. Build filesystem image (LittleFS with /data assets)
echo ""
echo ">>> Step 2/3  Building LittleFS filesystem image..."
pio run -e $ENV -t buildfs

# 3. Merge firmware + filesystem into one flash image
echo ""
echo ">>> Step 3/3  Merging into single flash image..."
pio run -e $ENV -t build_merged

echo ""
echo "============================================================"
echo "  BUILD COMPLETE"
echo ""
echo "  Merged flash image:"
echo "    .pio/build/$ENV/merged-flash.bin"
echo ""
echo "  Flash via web installer:"
echo "    1. Go to https://espressif.github.io/esptool-js/"
echo "       OR use the FluidDial web installer if available"
echo "    2. Connect ESP32 via USB"
echo "    3. Select merged-flash.bin"
echo "    4. Flash address: 0x0"
echo "============================================================"
