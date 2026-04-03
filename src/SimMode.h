// SimMode.h — Simulation mode for testing without connected hardware.
// TO REMOVE SIMULATION: delete SimMode.h and SimMode.cpp,
// then remove #include "SimMode.h" from TabScene.cpp and ardmain.cpp.
#pragma once

// Call once at boot to enable simulation mode
void simMode_enable();

// Returns true if simulation mode is active
bool simMode_active();

// Call from TabScene DRO tap handler — increments simulated axis by 0.1mm
// Returns true if the tap was consumed (sim mode active and tap in DRO area)
bool simMode_handleDROTap(int axis);  // axis 0-3

// Get simulated position for axis (0-3)
float simMode_getPos(int axis);

// Reset all simulated positions to 0
void simMode_reset();
