// SimMode.h — Full simulation mode for testing without hardware.
// TO REMOVE: delete SimMode.h + SimMode.cpp, remove #include "SimMode.h"
// from TabScene.cpp and ardmain.cpp, remove simMode_* call sites.
#pragma once
#include <string>
#include <vector>

struct SimFile { std::string name; bool isDir; int size; };

// Lifecycle
void simMode_enable();
bool simMode_active();

// Axis movement — returns true if tap consumed
bool simMode_handleDROTap(int axis);   // +0.1mm per tap
float simMode_getPos(int axis);        // 0-3

// Machine state simulation
void simMode_tick();                   // call every dispatch_events loop
void simMode_setEstop(bool pressed);   // feed from GPIO17 reading

// Simulated file list for Files tab
const std::vector<SimFile>& simMode_fileList();

// Inject sim data into FluidNC model globals (called by tick)
void simMode_injectState();

// Direct axis position control (used by path animation)
void simMode_setPos(int axis, float val);
void simMode_reset();
