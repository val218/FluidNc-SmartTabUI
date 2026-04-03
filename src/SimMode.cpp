// SimMode.cpp — Simulation mode implementation.
// TO REMOVE SIMULATION: delete this file and SimMode.h,
// then remove #include "SimMode.h" from TabScene.cpp and ardmain.cpp.

#include "SimMode.h"

static bool  _active       = false;
static float _axes[4]      = { 0, 0, 0, 0 };

void simMode_enable() {
    _active = true;
    for (int i = 0; i < 4; i++) _axes[i] = 0;
}

bool simMode_active() {
    return _active;
}

bool simMode_handleDROTap(int axis) {
    if (!_active || axis < 0 || axis > 3) return false;
    _axes[axis] += 0.1f;
    return true;
}

float simMode_getPos(int axis) {
    if (axis < 0 || axis > 3) return 0;
    return _axes[axis];
}

void simMode_reset() {
    for (int i = 0; i < 4; i++) _axes[i] = 0;
}
