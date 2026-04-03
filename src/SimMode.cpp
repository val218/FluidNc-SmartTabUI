// SimMode.cpp — Full simulation. Delete this file to remove simulation.

#include "SimMode.h"
#include "FluidNCModel.h"
#include "Scene.h"
#include "System.h"
#include "driver/gpio.h"
#include <cmath>
#include <cstdio>

static bool  _active    = false;
static float _axes[4]   = { 0, 0, 0, 0 };
static float _simFro    = 100.0f;

// ── Machine state simulation ─────────────────────────────────────────────────
// State cycles: Idle → Cycle (move) → Idle → ... with Hold and Alarm on e-stop
static enum { SIM_IDLE, SIM_CYCLE, SIM_HOLD, SIM_ALARM } _simState = SIM_IDLE;
static uint32_t _stateTimer   = 0;
static float    _simProgress  = 0;   // 0-100, used for cycle animation
static bool     _estopHeld    = false;

// Simulated file list shown in Files tab
static const std::vector<SimFile> _files = {
    { "gcode",          true,  0      },  // directory
    { "test_square.nc", false, 4200   },
    { "circle.nc",      false, 8800   },
    { "drill_grid.nc",  false, 2100   },
    { "facing.nc",      false, 15600  },
    { "contour.nc",     false, 31200  },
    { "my_part.nc",     false, 9400   },
};

// ── Public API ────────────────────────────────────────────────────────────────

void simMode_enable() {
    _active = true;
    for (int i = 0; i < 4; i++) _axes[i] = 0;
    _simState   = SIM_IDLE;
    _stateTimer = millis();
    _simProgress = 0;
}

bool simMode_active() { return _active; }

bool simMode_handleDROTap(int axis) {
    if (!_active || axis < 0 || axis > 3) return false;
    _axes[axis] += 0.1f;
    return true;
}

float simMode_getPos(int axis) {
    if (axis < 0 || axis > 3) return 0;
    return _axes[axis];
}

void simMode_setEstop(bool pressed) {
    if (!_active) return;
    if (pressed && !_estopHeld) {
        _estopHeld = true;
        _simState  = SIM_ALARM;
        _stateTimer = millis();
    }
    if (!pressed && _estopHeld) {
        _estopHeld = false;
        // Released — alarm clears after short delay (simulating $X)
        _stateTimer = millis();  // will clear in tick
    }
}

void simMode_tick() {
    if (!_active) return;
    uint32_t now = millis();
    uint32_t dt  = now - _stateTimer;

    switch (_simState) {
        case SIM_IDLE:
            // After 3s idle, start a cycle
            if (dt > 3000) {
                _simState    = SIM_CYCLE;
                _stateTimer  = now;
                _simProgress = 0;
            }
            break;

        case SIM_CYCLE:
            // Cycle runs for 6 seconds, axes move sinusoidally
            _simProgress = (float)dt / 6000.0f * 100.0f;
            if (_simProgress >= 100) {
                _simProgress = 0;
                _simState    = SIM_IDLE;
                _stateTimer  = now;
            }
            // Simulate axis movement during cycle
            _axes[0] = 50.0f * sinf(_simProgress * 0.063f);
            _axes[1] = 30.0f * cosf(_simProgress * 0.063f);
            _axes[2] = -5.0f * sinf(_simProgress * 0.126f);
            break;

        case SIM_HOLD:
            // Resume after 2s
            if (dt > 2000) {
                _simState   = SIM_CYCLE;
                _stateTimer = now;
            }
            break;

        case SIM_ALARM:
            // If e-stop released, clear alarm after 300ms
            if (!_estopHeld && dt > 300) {
                _simState   = SIM_IDLE;
                _stateTimer = now;
            }
            break;
    }
}

void simMode_injectState() {
    if (!_active) return;

    // Inject axis positions
    for (int i = 0; i < 4; i++) myAxes[i] = _axes[i];

    // Inject machine state
    state_t newState;
    const char* newStr;
    switch (_simState) {
        case SIM_CYCLE: newState = Cycle;  newStr = "Cycle";  break;
        case SIM_HOLD:  newState = Hold;   newStr = "Hold";   break;
        case SIM_ALARM: newState = Alarm;  newStr = "Alarm";  break;
        default:        newState = Idle;   newStr = "Idle";   break;
    }

    if (state != newState) {
        previous_state = state;
        state          = newState;
        my_state_string = newStr;
        act_on_state_change();
    }

    myFro = 100;  // keep feed override stable in simulation

    // Inject limit switches — trigger briefly at progress ~50 to show endstop
    myLimitSwitches[0] = (_simState == SIM_CYCLE && _simProgress > 49 && _simProgress < 52);
}

const std::vector<SimFile>& simMode_fileList() { return _files; }

void simMode_setPos(int axis, float val) {
    if (axis >= 0 && axis < 4) _axes[axis] = val;
}
