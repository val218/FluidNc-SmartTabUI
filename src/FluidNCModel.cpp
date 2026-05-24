// Copyright (c) 2023 Mitch Bradley
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "FluidNCModel.h"
extern void markDirty();
#include "FileParser.h"  // init_file_list()
#include <map>
#include "System.h"
#include "Scene.h"
#include "e4math.h"

extern Scene statusScene;

// local copies of status items
const char*        my_state_string    = "N/C";
state_t            state              = Disconnected;
int                n_axes             = 3;
pos_t              myAxes[6]          = { 0 };
bool               myLimitSwitches[6] = { false };
bool               myProbeSwitch      = false;
const char*        myFile             = "";  // running SD filename
const char*        myCtrlPins         = "";
file_percent_t     myPercent          = 0.0;  // percent conplete of SD file
override_percent_t myFro              = 100;  // Feed rate override
override_percent_t mySro              = 100;  // Spindle Override
override_percent_t myRro              = 100;  // Rapid Override
uint32_t           myFeed             = 0;
uint32_t           mySpeed            = 0;
uint32_t           mySelectedTool     = 0;

std::string myModes = "no data";

int      lastAlarm = 0;
int      lastError = 0;
bool     inInches  = false;
uint32_t errorExpire;

int num_digits() {
    return inInches ? 3 : 2;
}

// clang-format off
// Maps the state strings in status reports to internal state enum values
struct cmp_str {
   bool operator()(char const *a, char const *b) const    {
      return strcmp(a, b) < 0;
   }
};

std::map<const char *, state_t, cmp_str>  state_map = {
    { "Idle", Idle },
    { "Alarm", Alarm },
    { "Hold:0", Hold },
    { "Hold:1", Hold },
    { "Run", Cycle },
    { "Jog", Jog },
    { "Home", Homing },
    { "Door:0", DoorClosed },
    { "Door:1", DoorOpen },
    { "Check", CheckMode },
    { "Sleep", GrblSleep },
};
// clang-format on

bool decode_state_string(const char* state_string, state_t& state) {
    if (strcmp(my_state_string, state_string) != 0) {
        auto found = state_map.find(state_string);
        if (found != state_map.end()) {
            my_state_string = found->first;
            state           = found->second;
            return true;
        }
    }
    return false;
}

// ── Connection management — parser-driven, all on Core 1 ────────────────────
static uint32_t _lastStatusMs   = 0;
static uint32_t _nextPingMs     = 0;
static int      _missedPings    = 0;
static bool     _wasConnected   = false;
static const uint32_t PING_INTERVAL_MS   = 500;
static const uint32_t DISCONNECT_TIMEOUT = 5000;
static const int      DISCONNECT_PINGS   = 8;

void set_disconnected_state() {
    _wasConnected   = false;
    _missedPings    = 0;
    _lastStatusMs   = 0;
    state           = Disconnected;
    my_state_string = "N/C";
}

// clang-format off
std::map<int, const char*> error_map = {  // Do here so abreviations are right for the dial
    { 0, "None"},
    { 1, "GCode letter"},
    { 2, "GCode format"},
    { 3, "Bad $ command"},
    { 4, "Negative value"},
    { 5, "Setting Diabled"},
    { 10, "Soft limit error"},
    { 13, "Check door"},
    { 18, "No Homing Cycles"},
    { 20, "Unsupported GCode"},
    { 22, "Undefined feedrate"},
    { 19, "No single axis"},
    { 34, "Arc radius error"},
    { 39, "P Param Exceeded"},
};
// clang-format on

const char* decode_error_number(int error_num) {
    if (error_map.find(error_num) != error_map.end()) {
        return error_map[error_num];
    }
    static char retval[33];
    sprintf(retval, "%d", error_num);
    return retval;
}

extern "C" void begin_status_report() {
    myPercent = 0;
}

extern "C" void show_file(const char* filename, file_percent_t percent) {
    myPercent = percent;
}

extern "C" void show_overrides(override_percent_t feed_ovr, override_percent_t rapid_ovr, override_percent_t spindle_ovr) {
    myFro = feed_ovr;
    myRro = rapid_ovr;
    mySro = spindle_ovr;
}

extern "C" void show_feed_spindle(uint32_t feedrate, uint32_t spindle_speed) {
    myFeed  = feedrate;
    mySpeed = spindle_speed;
};

extern "C" void show_limits(bool probe, const bool* limits, size_t n_axis) {
    myProbeSwitch = probe;
    memcpy(myLimitSwitches, limits, n_axis * sizeof(*limits));
}

extern "C" void show_control_pins(const char* pins) {
    //dbg_printf("show_control_pins:%s\r\n", pins);
    myCtrlPins = pins;
}

#ifdef E4_POS_T
extern "C" void show_dro(const pos_t* axes, const pos_t* wco, bool isMpos, bool* limits, size_t n_axis) {
    n_axes = (int)n_axis;
    for (int axis = 0; axis < n_axis; axis++) {
        e4_t axis_val = axes[axis];
        if (isMpos) {
            axis_val -= wco[axis];
        }
        myAxes[axis] = inInches ? e4_mm_to_inch(axis_val) : axis_val;
    }
}
#else
pos_t fromMm(pos_t position) {
    return inInches ? position / 25.4 : position;
}
pos_t toMm(pos_t position) {
    return inInches ? position * 25.4 : position;
}

extern "C" void show_dro(const pos_t* axes, const pos_t* wco, bool isMpos, bool* limits, size_t n_axis) {
    for (int axis = 0; axis < n_axis; axis++) {
        myAxes[axis] = fromMm(axes[axis]);
        if (isMpos) {
            myAxes[axis] -= fromMm(wco[axis]);
        }
    }
}
#endif

void send_line(const char* s, int timeout) {
    fnc_send_line(s, timeout);
    dbg_println(s);
}
static void vsend_linef(const char* fmt, va_list va) {
    static char buf[128];
    vsnprintf(buf, 128, fmt, va);
    send_line(buf);
}
void send_linef(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vsend_linef(fmt, args);
    va_end(args);
}

char axisNumToChar(int axis) {
    return "XYZABC"[axis];
}

const char* axisNumToCStr(int axis) {
    static char ret[2] = { '\0', '\0' };
    ret[0]             = axisNumToChar(axis);
    return ret;
}

const char* intToCStr(int val) {
    static char buffer[20];
    sprintf(buffer, "%d", val);
    return buffer;
}

const char* mode_string() {
    return myModes.c_str();
}

state_t previous_state;
bool    awaiting_alarm = false;

extern "C" void show_state(const char* state_string) {
    previous_state = state;
    state_t new_state;
    mark_connected();  // any valid status report = connected
    if (decode_state_string(state_string, new_state) && state != new_state) {
        if (state == Disconnected) {
            fnc_realtime((realtime_cmd_t)0x0c);  // Ctrl-L - echo off
            send_line("$G");                     // Refresh GCode modes
            send_line("$G");                     // Refresh GCode modes
            send_line("$RI=200");
            init_file_list();
            
        }
        state = new_state;
        if (state == Alarm && lastAlarm == 0) {  // Unknown
            send_line("$A");                     // Get last alarm
            awaiting_alarm = true;
            return;
        }
        act_on_state_change();
    }
}

extern "C" void handle_other(char* line) {
    int alarmlen = strlen("Active alarm: ");
    if (strncmp(line, "Active alarm: ", alarmlen) == 0) {
        lastAlarm = atoi(line + alarmlen);
        if (awaiting_alarm) {
            dbg_printf("Got alarm %d\n", lastAlarm);
            awaiting_alarm = false;
            act_on_state_change();
        }
    }
}

extern "C" void show_error(int error) {
    errorExpire = milliseconds() + 1000;
    lastError   = error;
    tabui_onFluidNCError(error);  // declared in System.h
    current_scene->reDisplay();
}

extern "C" void show_timeout() {
    dbg_println("Timeout");
}
extern "C" void show_ok() {}

extern "C" void end_status_report() {
    markDirty();
}

extern "C" void show_alarm(int alarm) {
    lastAlarm = alarm;
    current_scene->reDisplay();
}

extern "C" void show_gcode_modes(struct gcode_modes* modes) {
    inInches = strcmp(modes->units, "In") == 0 || strcmp(modes->units, "G20") == 0;

    myModes = modes->wcs;
    myModes += " ";
    myModes += modes->units;
    myModes += " ";
    myModes += modes->distance;
    myModes += " ";
    myModes += modes->spindle;
    if (strcmp(modes->mist, "On") == 0) {
        myModes += " Mist";
    }
    if (strcmp(modes->flood, "On") == 0) {
        myModes += " Flood";
    }

    mySelectedTool = modes->tool;
    markDirty();
}

// (connection variables moved to top of file)

// Called by show_state() when any valid <State|...> is parsed — connection proven
void mark_connected() {
    _lastStatusMs = (uint32_t)millis();
    _missedPings  = 0;
    _wasConnected = true;
}

// Expose for diagnostics
uint32_t dbg_last_status_ms()  { return _lastStatusMs; }
int      dbg_missed_pings()    { return _missedPings; }

void request_status_report() {
    fnc_realtime(StatusReport);
    _nextPingMs = (uint32_t)millis() + PING_INTERVAL_MS;
}

bool fnc_is_connected() {
    extern volatile uint32_t fnc_rx_count;
    uint32_t now = (uint32_t)millis();

    // Send ping if due
    if ((now - _nextPingMs) < 0x80000000UL) {
        request_status_report();
        _missedPings++;
    }

    // Not yet connected (startup)
    if (!_wasConnected) {
        return false;
    }

    // Declare disconnect if too many missed pings or too long since last status
    uint32_t silenceMs = now - _lastStatusMs;
    if (_missedPings >= DISCONNECT_PINGS || silenceMs > DISCONNECT_TIMEOUT) {
        // Log reason to terminal so we know exactly what triggered it
        static uint32_t _lastDisconnLog = 0;
        if (now - _lastDisconnLog > 2000) {
            _lastDisconnLog = now;
            char msg[80];
            snprintf(msg, sizeof(msg),
                "[DISC] missed=%d silence=%lums rxCount=%lu",
                _missedPings, (unsigned long)silenceMs,
                (unsigned long)fnc_rx_count);
            fnc_term_inject(msg);
        }
        return false;
    }

    return true;
}

// Legacy — called by uart_reader_task, kept for compatibility but not used for timing
void update_rx_time() {}
