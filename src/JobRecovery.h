#pragma once
#include <string>
#include <vector>
#include "FluidNCModel.h"

// ── Job checkpoint saved to LittleFS at /job_checkpoint.bin ──────────────────
struct JobCheckpoint {
    char     magic[4];          // "JCP\0" — validates structure
    char     jobPath[128];      // full SD path e.g. /sd/job.nc
    uint32_t lastLine;          // last confirmed executed G-code line
    int32_t  axisX, axisY, axisZ; // machine position × 10000 (fixed point mm)
    int32_t  wcoX,  wcoY,  wcoZ;  // work coordinate offset × 10000
    char     wcs[8];            // active WCS e.g. "G54"
    char     units[4];          // "G21" or "G20"
    char     distMode[4];       // "G90" or "G91"
    uint32_t feedRate;          // mm/min
    uint32_t spindleSpeed;      // RPM
    uint32_t tool;              // tool number
    char     coolant[8];        // "M7","M8","M9"
    bool     dirty;             // true = interrupted, false = clean finish
    uint32_t savedAt;           // millis() when saved
};

// ── Recovery UI state ─────────────────────────────────────────────────────────
enum class RecoveryPhase {
    None,            // no recovery in progress
    Prompt,          // "Interrupted job found. Resume?"
    AskToolBreak,    // "Was a tool break or E-stop involved?"
    ShowLines,       // show last 10 G-code lines, ask user to confirm line
    ManualJog,       // prompt user to manually jog clear
    ToolChange,      // prompt tool change + probe
    Confirm,         // final confirmation before motion
    Executing,       // preamble executing
};

// ── Public API ────────────────────────────────────────────────────────────────
void     jobrecov_init();                                // call on boot
void     jobrecov_jobStarted(const char* path);          // called when job starts
void     jobrecov_lineExecuted(uint32_t line);           // call ~every 10-50 lines
void     jobrecov_jobComplete();                         // clean finish
void     jobrecov_saveNow();                             // force save

bool     jobrecov_hasDirty();                            // dirty checkpoint exists?
const JobCheckpoint& jobrecov_getCheckpoint();

// Resume flow entry points
void     jobrecov_startResume(bool toolBreak);           // begin resume sequence
void     jobrecov_cancelResume();
RecoveryPhase jobrecov_getPhase();
void     jobrecov_advance(int userChoice);               // step through wizard

// Draw the recovery overlay (called from TabScene reDisplay when phase != None)
void     jobrecov_draw(void* canvasPtr, int W, int H, int TOP, int NAV_Y);
// Touch handler — returns true if touch consumed
bool     jobrecov_onTouch(int x, int y);
void     jobrecov_showPrompt();
bool     jobrecov_wantsProbe();
void     jobrecov_clearProbe();
void     jobrecov_setG4142Warning(bool w);
void     jobrecov_scroll(int delta);
