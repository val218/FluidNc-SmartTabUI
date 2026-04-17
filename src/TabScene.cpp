// TabScene.cpp  — 5-tab landscape UI for FluidDial JC2432W328C (320x240)
// Copyright (c) 2024 FluidDial contributors. GPLv3 licence.

#include "Scene.h"
#include "SimMode.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/pcnt.h"
#include "FileParser.h"
#include "System.h"
extern volatile uint32_t fnc_tx_count;
extern override_percent_t mySro;
extern override_percent_t myRro;
extern volatile uint32_t fnc_rx_count;
extern "C" int fnc_term_count();
extern "C" const char* fnc_term_line(int idx);
extern "C" void fnc_term_inject(const char* line);
extern volatile int fnc_term_gen;
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>

// ─────────────────────────────────────────────────────────────────────────────
// Palette — RGB565 literals
// ─────────────────────────────────────────────────────────────────────────────
static int COL_BG      = 0x0862;
static int COL_PANEL   = 0x0883;
static int COL_PANEL2  = 0x10A3;
static int COL_PANEL3  = 0x0863;
static int COL_BORDER  = 0x1906;
static int COL_BORDER2 = 0x2147;
static int COL_DIM     = 0x31C9;
static int COL_DIM2    = 0x52F0;
static int COL_WHITE   = 0xDF1D;
static int COL_WHITE2  = 0x9D17;
static int COL_AX_X   = 0x06BF;
static int COL_AX_Y   = 0x0771;
static int COL_AX_Z   = 0xFEE8;
static int COL_AX_A   = 0xDB3F;

// ─────────────────────────────────────────────────────────────────────────────
// Layout
// ─────────────────────────────────────────────────────────────────────────────
static const int W       = 320;
static const int H       = 240;
static const int TOP     = 20;
static const int BOT     = 34;
static const int NAV_Y   = H - BOT;
static const int DROW    = 110;
static const int FEED_H  = 46;
static const int TAB_W   = 22;
static const int VIZ_X   = DROW;
static const int VIZ_Y   = TOP;
static const int VIZ_W   = W - DROW;        // full width to right edge
static const int VIZ_H   = NAV_Y - FEED_H - VIZ_Y;
static const int TAB_X   = W;               // tabs gone — pushed off screen
static const int DRO_ROW = (NAV_Y - TOP - FEED_H) / 4;
static const int N_TABS  = 5;
static const int CMD_H   = 34;
static const int OUT_LH  = 16;

// ─────────────────────────────────────────────────────────────────────────────
// Axis colour table
// ─────────────────────────────────────────────────────────────────────────────
struct AxisStyle { const char* letter; int col; };
static const AxisStyle AX_STYLES[4] = {
    { "X", COL_AX_X },
    { "Y", COL_AX_Y },
    { "Z", COL_AX_Z },
    { "A", COL_AX_A },
};

// ─────────────────────────────────────────────────────────────────────────────
// Tab names
// ─────────────────────────────────────────────────────────────────────────────
static const char* TAB_LABELS[N_TABS] = { "DRO", "Home", "Files", "Term", "Macros" };

// ─────────────────────────────────────────────────────────────────────────────
// Terminal
// ─────────────────────────────────────────────────────────────────────────────
struct TermLine { std::string txt; int col; };
static std::vector<TermLine> termLines;
static int termScroll   = 0;  // 0 = pinned to bottom, >0 = scrolled up by N lines
static int macroScroll  = 0;  // first visible macro index

// ── MPG handwheel state ───────────────────────────────────────────────────────
// Step size switch: gpio16=1mm, gpio4=10mm, neither=0.1mm
// Axis switch:      gpio17=X, gpio35=Y, gpio21=Z, gpio22=A, neither=off
// E-stop:           gpio8 active-low
static volatile int   mpgAxis     = -1;   // -1=off, 0=X,1=Y,2=Z,3=A
static volatile int   mpgStepIdx  = 0;    // 0=0.1, 1=1.0, 2=10.0
static float mpgSteps[]  = { 0.01f, 0.10f, 1.00f };
static const char* mpgStepLabels[] = { "x1", "x10", "x100" };
static bool  mpgEstopLast = true;   // debounce
static volatile int   mpgLastDir   = 0;      // last jog direction: +1 / -1 / 0
static volatile uint32_t mpgDirTime = 0;     // millis() when last jog fired (for fade-out)
static volatile bool  mpgEnable    = false;  // P6 enable button state
static int  _enableMode  = 0;  // EnableMode: 0=EnableGate 1=TouchOnly 2=JogOnly 3=MacroBtn 4=Disabled
static int  _enableMacro = 0;  // macro index for MacroBtn mode
static bool _p6Prev      = false;  // previous P6 state for edge detection
static volatile bool _p6MacroFire   = false;
static volatile bool mpgJogAllowed  = false;  // jog permitted (enable gating)
static int  _barSel      = 0;  // 0=none 1=feed selected 2=spindle selected
static int  _currentTab  = 0;
static int  _workX       = 1250;  // machine work area short axis (mm)
static int  _workY       = 2500;  // machine work area long axis (mm)
static int  _homeCorner  = 0;     // 0=BL 1=BR 2=TL 3=TR
static volatile bool mpgEstopActive = false;  // e-stop currently pressed
volatile bool _forceAlarm = false;  // set by mpgTask when e-stop pressed
static volatile bool _mpgChanged   = false; // set by Core0 readMpgSwitches, consumed on Core1

static int  _droAxesMode = 0;  // 0=XYZ 1=XYZA 2=XY 3=XYYZ (dual Y)

// Called by Settings to apply theme colours
void tabui_setTheme(int t) {
    switch (t) {
        case 1:  // Neutral — medium grey, light text
            COL_BG=0x31C8; COL_PANEL=0x4229; COL_PANEL2=0x4A8B; COL_PANEL3=0x39E9;
            COL_BORDER=0x5B2E; COL_BORDER2=0x73D1; COL_DIM=0x94D5; COL_DIM2=0xB5D9;
            COL_WHITE=0xE75E; COL_WHITE2=0xC65A;
            COL_AX_X=0x06BF; COL_AX_Y=0x0771; COL_AX_Z=0xFEE8; COL_AX_A=0xDB3F; break;
        case 2:  // Light — white/grey backgrounds, near-black text
            COL_BG=0xF79E; COL_PANEL=0xE73D; COL_PANEL2=0xD6BB; COL_PANEL3=0xEF7E;
            COL_BORDER=0xA556; COL_BORDER2=0x6BD1; COL_DIM=0x3A2A; COL_DIM2=0x1926;
            COL_WHITE=0x0862; COL_WHITE2=0x2147;
            COL_AX_X=0x0334; COL_AX_Y=0x0405; COL_AX_Z=0xA320; COL_AX_A=0x7814; break;
        default: // Dark — original deep blue-grey
            COL_BG=0x0862; COL_PANEL=0x0883; COL_PANEL2=0x10A3; COL_PANEL3=0x0863;
            COL_BORDER=0x1906; COL_BORDER2=0x2147; COL_DIM=0x31C9; COL_DIM2=0x52F0;
            COL_WHITE=0xDF1D; COL_WHITE2=0x9D17;
            COL_AX_X=0x06BF; COL_AX_Y=0x0771; COL_AX_Z=0xFEE8; COL_AX_A=0xDB3F; break;
    }
}
void tabui_setAxes(int a)         { _droAxesMode  = a; }
void tabui_setEnableMode(int m, int macro) { _enableMode = m; _enableMacro = macro; }
void tabui_setWorkArea(int wx, int wy, int hc) { _workX=wx; _workY=wy; _homeCorner=hc; }
int  tabui_getAxes()      { return _droAxesMode; }
bool mpgEnableHeld()      { return (bool)mpgEnable; }
void mpgSetEstop(bool v)  { mpgEstopActive = v; }
extern volatile bool _forceAlarm;
bool mpgConsumeChanged()  { if (!_mpgChanged) return false; _mpgChanged = false; return true; }
void mpgSignalChanged()   { _mpgChanged = true; }
// Touch gating: only applies to DRO tab in EnableGate/TouchOnly modes
// All other tabs always accept touch regardless of enable button
bool tabui_touchGated()   {
    if (_currentTab != 0) return false;  // never gate non-DRO tabs
    return (_enableMode==0||_enableMode==1) && !mpgEnable;
}
void setSimMode(bool sim) { if (sim) simMode_enable(); }
bool getSimMode()         { return simMode_active(); }

// Read PCF8574 I2C GPIO expander (address 0x20)
// Returns 0xFF on error (all pins HIGH = nothing active = safe default)
static bool    pcf8574Present = false;  // detected on first successful read
static uint8_t readPCF8574() {
    uint8_t val = 0xFF;
    esp_err_t err = i2c_master_read_from_device(I2C_NUM_1, 0x20, &val, 1, pdMS_TO_TICKS(20));
    if (err == ESP_OK) pcf8574Present = true;
    return (err == ESP_OK) ? val : 0xFF;
}

void readMpgSwitches() {
    static uint32_t lastRead = 0;
    uint32_t now = millis();
    if (now - lastRead < 100) return;
    lastRead = now;

    // If PCF8574 not connected, skip — all axes/steps remain at default (off/0.1mm)
    // Retry detection every 100ms until found
    if (!pcf8574Present) {
        readPCF8574();  // attempt detection, updates pcf8574Present if found
        return;
    }

    int prevAxis = (int)mpgAxis, prevStep = (int)mpgStepIdx;

    // Read all 8 switch inputs in one I2C transaction
    // PCF8574 pin map (active-LOW — switch closes pin to GND):
    //   P0 = Axis X
    //   P1 = Axis Y
    //   P2 = Axis Z
    //   P3 = Axis A
    //   P4 = Step 1mm
    //   P5 = Step 10mm
    //   P6 = Enable button (active-LOW — must be held to allow jogging)
    //   P7 = unused
    uint8_t pins = readPCF8574();

    bool axX    = !(pins & 0x01);  // P0
    bool axY    = !(pins & 0x02);  // P1
    bool axZ    = !(pins & 0x04);  // P2
    bool axA    = !(pins & 0x08);  // P3
    bool st1    = !(pins & 0x10);  // P4
    bool st10   = !(pins & 0x20);  // P5
    bool enable = !(pins & 0x40);  // P6 — enable button, must be held to jog
    mpgEnable = enable;

    // Step — always read regardless of enable/axis state
    if      (st10) mpgStepIdx = 2;
    else if (st1)  mpgStepIdx = 1;
    else           mpgStepIdx = 0;

    // Axis — always read for display; jogging gated separately in onEncoder
    if      (axA) mpgAxis = 3;
    else if (axZ) mpgAxis = 2;
    else if (axY) mpgAxis = 1;
    else if (axX) mpgAxis = 0;
    else          mpgAxis = -1;

    // Jog is allowed based on enableMode + enable button
    bool jogAllowed = false;
    switch (_enableMode) {
        case 0: jogAllowed = enable; break;   // EnableGate: must hold enable
        case 1: jogAllowed = true;   break;   // TouchOnly:  jog always free
        case 2: jogAllowed = enable; break;   // JogOnly:    must hold enable
        case 3: jogAllowed = true;   break;   // MacroBtn:   jog always free
        case 4: jogAllowed = true;   break;   // Disabled:   jog always free
    }
    // Store jog permission so onEncoder can check it
    mpgJogAllowed = jogAllowed;

    // MacroBtn mode: P6 rising edge — set a flag, fired in reDisplay context
    if (_enableMode == 3 && enable && !_p6Prev) {
        _p6MacroFire = true;
    }
    _p6Prev = enable;

    // Clear bar selection when enable released
    if (!enable) _barSel = 0;

    // Clear PCNT backlog when axis goes off → active
    if (prevAxis < 0 && mpgAxis >= 0) {
        pcnt_counter_clear(PCNT_UNIT_0);
    }

    if (mpgAxis != prevAxis || mpgStepIdx != prevStep) {
        _mpgChanged = true;  // picked up by dispatch_events on Core 1
    }
}
static const char* QUICK_CMDS[] = { "$H", "$?", "!", "~", "$X" };
static const int   N_QUICK_CMDS = 5;

// ─────────────────────────────────────────────────────────────────────────────
// Probe options
// ─────────────────────────────────────────────────────────────────────────────
struct ProbeOpt { const char* label; const char* sub; const char* cmd; int col; };
static const ProbeOpt PROBE_OPTS[] = {
    { "Tool Length",  "Z-probe with current tool",          "G38.2 Z-50 F100",          YELLOW     },
    { "Z Surface",    "Probe top of workpiece",             "G38.2 Z-50 F100",          CYAN       },
    { "XY Center",    "Find center of hole / boss",         "G38.2 X-50 F100",          GREEN      },
    { "X Edge",       "Find left or right X edge",          "G38.2 X-50 F100",          COL_AX_X   },
    { "Y Edge",       "Find front or back Y edge",          "G38.2 Y-50 F100",          COL_AX_Y   },
    { "Corner",       "Find XY corner of workpiece",        "G38.2 X-25 F100",          ORANGE     },
};
static const int N_PROBE_OPTS = 6;

// ─────────────────────────────────────────────────────────────────────────────
// Macros
// ─────────────────────────────────────────────────────────────────────────────
struct MacroBtn { std::string name; int col; std::string cmd; };
// Macro list — add as many as needed, scrollable at runtime
static std::vector<MacroBtn> MACROS = {
    { "Tool Change", ORANGE,    "M6"                   },
    { "Park",        CYAN,      "G53 G0 Z0"            },
    { "Coolant On",  BLUE,      "M8"                   },
    { "Coolant Off", BLUE,      "M9"                   },
    { "Probe Z",     YELLOW,    "G38.2 Z-50 F100"      },
    { "Zero X",      COL_AX_X,  "G10 L20 P1 X0"        },
    { "Zero Y",      COL_AX_Y,  "G10 L20 P1 Y0"        },
    { "Zero Z",      COL_AX_Z,  "G10 L20 P1 Z0"        },
    { "Zero All",    ORANGE,    "G10 L20 P1 X0 Y0 Z0"  },
    { "Spindle CW",  GREEN,     "M3 S10000"            },
    { "Spindle CCW", GREEN,     "M4 S10000"            },
    { "Spindle Off", COL_DIM2,  "M5"                   },
    { "Feed Hold",   RED,       "!"                    },
    { "Cycle Start", GREEN,     "~"                    },
    { "Unlock",      ORANGE,    "$X"                   },
    { "Home All",    CYAN,      "$H"                   },
};

// ─────────────────────────────────────────────────────────────────────────────
// File list
// ─────────────────────────────────────────────────────────────────────────────
struct FileEntry { std::string name; bool isDir; int size; };
static std::vector<FileEntry> fileList;
static int  fileScroll   = 0;
static int  fileSelected    = -1;
static bool filePreviewMode  = false;      // showing gcode preview panel
static bool simJobRunning    = false;      // sim: job is running (show in DRO viz)
static std::string simJobName;             // name of file being sim-run
static std::vector<std::pair<float,float>> simPath;  // XY path parsed from gcode for viz
static int  simPathIdx       = 0;          // current position along path (animation)
static std::vector<std::string> previewLines; // loaded gcode lines
static std::vector<std::pair<float,float>> vizPath;   // path for DRO viz overlay
static std::string vizJobName;              // filename shown on DRO viz
static int  vizPathExecuted  = 0;           // segments drawn as "executed" (green)
static int  previewScroll   = 0;           // first visible preview line
static int  previewFirstLine = 0;           // line offset in file
static std::string filePath = "/sd";

// ─────────────────────────────────────────────────────────────────────────────
// Drawing helpers — thin wrappers so draw code stays clean
// ─────────────────────────────────────────────────────────────────────────────
static void fillR(int x, int y, int w, int h, int r, int col) {
    canvas.fillRoundRect(x, y, w, h, r, col);
}
static void strokeR(int x, int y, int w, int h, int r, int col) {
    canvas.drawRoundRect(x, y, w, h, r, col);
}
// Draw a tinted-fill + border rounded rect. alpha 0..255.
static void tintStrokeR(int x, int y, int w, int h, int r, int fillCol, int strokeCol, int alpha = 30) {
    // Extract RGB565 components, scale by alpha/255
    int rr = ((fillCol >> 11) & 0x1F) * alpha / 255;
    int gg = ((fillCol >> 5)  & 0x3F) * alpha / 255;
    int bb = ((fillCol >> 0)  & 0x1F) * alpha / 255;
    int tint = (rr << 11) | (gg << 5) | bb;
    fillR(x, y, w, h, r, tint);
    strokeR(x, y, w, h, r, strokeCol);
}
static void hline(int x, int y, int w, int col) {
    canvas.drawFastHLine(x, y, w, col);
}
static void vline(int x, int y, int h, int col) {
    canvas.drawFastVLine(x, y, h, col);
}
static void txt(const char* s, int x, int y, int col, fontnum_t f = TINY, int datum = middle_center) {
    text(s, x, y, col, f, datum);
}
static void txts(const std::string& s, int x, int y, int col, fontnum_t f = TINY, int datum = middle_center) {
    text(s, x, y, col, f, datum);
}
// Font2 (10x12 bitmap, not bold) — used for all screen content text
static void f2(const char* s, int x, int y, int col, int datum = middle_center) {
    canvas.setFont(&fonts::Font2);
    canvas.setTextDatum(datum);
    canvas.setTextColor(col);
    canvas.drawString(s, x, y);
}
static void f2s(const std::string& s, int x, int y, int col, int datum = middle_center) {
    f2(s.c_str(), x, y, col, datum);
}

// ─────────────────────────────────────────────────────────────────────────────
// TabScene
// ─────────────────────────────────────────────────────────────────────────────
class TabScene : public Scene {
private:
    int  _tab      = 0;
    int  _vizView  = 0;
    bool _probeOpen = false;
    bool _alarmOpen = false;

    struct Rect { int x, y, w, h; };
    static bool hit(const Rect& r, int px, int py) {
        return px >= r.x && px < r.x + r.w && py >= r.y && py < r.y + r.h;
    }

    Rect _navTabs[N_TABS];
    Rect _vizTabs[3];
    Rect _feedMinus, _feedPlus;
    Rect _spndMinus, _spndPlus, _spndPill, _spdPill;
    Rect _homeAllBtn, _probeBtnR;
    Rect _axisHomeBtns[3];
    Rect _zeroWcsBtns[4];
    Rect _unlockBtn;
    Rect _probeClose;
    Rect _probeRows[N_PROBE_OPTS];
    Rect _cmdBtns[N_QUICK_CMDS];
    Rect _macroBtns[8];  // slots for visible macros (max visible at once)

    // ── state colour ─────────────────────────────────────────────────────────
    int stateCol() {
        switch (state) {
            case Cycle: return CYAN;
            case Hold:  return YELLOW;
            case Alarm: return RED;
            case Jog:   return 0xF81F;
            case Homing:return CYAN;
            default:    return GREEN;
        }
    }

    // ── header ───────────────────────────────────────────────────────────────
    // Use the compact 6x8 bitmap font for the header - fits in 20px and not bold
    void hdrTxt(const char* s, int x, int y, int col, int datum = middle_center) {
        canvas.setFont(&fonts::Font0);
        canvas.setTextDatum(datum);
        canvas.setTextColor(col);
        canvas.drawString(s, x, y);
    }

    void drawHeader() {
        canvas.fillRect(0, 0, W, TOP, COL_PANEL);
        hline(0, TOP - 1, W, COL_BORDER);

        int sc = stateCol();
        // ── LEFT: machine state badge ────────────────────────────────────────
        int tint = (((sc>>11)&0x1F)*20/255)<<11|(((sc>>5)&0x3F)*20/255)<<5|(((sc>>0)&0x1F)*20/255);
        fillR(2, 2, 42, 16, 3, tint);
        strokeR(2, 2, 42, 16, 3, sc);
        hdrTxt(my_state_string, 23, 10, sc);

        // STP badge directly right of state badge (only when e-stop active)
        if (mpgEstopActive) {
            fillR(46, 2, 30, 16, 3, 0x4000);
            strokeR(46, 2, 30, 16, 3, RED);
            hdrTxt("STP", 61, 10, RED);
        }

        // ── RIGHT: EN  ◄►  TX  RX  — fixed slots ────────────────────────────
        // Slots from right edge, each ~18px:
        // RX  = W-2  (right-aligned)
        // TX  = W-20
        // ◄►  = W-38 (direction arrows slot, 16px wide)
        // EN  = W-56
        {
            static uint32_t lastTx=0,lastRx=0,txFlash=0,rxFlash=0;
            uint32_t now2 = millis();
            if (fnc_tx_count!=lastTx){lastTx=fnc_tx_count;txFlash=now2;}
            if (fnc_rx_count!=lastRx){lastRx=fnc_rx_count;rxFlash=now2;}
            bool txOn=(now2-txFlash)<80, rxOn=(now2-rxFlash)<80;

            // EN
            hdrTxt("EN", W-56, 10, mpgEnable?GREEN:COL_DIM, middle_right);

            // ◄► direction arrows in their own fixed slot
            if (mpgLastDir != 0 && (millis() - mpgDirTime) < 300) {
                static const int axCols[] = { COL_AX_X, COL_AX_Y, COL_AX_Z, COL_AX_A };
                int ac = (mpgAxis >= 0) ? axCols[mpgAxis] : COL_WHITE2;
                auto drawTri = [&](int cx2, bool pr, int col) {
                    for (int r=0;r<7;r++){
                        int h=(r<4)?r:(6-r);
                        int lx=pr?(cx2-3):(cx2+3-h), rx2=pr?(cx2-3+h):(cx2+3);
                        if(rx2>lx) canvas.drawFastHLine(lx,3+r,rx2-lx+1,col);
                    }
                };
                canvas.fillRect(W-38, 2, 16, 16, COL_PANEL);
                drawTri(W-31, false, mpgLastDir<0 ? ac : COL_DIM);
                drawTri(W-24, true,  mpgLastDir>0 ? ac : COL_DIM);
            }

            // TX  RX
            hdrTxt("TX", W-20, 10, txOn?ORANGE:COL_DIM, middle_right);
            hdrTxt("RX", W-2,  10, rxOn?GREEN:COL_DIM,  middle_right);
        }

        // ── CENTRE: tab name or JOG label ────────────────────────────────────
        if (mpgEnable && mpgAxis >= 0) {
            static const int axCols[] = { COL_AX_X, COL_AX_Y, COL_AX_Z, COL_AX_A };
            static const char axNames[] = { 'X', 'Y', 'Z', 'A' };
            int ac = axCols[mpgAxis];
            char jl[14]; snprintf(jl, sizeof(jl), "JOG %c %smm", axNames[mpgAxis], mpgStepLabels[(int)mpgStepIdx]);
            canvas.fillRect(W/2-44, 2, 88, 16, 0x0000);
            strokeR(W/2-44, 2, 88, 16, 3, ac);
            hdrTxt(jl, W/2, 10, ac);
        } else if (simMode_active()) {
            hdrTxt("SIM", W/2-20, 10, ORANGE);
            hdrTxt(TAB_LABELS[_tab], W/2+12, 10, COL_WHITE2);
        } else {
            hdrTxt(TAB_LABELS[_tab], W/2, 10, COL_WHITE2);
        }
    }

    // ── nav bar ──────────────────────────────────────────────────────────────
    void navTxt(const char* s, int x, int y, int col) {
        canvas.setFont(&fonts::Font2);   // 10x12 bitmap — compact, not bold
        canvas.setTextDatum(middle_center);
        canvas.setTextColor(col);
        canvas.drawString(s, x, y);
    }

    void drawNav() {
        canvas.fillRect(0, NAV_Y, W, BOT, 0x0841);
        hline(0, NAV_Y, W, COL_BORDER);
        int tw = W / N_TABS;
        for (int i = 0; i < N_TABS; i++) {
            int x = i * tw;
            _navTabs[i] = { x, NAV_Y, tw, BOT };
            if (i == _tab) {
                canvas.fillRect(x, NAV_Y, tw, BOT, 0x0019);
                canvas.fillRect(x, NAV_Y, tw, 3, BLUE);
            }
            if (i > 0) vline(x, NAV_Y + 5, BOT - 10, COL_BORDER);
            int col = (i == _tab) ? COL_WHITE : COL_WHITE2;
            navTxt(TAB_LABELS[i], x + tw / 2, NAV_Y + BOT / 2, col);
        }
    }

    // ── DRO screen ───────────────────────────────────────────────────────────
    void drawDROScreen() {
        // DRO rows
        // DRO axis count from settings: 0=XYZ(3) 1=XYZA(4) 2=XY(2) 3=XYYZ(4)
        static const int axisCount[] = { 3, 4, 2, 4 };
        int nAxes = axisCount[_droAxesMode];
        int droRow = (NAV_Y - TOP - FEED_H) / nAxes;  // dynamic row height
        for (int i = 0; i < nAxes; i++) {
            int ry    = TOP + i * droRow;
            int axcol = AX_STYLES[i].col;
            bool isMpg = (mpgAxis == i);

            // Row background — bright when this axis is MPG active
            int rowbg = isMpg ? COL_PANEL2 : ((i % 2 == 0) ? COL_PANEL : COL_PANEL3);
            canvas.fillRect(0, ry, DROW, droRow, rowbg);
            canvas.fillRect(0, ry, isMpg ? 5 : 3, droRow, axcol);
            hline(0, ry + droRow - 1, DROW, COL_BORDER);

            // Axis letter centred in row
            const char* axLetter = (_droAxesMode == 3 && i == 3) ? "Y2" : AX_STYLES[i].letter;
            int midY = ry + droRow / 2;
            txt(axLetter, 8, midY, isMpg ? COL_WHITE : axcol, TINY, middle_left);

            // Position value right-aligned
            int ndig = inInches ? 4 : 3;
            const char* posStr = simMode_active()
                ? pos_to_cstr((pos_t)simMode_getPos(i), 3)
                : pos_to_cstr(myAxes[i], ndig);
            txt(posStr, DROW - 5, midY,
                simMode_active() ? ORANGE : (isMpg ? COL_WHITE : COL_WHITE2),
                TINY, middle_right);

            // Step pill badge at bottom of active row
            if (isMpg) {
                const char* sl = mpgStepLabels[(int)mpgStepIdx];
                canvas.setFont(&fonts::Font0);
                int sw = canvas.textWidth(sl);
                int bx = 6, by2 = ry + droRow - 11;
                canvas.fillRoundRect(bx, by2, sw + 8, 9, 2, axcol);
                canvas.setTextDatum(middle_center);
                canvas.setTextColor(0x0000);
                canvas.drawString(sl, bx + (sw + 8) / 2, by2 + 5);
            }
        }
        vline(DROW - 1, TOP, NAV_Y - TOP - FEED_H, COL_BORDER);

        // Timed path animation (80ms/step) driven here during drawing
        if (simJobRunning && !simPath.empty()) {
            static uint32_t _lastStep = 0;
            uint32_t _now = millis();
            if (_now - _lastStep >= 80) {
                _lastStep = _now;
                if (simPathIdx < (int)simPath.size()-1) {
                    simPathIdx++;
                    simMode_setPos(0, simPath[simPathIdx].first);
                    simMode_setPos(1, simPath[simPathIdx].second);
                    reDisplay();
                } else {
                    simJobRunning = false;
                }
            }
        }

        // Visualizer area — simple blank until job running
        canvas.fillRect(VIZ_X, VIZ_Y, VIZ_W, VIZ_H, COL_PANEL2);

        // Work area map — landscape orientation: workY (long axis) runs horizontally
        // Screen X = machine Y axis (0..workY left→right)
        // Screen Y = machine X axis (0..workX top→bottom, inverted for Y-up)
        {
            int pad3=2, mapW=VIZ_W-2*pad3, mapH=VIZ_H-2*pad3;
            int mapX=VIZ_X+pad3, mapY=VIZ_Y+pad3;

            // Scale: workY along screen-X, workX along screen-Y (landscape)
            // Use the SAME scale for both axes so distances are proportional
            float scH=(float)mapW/_workY;  // px/mm horizontal (for Y machine axis)
            float scV=(float)mapH/_workX;  // px/mm vertical   (for X machine axis)
            float sc=std::min(scH,scV);    // limiting scale preserves proportions

            // Drawn area size in pixels
            int drawnW=(int)(_workY*sc);  // machine Y → screen width
            int drawnH=(int)(_workX*sc);  // machine X → screen height

            // Centre the work area in the viz panel
            int offX=mapX+(mapW-drawnW)/2;
            int offY=mapY+(mapH-drawnH)/2;

            // Background and border
            canvas.fillRect(offX,offY,drawnW,drawnH,COL_PANEL3);
            canvas.drawRect(offX,offY,drawnW,drawnH,COL_BORDER2);

            // Grid lines (every 25%)
            for(int gi=1;gi<4;gi++){
                canvas.drawFastVLine(offX+drawnW*gi/4,offY,drawnH,COL_BORDER);
                canvas.drawFastHLine(offX,offY+drawnH*gi/4,drawnW,COL_BORDER);
            }

            // Home corner position (green dot)
            // homeCorner: 0=BL 1=BR 2=TL 3=TR
            // In landscape: BL = screen bottom-left (Y=0 left, X=0 bottom)
            int hx=(_homeCorner==1||_homeCorner==3)?offX+drawnW:offX;
            int hy=(_homeCorner==0||_homeCorner==1)?offY+drawnH:offY;
            canvas.fillCircle(hx,hy,2,GREEN);

            // XY reference arrows from home corner (thin, 20px)
            // X arrow: points in machine-X+ direction → screen upward (machine X = screen Y inverted)
            // Y arrow: points in machine-Y+ direction → screen rightward
            int arrowLen=18;
            // Determine arrow directions based on home corner
            int xDir = (_homeCorner==0||_homeCorner==1) ? -1 : 1;  // X+ → screen up(-1) or down(+1)
            int yDir = (_homeCorner==1||_homeCorner==3) ? -1 : 1;  // Y+ → screen left(-1) or right(+1)
            // Y axis arrow (horizontal) — green, points in machine Y+ direction
            int yax0 = (yDir>0) ? hx : hx+yDir*arrowLen;
            canvas.drawFastHLine(yax0, hy,   arrowLen, GREEN);
            canvas.drawFastHLine(yax0, hy+1, arrowLen, GREEN);  // 2px thick
            canvas.setFont(&fonts::Font0);
            canvas.setTextDatum(middle_center);
            canvas.setTextColor(GREEN);
            canvas.drawString("Y", hx+yDir*(arrowLen+5), hy);
            // X axis arrow (vertical) — red, points in machine X+ direction
            int xax0 = (xDir>0) ? hy : hy+xDir*arrowLen;
            canvas.drawFastVLine(hx,   xax0, arrowLen, RED);
            canvas.drawFastVLine(hx+1, xax0, arrowLen, RED);    // 2px thick
            canvas.setTextColor(RED);
            canvas.drawString("X", hx, hy+xDir*(arrowLen+5));

            // Real-time position dot
            // Machine coords → screen coords:
            //   screenX = offX + machY * sc  (Y goes left→right)
            //   screenY = offY + drawnH - machX * sc  (X inverted: 0=bottom)
            // Adjusted for home corner:
            // myAxes is e4 fixed-point (mm × 10000) when E4_POS_T defined
            // simMode positions are plain mm floats
            float machX = simMode_active() ? (float)simMode_getPos(0)
                                           : (float)myAxes[0] / 10000.0f;
            float machY = simMode_active() ? (float)simMode_getPos(1)
                                           : (float)myAxes[1] / 10000.0f;
            // Clamp to work area
            machX=std::max(0.0f,std::min(machX,(float)_workX));
            machY=std::max(0.0f,std::min(machY,(float)_workY));

            int dotX,dotY;
            // Map machine Y → screen X, machine X → screen Y (landscape)
            float sX = machY * sc;  // screen offset in X direction (from home Y edge)
            float sY = machX * sc;  // screen offset in Y direction (from home X edge)
            if(_homeCorner==0){      // BL: Y→right, X→up
                dotX=offX+(int)sX; dotY=offY+drawnH-(int)sY;
            } else if(_homeCorner==1){ // BR: Y→left, X→up
                dotX=offX+drawnW-(int)sX; dotY=offY+drawnH-(int)sY;
            } else if(_homeCorner==2){ // TL: Y→right, X→down
                dotX=offX+(int)sX; dotY=offY+(int)sY;
            } else {                  // TR: Y→left, X→down
                dotX=offX+drawnW-(int)sX; dotY=offY+(int)sY;
            }
            canvas.fillCircle(dotX,dotY,2,ORANGE);
            canvas.drawCircle(dotX,dotY,2,COL_WHITE);

            // Scale label bottom-right of work area
            canvas.setFont(&fonts::Font0);
            canvas.setTextColor(COL_DIM);
            canvas.setTextDatum(bottom_right);
            char wdim[24]; snprintf(wdim,sizeof(wdim),"%dx%dmm",_workY,_workX);
            canvas.drawString(wdim,offX+drawnW-1,offY+drawnH-1);
        }

        // Tool path overlay — uses same work-area coordinate system
        // vizPath is populated when a file is selected/run from Files tab
        if (!vizPath.empty()) {
            // Recompute the same offsets as the work area map above
            int pad3v=2, mapWv=VIZ_W-2*pad3v, mapHv=VIZ_H-2*pad3v;
            float scHv=(float)mapWv/_workY, scVv=(float)mapHv/_workX;
            float scv=std::min(scHv,scVv);
            int drawnWv=(int)(_workY*scv), drawnHv=(int)(_workX*scv);
            int offXv=(VIZ_X+pad3v)+(mapWv-drawnWv)/2;
            int offYv=(VIZ_Y+pad3v)+(mapHv-drawnHv)/2;

            // Map machine coords to screen using same landscape transform as work area dot
            auto pathToScreen = [&](float mx, float my, int& sx, int& sy) {
                float sXv = my * scv;
                float sYv = mx * scv;
                if (_homeCorner==0) { sx=offXv+(int)sXv; sy=offYv+drawnHv-(int)sYv; }
                else if (_homeCorner==1) { sx=offXv+drawnWv-(int)sXv; sy=offYv+drawnHv-(int)sYv; }
                else if (_homeCorner==2) { sx=offXv+(int)sXv; sy=offYv+(int)sYv; }
                else { sx=offXv+drawnWv-(int)sXv; sy=offYv+(int)sYv; }
            };

            // Update executed segment count from myPercent (real job) or simPathIdx (sim)
            if (simJobRunning) {
                vizPathExecuted = simPathIdx;
            } else if (myPercent > 0) {
                vizPathExecuted = (int)vizPath.size() * (int)myPercent / 100;
            }
            int executed = std::min(vizPathExecuted, (int)vizPath.size()-1);

            // Draw path: dim = not yet executed, cyan = executed
            for (int pi=1; pi<(int)vizPath.size(); pi++) {
                int x1,y1,x2,y2;
                pathToScreen(vizPath[pi-1].first, vizPath[pi-1].second, x1, y1);
                pathToScreen(vizPath[pi].first,   vizPath[pi].second,   x2, y2);
                int col = (pi <= executed) ? CYAN : COL_DIM;
                canvas.drawLine(x1,y1,x2,y2,col);
            }

            // Current position marker on path
            if (executed > 0 && executed < (int)vizPath.size()) {
                int dx,dy;
                pathToScreen(vizPath[executed].first, vizPath[executed].second, dx, dy);
                canvas.fillCircle(dx, dy, 2, GREEN);
            }

            // Filename — scrolling marquee at top of viz area (max 26 chars fit)
            if (!vizJobName.empty()) {
                static int _nameScroll = 0;
                static uint32_t _nameTime = 0;
                if (millis() - _nameTime > 300) { _nameTime = millis(); _nameScroll++; }
                std::string display_name = vizJobName;
                if ((int)display_name.size() > 26) {
                    int pos = _nameScroll % (int)(display_name.size() + 4);
                    std::string padded = display_name + "    " + display_name;
                    display_name = padded.substr(pos, 26);
                }
                canvas.setFont(&fonts::Font0);
                canvas.setTextDatum(middle_left);
                canvas.setTextColor(CYAN);
                canvas.drawString(display_name.c_str(), offXv+2, offYv+5);
            }

            // Progress bar — 5px tall, at bottom of work area
            int pbY = offYv + drawnHv - 6;
            int pbPct = vizPath.size()>1 ? executed*100/((int)vizPath.size()-1) : 0;
            if (myPercent > 0) pbPct = (int)myPercent;
            int pbW = drawnWv * pbPct / 100;
            canvas.fillRect(offXv, pbY, drawnWv, 5, COL_BORDER);
            if (pbW > 0) canvas.fillRect(offXv, pbY, pbW, 5, CYAN);
            // Percentage text
            char pctStr[8]; snprintf(pctStr, sizeof(pctStr), "%d%%", pbPct);
            canvas.setFont(&fonts::Font0);
            canvas.setTextDatum(middle_right);
            canvas.setTextColor(pbPct>0 ? CYAN : COL_DIM);
            canvas.drawString(pctStr, offXv+drawnWv-1, pbY-4);


        // File progress bar
        if (myPercent > 0) {
            int bw = (int)((VIZ_W - 10) * (int)myPercent / 100);
            canvas.fillRect(VIZ_X + 5, NAV_Y - FEED_H - 6, VIZ_W - 10, 3, COL_BORDER2);
            if (bw > 0) canvas.fillRect(VIZ_X + 5, NAV_Y - FEED_H - 6, bw, 3, CYAN);
            char pct[12];
            snprintf(pct, sizeof(pct), "%d%%", (int)myPercent);
            txt(pct, TAB_X - 3, VIZ_Y + 5, COL_DIM2, TINY, middle_right);
        }

        // Viz view tabs
        canvas.fillRect(TAB_X, VIZ_Y, TAB_W, VIZ_H, COL_PANEL);
        vline(TAB_X, VIZ_Y, VIZ_H, COL_BORDER);
        const char* vt[] = { "XY", "XZ", "3D" };
        int tabH = VIZ_H / 3;
        for (int i = 0; i < 3; i++) {
            int ty = VIZ_Y + i * tabH;
            _vizTabs[i] = { TAB_X, ty, TAB_W, tabH };
            if (i == _vizView) {
                canvas.fillRect(TAB_X, ty, TAB_W, tabH, 0x0019);
                canvas.fillRect(TAB_X, ty, 2, tabH, BLUE);
            }
            hline(TAB_X, ty + tabH - 1, TAB_W, COL_BORDER);
            {
                int tcol2 = (i == _vizView) ? COL_WHITE : COL_DIM2;
                int bg    = (i == _vizView) ? 0x0019 : COL_PANEL;
                LGFX_Sprite tmp(&canvas);
                tmp.setColorDepth(8);
                // Create sprite sized for the text, rotate 90 CCW when pushing
                tmp.createSprite(tabH - 2, TAB_W - 2);
                tmp.fillSprite(bg);
                tmp.setFont(&fonts::Font2);
                tmp.setTextColor(tcol2);
                tmp.setTextDatum(middle_center);
                tmp.drawString(vt[i], (tabH - 2) / 2, (TAB_W - 2) / 2);
                // 90 = CW = text reads bottom-to-top; 270 = CCW = text reads top-to-bottom
                // Use bg+1 as transparent key so bg fills the cell correctly
                tmp.pushRotateZoom(&canvas, TAB_X + TAB_W / 2, ty + tabH / 2,
                                   270.0f, 1.0f, 1.0f, bg + 1);
                tmp.deleteSprite();
            }
        }

        }  // end tool path if-block

        // ── Feed / Speed / Spindle bar — tap to select, MPG adjusts ──────────
        int fy  = NAV_Y - FEED_H;
        canvas.fillRect(0, fy, W, FEED_H, COL_PANEL2);
        hline(0, fy, W, COL_BORDER);

        auto barTxt = [&](const char* s, int x, int y2, int col, int datum = middle_center) {
            canvas.setFont(&fonts::Font2);
            canvas.setTextDatum(datum);
            canvas.setTextColor(col);
            canvas.drawString(s, x, y2);
        };

        // 3 equal pills: FEED | RAPID | SPND — each selectable for MPG control
        int pad2 = 4, gap3 = 3;
        int pillW3 = (W - 2*pad2 - 2*gap3) / 3;
        int fh = FEED_H - 8, fy2 = fy + 4;
        int px0 = pad2;
        int px1 = px0 + pillW3 + gap3;
        int px2 = px1 + pillW3 + gap3;

        int lbl_y = fy + FEED_H*32/100;
        int val_y = fy + FEED_H*70/100;

        auto drawPill3 = [&](int px, int pw, const char* label, const char* val, int col, bool sel) {
            if (sel) {
                canvas.fillRoundRect(px, fy2, pw, fh, 4, col);
                canvas.drawRoundRect(px, fy2, pw, fh, 4, COL_WHITE);
                barTxt(label, px + pw/2, lbl_y, COL_BG);
                barTxt(val,   px + pw/2, val_y, COL_BG);
            } else {
                tintStrokeR(px, fy2, pw, fh, 4, col, col, 18);
                barTxt(label, px + pw/2, lbl_y, col);
                barTxt(val,   px + pw/2, val_y, COL_WHITE);
            }
        };

        // FEED
        { int col=(myFro<80)?RED:(myFro>120)?ORANGE:CYAN;
          char v[10]; snprintf(v,sizeof(v),"%d%%",(int)myFro);
          _feedMinus={px0,fy,pillW3,FEED_H};
          drawPill3(px0,pillW3,"FEED",v,col,_barSel==1); }

        vline(px1-1, fy+4, fh, COL_BORDER2);

        // RAPID
        { int col=(myRro<80)?RED:(myRro>120)?ORANGE:YELLOW;
          char v[10]; snprintf(v,sizeof(v),"%d%%",(int)myRro);
          _spdPill={px1,fy,pillW3,FEED_H};
          drawPill3(px1,pillW3,"RAPID",v,col,_barSel==2); }

        vline(px2-1, fy+4, fh, COL_BORDER2);

        // SPND
        { int col=(mySro<80)?RED:(mySro>120)?ORANGE:0xF81F;
          char v[10]; snprintf(v,sizeof(v),"%d%%",(int)mySro);
          _spndPill={px2,fy,pillW3,FEED_H};
          drawPill3(px2,pillW3,"SPND",v,col,_barSel==3); }
    }  // end drawDROScreen

    // ── Homing screen ────────────────────────────────────────────────────────
    void drawHomingScreen() {
        int pad = 10, gap = 5, bw = W - 2 * pad;
        int homeH = 28, axH = 24, esH = 22, zeroH = 26;
        int divH = 1, labH = 12, secGap = 5;
        int total = homeH + (secGap + divH + labH + axH)
                          + (secGap + divH + labH + esH)
                          + (secGap + divH + labH + zeroH);
        int y = TOP + std::max(4, (NAV_Y - TOP - total) / 2);

        // ── Home All + Probe ─────────────────────────────────────────────
        int btnGap = 6;
        int homeW  = bw * 56 / 100;
        int probW  = bw - homeW - btnGap;
        _homeAllBtn = { pad, y, homeW, homeH };
        _probeBtnR  = { pad + homeW + btnGap, y, probW, homeH };

        // Home All: solid white border, dark fill — primary action
        canvas.fillRoundRect(pad, y, homeW, homeH, 4, COL_PANEL2);
        canvas.drawRoundRect(pad, y, homeW, homeH, 4, COL_WHITE);
        f2("Home All  ($H)", pad + homeW / 2, y + homeH / 2, COL_WHITE);

        // Probe: amber accent border only
        canvas.fillRoundRect(pad + homeW + btnGap, y, probW, homeH, 4, COL_PANEL2);
        canvas.drawRoundRect(pad + homeW + btnGap, y, probW, homeH, 4, ORANGE);
        f2("Probe", pad + homeW + btnGap + probW / 2, y + homeH / 2, ORANGE);
        y += homeH + secGap;

        // ── Individual axis ───────────────────────────────────────────────
        hline(pad, y, bw, COL_BORDER); y += divH + 2;
        f2("Individual axis", pad, y + labH / 2, COL_DIM2, middle_left); y += labH;
        int aw = (bw - 2 * gap) / 3;
        int axcols[3] = { COL_AX_X, COL_AX_Y, COL_AX_Z };
        const char* axlet[3] = { "X", "Y", "Z" };
        for (int i = 0; i < 3; i++) {
            int hx = pad + i * (aw + gap);
            _axisHomeBtns[i] = { hx, y, aw, axH };
            // Neutral dark bg, thin coloured left accent bar, white text
            canvas.fillRoundRect(hx, y, aw, axH, 3, COL_PANEL2);
            canvas.drawRoundRect(hx, y, aw, axH, 3, COL_BORDER2);
            canvas.fillRect(hx, y, 3, axH, axcols[i]);  // coloured left strip
            char lbl[12]; snprintf(lbl, sizeof(lbl), "Home %s", axlet[i]);
            f2(lbl, hx + aw / 2 + 2, y + axH / 2, COL_WHITE);
        }
        y += axH + secGap;

        // ── Endstop status ────────────────────────────────────────────────
        hline(pad, y, bw, COL_BORDER); y += divH + 2;
        f2("Endstop status", pad, y + labH / 2, COL_DIM2, middle_left); y += labH;
        int esW = (bw - 2 * gap) / 3;
        for (int i = 0; i < 3; i++) {
            int ex = pad + i * (esW + gap);
            bool trig = myLimitSwitches[i];
            // Uniform dark bg, red fill when triggered
            canvas.fillRoundRect(ex, y, esW, esH, 3, trig ? 0x6000 : COL_PANEL2);
            canvas.drawRoundRect(ex, y, esW, esH, 3, trig ? RED : COL_BORDER2);
            // Status dot
            canvas.fillCircle(ex + 10, y + esH / 2, 3, trig ? RED : COL_DIM2);
            // Axis letter in white always
            f2(axlet[i], ex + 20, y + esH / 2, COL_WHITE, middle_left);
            // Status text
            f2(trig ? "TRIG" : "open", ex + esW - 5, y + esH / 2,
               trig ? RED : COL_DIM2, middle_right);
        }
        y += esH + secGap;

        // ── Zero Work Coordinate ──────────────────────────────────────────
        hline(pad, y, bw, COL_BORDER); y += divH + 2;
        f2("Zero Work Coordinate", pad, y + labH / 2, COL_DIM2, middle_left); y += labH;
        const char* zla[]  = { "X", "Y", "Z", "All" };
        int zcols[]        = { COL_AX_X, COL_AX_Y, COL_AX_Z, ORANGE };
        int zw = (bw - 3 * gap) / 4;
        for (int k = 0; k < 4; k++) {
            int zx = pad + k * (zw + gap);
            _zeroWcsBtns[k] = { zx, y, zw, zeroH };
            canvas.fillRoundRect(zx, y, zw, zeroH, 3, COL_PANEL2);
            canvas.drawRoundRect(zx, y, zw, zeroH, 3, COL_BORDER2);
            canvas.fillRect(zx, y + 2, 3, zeroH - 4, zcols[k]);
            // Single line: "0 X" / "0 Y" / "0 Z" / "0 All"
            char zlbl[8]; snprintf(zlbl, sizeof(zlbl), "0 %s", zla[k]);
            f2(zlbl, zx + zw / 2 + 2, y + zeroH / 2, zcols[k]);
        }
    }

    // ── G-code preview overlay (bottom sheet, ~60% height) ──────────────────
    void drawGcodePreview() {
        int panelY = TOP + 30;   // starts 30px below header — shows file list peeking above
        int panelH = NAV_Y - panelY;
        int titleH = 18;
        int abH    = 22;
        int listY  = panelY + titleH;
        int listH  = panelH - titleH - abH;
        int lh     = 13;
        int total  = (int)previewLines.size();
        int maxL   = listH / lh;

        // Semi-transparent dim above panel
        canvas.fillRect(0, TOP, W, 30, 0x0000);

        // Panel background
        canvas.fillRect(0, panelY, W, panelH, 0x0883);
        hline(0, panelY, W, CYAN);

        // Title bar
        canvas.fillRect(0, panelY, W, titleH, 0x10A3);
        const std::string& fname = fileList[fileSelected].name;
        f2s(fname, 8, panelY + titleH/2, CYAN, middle_left);
        tintStrokeR(W-32, panelY+2, 28, 14, 3, COL_BORDER2, COL_BORDER, 60);
        f2("X", W-18, panelY+titleH/2, COL_DIM2);

        // Scroll indicator
        if (total > maxL) {
            int thumbH = std::max(4, listH * maxL / total);
            int thumbY = listY + (listH-thumbH) * previewScroll / std::max(1, total-maxL);
            canvas.fillRect(W-3, listY, 3, listH, COL_BORDER);
            canvas.fillRect(W-3, thumbY, 3, thumbH, COL_DIM2);
        }

        // G-code lines
        for (int i = 0; i < maxL && (previewScroll+i) < total; i++) {
            int ly  = listY + i*lh;
            int idx = previewScroll+i;
            if (i%2==0) canvas.fillRect(0, ly, W-4, lh, 0x0841);
            const std::string& ln = previewLines[idx];
            int col = COL_DIM2;
            if (!ln.empty()) {
                char c = ln[0];
                if      (c=='G'||c=='g') col = CYAN;
                else if (c=='M'||c=='m') col = YELLOW;
                else if (c==';'||c=='(') col = COL_DIM;
                else if (c=='F'||c=='f') col = GREEN;
                else                     col = COL_WHITE2;
            }
            canvas.setFont(&fonts::Font0);
            canvas.setTextDatum(middle_right);
            canvas.setTextColor(COL_DIM);
            char lnum[6]; snprintf(lnum,sizeof(lnum),"%d",previewFirstLine+idx+1);
            canvas.drawString(lnum, 26, ly+lh/2);
            canvas.setTextDatum(middle_left);
            canvas.setTextColor(col);
            canvas.drawString(ln.substr(0,44).c_str(), 30, ly+lh/2);
        }

        // Action bar
        int abY = NAV_Y - abH;
        canvas.fillRect(0, abY, W, abH, 0x10A3);
        hline(0, abY, W, COL_BORDER);
        char info[20]; snprintf(info,sizeof(info),"%d lines", total);
        f2(info, 8, abY+abH/2, COL_DIM2, middle_left);
        int rbw=70, rbh=16;
        tintStrokeR(W-rbw-4, abY+3, rbw, rbh, 3, GREEN, GREEN, 40);
        f2(simMode_active() ? "Sim Run" : "Run", W-4-rbw/2, abY+abH/2, GREEN, middle_center);
    }

    // ── Files screen ─────────────────────────────────────────────────────────
    void drawFilesScreen() {
        // Draw file list always (preview is an overlay on top)
        canvas.fillRect(0, TOP, W, 18, COL_PANEL);
        hline(0, TOP + 18, W, COL_BORDER);
        f2s(filePath, 18, TOP + 9, CYAN, middle_left);
        if (filePath != "/sd") {
            tintStrokeR(W - 34, TOP + 2, 28, 14, 2, COL_BORDER2, COL_BORDER, 60);
            f2("Up", W - 20, TOP + 9, COL_DIM2);
        }

        int y = TOP + 19, rowH = 22;
        int maxR = (NAV_Y - y - 22) / rowH;
        for (int i = 0; i < maxR; i++) {
            int fi = fileScroll + i;
            int ry = y + i * rowH;
            int rowbg = (i % 2 == 0) ? 0x0883 : COL_PANEL3;
            if (fi >= (int)fileList.size()) {
                canvas.fillRect(0, ry, W, rowH, rowbg);
                continue;
            }
            bool sel = (fi == fileSelected);
            canvas.fillRect(0, ry, W, rowH, sel ? 0x0019 : rowbg);
            if (sel) canvas.fillRect(0, ry, 2, rowH, BLUE);
            hline(0, ry + rowH - 1, W, COL_BORDER);
            const FileEntry& f = fileList[fi];
            if (f.isDir) {
                canvas.fillRoundRect(6, ry + rowH / 2 - 5, 11, 8, 1, 0x3240);
                canvas.drawRoundRect(6, ry + rowH / 2 - 5, 11, 8, 1, YELLOW);
                std::string dn = f.name + "/";
                f2s(dn, 22, ry + rowH / 2, sel ? COL_WHITE : YELLOW, middle_left);
            } else {
                canvas.drawRoundRect(6, ry + 4, 10, rowH - 8, 1, sel ? CYAN : COL_DIM2);
                f2s(f.name, 22, ry + rowH / 2, sel ? COL_WHITE : COL_WHITE2, middle_left);
                if (f.size > 0) {
                    char sz[20];
                    if (f.size >= 1000000)
                        snprintf(sz, sizeof(sz), "%.1f MB", f.size / 1000000.0);
                    else if (f.size >= 1000)
                        snprintf(sz, sizeof(sz), "%.1f KB", f.size / 1000.0);
                    else
                        snprintf(sz, sizeof(sz), "%d B", f.size);
                    f2(sz, W - 6, ry + rowH / 2, COL_DIM2, middle_right);
                }
            }
        }
        // Selection indicator in action bar (no overlay)
        if (fileSelected >= 0 && fileSelected < (int)fileList.size() && !fileList[fileSelected].isDir) {
            int abY = NAV_Y - 20;
            canvas.fillRect(0, abY, W, 20, COL_PANEL);
            hline(0, abY, W, COL_BORDER);
            f2s(fileList[fileSelected].name, 6, abY + 10, CYAN, middle_left);
            tintStrokeR(W - 58, abY + 2, 26, 16, 2, COL_BORDER2, COL_BORDER, 60);
            f2("View", W - 45, abY + 10, COL_WHITE2);
            tintStrokeR(W - 28, abY + 2, 24, 16, 2, GREEN, GREEN, 40);
            f2("Run", W - 16, abY + 10, GREEN);
        }

        // G-code overlay panel (bottom sheet) — drawn on top of file list
        if (filePreviewMode && fileSelected >= 0) {
            drawGcodePreview();
        }
    }

    // ── Terminal screen ───────────────────────────────────────────────────────
    void drawTerminalScreen() {
        int cmdY = NAV_Y - CMD_H;
        int outH = cmdY - TOP - 4;
        int outY = TOP + 4;

        canvas.fillRect(0, outY, W, outH, COL_PANEL3);
        hline(0, outY + outH, W, COL_BORDER);

        // Build merged view: raw UART RX lines + TX commands from termLines
        // Use font0 for compact display — fits more lines
        int lh      = 12;
        int maxLines = (outH - 4) / lh;
        int rxTotal  = fnc_term_count();
        int txTotal  = (int)termLines.size();
        // Show RX raw lines, scrollable
        int total = rxTotal;
        int start = std::max(0, total - maxLines - termScroll);
        start     = std::min(start, std::max(0, total - maxLines));

        // Scroll indicator
        if (total > maxLines) {
            int trackH = outH - 8;
            int thumbH = std::max(6, trackH * maxLines / total);
            int thumbY = outY + 4 + (trackH-thumbH) * start / std::max(1, total-maxLines);
            canvas.fillRect(W-3, outY+4, 3, trackH, COL_BORDER);
            canvas.fillRect(W-3, thumbY, 3, thumbH, COL_DIM2);
        }

        // Draw RX lines from UART capture
        for (int i = 0; i < maxLines && (start + i) < total; i++) {
            int ly = outY + 4 + i * lh;
            if (i % 2 == 0) canvas.fillRect(0, ly, W-4, lh, 0x0841);
            const char* line = fnc_term_line(start + i);
            // Colour by content
            int col = COL_DIM2;
            if (line[0] == '<') col = GREEN;           // status reports
            else if (line[0] == '[') col = CYAN;        // [MSG:] info
            else if (line[0] == 'e' || line[0]=='E') col = RED;  // error
            else if (line[0] == 'o') col = COL_WHITE2;  // ok
            else if (line[0] == '>') col = YELLOW;      // echo of sent commands
            canvas.setFont(&fonts::Font0);
            canvas.setTextDatum(middle_left);
            canvas.setTextColor(col);
            canvas.drawString(line, 6, ly + lh/2);
        }

        // Command buttons bar
        canvas.fillRect(0, cmdY, W, CMD_H, COL_PANEL2);
        hline(0, cmdY, W, COL_BORDER);
        int totalGaps = (N_QUICK_CMDS - 1) * 4;
        int cmdW      = (W - 8 - totalGaps) / N_QUICK_CMDS;
        for (int ci = 0; ci < N_QUICK_CMDS; ci++) {
            int bx = 4 + ci * (cmdW + 4);
            int bh = CMD_H - 8;
            _cmdBtns[ci] = { bx, cmdY + 4, cmdW, bh };
            tintStrokeR(bx, cmdY + 4, cmdW, bh, 3, 0xF81F, 0xF81F, 25);
            f2(QUICK_CMDS[ci], bx + cmdW / 2, cmdY + CMD_H / 2, 0xF81F);
        }
    }

    // ── Macros screen ─────────────────────────────────────────────────────────
    void drawMacrosScreen() {
        int pad = 6, gap = 5;
        int bh  = 36;  // tall enough for comfortable tapping
        int avH = NAV_Y - TOP - pad;
        int visible = std::min((int)MACROS.size(), avH / (bh + gap));

        // Clamp scroll
        int maxScroll = std::max(0, (int)MACROS.size() - visible);
        macroScroll = std::max(0, std::min(macroScroll, maxScroll));

        // Scroll indicator bar (right edge, 3px wide)
        if ((int)MACROS.size() > visible) {
            int trackH = avH;
            int thumbH = std::max(8, trackH * visible / (int)MACROS.size());
            int thumbY = TOP + pad + (trackH - thumbH) * macroScroll / std::max(1, maxScroll);
            canvas.fillRect(W - 3, TOP + pad, 3, trackH, COL_BORDER);
            canvas.fillRect(W - 3, thumbY, 3, thumbH, COL_DIM2);
        }

        int bw = W - 2 * pad - 6;  // leave 6px for scroll bar
        for (int vi = 0; vi < visible; vi++) {
            int mi = macroScroll + vi;
            int by = TOP + pad + vi * (bh + gap);
            int rh = std::min(bh, NAV_Y - by - 2);  // clamp to screen
            if (rh < 8) break;                        // too small, stop
            _macroBtns[vi] = { pad, by, bw, rh };

            // Background
            canvas.fillRoundRect(pad, by, bw, rh, 4, COL_PANEL2);
            canvas.drawRoundRect(pad, by, bw, rh, 4, COL_BORDER2);

            // Coloured left accent strip
            canvas.fillRect(pad, by + 2, 4, rh - 4, MACROS[mi].col);

            // Dot indicator
            canvas.fillCircle(pad + 14, by + rh / 2, 4, MACROS[mi].col);

            // Name — TINY font, vertically centred
            txt(MACROS[mi].name.c_str(), pad + 26, by + rh / 2,
                MACROS[mi].cmd.empty() ? COL_DIM : COL_WHITE, TINY, middle_left);

            // Command preview in small font
            if (!MACROS[mi].cmd.empty()) {
                canvas.setFont(&fonts::Font0);
                canvas.setTextDatum(middle_right);
                canvas.setTextColor(COL_DIM2);
                canvas.drawString(MACROS[mi].cmd.c_str(), W - 10, by + rh / 2);
            }
        }
        // Clear unused slots
        for (int vi = visible; vi < 8; vi++) _macroBtns[vi] = { 0, 0, 0, 0 };
    }

    // ── Probe overlay ─────────────────────────────────────────────────────────
    void drawProbeOverlay() {
        canvas.fillRect(0, TOP, W, NAV_Y - TOP, COL_BG);

        // Title bar
        int titleH = 24;
        canvas.fillRect(0, TOP, W, titleH, COL_PANEL2);
        hline(0, TOP + titleH, W, COL_BORDER);
        canvas.fillCircle(12, TOP + titleH/2, 4, CYAN);
        f2("Select Probe Operation", 22, TOP + titleH/2, COL_WHITE, middle_left);
        _probeClose = { W - 36, TOP, 36, titleH };
        tintStrokeR(W-34, TOP+3, 30, titleH-6, 3, COL_BORDER2, COL_BORDER, 60);
        f2("X", W - 19, TOP + titleH/2, COL_DIM);

        // Cards — 6 rows with label + description + command
        int listY = TOP + titleH + 3;
        int avail = NAV_Y - listY - 2;
        int rowH  = avail / N_PROBE_OPTS;

        for (int i = 0; i < N_PROBE_OPTS; i++) {
            int ry = listY + i * rowH;
            int rh = rowH - 2;
            _probeRows[i] = { 0, ry, W, rh };

            int c = PROBE_OPTS[i].col;

            // Row background — tinted with probe colour
            canvas.fillRect(0, ry, W, rh, COL_PANEL);
            // Left colour bar (8px)
            canvas.fillRect(0, ry, 8, rh, c);
            // Subtle tint on left side
            canvas.fillRect(8, ry, 60, rh, COL_PANEL2);

            // Colour icon circle
            canvas.fillCircle(28, ry + rh/2, rh/2 - 3, c);
            canvas.drawCircle(28, ry + rh/2, rh/2 - 3, COL_WHITE);

            // Probe name — TINY, bold, left of centre
            txt(PROBE_OPTS[i].label, 52, ry + rh*38/100, COL_WHITE, TINY, middle_left);

            // Description — Font0, dim, below name
            canvas.setFont(&fonts::Font0);
            canvas.setTextDatum(middle_left);
            canvas.setTextColor(COL_DIM2);
            canvas.drawString(PROBE_OPTS[i].sub, 52, ry + rh*70/100);

            // G-code command — right side, coloured
            canvas.setTextDatum(middle_right);
            canvas.setTextColor(c);
            canvas.drawString(PROBE_OPTS[i].cmd, W - 6, ry + rh/2);

            hline(0, ry + rh + 1, W, COL_BORDER);
        }
    }

    // ── Alarm overlay ─────────────────────────────────────────────────────────
    void drawAlarmOverlay() {
        canvas.fillRect(0, TOP, W, NAV_Y - TOP, 0x8000);

        int pw = 220, ph = 130;
        int cx = W / 2, cy = (NAV_Y + TOP) / 2;
        fillR(cx - pw/2, cy - ph/2, pw, ph, 8, 0x2800);
        strokeR(cx - pw/2, cy - ph/2, pw, ph, 8, RED);

        if (mpgEstopActive) {
            f2("! E-STOP ACTIVE", cx, cy - ph/2 + 22, RED);
            f2("Machine in Alarm state", cx, cy - ph/2 + 44, COL_WHITE2);
            f2("Release to clear and resume", cx, cy - ph/2 + 62, COL_WHITE2);
        } else {
            f2("! ALARM", cx, cy - ph/2 + 22, RED);
            f2("Press UNLOCK to clear", cx, cy - ph/2 + 44, COL_WHITE2);
        }

        int ubw = 180, ubh = 36;
        int ubx = cx - ubw/2, uby = cy + ph/2 - ubh - 8;
        _unlockBtn = { ubx, uby, ubw, ubh };
        if (!mpgEstopActive) {
            tintStrokeR(ubx, uby, ubw, ubh, 6, RED, RED, 60);
            f2("UNLOCK  ($X)", cx, uby + ubh/2, COL_WHITE);
        }
    }

public:
    TabScene() : Scene("TabUI", 4) {}  // encoder_scale=4: X4 PCNT quadrature, 1 detent = 1 step

    void onEntry(void* arg = nullptr) override {
        sprite_offset = { 0, 0 };
        termLines.clear();
        fnc_term_inject("> $I");
        termLines.push_back({ "[MSG:FluidDial UI]", GREEN   });
        fnc_realtime(StatusReport);
        if (simMode_active()) {
            fileList.clear(); fileScroll = 0; fileSelected = -1;
            for (auto& f : simMode_fileList())
                fileList.push_back({ f.name, f.isDir, f.size });
        } else {
            request_file_list("/sd");
        }
    }

    void onDROChange()    override { if (_tab == 0) reDisplay(); }
    void onLineReceived()  override { if (_tab == 3) reDisplay(); }
    void onLimitsChange() override { if (_tab == 1) reDisplay(); }

    void onStateChange(state_t old_state) override {
        if (state == Alarm) _alarmOpen = true;
        else _alarmOpen = false;
        reDisplay();
    }

    void onMessage(char* command, char* arguments) override {
        std::string line = std::string("[MSG:") + arguments + "]";
        termLines.push_back({ line, GREEN });
        if (termLines.size() > 200) termLines.erase(termLines.begin());
        if (termScroll == 0 && _tab == 3) reDisplay();  // live update when pinned to bottom
        if (_tab == 3) reDisplay();
    }

    void onFileLines(int firstLine, const std::vector<std::string>& lines) override {
        previewLines   = lines;
        previewFirstLine = firstLine;
        previewScroll  = 0;
        filePreviewMode = true;
        // Parse path for DRO viz
        if (fileSelected>=0 && fileSelected<(int)fileList.size())
            parseGcodeToVizPath(lines, fileList[fileSelected].name);
        if (_tab == 2) reDisplay();
    }

    void onFilesList() override {
        fileList.clear();
        fileScroll   = 0;
        fileSelected = -1;
        for (size_t i = 0; i < fileVector.size(); i++) {
            FileEntry fe;
            fe.name  = fileVector[i].fileName;
            fe.isDir = fileVector[i].isDir();
            fe.size  = fe.isDir ? 0 : fileVector[i].fileSize;
            fileList.push_back(fe);
        }
        if (_tab == 2) reDisplay();
    }

    void onEncoder(int delta) override {
        // Always update direction indicator immediately
        mpgLastDir = (delta > 0) ? 1 : -1;
        mpgDirTime = millis();

        // Jogging ONLY on DRO tab (tab 0) — all other tabs use wheel for scrolling
        if (_tab == 0 && mpgAxis >= 0 && _barSel == 0) {
            reDisplay();  // update DRO to show axis/step highlight
            if (mpgJogAllowed) {
                static const char axChar[] = { 'X', 'Y', 'Z', 'A' };
                float dist = delta * mpgSteps[(int)mpgStepIdx];
                static const int jogFeed[] = { 100, 500, 2000 };
                send_linef("$J=G91 %c%.3f F%d", axChar[(int)mpgAxis], dist, jogFeed[(int)mpgStepIdx]);
            }
            return;
        }

        // MPG wheel controls selected bar override (feed or spindle)
        if (_tab == 0 && _barSel != 0) {
            if (_barSel == 1) {
                // Feed override: coarse ±10% per detent
                fnc_realtime(delta>0 ? FeedOvrCoarsePlus : FeedOvrCoarseMinus);
            } else if (_barSel == 2) {
                // Rapid override: only 3 fixed levels — 100%(0x95), 50%(0x96), 25%(0x97)
                // Cycle through them: CW = lower, CCW = higher
                uint8_t rapid_cmd;
                // CW (delta>0) = faster (higher %), CCW (delta<0) = slower (lower %)
                // Levels: 0x95=100%, 0x96=50%, 0x97=25%
                if (myRro >= 100)     rapid_cmd = delta>0 ? 0x95 : 0x96;  // at 100%: CW→stays, CCW→50%
                else if (myRro >= 50) rapid_cmd = delta>0 ? 0x95 : 0x97;  // at 50%:  CW→100%, CCW→25%
                else                  rapid_cmd = delta>0 ? 0x96 : 0x97;  // at 25%:  CW→50%,  CCW→stays
                fnc_realtime((realtime_cmd_t)rapid_cmd);
            } else {
                // Spindle override: coarse ±10% per detent
                fnc_realtime(delta>0 ? (realtime_cmd_t)0x9A : (realtime_cmd_t)0x9B);
            }
            reDisplay();
            return;
        }

        // All other cases: scroll the active tab (axis switch position ignored)
        if (_tab == 2) {
            if (filePreviewMode) {
                // Scroll gcode preview
                int maxS = std::max(0, (int)previewLines.size() - 1);
                previewScroll = std::max(0, std::min(previewScroll + delta, maxS));
            } else {
                fileScroll = std::max(0, fileScroll + delta);
            }
        } else if (_tab == 3) {
            termScroll = std::max(0, termScroll - delta);
            int cmdY      = NAV_Y - CMD_H;
            int outH      = cmdY - TOP - 4;
            int lh2       = 12;
            int maxL      = (outH - 4) / lh2;
            int maxScroll = std::max(0, fnc_term_count() - maxL);
            termScroll    = std::min(termScroll, maxScroll);
        } else if (_tab == 4) {
            macroScroll = std::max(0, macroScroll + delta);
        }
        reDisplay();  // always redraw so header arrow is immediate on any tab
    }

    // Parse G-code lines into vizPath (machine mm XY coords)
    void parseGcodeToVizPath(const std::vector<std::string>& lines, const std::string& name) {
        vizPath.clear(); vizPathExecuted = 0; vizJobName = name;
        float cx=0, cy=0; bool isAbs=true;
        for (const auto& ln : lines) {
            const char* p = ln.c_str();
            while (*p==' ') p++;
            if (!*p || *p==';' || *p=='(') continue;
            if ((*p=='G'||*p=='g') && p[1]=='9') {
                if (p[2]=='0') isAbs=true;
                else if (p[2]=='1') isAbs=false;
                continue;
            }
            float nx=cx, ny=cy; bool mv=false;
            for (const char* q=p; *q; q++) {
                char c=*q;
                bool atWord=(q==p||*(q-1)==' '||*(q-1)=='	');
                if ((c=='X'||c=='x')&&atWord){nx=isAbs?strtof(q+1,nullptr):cx+strtof(q+1,nullptr);mv=true;}
                if ((c=='Y'||c=='y')&&atWord){ny=isAbs?strtof(q+1,nullptr):cy+strtof(q+1,nullptr);mv=true;}
            }
            if (mv && (nx!=cx || ny!=cy)) { vizPath.push_back({nx,ny}); cx=nx; cy=ny; }
        }
    }

    void onTouchClick() override {
        int x = touchX, y = touchY;

        // Alarm overlay — intercepts all touches
        if (_alarmOpen && state == Alarm) {
            if (hit(_unlockBtn, x, y)) {
                send_line("$X");
                fnc_term_inject("> $X");
                termLines.push_back({ "[MSG:Unlocked]", GREEN  });
            }
            return;
        }

        // Probe overlay
        if (_probeOpen) {
            if (hit(_probeClose, x, y)) { _probeOpen = false; reDisplay(); return; }
            for (int i = 0; i < N_PROBE_OPTS; i++) {
                if (hit(_probeRows[i], x, y)) {
                    _probeOpen = false;
                    if (PROBE_OPTS[i].cmd[0]) {
                        send_line(PROBE_OPTS[i].cmd);
                        fnc_term_inject((std::string("> ") + PROBE_OPTS[i].cmd).c_str());
                    }
                    reDisplay(); return;
                }
            }
            _probeOpen = false; reDisplay(); return;
        }

        // Nav bar
        if (y >= NAV_Y) {
            for (int i = 0; i < N_TABS; i++) {
                if (hit(_navTabs[i], x, y)) {
                    _tab = i;
                    if (_tab == 2) {
                        if (simMode_active()) {
                            // Load simulated file list
                            fileList.clear(); fileScroll = 0; fileSelected = -1;
                            for (auto& f : simMode_fileList()) {
                                fileList.push_back({ f.name, f.isDir, f.size });
                            }
                        } else {
                            request_file_list(filePath.c_str());
                        }
                    }
                    reDisplay(); return;
                }
            }
            return;
        }

        // DRO screen
        if (_tab == 0) {
            // Simulation mode: tap a DRO axis row to simulate +0.1mm jog
            if (simMode_active() && x < DROW && y >= TOP && y < NAV_Y - FEED_H) {
                static const int _axCounts[] = {3,4,2,4};
                int _dr = (NAV_Y - TOP - FEED_H) / _axCounts[_droAxesMode];
                int axis = (y - TOP) / _dr;
                if (simMode_handleDROTap(axis)) { reDisplay(); return; }
            }
            // Tap FEED or SPND pill to toggle MPG wheel control
            // Centre (SPEED) pill is display-only
            if (y >= NAV_Y - FEED_H) {
                int third = W / 3;
                if      (x < third)    { _barSel = (_barSel==1)?0:1; reDisplay(); return; }
                else if (x < 2*third)  { _barSel = (_barSel==2)?0:2; reDisplay(); return; }
                else                   { _barSel = (_barSel==3)?0:3; reDisplay(); return; }
            }

        }

        // Homing screen
        if (_tab == 1) {
            if (hit(_homeAllBtn, x, y)) {
                send_line("$H");
                fnc_term_inject("> $H");
                reDisplay(); return;
            }
            if (hit(_probeBtnR, x, y)) {
                _probeOpen = true; reDisplay(); return;
            }
            const char* axcmds[] = { "$HX", "$HY", "$HZ" };
            for (int i = 0; i < 3; i++) {
                if (hit(_axisHomeBtns[i], x, y)) {
                    send_line(axcmds[i]);
                    fnc_term_inject((std::string("> ") + axcmds[i]).c_str());
                    reDisplay(); return;
                }
            }
            const char* zcmds[] = {
                "G10 L20 P1 X0", "G10 L20 P1 Y0",
                "G10 L20 P1 Z0", "G10 L20 P1 X0 Y0 Z0"
            };
            for (int k = 0; k < 4; k++) {
                if (hit(_zeroWcsBtns[k], x, y)) {
                    send_line(zcmds[k]);
                    fnc_term_inject((std::string("> ") + zcmds[k]).c_str());
                    termLines.push_back({ "ok", GREEN });
                    reDisplay(); return;
                }
            }
        }

        // Files screen
        if (_tab == 2) {
            // Up button
            if (filePath != "/sd" && x >= W - 34 && y >= TOP + 2 && y <= TOP + 16) {
                size_t pos = filePath.rfind('/');
                if (pos != std::string::npos && pos > 0) filePath = filePath.substr(0, pos);
                else filePath = "/sd";
                fileScroll = 0; fileSelected = -1;
                request_file_list(filePath.c_str()); return;
            }
            int rowH = 22, yOff = TOP + 19;
            if (y >= yOff && y < NAV_Y - 22) {
                int ri = (y - yOff) / rowH;
                int fi = fileScroll + ri;
                if (fi < (int)fileList.size()) {
                    if (fileList[fi].isDir) {
                        filePath += "/" + fileList[fi].name;
                        fileScroll = 0; fileSelected = -1;
                        if (simMode_active()) {
                            fileList.clear();
                            for (auto& f : simMode_fileList())
                                fileList.push_back({ f.name, f.isDir, f.size });
                        } else {
                            request_file_list(filePath.c_str());
                        }
                    } else {
                        fileSelected     = fi;
                        filePreviewMode  = false;
                        previewLines.clear();
                        previewScroll    = 0;
                        // Load preview: request 60 lines from start of file
                        if (simMode_active()) {
                            // Unique gcode per file index so each shows a different path
                            previewLines.clear();
                            const std::string& fn = fileList[fi].name;
                            previewLines.push_back(std::string("; ") + fn);
                            previewLines.push_back("G21 G90 G17");
                            previewLines.push_back("G0 Z5");
                            int fidx = fi % 6;  // 6 distinct patterns
                            if (fidx == 0) {
                                // Circle (16-segment)
                                previewLines.push_back("G0 X25 Y0");
                                previewLines.push_back("M3 S10000");
                                previewLines.push_back("G1 F300");
                                for (int si=1; si<=16; si++) {
                                    float a=si*3.14159f/8.0f;
                                    char b[32]; snprintf(b,sizeof(b),"G1 X%.2f Y%.2f",25+25*cosf(a),25+25*sinf(a));
                                    previewLines.push_back(b);
                                }
                            } else if (fidx == 1) {
                                // L-shaped contour
                                previewLines.push_back("G0 X0 Y0");
                                previewLines.push_back("M3 S12000");
                                previewLines.push_back("G1 F400");
                                const float pts[][2]={{0,0},{70,0},{70,20},{30,20},{30,60},{0,60},{0,0}};
                                for (auto& pp2 : pts) {
                                    char b[32]; snprintf(b,sizeof(b),"G1 X%.0f Y%.0f",pp2[0],pp2[1]);
                                    previewLines.push_back(b);
                                }
                            } else if (fidx == 2) {
                                // Drill grid 4x3
                                previewLines.push_back("M3 S8000");
                                for (int xi=0;xi<4;xi++) for (int yi=0;yi<3;yi++) {
                                    char b[32]; snprintf(b,sizeof(b),"G0 X%.0f Y%.0f",10.f+xi*16,10.f+yi*16);
                                    previewLines.push_back(b);
                                    previewLines.push_back("G1 Z-8 F100");
                                    previewLines.push_back("G0 Z5");
                                }
                            } else if (fidx == 3) {
                                // Star / cross
                                previewLines.push_back("G0 X35 Y5");
                                previewLines.push_back("M3 S9000");
                                previewLines.push_back("G1 F350");
                                const float pts[][2]={{35,5},{45,25},{65,25},{50,40},{55,60},{35,48},{15,60},{20,40},{5,25},{25,25},{35,5}};
                                for (auto& pp2 : pts) {
                                    char b[32]; snprintf(b,sizeof(b),"G1 X%.0f Y%.0f",pp2[0],pp2[1]);
                                    previewLines.push_back(b);
                                }
                            } else if (fidx == 4) {
                                // Zigzag facing
                                previewLines.push_back("G0 X0 Y0");
                                previewLines.push_back("M3 S15000");
                                previewLines.push_back("G1 F800");
                                for (int row=0;row<8;row++) {
                                    char b[32];
                                    float fy=row*8.0f;
                                    snprintf(b,sizeof(b),"G1 X%.0f Y%.0f",(row%2==0?70.0f:0.0f),fy);
                                    previewLines.push_back(b);
                                    snprintf(b,sizeof(b),"G1 Y%.0f",fy+8.0f);
                                    previewLines.push_back(b);
                                }
                            } else {
                                // Rounded pocket spiral
                                previewLines.push_back("G0 X35 Y30");
                                previewLines.push_back("M3 S11000");
                                previewLines.push_back("G1 F500");
                                for (int r=5;r<=30;r+=5) {
                                    for (int si=0;si<=16;si++) {
                                        float a=si*3.14159f/8.0f;
                                        char b[32]; snprintf(b,sizeof(b),"G1 X%.2f Y%.2f",35+r*cosf(a),30+r*sinf(a));
                                        previewLines.push_back(b);
                                    }
                                }
                            }
                            previewLines.push_back("G0 Z5");
                            previewLines.push_back("M5");
                            previewLines.push_back("M30");
                            filePreviewMode = true;
                        } else {
                            std::string path = filePath + "/" + fileList[fi].name;
                            request_file_preview(path.c_str(), 0, 60);
                        }
                        reDisplay();
                    }
                }
                return;
            }
            // G-code preview overlay touches
            if (filePreviewMode) {
                int panelTop = TOP + 30;
                if (y < panelTop) {
                    // Touch above panel → close overlay, fall through to file list tap
                    filePreviewMode = false;
                    reDisplay();
                    // intentional fall-through to file list handler below
                } else {
                    // Touch inside panel
                    int abH = 22;
                    // X close button
                    if (y <= panelTop + 18 && x >= W - 32) {
                        filePreviewMode = false; reDisplay(); return;
                    }
                    // Run button in action bar
                    if (y >= NAV_Y - abH && x >= W - 74) {
                        // Parse gcode XY path
                        simPath.clear(); simPathIdx = 0;
                        parseGcodeToVizPath(previewLines, fileList[fileSelected].name);
                        simPath = vizPath;  // share for sim animation
                                                simJobName = fileList[fileSelected].name;
                        if (!simMode_active()) {
                            std::string path = filePath + "/" + fileList[fileSelected].name;
                            send_linef("$Localfs/Run=%s", path.c_str());
                            termLines.push_back({ "> Run: " + simJobName, COL_DIM2 });
                        } else {
                            simJobRunning = true; simPathIdx = 0;
                            if (!simPath.empty()) {
                                simMode_setPos(0, simPath[0].first);
                                simMode_setPos(1, simPath[0].second);
                            }
                            termLines.push_back({ "> [SIM] Run: " + simJobName, ORANGE });
                        }
                        filePreviewMode = false;
                        _tab = 0; reDisplay(); return;
                    }
                    return;  // consume all other panel touches
                }
            }
            // Run button (action bar in list mode)
            if (fileSelected >= 0 && y >= NAV_Y - 20 && x >= W - 42) {
                std::string path = filePath + "/" + fileList[fileSelected].name;
                send_linef("$Localfs/Run=%s", path.c_str());
                termLines.push_back({ std::string("> Run: ") + fileList[fileSelected].name, COL_DIM2 });
                _tab = 3; reDisplay();
                return;
            }
        }

        // Terminal screen
        if (_tab == 3) {
            for (int ci = 0; ci < N_QUICK_CMDS; ci++) {
                if (hit(_cmdBtns[ci], x, y)) {
                    char _echo[68];
                    snprintf(_echo, sizeof(_echo), "> %s", QUICK_CMDS[ci]);
                    fnc_term_inject(_echo);
                    send_line(QUICK_CMDS[ci]);
                    reDisplay(); return;
                }
            }
        }

        // Macros screen — _macroBtns maps visible slots to MACROS[macroScroll+vi]
        if (_tab == 4) {
            for (int vi = 0; vi < 8; vi++) {
                if (_macroBtns[vi].w == 0) break;
                if (hit(_macroBtns[vi], x, y)) {
                    int mi = macroScroll + vi;
                    if (mi < (int)MACROS.size() && !MACROS[mi].cmd.empty()) {
                        send_line(MACROS[mi].cmd.c_str());
                        fnc_term_inject((std::string("> ") + MACROS[mi].cmd).c_str());
                        reDisplay(); return;
                    }
                }
            }
        }
    }

    void onLeftFlick()  override {}
    void onRightFlick() override {}
    void onUpFlick() override {
        if (_tab == 2 && filePreviewMode) { previewScroll = std::max(0, previewScroll - 5); reDisplay(); }
        if (_tab == 3) { termScroll = std::max(0, termScroll - 5); reDisplay(); }
        if (_tab == 4) { macroScroll = std::max(0, macroScroll - 3); reDisplay(); }
    }
    void onDownFlick() override {
        if (_tab == 2 && filePreviewMode) { previewScroll += 5; reDisplay(); }
        if (_tab == 3) {
            termScroll += 5;
            int cmdY  = NAV_Y - CMD_H;
            int outH  = cmdY - TOP - 4;
            int lh3   = 12;
            int maxL  = (outH - 4) / lh3;
            int maxSc = std::max(0, fnc_term_count() - maxL);
            termScroll = std::min(termScroll, maxSc);
            reDisplay();
        }
        if (_tab == 4) { macroScroll += 3; reDisplay(); }
    }

    void reDisplay() override {
        _currentTab = _tab;
        canvas.fillSprite(COL_BG);
        drawHeader();
        canvas.fillRect(0, TOP, W, NAV_Y - TOP, COL_BG);
        switch (_tab) {
            case 0: drawDROScreen();      break;
            case 1: drawHomingScreen();   break;
            case 2: drawFilesScreen();    break;
            case 3: drawTerminalScreen(); break;
            case 4: drawMacrosScreen();   break;
        }
        if (_probeOpen)              drawProbeOverlay();
        if (_alarmOpen || _forceAlarm) drawAlarmOverlay();
        drawNav();
        refreshDisplay();
    }
};

static TabScene tabScene;

void mpgCheckMacroFire() {
    if (!_p6MacroFire) return;
    _p6MacroFire = false;
    if (_enableMacro >= 0 && _enableMacro < (int)MACROS.size()
        && !MACROS[_enableMacro].cmd.empty()) {
        send_line(MACROS[_enableMacro].cmd.c_str());
    }
}

Scene* getTabScene() { return &tabScene; }
