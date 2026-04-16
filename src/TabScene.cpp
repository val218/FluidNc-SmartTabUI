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
static const int COL_AX_X   = 0x06BF;
static const int COL_AX_Y   = 0x0771;
static const int COL_AX_Z   = 0xFEE8;
static const int COL_AX_A   = 0xDB3F;

// ─────────────────────────────────────────────────────────────────────────────
// Layout
// ─────────────────────────────────────────────────────────────────────────────
static const int W       = 320;
static const int H       = 240;
static const int TOP     = 20;
static const int BOT     = 34;
static const int NAV_Y   = H - BOT;
static const int DROW    = 90;
static const int FEED_H  = 46;
static const int TAB_W   = 22;
static const int VIZ_X   = DROW;
static const int VIZ_Y   = TOP;
static const int VIZ_W   = W - DROW - TAB_W;
static const int VIZ_H   = NAV_Y - FEED_H - VIZ_Y;
static const int TAB_X   = W - TAB_W;
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
static int  _droAxesMode = 0;  // 0=XYZ 1=XYZA 2=XY 3=XYYZ (dual Y)

// Called by Settings to apply theme colours
void tabui_setTheme(int t) {
    switch (t) {
        case 1:  // Neutral — medium grey, light text
            COL_BG=0x31C8; COL_PANEL=0x4229; COL_PANEL2=0x4A8B; COL_PANEL3=0x39E9;
            COL_BORDER=0x5B2E; COL_BORDER2=0x73D1; COL_DIM=0x94D5; COL_DIM2=0xB5D9;
            COL_WHITE=0xE75E; COL_WHITE2=0xC65A; break;
        case 2:  // Light — near-white backgrounds, dark text
            COL_BG=0xF79E; COL_PANEL=0xE75D; COL_PANEL2=0xD6FC; COL_PANEL3=0xEF7E;
            COL_BORDER=0xB5F9; COL_BORDER2=0x8CB4; COL_DIM=0x636F; COL_DIM2=0x426B;
            COL_WHITE=0x10A3; COL_WHITE2=0x2987; break;
        default: // Dark — original deep blue-grey
            COL_BG=0x0862; COL_PANEL=0x0883; COL_PANEL2=0x10A3; COL_PANEL3=0x0863;
            COL_BORDER=0x1906; COL_BORDER2=0x2147; COL_DIM=0x31C9; COL_DIM2=0x52F0;
            COL_WHITE=0xDF1D; COL_WHITE2=0x9D17; break;
    }
}
void tabui_setAxes(int a) { _droAxesMode = a; }
int  tabui_getAxes()      { return _droAxesMode; }
bool mpgEnableHeld()      { return (bool)mpgEnable; }
void setSimMode(bool sim) { if (sim) simMode_enable(); }
bool getSimMode()         { return simMode_active(); }

// Read PCF8574 I2C GPIO expander (address 0x20)
// Returns 0xFF on error (all pins HIGH = nothing active = safe default)
static bool    pcf8574Present = false;  // detected on first successful read
static uint8_t readPCF8574() {
    uint8_t val = 0xFF;
    esp_err_t err = i2c_master_read_from_device(I2C_NUM_1, 0x20, &val, 1, pdMS_TO_TICKS(2));
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

    // Axis — only active when enable button (P6) is held
    // This prevents accidental jogging when reaching for the pendant
    if (!enable) {
        mpgAxis = -1;  // enable not held → no jogging
    } else if (axA) mpgAxis = 3;
    else if (axZ)   mpgAxis = 2;
    else if (axY)   mpgAxis = 1;
    else if (axX)   mpgAxis = 0;
    else            mpgAxis = -1;  // enable held but no axis selected

    // Step — only meaningful when axis is selected
    if (mpgAxis >= 0) {
        if      (st10) mpgStepIdx = 2;  // x100
        else if (st1)  mpgStepIdx = 1;  // x10
        else           mpgStepIdx = 0;  // x1
    }

    // Clear PCNT backlog when axis goes off → active
    if (prevAxis < 0 && mpgAxis >= 0) {
        pcnt_counter_clear(PCNT_UNIT_0);
    }

    if ((mpgAxis != prevAxis || mpgStepIdx != prevStep) && current_scene) {
        current_scene->reDisplay();
    }
}
static const char* QUICK_CMDS[] = { "$H", "$?", "!", "~", "$X" };
static const int   N_QUICK_CMDS = 5;

// ─────────────────────────────────────────────────────────────────────────────
// Probe options
// ─────────────────────────────────────────────────────────────────────────────
struct ProbeOpt { const char* label; const char* sub; const char* cmd; int col; };
static const ProbeOpt PROBE_OPTS[] = {
    { "Tool Length", "Probe Z with current tool",  "G38.2 Z-50 F100", YELLOW  },
    { "Z Surface",   "Probe top of workpiece",     "G38.2 Z-50 F100", CYAN    },
    { "XY Center",   "Find center of hole/boss",   "G38.2 X-50 F100", GREEN   },
    { "X Edge",      "Find X axis edge",           "G38.2 X-50 F100", COL_AX_X   },
    { "Y Edge",      "Find Y axis edge",           "G38.2 Y-50 F100", COL_AX_Y   },
    { "Corner",      "Find XY workpiece corner",   "G38.2 X-25 F100", ORANGE  },
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
        // state badge - compact, 40px wide
        int tint = (((sc >> 11) & 0x1F) * 20 / 255) << 11 |
                   (((sc >> 5)  & 0x3F) * 20 / 255) << 5  |
                   (((sc >> 0)  & 0x1F) * 20 / 255);
        fillR(2, 2, 42, 16, 3, tint);
        strokeR(2, 2, 42, 16, 3, sc);
        hdrTxt(my_state_string, 23, 10, sc);

        // screen name centred (with SIM badge if in simulation mode)
        if (simMode_active()) {
            hdrTxt("SIM", W / 2 - 20, 10, ORANGE);
            hdrTxt(TAB_LABELS[_tab], W / 2 + 12, 10, COL_WHITE2);
        } else {
            hdrTxt(TAB_LABELS[_tab], W / 2, 10, COL_WHITE2);
        }

        // Enable button indicator — green when held, dim when not
        int enCol = mpgEnable ? GREEN : COL_DIM;
        canvas.fillCircle(W - 92, 10, 3, enCol);
        hdrTxt("EN", W - 87, 10, enCol, middle_left);

        // TX/RX indicators
        canvas.fillCircle(W - 72, 10, 2, COL_DIM2);
        hdrTxt("TX", W - 68, 10, COL_DIM2, middle_left);
        canvas.fillCircle(W - 52, 10, 2, COL_DIM2);
        hdrTxt("RX", W - 48, 10, COL_DIM2, middle_left);

        // MPG axis + step label — always visible when enable held and axis selected
        if ((int)mpgEnable && mpgAxis >= 0) {
            static const int axCols[] = { COL_AX_X, COL_AX_Y, COL_AX_Z, COL_AX_A };
            static const char axNames[] = { 'X', 'Y', 'Z', 'A' };
            int ac = axCols[mpgAxis];
            char jogLabel[12];
            snprintf(jogLabel, sizeof(jogLabel), "JOG %c %smm",
                     axNames[mpgAxis], mpgStepLabels[(int)mpgStepIdx]);
            // Draw over the tab name area — highlight with axis colour
            canvas.fillRect(W/2 - 42, 2, 84, 16, 0x0000);
            strokeR(W/2 - 42, 2, 84, 16, 3, ac);
            hdrTxt(jogLabel, W/2, 10, ac);
        }

        // MPG direction indicator — shown for 200ms after any wheel movement
        if (mpgLastDir != 0 && (millis() - mpgDirTime) < 200) {
            static const int axCols[] = { COL_AX_X, COL_AX_Y, COL_AX_Z, COL_AX_A };
            int ac = (mpgAxis >= 0) ? axCols[mpgAxis] : COL_WHITE2;
            // Two triangles side by side: active direction bright, inactive dim
            // Left arrow at W-30, right arrow at W-18
            auto drawTri = [&](int cx, bool pointRight, int col) {
                for (int r = 0; r < 7; r++) {
                    int half = (r < 4) ? r : (6 - r);
                    int lx = pointRight ? (cx - 3)          : (cx + 3 - half);
                    int rx = pointRight ? (cx - 3 + half)   : (cx + 3);
                    if (rx > lx) canvas.drawFastHLine(lx, 3 + r, rx - lx + 1, col);
                }
            };
            int dimCol = COL_DIM;  // inactive arrow colour
            if (mpgLastDir < 0) {   // spinning left (CCW)
                drawTri(W - 28, false, ac);      // ◄ bright
                drawTri(W - 16, true,  dimCol);  // ► dim
            } else {                // spinning right (CW)
                drawTri(W - 28, false, dimCol);  // ◄ dim
                drawTri(W - 16, true,  ac);      // ► bright
            }
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
            bool isMpg = (mpgAxis == i);  // this row is the active MPG axis

            // Background: highlighted when this axis is selected on MPG
            int rowbg = isMpg ? 0x0925 : ((i % 2 == 0) ? 0x0883 : 0x0863);
            canvas.fillRect(0, ry, DROW, droRow, rowbg);

            // Left accent strip: thicker + brighter when MPG active on this axis
            int stripW = isMpg ? 5 : 3;
            canvas.fillRect(0, ry, stripW, droRow, axcol);
            hline(0, ry + droRow - 1, DROW, COL_BORDER);

            // Axis letter (top of row) — XYYZ mode: axis 3 = Y2
            const char* axLetter = (_droAxesMode == 3 && i == 3) ? "Y2" : AX_STYLES[i].letter;
            txt(axLetter, 7, ry + droRow * 28 / 100, axcol, TINY, middle_left);

            // Position value (middle of row)
            // Show real or simulated position
            // 3 decimal places in mm (e.g. -123.456), 4 in inches
            int ndig = inInches ? 4 : 3;
            const char* posStr = simMode_active()
                ? pos_to_cstr(simMode_getPos(i), 3)
                : pos_to_cstr(myAxes[i], ndig);
            txt(posStr, DROW - 5, ry + droRow * 58 / 100,
                simMode_active() ? ORANGE : COL_WHITE, TINY, middle_right);

            // Step size indicator — shown on active axis row, always readable
            if (isMpg) {
                char stepStr[10];
                snprintf(stepStr, sizeof(stepStr), "âº%smm", mpgStepLabels[(int)mpgStepIdx]);
                canvas.setFont(&fonts::Font0);
                canvas.setTextDatum(middle_right);
                canvas.setTextColor(axcol);
                canvas.drawString(stepStr, DROW - 5, ry + droRow * 85 / 100);
            } else if ((int)mpgEnable && mpgAxis < 0) {
                // Enable held but no axis — show step size dimly on first row only
                if (i == 0) {
                    char stepStr[8];
                    snprintf(stepStr, sizeof(stepStr), "%smm", mpgStepLabels[(int)mpgStepIdx]);
                    canvas.setFont(&fonts::Font0);
                    canvas.setTextDatum(middle_right);
                    canvas.setTextColor(COL_DIM);
                    canvas.drawString(stepStr, DROW - 5, ry + droRow * 85 / 100);
                }
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

        if (simJobRunning || (!simPath.empty() && simPathIdx > 0)) {
            // Draw tool path
            float minX=simPath[0].first, maxX=minX;
            float minY=simPath[0].second, maxY=minY;
            for (auto& pt : simPath) {
                minX=std::min(minX,pt.first); maxX=std::max(maxX,pt.first);
                minY=std::min(minY,pt.second); maxY=std::max(maxY,pt.second);
            }
            float rx=maxX-minX, ry=maxY-minY;
            if (rx<1.0f) rx=1.0f; if (ry<1.0f) ry=1.0f;
            // Shrink bottom 10px to leave room for progress bar
            int vizH2 = VIZ_H - 12;
            float scale = std::min((VIZ_W-8)/(float)rx, (float)vizH2/(float)ry);
            float ox = VIZ_X+4 + (VIZ_W-8 - rx*scale)/2.0f;
            float oy = VIZ_Y+4 + ((float)vizH2 - ry*scale)/2.0f + ry*scale;

            auto toVX = [&](float vx){ return (int)(ox + (vx-minX)*scale); };
            auto toVY = [&](float vy){ return (int)(oy - (vy-minY)*scale); };

            int drawn = std::min(simPathIdx, (int)simPath.size()-1);
            for (int pi=1; pi<(int)simPath.size(); pi++) {
                int x1=toVX(simPath[pi-1].first), y1=toVY(simPath[pi-1].second);
                int x2=toVX(simPath[pi].first),   y2=toVY(simPath[pi].second);
                canvas.drawLine(x1,y1,x2,y2, pi<=drawn ? CYAN : COL_DIM);
            }
            if (drawn < (int)simPath.size()) {
                canvas.fillCircle(toVX(simPath[drawn].first), toVY(simPath[drawn].second), 3, GREEN);
            }

            // Progress bar at bottom of viz area
            int pbY = VIZ_Y + VIZ_H - 10;
            int pct = simPath.size()>1 ? simPathIdx*100/((int)simPath.size()-1) : 100;
            int pbW = (VIZ_W-4) * pct / 100;
            canvas.fillRect(VIZ_X+2, pbY, VIZ_W-4, 7, COL_BORDER);
            if (pbW>0) canvas.fillRect(VIZ_X+2, pbY, pbW, 7, CYAN);
            // Job name top-left
            if (!simJobName.empty())
                hdrTxt(simJobName.c_str(), VIZ_X+3, VIZ_Y+6, COL_DIM2, middle_left);
        } else {
            // Idle — simple placeholder
            txt("Visualizer", VIZ_X+VIZ_W/2, VIZ_Y+VIZ_H/2-6, COL_DIM, TINY, middle_center);
            txt(simMode_active()?"Run a file to preview":"(no job running)",
                VIZ_X+VIZ_W/2, VIZ_Y+VIZ_H/2+8, COL_DIM, TINY, middle_center);
        }

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

        // Feed bar
        int fc = (myFro < 80) ? RED : (myFro > 120) ? ORANGE : CYAN;
        int fy = NAV_Y - FEED_H;
        canvas.fillRect(0, fy, W, FEED_H, COL_PANEL2);
        hline(0, fy, W, COL_BORDER);

        int pad = 6, gap = 4;
        int avail = W - 2 * pad - 2 * gap;
        int pillW = avail * 28 / 100;
        int btnW  = (avail - pillW) / 2;

        // Helper: draw feed bar text using compact Font2 (not bold)
        auto feedTxt = [&](const char* s, int x, int y, int col, int datum = middle_center) {
            canvas.setFont(&fonts::Font2);
            canvas.setTextDatum(datum);
            canvas.setTextColor(col);
            canvas.drawString(s, x, y);
        };

        // -10% button
        int b1x = pad;
        _feedMinus = { b1x, fy, btnW, FEED_H };
        tintStrokeR(b1x, fy + 4, btnW, FEED_H - 8, 4, fc, fc, 25);
        feedTxt("-10%", b1x + btnW / 2, fy + FEED_H / 2, fc);

        // FEED pill — "FEED" label top, percentage below
        int px = pad + btnW + gap;
        tintStrokeR(px, fy + 4, pillW, FEED_H - 8, 4, fc, fc, 18);
        feedTxt("FEED", px + pillW / 2, fy + FEED_H * 30 / 100, fc);
        char fpct[12];
        snprintf(fpct, sizeof(fpct), "%d%%", (int)myFro);
        feedTxt(fpct, px + pillW / 2, fy + FEED_H * 68 / 100, COL_WHITE);

        // +10% button
        int b2x = px + pillW + gap;
        _feedPlus = { b2x, fy, btnW, FEED_H };
        tintStrokeR(b2x, fy + 4, btnW, FEED_H - 8, 4, fc, fc, 25);
        feedTxt("+10%", b2x + btnW / 2, fy + FEED_H / 2, fc);
    }

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

        int maxLines = (outH - 4) / OUT_LH;
        int total    = (int)termLines.size();
        // start index: 0=bottom, termScroll>0 means scrolled up
        int start = std::max(0, total - maxLines - termScroll);
        start     = std::min(start, std::max(0, total - maxLines));
        // scroll indicator on right edge
        if (total > maxLines) {
            int trackH = outH - 8;
            int thumbH = std::max(6, trackH * maxLines / total);
            int thumbY = outY + 4 + (trackH - thumbH) * start / std::max(1, total - maxLines);
            canvas.fillRect(W - 3, outY + 4, 3, trackH, COL_BORDER);
            canvas.fillRect(W - 3, thumbY, 3, thumbH, COL_DIM2);
        }
        for (int i = 0; i < std::min(maxLines, total - start); i++) {
            if (i % 2 == 0) canvas.fillRect(0, outY + 4 + i * OUT_LH, W, OUT_LH, 0x0841);
            f2s(termLines[(size_t)(start + i)].txt,
                 8, outY + 4 + i * OUT_LH + OUT_LH / 2,
                 termLines[(size_t)(start + i)].col, middle_left);
        }

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
            _macroBtns[vi] = { pad, by, bw, bh };

            // Background
            canvas.fillRoundRect(pad, by, bw, bh, 4, COL_PANEL2);
            canvas.drawRoundRect(pad, by, bw, bh, 4, COL_BORDER2);

            // Coloured left accent strip
            canvas.fillRect(pad, by + 2, 4, bh - 4, MACROS[mi].col);

            // Dot indicator
            canvas.fillCircle(pad + 14, by + bh / 2, 4, MACROS[mi].col);

            // Name — TINY font, vertically centred
            txt(MACROS[mi].name.c_str(), pad + 26, by + bh / 2,
                MACROS[mi].cmd.empty() ? COL_DIM : COL_WHITE, TINY, middle_left);

            // Command preview in small font
            if (!MACROS[mi].cmd.empty()) {
                canvas.setFont(&fonts::Font0);
                canvas.setTextDatum(middle_right);
                canvas.setTextColor(COL_DIM2);
                canvas.drawString(MACROS[mi].cmd.c_str(), W - 10, by + bh / 2);
            }
        }
        // Clear unused slots
        for (int vi = visible; vi < 8; vi++) _macroBtns[vi] = { 0, 0, 0, 0 };
    }

    // ── Probe overlay ─────────────────────────────────────────────────────────
    void drawProbeOverlay() {
        // Full-screen dark overlay
        canvas.fillRect(0, TOP, W, NAV_Y - TOP, 0x0863);

        // Title bar
        int titleH = 26;
        canvas.fillRect(0, TOP, W, titleH, 0x10A3);
        hline(0, TOP + titleH, W, COL_BORDER);
        canvas.fillCircle(14, TOP + titleH / 2, 4, CYAN);
        f2("Probe Operation", 26, TOP + titleH / 2, COL_WHITE, middle_left);
        _probeClose = { W - 30, TOP, 30, titleH };
        f2("X", W - 15, TOP + titleH / 2, COL_DIM2);

        // Rows — full width, generous height for easy tapping
        int listY = TOP + titleH + 4;
        int avail = NAV_Y - listY - 4;
        int rowH  = avail / N_PROBE_OPTS;

        for (int i = 0; i < N_PROBE_OPTS; i++) {
            int ry = listY + i * rowH;
            int rh = rowH - 3;
            _probeRows[i] = { 0, ry, W, rh };

            // Alternating background
            canvas.fillRect(0, ry, W, rh, i % 2 == 0 ? 0x0883 : 0x0841);

            // Coloured left accent strip
            canvas.fillRect(0, ry, 5, rh, PROBE_OPTS[i].col);

            // Label centred in row — TINY font, single line
            txt(PROBE_OPTS[i].label, 14, ry + rh * 42 / 100,
                COL_WHITE, TINY, middle_left);
            // Sub-description in Font0 below label
            canvas.setFont(&fonts::Font0);
            canvas.setTextDatum(middle_left);
            canvas.setTextColor(COL_DIM2);
            canvas.drawString(PROBE_OPTS[i].sub, 14, ry + rh * 78 / 100);

            // Arrow indicator
            canvas.setFont(&fonts::Font2);
            canvas.setTextDatum(middle_right);
            canvas.setTextColor(PROBE_OPTS[i].col);
            canvas.drawString(">", W - 8, ry + rh / 2);

            hline(0, ry + rh, W, COL_BORDER);
        }
    }

    // ── Alarm overlay ─────────────────────────────────────────────────────────
    void drawAlarmOverlay() {
        // 50% red wash
        canvas.fillRect(0, TOP, W, NAV_Y - TOP, 0x8000);

        int pw = 200, ph = 120;
        int cx = W / 2, cy = H / 2 + 10;
        fillR(cx - pw / 2, cy - ph / 2, pw, ph, 8, 0x2800);
        strokeR(cx - pw / 2, cy - ph / 2, pw, ph, 8, RED);
        f2("!  ALARM", cx, cy - ph / 2 + 20, RED);
        f2("Release e-stop to unlock", cx, cy - ph / 2 + 40, COL_WHITE2);

        int ubw = 160, ubh = 44;
        int ubx = cx - ubw / 2, uby = cy - ubh / 2 + 12;
        _unlockBtn = { ubx, uby, ubw, ubh };
        tintStrokeR(ubx, uby, ubw, ubh, 6, RED, RED, 60);
        f2("UNLOCK  ($X)", cx, uby + ubh / 2, COL_WHITE);
    }

public:
    TabScene() : Scene("TabUI", 4) {}  // encoder_scale=4: X4 PCNT quadrature, 1 detent = 1 step

    void onEntry(void* arg = nullptr) override {
        sprite_offset = { 0, 0 };
        termLines.clear();
        termLines.push_back({ "> $I",              COL_DIM2 });
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

        // Jog when axis selected
        if (mpgAxis >= 0) {
            static const char axChar[] = { 'X', 'Y', 'Z', 'A' };
            float dist = delta * mpgSteps[mpgStepIdx];
            static const int jogFeed[] = { 100, 500, 2000 };
            send_linef("$J=G91 %c%.3f F%d", axChar[mpgAxis], dist, jogFeed[mpgStepIdx]);
            reDisplay();
            return;
        }

        // Axis=off: scroll active tab + always update header arrow
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
            int maxL      = (outH - 4) / OUT_LH;
            int maxScroll = std::max(0, (int)termLines.size() - maxL);
            termScroll    = std::min(termScroll, maxScroll);
        } else if (_tab == 4) {
            macroScroll = std::max(0, macroScroll + delta);
        }
        reDisplay();  // always redraw so header arrow is immediate on any tab
    }

    void onTouchClick() override {
        int x = touchX, y = touchY;

        // Alarm overlay — intercepts all touches
        if (_alarmOpen && state == Alarm) {
            if (hit(_unlockBtn, x, y)) {
                send_line("$X");
                termLines.push_back({ "> $X",          COL_DIM2   });
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
                        termLines.push_back({ std::string("> ") + PROBE_OPTS[i].cmd, COL_DIM2 });
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
            if (hit(_feedMinus, x, y)) {
                fnc_realtime(FeedOvrCoarseMinus);
                reDisplay(); return;
            }
            if (hit(_feedPlus, x, y)) {
                fnc_realtime(FeedOvrCoarsePlus);
                reDisplay(); return;
            }
            // Viz tabs
            int tabH = VIZ_H / 3;
            if (x >= TAB_X - 18 && x <= W && y >= VIZ_Y && y < NAV_Y - FEED_H) {
                int ti = (y - VIZ_Y) / tabH;
                if (ti >= 0 && ti < 3) { _vizView = ti; reDisplay(); return; }
            }
        }

        // Homing screen
        if (_tab == 1) {
            if (hit(_homeAllBtn, x, y)) {
                send_line("$H");
                termLines.push_back({ "> $H", COL_DIM2 });
                reDisplay(); return;
            }
            if (hit(_probeBtnR, x, y)) {
                _probeOpen = true; reDisplay(); return;
            }
            const char* axcmds[] = { "$HX", "$HY", "$HZ" };
            for (int i = 0; i < 3; i++) {
                if (hit(_axisHomeBtns[i], x, y)) {
                    send_line(axcmds[i]);
                    termLines.push_back({ std::string("> ") + axcmds[i], COL_DIM2 });
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
                    termLines.push_back({ std::string("> ") + zcmds[k], COL_DIM2 });
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
                        float _cx=0, _cy=0; bool _abs=true;
                        for (auto& ln : previewLines) {
                            const char* p = ln.c_str();
                            while (*p==' ') p++;
                            if (!*p||*p==';'||*p=='(') continue;
                            if ((*p=='G'||*p=='g')&&p[1]=='9') { if(p[2]=='0')_abs=true; if(p[2]=='1')_abs=false; }
                            float nx=_cx, ny=_cy; bool mv=false;
                            for (const char* q=p; *q; q++) {
                                if ((*q=='X'||*q=='x')&&(q==p||*(q-1)==' '||*(q-1)=='	')) { nx=_abs?strtof(q+1,nullptr):_cx+strtof(q+1,nullptr); mv=true; }
                                if ((*q=='Y'||*q=='y')&&(q==p||*(q-1)==' '||*(q-1)=='	')) { ny=_abs?strtof(q+1,nullptr):_cy+strtof(q+1,nullptr); mv=true; }
                            }
                            if (mv&&(nx!=_cx||ny!=_cy)) { simPath.push_back({nx,ny}); _cx=nx; _cy=ny; }
                        }
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
                    send_line(QUICK_CMDS[ci]);
                    termLines.push_back({ std::string("> ") + QUICK_CMDS[ci], COL_DIM2 });
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
                        termLines.push_back({ std::string("> ") + MACROS[mi].cmd, COL_DIM2 });
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
        if (_tab == 3) { termScroll = std::max(0, termScroll - 3); reDisplay(); }
        if (_tab == 4) { macroScroll = std::max(0, macroScroll - 3); reDisplay(); }
    }
    void onDownFlick() override {
        if (_tab == 2 && filePreviewMode) { previewScroll += 5; reDisplay(); }
        if (_tab == 3) {
            termScroll += 3;
            int cmdY  = NAV_Y - CMD_H;
            int outH  = cmdY - TOP - 4;
            int maxL  = (outH - 4) / OUT_LH;
            int maxSc = std::max(0, (int)termLines.size() - maxL);
            termScroll = std::min(termScroll, maxSc);
            reDisplay();
        }
        if (_tab == 4) { macroScroll += 3; reDisplay(); }
    }

    void reDisplay() override {
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
        if (_alarmOpen && state == Alarm) drawAlarmOverlay();
        drawNav();
        refreshDisplay();
    }
};

static TabScene tabScene;

Scene* getTabScene() { return &tabScene; }
