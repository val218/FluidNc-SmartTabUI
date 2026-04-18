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
static const int VIZ_H   = NAV_Y - FEED_H - VIZ_Y - 14; // 14px for filename+bar strip below
static const int TAB_X   = W;               // tabs gone — pushed off screen
static const int DRO_ROW = (NAV_Y - TOP - FEED_H) / 4;
static const int N_TABS  = 5;
static const int CMD_H   = 54;
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
static int _homeScroll  = 0;  // home screen scroll offset

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
static bool _showMPos    = false;  // false=WPos true=MPos
static bool _inInchMode  = false;  // display inch instead of mm
static uint32_t _jobStartTime = 0; // millis when job started
static uint32_t _jobElapsed   = 0; // accumulated elapsed ms
static int  _currentTab  = 0;
static int  _workX       = 1250;  // machine work area short axis (mm)
static int  _workY       = 2500;  // machine work area long axis (mm)
static int  _homeCorner  = 0;     // 0=BL 1=BR 2=TL 3=TR
static volatile bool mpgEstopActive = false;  // e-stop currently pressed
volatile bool _forceAlarm = false;  // set by mpgTask when e-stop pressed
static volatile bool _mpgChanged   = false; // set by Core0 readMpgSwitches, consumed on Core1

static int  _droAxesMode = 0;  // 0=XYZ 1=XYZA 2=XY 3=XYYZ (dual Y)
static char _axisLetters[6];   // axis letters from FluidNC: X,Y,Z,A,B,C
static bool _axisAutoDetected; // true when letters received from FluidNC

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
void setSimMode(bool sim) { (void)sim; }  // simulation removed
bool getSimMode()         { return false; }  // simulation removed

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
    // Use markDirty for cross-core safety
    if (prevAxis < 0 && mpgAxis >= 0) {
        pcnt_counter_clear(PCNT_UNIT_0);
    }

    if (mpgAxis != prevAxis || mpgStepIdx != prevStep) {
        _mpgChanged = true;  // picked up by dispatch_events on Core 1
    }
}
static const char* QUICK_CMDS[] = { "$H", "$?", "!", "~", "$X" };
static const char* SPINDLE_CMDS[] = { "M3 S10000", "M4 S10000", "M5", "M8", "M9" };
static const char* SPINDLE_LBLS[] = { "CW", "CCW", "Stop", "Flood", "M-off" };
static const int   N_SPINDLE = 5;
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
static bool _jobSentToFluidNC = false;  // real job was sent, expect Cycle state
static std::string simJobName;             // name of file being sim-run
static std::vector<std::pair<float,float>> simPath;  // XY path parsed from gcode for viz
static int  simPathIdx       = 0;          // current position along path (animation)
static std::vector<std::string> previewLines; // loaded gcode lines
static std::vector<std::pair<float,float>> vizPath;   // path for DRO viz overlay
static std::string vizJobName;              // filename shown on DRO viz
static int  vizPathExecuted  = 0;           // segments drawn as "executed" (green)
static int  previewScroll   = 0;           // first visible preview line
static int  previewFirstLine = 0;           // line offset in file
static std::vector<std::string> allFileLines;  // accumulated lines across batches
static std::string _loadingPath;            // path being batch-loaded
static int  _loadingBatch = 0;              // next batch start line
static bool _loadingDone  = false;          // all batches received

// ── LittleFS viz path cache ───────────────────────────────────────────────────
// Key: /viz/XXXXXXXX.bin where X = CRC32 of (name + size)
// Format: uint32 count, then count * (float x, float y)
static bool _vizCacheReady = false;

static uint32_t vizCacheKey(const std::string& name, int size) {
    uint32_t crc = 0xFFFFFFFF;
    for (char c : name)  { crc ^= (uint8_t)c; for(int j=0;j<8;j++) crc=(crc>>1)^(0xEDB88320&-(crc&1)); }
    crc ^= (uint32_t)size;            for(int j=0;j<8;j++) crc=(crc>>1)^(0xEDB88320&-(crc&1));
    return crc ^ 0xFFFFFFFF;
}

static bool vizCacheLoad(const std::string& name, int fsize,
                          std::vector<std::pair<float,float>>& out) {
    if (!_vizCacheReady) return false;
    char path[32]; snprintf(path,sizeof(path),"/viz/%08X.bin",vizCacheKey(name,fsize));
    File f = LittleFS.open(path,"r"); if(!f) return false;
    uint32_t cnt=0; f.read((uint8_t*)&cnt,4);
    if(cnt==0||cnt>20000){f.close();return false;}
    out.clear(); out.reserve(cnt);
    for(uint32_t i=0;i<cnt;i++){
        float x,y; f.read((uint8_t*)&x,4); f.read((uint8_t*)&y,4);
        out.push_back({x,y});
    }
    f.close(); return true;
}

static void vizCacheSave(const std::string& name, int fsize,
                          const std::vector<std::pair<float,float>>& pts) {
    if (!_vizCacheReady || pts.empty()) return;
    LittleFS.mkdir("/viz");
    char path[32]; snprintf(path,sizeof(path),"/viz/%08X.bin",vizCacheKey(name,fsize));
    File f = LittleFS.open(path,"w"); if(!f) return;
    uint32_t cnt=pts.size(); f.write((uint8_t*)&cnt,4);
    for(auto& p:pts){ f.write((uint8_t*)&p.first,4); f.write((uint8_t*)&p.second,4); }
    f.close();
}

static void vizCacheInit() {
    _vizCacheReady = LittleFS.begin(false);
    if(!_vizCacheReady) _vizCacheReady = LittleFS.begin(true); // format if needed
}

// Evict old cache entries if LittleFS >80% full
static void vizCacheEvictOld() {
    if(!_vizCacheReady) return;
    size_t used=LittleFS.usedBytes(), total=LittleFS.totalBytes();
    if(used < total*8/10) return; // <80% full, ok
    // Delete oldest file in /viz
    File dir=LittleFS.open("/viz"); if(!dir) return;
    File oldest; uint32_t oldTime=UINT32_MAX;
    File entry=dir.openNextFile();
    while(entry){ if(!entry.isDirectory()&&entry.getLastWrite()<oldTime){oldTime=entry.getLastWrite();oldest=entry;} entry=dir.openNextFile(); }
    if(oldest) LittleFS.remove(oldest.path());
}
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
    Rect _holdBtn, _abortBtn;
    Rect _vizTabs[3];
    Rect _feedMinus, _feedPlus;
    Rect _spndMinus, _spndPlus, _spndPill, _spdPill;
    Rect _homeAllBtn, _probeBtnR;
    Rect _axisHomeBtns[3];
    Rect _zeroWcsBtns[4];
    int  _homePressedId = -1;  // which home button was last pressed (-1=none)
    uint32_t _homePressTime = 0;  // when it was pressed
    bool _previewShowPath = false;  // false=gcode text, true=path viz
    bool _spindleMenuOpen = false;  // spindle sub-menu open in macros tab
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

        // ── RIGHT: EN ◄► TX RX — EN leftmost, arrows next to it right side ──
        // Slots: EN=W-74, ◄►=W-56..W-40, TX=W-34, RX=W-18
        {
            static uint32_t lastTx=0,lastRx=0,txFlash=0,rxFlash=0;
            uint32_t now2 = millis();
            bool txAct=(fnc_tx_count!=lastTx); if(txAct){lastTx=fnc_tx_count;txFlash=now2;}
            bool rxAct=(fnc_rx_count!=lastRx); if(rxAct){lastRx=fnc_rx_count;rxFlash=now2;}
            // Flash: bright for 120ms then off — blink effect
            bool txOn=((now2-txFlash)<120), rxOn=((now2-rxFlash)<120);

            // EN — leftmost of the right cluster
            hdrTxt("EN", W-74, 10, mpgEnable?GREEN:COL_DIM, middle_right);

            // ◄► direction arrows — immediately right of EN
            {
                static const int axCols[] = { COL_AX_X, COL_AX_Y, COL_AX_Z, COL_AX_A };
                int ac = (mpgAxis >= 0) ? axCols[mpgAxis] : COL_WHITE2;
                bool active = (mpgLastDir != 0 && (now2 - mpgDirTime) < 300);
                auto drawTri = [&](int cx2, bool pr, int col) {
                    for (int r=0;r<7;r++){
                        int h=(r<4)?r:(6-r);
                        int lx=pr?(cx2-3):(cx2+3-h), rx2=pr?(cx2-3+h):(cx2+3);
                        if(rx2>lx) canvas.drawFastHLine(lx,3+r,rx2-lx+1,col);
                    }
                };
                canvas.fillRect(W-56, 2, 16, 16, COL_PANEL);
                drawTri(W-49, false, active&&mpgLastDir<0 ? ac : COL_DIM);
                drawTri(W-42, true,  active&&mpgLastDir>0 ? ac : COL_DIM);
            }

            // TX — flashes orange when data sent
            canvas.setFont(&fonts::Font0); canvas.setTextDatum(middle_right);
            if(txOn){ canvas.fillRoundRect(W-36,3,16,14,2,0x3200); }
            canvas.setTextColor(txOn?ORANGE:COL_DIM);
            canvas.drawString("TX", W-28, 10);

            // RX — flashes green when data received
            if(rxOn){ canvas.fillRoundRect(W-20,3,18,14,2,0x0320); }
            canvas.setTextColor(rxOn?GREEN:COL_DIM);
            canvas.drawString("RX", W-10, 10);
        }

        // ── CENTRE: tab name or JOG label ────────────────────────────────────
        if (mpgEnable && mpgAxis >= 0) {
            static const int axCols[] = { COL_AX_X, COL_AX_Y, COL_AX_Z, COL_AX_A };
            static const char axNames[] = { 'X', 'Y', 'Z', 'A' };
            int ac = axCols[mpgAxis];
            // Y2 label for XYYZ mode
            const char ax4 = (_droAxesMode==3 && mpgAxis==3) ? '2' : axNames[mpgAxis];
            char axPfx[4] = {(_droAxesMode==3 && mpgAxis==3)?'Y':ax4, 0};
            if(_droAxesMode==3 && mpgAxis==3) { axPfx[0]='Y'; axPfx[1]='2'; axPfx[2]=0; }
            char jl[16]; snprintf(jl, sizeof(jl), "JOG %s %smm", axPfx, mpgStepLabels[(int)mpgStepIdx]);
            canvas.fillRect(W/2-44, 2, 88, 16, 0x0000);
            strokeR(W/2-44, 2, 88, 16, 3, ac);
            hdrTxt(jl, W/2, 10, ac);
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

        // When job running on DRO tab: show Hold and Abort instead of tab bar
        bool jobActive = (state == Cycle || (state == Hold && _jobSentToFluidNC) || simJobRunning);
        if (jobActive && _tab == 0) {
            int half = W / 2;
            bool inHold = (state == Hold);

            // Hold / Resume button (left half)
            _holdBtn = { 0, NAV_Y, half, BOT };
            int hCol = inHold ? GREEN : YELLOW;
            tintStrokeR(2, NAV_Y+3, half-4, BOT-6, 4, hCol, hCol, 40);
            navTxt(inHold ? "Resume (~)" : "Hold (!)", half/2, NAV_Y+BOT/2, hCol);

            // Abort button (right half)
            _abortBtn = { half, NAV_Y, half, BOT };
            tintStrokeR(half+2, NAV_Y+3, half-4, BOT-6, 4, RED, RED, 40);
            navTxt("Abort (x)", half + half/2, NAV_Y+BOT/2, RED);

            // Invalidate nav tabs so normal tab touch doesn't fire
            for (int i = 0; i < N_TABS; i++) _navTabs[i] = {0,0,0,0};
            return;
        }

        // Normal tab bar
        _holdBtn = {0,0,0,0}; _abortBtn = {0,0,0,0};
        int tw = W / N_TABS;
        for (int i = 0; i < N_TABS; i++) {
            int x = i * tw;
            int w = (i == N_TABS-1) ? W - x : tw;  // last tab fills to edge
            _navTabs[i] = { x, NAV_Y, w, BOT };
            if (i == _tab) {
                canvas.fillRect(x, NAV_Y, w, BOT, 0x0019);
                canvas.fillRect(x, NAV_Y, w, 3, BLUE);
            }
            if (i > 0 && i < N_TABS) vline(x, NAV_Y + 5, BOT - 10, COL_BORDER);
            int col = (i == _tab) ? COL_WHITE : COL_WHITE2;
            navTxt(TAB_LABELS[i], x + w/2, NAV_Y + BOT/2, col);
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
            const char* posStr = pos_to_cstr(myAxes[i], ndig);
            txt(posStr, DROW - 5, midY,
                isMpg ? COL_WHITE : COL_WHITE2,
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

        // DRO mode pills: MPos/WPos and mm/in toggles
        { int px=DROW+2, py=TOP+2, pw=36, ph=10, g=3;
          // MPos/WPos toggle
          tintStrokeR(px, py, pw, ph, 2, _showMPos?COL_BORDER:0x0019, _showMPos?COL_DIM:CYAN, 30);
          canvas.setFont(&fonts::Font0); canvas.setTextDatum(middle_center);
          canvas.setTextColor(_showMPos?COL_DIM:CYAN);
          canvas.drawString("WPos", px+pw/2, py+ph/2);
          px+=pw+g;
          tintStrokeR(px, py, pw, ph, 2, _showMPos?0x0019:COL_BORDER, _showMPos?CYAN:COL_DIM, 30);
          canvas.setTextColor(_showMPos?CYAN:COL_DIM);
          canvas.drawString("MPos", px+pw/2, py+ph/2);
          px+=pw+g+4;
          // mm/in toggle
          tintStrokeR(px, py, 24, ph, 2, inInches?COL_BORDER:0x0019, inInches?COL_DIM:YELLOW, 30);
          canvas.setTextColor(inInches?COL_DIM:YELLOW);
          canvas.drawString("mm", px+12, py+ph/2);
          px+=27;
          tintStrokeR(px, py, 24, ph, 2, inInches?0x0019:COL_BORDER, inInches?YELLOW:COL_DIM, 30);
          canvas.setTextColor(inInches?YELLOW:COL_DIM);
          canvas.drawString("in", px+12, py+ph/2);
        }

        // Job timer in viz area top-right
        if (_jobStartTime > 0 || _jobElapsed > 0) {
          uint32_t elapsed = _jobElapsed + (state==Cycle ? millis()-_jobStartTime : 0);
          uint32_t s2=elapsed/1000, m=s2/60, h=m/60;
          char tstr[12];
          if(h>0) snprintf(tstr,sizeof(tstr),"%02lu:%02lu:%02lu",h,m%60,s2%60);
          else    snprintf(tstr,sizeof(tstr),"%02lu:%02lu",m,s2%60);
          canvas.setFont(&fonts::Font0); canvas.setTextDatum(middle_right);
          canvas.setTextColor(state==Cycle?CYAN:COL_DIM2);
          canvas.drawString(tstr, VIZ_X+VIZ_W-2, TOP+7);
        }




        // ── Visualizer: shows path (auto-scaled) OR work area map ────────────
        canvas.fillRect(VIZ_X, VIZ_Y, VIZ_W, VIZ_H, COL_PANEL2);

        if (!vizPath.empty()) {
            // ── MODE A: Auto-scaled path view ────────────────────────────────
            // Path fills the viz area; work area grid not shown (different scale)
            int pad3v=6, mapWv=VIZ_W-2*pad3v, mapHv=VIZ_H-2*pad3v;

            // Bounding box of path in machine coords
            float pMinX=vizPath[0].first, pMaxX=pMinX;
            float pMinY=vizPath[0].second, pMaxY=pMinY;
            for (auto& pt : vizPath) {
                pMinX=std::min(pMinX,pt.first); pMaxX=std::max(pMaxX,pt.first);
                pMinY=std::min(pMinY,pt.second); pMaxY=std::max(pMaxY,pt.second);
            }
            float rangeX=std::max(pMaxX-pMinX, 1.0f);
            float rangeY=std::max(pMaxY-pMinY, 1.0f);

            // Scale to fill viz with 10% padding
            float scvX=(float)mapWv / (rangeY*1.2f);
            float scvY=(float)mapHv / (rangeX*1.2f);
            float scv=std::min(scvX,scvY);

            float pathScreenW=rangeY*scv, pathScreenH=rangeX*scv;
            int offXv=VIZ_X+pad3v+(int)((mapWv-pathScreenW)/2.0f);
            int offYv=VIZ_Y+pad3v+(int)((mapHv-pathScreenH)/2.0f);

            // Path background
            canvas.fillRect(offXv, offYv, (int)pathScreenW, (int)pathScreenH, COL_PANEL3);
            canvas.drawRect(offXv, offYv, (int)pathScreenW, (int)pathScreenH, COL_BORDER2);

            // Map machine coords → screen (landscape: Y→X, X→Y inverted)
            auto pathToScreen = [&](float mx, float my, int& sx, int& sy) {
                sx = offXv + (int)((my - pMinY) * scv);
                sy = offYv + (int)(pathScreenH - (mx - pMinX) * scv);
            };

            // Update executed count
            if (myPercent > 0) vizPathExecuted = (int)vizPath.size() * (int)myPercent / 100;
            int executed = std::min(vizPathExecuted, (int)vizPath.size()-1);

            // Draw path lines: dim=pending, cyan=done
            // Limit to 2000 segments to prevent freeze on large files
            int vizStep = std::max(1, (int)vizPath.size() / 2000);
            for (int pi=1; pi<(int)vizPath.size(); pi+=vizStep) {
                int x1,y1,x2,y2;
                pathToScreen(vizPath[pi-1].first,vizPath[pi-1].second,x1,y1);
                pathToScreen(vizPath[pi].first,  vizPath[pi].second,  x2,y2);
                canvas.drawLine(x1,y1,x2,y2, pi<=executed ? CYAN : COL_DIM);
            }

            // Current position marker on path
            if (executed>0 && executed<(int)vizPath.size()) {
                int dx,dy; pathToScreen(vizPath[executed].first,vizPath[executed].second,dx,dy);
                canvas.fillCircle(dx,dy,2,GREEN);
            }

            // Real machine position dot — mapped to same auto-scale
            float machX=(float)myAxes[0]/10000.0f;
            float machY=(float)myAxes[1]/10000.0f;
            int dotX,dotY; pathToScreen(machX,machY,dotX,dotY);
            // Only draw if within the scaled view
            if (dotX>=offXv && dotX<offXv+(int)pathScreenW && dotY>=offYv && dotY<offYv+(int)pathScreenH) {
                canvas.fillCircle(dotX,dotY,2,ORANGE);
                canvas.drawCircle(dotX,dotY,2,COL_WHITE);
            }

            // Bounding box label
            canvas.setFont(&fonts::Font0); canvas.setTextColor(COL_DIM);
            canvas.setTextDatum(bottom_right);
            char pdim[24]; snprintf(pdim,sizeof(pdim),"%.0fx%.0fmm",rangeY,rangeX);
            canvas.drawString(pdim,offXv+(int)pathScreenW-1,offYv+(int)pathScreenH-1);

            // Clear [×] button — top-right corner of viz area
            int xbx=VIZ_X+VIZ_W-16, xby=VIZ_Y+1, xbw=15, xbh=13;
            canvas.fillRoundRect(xbx,xby,xbw,xbh,2,0x4000);
            canvas.drawRoundRect(xbx,xby,xbw,xbh,2,COL_BORDER2);
            canvas.setFont(&fonts::Font0); canvas.setTextDatum(middle_center);
            canvas.setTextColor(COL_DIM2);
            canvas.drawString("x",xbx+xbw/2,xby+xbh/2);

        } else {
            // ── MODE B: Work area map with real-time position ────────────────
            int pad3=2, mapW=VIZ_W-2*pad3, mapH=VIZ_H-2*pad3;
            int mapX=VIZ_X+pad3, mapY=VIZ_Y+pad3;
            float scH=(float)mapW/_workY, scV=(float)mapH/_workX;
            float sc=std::min(scH,scV);
            int drawnW=(int)(_workY*sc), drawnH=(int)(_workX*sc);
            int offX=mapX+(mapW-drawnW)/2, offY=mapY+(mapH-drawnH)/2;

            canvas.fillRect(offX,offY,drawnW,drawnH,COL_PANEL3);
            canvas.drawRect(offX,offY,drawnW,drawnH,COL_BORDER2);
            for(int gi=1;gi<4;gi++){
                canvas.drawFastVLine(offX+drawnW*gi/4,offY,drawnH,COL_BORDER);
                canvas.drawFastHLine(offX,offY+drawnH*gi/4,drawnW,COL_BORDER);
            }

            // Home corner dot + XY arrows
            int hx=(_homeCorner==1||_homeCorner==3)?offX+drawnW:offX;
            int hy=(_homeCorner==0||_homeCorner==1)?offY+drawnH:offY;
            canvas.fillCircle(hx,hy,2,GREEN);
            int xDir=(_homeCorner==0||_homeCorner==1)?-1:1;
            int yDir=(_homeCorner==1||_homeCorner==3)?-1:1;
            int al=16;
            int yax0=(yDir>0)?hx:hx+yDir*al;
            canvas.drawFastHLine(yax0,hy,  al,GREEN);
            canvas.drawFastHLine(yax0,hy+1,al,GREEN);
            canvas.setFont(&fonts::Font0); canvas.setTextDatum(middle_center);
            canvas.setTextColor(GREEN);
            canvas.drawString("Y",hx+yDir*(al+5),hy);
            int xax0=(xDir>0)?hy:hy+xDir*al;
            canvas.drawFastVLine(hx,  xax0,al,RED);
            canvas.drawFastVLine(hx+1,xax0,al,RED);
            canvas.setTextColor(RED);
            canvas.drawString("X",hx,hy+xDir*(al+5));

            // Real-time position dot (machine scale)
            float machX=(float)myAxes[0]/10000.0f;
            float machY=(float)myAxes[1]/10000.0f;
            machX=std::max(0.0f,std::min(machX,(float)_workX));
            machY=std::max(0.0f,std::min(machY,(float)_workY));
            int dotX,dotY;
            if(_homeCorner==0){dotX=offX+(int)(machY*sc);dotY=offY+drawnH-(int)(machX*sc);}
            else if(_homeCorner==1){dotX=offX+drawnW-(int)(machY*sc);dotY=offY+drawnH-(int)(machX*sc);}
            else if(_homeCorner==2){dotX=offX+(int)(machY*sc);dotY=offY+(int)(machX*sc);}
            else{dotX=offX+drawnW-(int)(machY*sc);dotY=offY+(int)(machX*sc);}
            canvas.fillCircle(dotX,dotY,2,ORANGE);
            canvas.drawCircle(dotX,dotY,2,COL_WHITE);

            canvas.setFont(&fonts::Font0); canvas.setTextColor(COL_DIM);
            canvas.setTextDatum(bottom_right);
            char wdim[24]; snprintf(wdim,sizeof(wdim),"%dx%dmm",_workY,_workX);
            canvas.drawString(wdim,offX+drawnW-1,offY+drawnH-1);
        }

        // ── G-code info strip: filename above, progress bar below ─────────────
        if (!vizJobName.empty()) {
            int stripY = VIZ_Y + VIZ_H + 1;  // 1px gap below viz area
            int nameH  = 8;   // filename row height
            int barH   = 5;   // progress bar height
            // Background
            canvas.fillRect(VIZ_X, stripY, VIZ_W, nameH + barH, COL_PANEL2);

            // Filename — scrolling, above the bar
            static int _ns=0; static uint32_t _nt=0;
            if (millis()-_nt>350){_nt=millis();_ns++;}
            std::string dn=vizJobName;
            int maxChars = (VIZ_W - 4) / 6;  // Font0 is ~6px wide
            if ((int)dn.size() > maxChars) {
                int p = _ns % (int)(dn.size() + 4);
                std::string pd = dn + "    " + dn;
                dn = pd.substr(p, maxChars);
            }
            canvas.setFont(&fonts::Font0);
            canvas.setTextDatum(middle_left);
            canvas.setTextColor(CYAN);
            canvas.drawString(dn.c_str(), VIZ_X + 2, stripY + nameH/2);

            // Progress bar — below filename
            int pbPct = 0;
            if (simJobRunning && !vizPath.empty())
                pbPct = vizPathExecuted * 100 / std::max(1, (int)vizPath.size()-1);
            else if (myPercent > 0)
                pbPct = (int)myPercent;
            int pbY = stripY + nameH;
            canvas.fillRect(VIZ_X, pbY, VIZ_W, barH, COL_BORDER);
            int pbFill = VIZ_W * pbPct / 100;
            if (pbFill > 0) canvas.fillRect(VIZ_X, pbY, pbFill, barH, CYAN);
            // Percentage right-aligned on filename row
            char pctS[8]; snprintf(pctS,sizeof(pctS),"%d%%",pbPct);
            canvas.setTextDatum(middle_right);
            canvas.setTextColor(pbPct>0 ? CYAN : COL_DIM);
            canvas.drawString(pctS, VIZ_X+VIZ_W-1, stripY + nameH/2);
        }

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

    // ── Probe screen ─────────────────────────────────────────────────────────
    // WCS selector, probing operations, tool length
    int _wcsNum = 1;  // active WCS: 1=G54..6=G59
    int _probeStep = 0;  // 0=idle, 1-N=wizard step
    struct ProbeWizard { const char* title; const char* desc; const char* cmd; } ;

    void drawProbeScreen() {
        int pad=8, gap=6;
        int W2=W, y=TOP+4;

        // ── WCS selector G54–G59 ─────────────────────────────────────────────
        canvas.setFont(&fonts::Font0);
        canvas.setTextDatum(middle_left);
        canvas.setTextColor(COL_DIM);
        canvas.drawString("WCS:", pad, y+8);
        const char* wcsNames[]={"G54","G55","G56","G57","G58","G59"};
        int bw=(W2-pad*2-4*5)/6;
        for(int i=0;i<6;i++){
            bool sel=(i==_wcsNum-1);
            int bx=pad+(bw+5)*i;
            if(sel){canvas.fillRoundRect(bx,y,bw,16,3,0x0019);canvas.drawRoundRect(bx,y,bw,16,3,CYAN);}
            else tintStrokeR(bx,y,bw,16,3,COL_BORDER,COL_BORDER,30);
            canvas.setTextDatum(middle_center);
            canvas.setTextColor(sel?CYAN:COL_DIM2);
            canvas.drawString(wcsNames[i],bx+bw/2,y+8);
        }
        y+=22; hline(0,y,W2,COL_BORDER); y+=4;

        // ── Axis zero buttons ────────────────────────────────────────────────
        canvas.setTextDatum(middle_left); canvas.setTextColor(COL_DIM);
        canvas.setFont(&fonts::Font0);
        canvas.drawString("Zero:", pad, y+10);
        const char* axNames[]={"X","Y","Z","All"};
        int zbw=(W2-pad*2-3*gap)/4;
        for(int i=0;i<4;i++){
            int bx=pad+(zbw+gap)*i;
            tintStrokeR(bx,y,zbw,20,3,COL_BORDER2,COL_BORDER,40);
            canvas.setTextDatum(middle_center);
            int pc2=(i==0)?COL_AX_X:(i==1)?COL_AX_Y:(i==2)?COL_AX_Z:ORANGE;
            canvas.setTextColor(pc2);
            canvas.drawString(axNames[i],bx+zbw/2,y+10);
        }
        y+=26; hline(0,y,W2,COL_BORDER); y+=4;

        // ── Probing operations ───────────────────────────────────────────────
        struct { const char* label; const char* sub; int col; } ops[]={
            {"Tool Length", "Touch-off Z to surface",   YELLOW  },
            {"Z Surface",   "Probe workpiece top",       CYAN    },
            {"X Edge",      "Find X axis edge",          COL_AX_X},
            {"Y Edge",      "Find Y axis edge",          COL_AX_Y},
            {"XY Corner",   "Find corner of workpiece",  ORANGE  },
            {"XY Center",   "Center of hole/boss",       GREEN   },
        };
        int nOps=6, opH=22, opW=W2-2*pad;
        for(int i=0;i<nOps;i++){
            int oy=y+i*(opH+3);
            if(oy+opH>NAV_Y-4) break;
            canvas.fillRoundRect(pad,oy,opW,opH,3,COL_PANEL2);
            canvas.drawRoundRect(pad,oy,opW,opH,3,COL_BORDER);
            canvas.fillRect(pad,oy+2,4,opH-4,ops[i].col);
            canvas.setFont(&fonts::Font0); canvas.setTextDatum(middle_left);
            canvas.setTextColor(COL_WHITE);
            canvas.drawString(ops[i].label,pad+10,oy+opH*36/100);
            canvas.setTextColor(COL_DIM2);
            canvas.drawString(ops[i].sub,pad+10,oy+opH*72/100);
        }
    }

    // ── Homing screen ────────────────────────────────────────────────────────
    void drawHomingScreen() {
        int pad=8, gap=4, bw=W-2*pad;
        bool fl=(millis()-_homePressTime)<300;
        int btnH=32, secLH=13, secGap=6;
        int secH=secLH+btnH+secGap;
        int totalH=4*secH;
        int avH=NAV_Y-TOP-4;

        _homeScroll=std::max(0,std::min(_homeScroll,std::max(0,totalH-avH)));

        // Scroll indicator
        if(totalH>avH){
            int thumbH=std::max(10,avH*avH/totalH);
            int thumbY=TOP+2+(avH-thumbH)*_homeScroll/std::max(1,totalH-avH);
            canvas.fillRect(W-3,TOP+2,3,avH,COL_BORDER);
            canvas.fillRect(W-3,thumbY,3,thumbH,COL_DIM2);
        }

        int vy=TOP+2-_homeScroll;
        int axcols[3]={COL_AX_X,COL_AX_Y,COL_AX_Z};
        const char* axlet[3]={"X","Y","Z"};

        auto secLabel=[&](const char* lbl){
            if(vy>TOP-secLH && vy<NAV_Y){
                canvas.setFont(&fonts::Font0); canvas.setTextColor(COL_DIM);
                canvas.setTextDatum(middle_left);
                canvas.drawString(lbl,pad,vy+secLH/2);
            }
            vy+=secLH;
        };
        auto visible=[&](){ return vy+btnH>TOP && vy<NAV_Y; };

        // ── Home All + Probe ──────────────────────────────────────────────
        secLabel("Homing");
        if(visible()){
            int homeW=bw*58/100, probW=bw-homeW-gap;
            _homeAllBtn={pad,vy,homeW,btnH};
            _probeBtnR={pad+homeW+gap,vy,probW,btnH};
            if(fl&&_homePressedId==0){canvas.fillRoundRect(pad,vy,homeW,btnH,4,COL_WHITE2);canvas.setTextColor(COL_BG);}
            else{tintStrokeR(pad,vy,homeW,btnH,4,COL_WHITE2,COL_WHITE,50);canvas.setTextColor(COL_WHITE);}
            canvas.setFont(&fonts::Font2);canvas.setTextDatum(middle_center);
            canvas.drawString("Home All ($H)",pad+homeW/2,vy+btnH/2);
            if(fl&&_homePressedId==1){canvas.fillRoundRect(pad+homeW+gap,vy,probW,btnH,4,ORANGE);canvas.setTextColor(COL_BG);}
            else{tintStrokeR(pad+homeW+gap,vy,probW,btnH,4,ORANGE,ORANGE,40);canvas.setTextColor(ORANGE);}
            canvas.setFont(&fonts::Font2);canvas.setTextDatum(middle_center);
            canvas.drawString("Probe",pad+homeW+gap+probW/2,vy+btnH/2);
        } else {_homeAllBtn={0,0,0,0};_probeBtnR={0,0,0,0};}
        vy+=btnH+secGap;

        // ── Home individual axis ──────────────────────────────────────────
        secLabel("Home Axis");
        if(visible()){
            int aw=(bw-2*gap)/3;
            for(int i=0;i<3;i++){
                int hx=pad+i*(aw+gap);
                _axisHomeBtns[i]={hx,vy,aw,btnH};
                bool ap=(fl&&_homePressedId==10+i);
                if(ap) canvas.fillRoundRect(hx,vy,aw,btnH,3,axcols[i]);
                else{canvas.fillRoundRect(hx,vy,aw,btnH,3,COL_PANEL2);canvas.drawRoundRect(hx,vy,aw,btnH,3,axcols[i]);}
                canvas.fillRect(hx+1,vy+2,3,btnH-4,axcols[i]);
                canvas.setFont(&fonts::Font2);canvas.setTextDatum(middle_center);
                canvas.setTextColor(ap?COL_BG:axcols[i]);
                canvas.drawString(axlet[i],hx+aw/2,vy+btnH/2);
            }
        } else{for(int i=0;i<3;i++) _axisHomeBtns[i]={0,0,0,0};}
        vy+=btnH+secGap;

        // ── Endstop status ────────────────────────────────────────────────
        secLabel("Endstops");
        if(visible()){
            int esW=(bw-2*gap)/3;
            for(int i=0;i<3;i++){
                int ex=pad+i*(esW+gap);
                bool trig=myLimitSwitches[i];
                canvas.fillRoundRect(ex,vy,esW,btnH,3,trig?0x6000:COL_PANEL2);
                canvas.drawRoundRect(ex,vy,esW,btnH,3,trig?RED:COL_BORDER2);
                canvas.fillCircle(ex+10,vy+btnH/2,4,trig?RED:COL_DIM2);
                canvas.setFont(&fonts::Font2);canvas.setTextDatum(middle_left);
                canvas.setTextColor(COL_WHITE);
                canvas.drawString(axlet[i],ex+20,vy+btnH/2);
                canvas.setTextDatum(middle_right);canvas.setTextColor(trig?RED:COL_DIM2);
                canvas.drawString(trig?"TRIG":"open",ex+esW-4,vy+btnH/2);
            }
        }
        vy+=btnH+secGap;

        // ── Zero WCS ─────────────────────────────────────────────────────
        secLabel("Zero WCS");
        if(visible()){
            const char* zla[]={"X","Y","Z","All"};
            int zcols[]={COL_AX_X,COL_AX_Y,COL_AX_Z,ORANGE};
            int zw=(bw-3*gap)/4;
            for(int k=0;k<4;k++){
                int zx=pad+k*(zw+gap);
                _zeroWcsBtns[k]={zx,vy,zw,btnH};
                bool zp=(fl&&_homePressedId==20+k);
                if(zp) canvas.fillRoundRect(zx,vy,zw,btnH,3,zcols[k]);
                else{canvas.fillRoundRect(zx,vy,zw,btnH,3,COL_PANEL2);canvas.drawRoundRect(zx,vy,zw,btnH,3,zcols[k]);}
                canvas.setFont(&fonts::Font2);canvas.setTextDatum(middle_center);
                canvas.setTextColor(zp?COL_BG:zcols[k]);
                char zlbl[8]; snprintf(zlbl,sizeof(zlbl),"%s=0",zla[k]);
                canvas.drawString(zlbl,zx+zw/2,vy+btnH/2);
            }
        } else{for(int k=0;k<4;k++) _zeroWcsBtns[k]={0,0,0,0};}
    }

    // ── G-code preview — full screen with viz/text toggle ───────────────────
    void drawGcodePreview() {
        // Full-screen overlay
        canvas.fillRect(0, TOP, W, NAV_Y-TOP, COL_BG);

        const std::string& fname = (fileSelected>=0&&fileSelected<(int)fileList.size())
                                   ? fileList[fileSelected].name : std::string("?");

        // ── Title bar ────────────────────────────────────────────────────────
        int titleH=18;
        canvas.fillRect(0,TOP,W,titleH,COL_PANEL);
        hline(0,TOP+titleH,W,COL_BORDER);
        f2s(fname,8,TOP+titleH/2,CYAN,middle_left);
        // Path/lines count right of title
        if(_previewShowPath){
            char pc[16]; snprintf(pc,sizeof(pc),"%d pts",(int)vizPath.size());
            f2(pc,W-6,TOP+titleH/2,COL_DIM2,middle_right);
        } else {
            char lc[16]; snprintf(lc,sizeof(lc),"%d lines",(int)previewLines.size());
            f2(lc,W-6,TOP+titleH/2,COL_DIM2,middle_right);
        }

        // ── Content area ─────────────────────────────────────────────────────
        int contentY=TOP+titleH, contentH=NAV_Y-titleH-TOP-28;

        if (_previewShowPath && !vizPath.empty()) {
            // ── Path visualizer — full content area ──────────────────────────
            canvas.fillRect(0,contentY,W,contentH,COL_PANEL2);

            // Bounding box
            float pMinX=vizPath[0].first,pMaxX=pMinX;
            float pMinY=vizPath[0].second,pMaxY=pMinY;
            for(auto& pt:vizPath){
                pMinX=std::min(pMinX,pt.first);pMaxX=std::max(pMaxX,pt.first);
                pMinY=std::min(pMinY,pt.second);pMaxY=std::max(pMaxY,pt.second);
            }
            float rX=std::max(pMaxX-pMinX,1.0f), rY=std::max(pMaxY-pMinY,1.0f);
            int pad2=8;
            float scX=(float)(W-2*pad2)/(rY*1.1f);
            float scY=(float)(contentH-2*pad2)/(rX*1.1f);
            float sc2=std::min(scX,scY);
            float pW=rY*sc2, pH=rX*sc2;
            int ox=(int)(pad2+(W-2*pad2-pW)/2.0f);
            int oy=(int)(contentY+pad2+(contentH-2*pad2-pH)/2.0f);

            // Background rect
            canvas.fillRect(ox,oy,(int)pW,(int)pH,COL_PANEL3);
            canvas.drawRect(ox,oy,(int)pW,(int)pH,COL_BORDER2);

            // Draw all path lines in cyan
            for(int pi=1;pi<(int)vizPath.size();pi++){
                int x1=ox+(int)((vizPath[pi-1].second-pMinY)*sc2);
                int y1=oy+(int)(pH-(vizPath[pi-1].first-pMinX)*sc2);
                int x2=ox+(int)((vizPath[pi].second-pMinY)*sc2);
                int y2=oy+(int)(pH-(vizPath[pi].first-pMinX)*sc2);
                canvas.drawLine(x1,y1,x2,y2,CYAN);
            }

            // Start dot (green) and end dot (red)
            if(vizPath.size()>0){
                int sx=ox+(int)((vizPath[0].second-pMinY)*sc2);
                int sy=oy+(int)(pH-(vizPath[0].first-pMinX)*sc2);
                canvas.fillCircle(sx,sy,3,GREEN);
            }
            if(vizPath.size()>1){
                int ex2=ox+(int)((vizPath.back().second-pMinY)*sc2);
                int ey2=oy+(int)(pH-(vizPath.back().first-pMinX)*sc2);
                canvas.fillCircle(ex2,ey2,3,RED);
            }

            // Dimensions label
            canvas.setFont(&fonts::Font0); canvas.setTextColor(COL_DIM);
            canvas.setTextDatum(bottom_right);
            char pdim[24]; snprintf(pdim,sizeof(pdim),"%.0fx%.0fmm",rY,rX);
            canvas.drawString(pdim,ox+(int)pW-1,oy+(int)pH-1);

        } else {
            // ── G-code text view ─────────────────────────────────────────────
            int lh=13, maxL=contentH/lh;
            int total=(int)previewLines.size();

            // Scroll bar
            if(total>maxL){
                int thumbH=std::max(4,contentH*maxL/total);
                int thumbY=contentY+(contentH-thumbH)*previewScroll/std::max(1,total-maxL);
                canvas.fillRect(W-3,contentY,3,contentH,COL_BORDER);
                canvas.fillRect(W-3,thumbY,3,thumbH,COL_DIM2);
            }
            for(int i=0;i<maxL&&(previewScroll+i)<total;i++){
                int ly=contentY+i*lh;
                int idx=previewScroll+i;
                if(i%2==0) canvas.fillRect(0,ly,W-4,lh,0x0841);
                const std::string& ln=previewLines[idx];
                int col=COL_DIM2;
                if(!ln.empty()){
                    char c=ln[0];
                    if(c=='G'||c=='g') col=CYAN;
                    else if(c=='M'||c=='m') col=YELLOW;
                    else if(c==';'||c=='(') col=COL_DIM;
                    else if(c=='T'||c=='t') col=ORANGE;
                    else col=COL_WHITE2;
                }
                canvas.setFont(&fonts::Font0);
                canvas.setTextDatum(middle_right);
                canvas.setTextColor(COL_DIM);
                char lnum[6]; snprintf(lnum,sizeof(lnum),"%d",idx+1);
                canvas.drawString(lnum,26,ly+lh/2);
                canvas.setTextDatum(middle_left);
                canvas.setTextColor(col);
                canvas.drawString(ln.substr(0,44).c_str(),30,ly+lh/2);
            }
        }

        // ── Action bar: [Exit] [G-code|Path] [Run] ───────────────────────────
        int abY=NAV_Y-28, abH2=28;
        canvas.fillRect(0,abY,W,abH2,COL_PANEL);
        hline(0,abY,W,COL_BORDER);

        int bw3=(W-16)/3, bh3=22, by3=abY+3;

        // Exit — left
        tintStrokeR(4,by3,bw3,bh3,3,COL_BORDER2,COL_BORDER,40);
        f2("Exit",4+bw3/2,abY+abH2/2,COL_DIM2,middle_center);

        // Toggle G-code / Path — centre
        bool hasPath=!vizPath.empty();
        int togCol=hasPath?YELLOW:COL_DIM;
        if(_previewShowPath){
            canvas.fillRoundRect(8+bw3,by3,bw3,bh3,3,0x0019);
            canvas.drawRoundRect(8+bw3,by3,bw3,bh3,3,togCol);
            f2("G-code",8+bw3+bw3/2,abY+abH2/2,togCol,middle_center);
        } else {
            tintStrokeR(8+bw3,by3,bw3,bh3,3,togCol,togCol,hasPath?30:15);
            f2("Path",8+bw3+bw3/2,abY+abH2/2,togCol,middle_center);
        }

        // Run — right
        canvas.fillRoundRect(12+2*bw3,by3,bw3,bh3,3,0x0440);
        canvas.drawRoundRect(12+2*bw3,by3,bw3,bh3,3,GREEN);
        f2("Run",12+2*bw3+bw3/2,abY+abH2/2,GREEN,middle_center);
    }

    // ── Files screen ─────────────────────────────────────────────────────────
    void drawFilesScreen() {
        // Layout: [24px header] [rows 30px each] [38px action bar]
        int hdrH=24, abH=38, rowH=30;
        int listY=TOP+hdrH;
        int abY=NAV_Y-abH;
        int maxR=(abY-listY)/rowH;

        // ── Header: path + Up button ─────────────────────────────────────────
        canvas.fillRect(0,TOP,W,hdrH,COL_PANEL);
        hline(0,TOP+hdrH,W,COL_BORDER);
        // Up button — full-height, easy to tap
        if (filePath != "/sd") {
            tintStrokeR(W-42,TOP+3,38,hdrH-6,3,COL_BORDER2,COL_BORDER,50);
            canvas.setFont(&fonts::Font2); canvas.setTextDatum(middle_center);
            canvas.setTextColor(COL_WHITE2);
            canvas.drawString("Up",W-23,TOP+hdrH/2);
        }
        canvas.setFont(&fonts::Font0); canvas.setTextDatum(middle_left);
        canvas.setTextColor(CYAN);
        // Truncate path to fit left of Up button
        std::string dispPath = filePath;
        if((int)dispPath.size()>28) dispPath="..."+dispPath.substr(dispPath.size()-25);
        canvas.drawString(dispPath.c_str(),5,TOP+hdrH/2);

        // ── File rows ─────────────────────────────────────────────────────────
        for(int i=0;i<maxR;i++){
            int fi=fileScroll+i;
            int ry=listY+i*rowH;
            int rowbg=(i%2==0)?0x0883:COL_PANEL3;
            if(fi>=(int)fileList.size()){
                canvas.fillRect(0,ry,W,rowH,rowbg); continue;
            }
            bool sel=(fi==fileSelected);
            canvas.fillRect(0,ry,W,rowH,sel?0x0019:rowbg);
            if(sel) canvas.fillRect(0,ry,3,rowH,BLUE);
            hline(0,ry+rowH-1,W,COL_BORDER);
            const FileEntry& fe=fileList[fi];
            if(fe.isDir){
                // Folder icon
                canvas.fillRoundRect(6,ry+rowH/2-6,14,10,2,0x3240);
                canvas.drawRoundRect(6,ry+rowH/2-6,14,10,2,YELLOW);
                canvas.fillRect(6,ry+rowH/2-9,7,4,0x3240);
                std::string dn=fe.name+"/";
                canvas.setFont(&fonts::Font2); canvas.setTextDatum(middle_left);
                canvas.setTextColor(sel?COL_WHITE:YELLOW);
                canvas.drawString(dn.c_str(),26,ry+rowH/2);
            } else {
                // File icon
                canvas.fillRoundRect(6,ry+4,12,rowH-8,2,sel?0x0019:COL_PANEL3);
                canvas.drawRoundRect(6,ry+4,12,rowH-8,2,sel?CYAN:COL_DIM2);
                canvas.setFont(&fonts::Font0); canvas.setTextDatum(middle_center);
                canvas.setTextColor(sel?CYAN:COL_DIM2);
                canvas.drawString("NC",12,ry+rowH/2);
                // Filename
                canvas.setFont(&fonts::Font2); canvas.setTextDatum(middle_left);
                canvas.setTextColor(sel?COL_WHITE:COL_WHITE2);
                // Truncate long names
                std::string fname=fe.name;
                if((int)fname.size()>22) fname=fname.substr(0,19)+"...";
                canvas.drawString(fname.c_str(),24,ry+rowH/2-4);
                // Size below name
                if(fe.size>0){
                    char sz[16];
                    if(fe.size>=1000000) snprintf(sz,sizeof(sz),"%.1fMB",fe.size/1000000.0);
                    else if(fe.size>=1000) snprintf(sz,sizeof(sz),"%.1fKB",fe.size/1000.0);
                    else snprintf(sz,sizeof(sz),"%dB",fe.size);
                    canvas.setFont(&fonts::Font0); canvas.setTextDatum(middle_left);
                    canvas.setTextColor(COL_DIM2);
                    canvas.drawString(sz,24,ry+rowH/2+6);
                }
            }
        }

        // ── Action bar: full-width View and Run buttons ───────────────────────
        canvas.fillRect(0,abY,W,abH,COL_PANEL);
        hline(0,abY,W,COL_BORDER);

        if(fileSelected>=0&&fileSelected<(int)fileList.size()&&!fileList[fileSelected].isDir){
            // Show selected filename at top of action bar
            canvas.setFont(&fonts::Font0); canvas.setTextDatum(middle_left);
            canvas.setTextColor(CYAN);
            std::string sn=fileList[fileSelected].name;
            if((int)sn.size()>26) sn=sn.substr(0,23)+"...";
            canvas.drawString(sn.c_str(),6,abY+8);
            // Two big buttons: [View] [Run]
            int btnY=abY+14, btnH=abH-16, btnW=(W-12)/2;
            // View button
            tintStrokeR(4,btnY,btnW,btnH,4,COL_BORDER2,COL_WHITE2,50);
            canvas.setFont(&fonts::Font2); canvas.setTextDatum(middle_center);
            canvas.setTextColor(COL_WHITE2);
            canvas.drawString("View",4+btnW/2,btnY+btnH/2);
            // Run button
            tintStrokeR(8+btnW,btnY,btnW,btnH,4,GREEN,GREEN,40);
            canvas.setTextColor(GREEN);
            canvas.drawString("Run",8+btnW+btnW/2,btnY+btnH/2);
        } else {
            canvas.setFont(&fonts::Font2); canvas.setTextDatum(middle_center);
            canvas.setTextColor(COL_DIM);
            canvas.drawString("Tap a file to select",W/2,abY+abH/2);
        }

        // G-code preview overlay
        if(filePreviewMode&&fileSelected>=0) drawGcodePreview();
    }

    // ── Terminal screen ───────────────────────────────────────────────────────
    void drawTerminalScreen() {
        // Layout: [spindle row 22px] [term output] [quick cmd row 28px] [nav]
        int sRowH = 22;            // spindle/coolant row height (top)
        int qRowH = 28;            // quick commands row height (bottom, easier to press)
        int sRowY = TOP;           // spindle row just below header
        int qRowY = NAV_Y - qRowH; // quick cmds just above nav bar
        int outY  = sRowY + sRowH + 1;
        int outH  = qRowY - outY - 1;

        // ── Top row: Soft Reset button ───────────────────────────────────────
        canvas.fillRect(0, sRowY, W, sRowH, COL_PANEL2);
        hline(0, sRowY + sRowH, W, COL_BORDER);
        tintStrokeR(4, sRowY+2, W-8, sRowH-4, 3, RED, RED, 25);
        canvas.setFont(&fonts::Font2); canvas.setTextDatum(middle_center);
        canvas.setTextColor(RED);
        canvas.drawString("Soft Reset (Ctrl-X)", W/2, sRowY+sRowH/2);

        // ── Terminal output area ─────────────────────────────────────────────
        canvas.fillRect(0, outY, W, outH, COL_PANEL3);
        int lh = 13;
        int maxLines = (outH - 2) / lh;
        int total = fnc_term_count();
        int start = std::max(0, total - maxLines - termScroll);
        start     = std::min(start, std::max(0, total - maxLines));

        // Scroll indicator
        if (total > maxLines) {
            int trackH = outH - 4;
            int thumbH = std::max(6, trackH * maxLines / total);
            int thumbY = outY + 2 + (trackH-thumbH) * start / std::max(1, total-maxLines);
            canvas.fillRect(W-3, outY+2, 3, trackH, COL_BORDER);
            canvas.fillRect(W-3, thumbY, 3, thumbH, COL_DIM2);
        }

        for (int i = 0; i < maxLines && (start + i) < total; i++) {
            int ly = outY + 2 + i * lh;
            if (i % 2 == 0) canvas.fillRect(0, ly, W-4, lh, 0x0841);
            const char* line = fnc_term_line(start + i);
            int col = COL_DIM2;
            if (line[0]=='<') col=GREEN;
            else if (line[0]=='[') col=CYAN;
            else if (line[0]=='e'||line[0]=='E'||line[0]=='A') col=RED;
            else if (line[0]=='o') col=COL_WHITE2;
            else if (line[0]=='>') col=YELLOW;
            canvas.setFont(&fonts::Font0);
            canvas.setTextDatum(middle_left);
            canvas.setTextColor(col);
            canvas.drawString(line, 5, ly+lh/2);
        }

        // ── Bottom row: Quick GCode commands (large, easy to press) ─────────
        canvas.fillRect(0, qRowY, W, qRowH, COL_PANEL2);
        hline(0, qRowY, W, COL_BORDER);
        int totalGaps = (N_QUICK_CMDS - 1) * 3;
        int cmdW = (W - 6 - totalGaps) / N_QUICK_CMDS;
        for (int ci = 0; ci < N_QUICK_CMDS; ci++) {
            int bx = 3 + ci * (cmdW + 3);
            _cmdBtns[ci] = { bx, qRowY+2, cmdW, qRowH-4 };
            tintStrokeR(bx, qRowY+2, cmdW, qRowH-4, 3, 0xF81F, 0xF81F, 20);
            canvas.setFont(&fonts::Font2); canvas.setTextDatum(middle_center);
            canvas.setTextColor(0xF81F);
            canvas.drawString(QUICK_CMDS[ci], bx+cmdW/2, qRowY+qRowH/2);
        }
    }

    // ── Macros screen ─────────────────────────────────────────────────────────
    void drawMacrosScreen() {
        // Spindle sub-menu overlay
        if (_spindleMenuOpen) {
            canvas.fillRect(0,TOP,W,NAV_Y-TOP,COL_BG);
            canvas.fillRect(0,TOP,W,16,COL_PANEL);
            hline(0,TOP+16,W,COL_BORDER);
            canvas.setFont(&fonts::Font2); canvas.setTextDatum(middle_center);
            canvas.setTextColor(0xF81F);
            canvas.drawString("Spindle / Coolant",W/2,TOP+8);
            int sy=TOP+20, sbh=26, sbw=W-12, spad=6;
            for(int ci=0;ci<N_SPINDLE;ci++){
                int col2=(ci<2)?GREEN:(ci==2)?RED:(ci==3)?CYAN:COL_DIM2;
                tintStrokeR(spad,sy,sbw,sbh,3,col2,col2,25);
                canvas.setFont(&fonts::Font2); canvas.setTextDatum(middle_center);
                canvas.setTextColor(col2);
                canvas.drawString(SPINDLE_LBLS[ci],W/2,sy+sbh/2);
                sy+=sbh+4;
            }
            // Close button
            tintStrokeR(spad,sy+4,sbw,sbh,3,COL_BORDER2,COL_BORDER,40);
            canvas.setTextColor(COL_DIM2);
            canvas.drawString("Close",W/2,sy+4+sbh/2);
            return;
        }

        int pad=6, gap=3, bh=40;
        int avH=NAV_Y-TOP-pad;

        // Total items = 1 (spindle) + MACROS.size()
        int totalItems=1+(int)MACROS.size();
        int visible=avH/(bh+gap);
        int maxScroll=std::max(0,totalItems-visible);
        macroScroll=std::max(0,std::min(macroScroll,maxScroll));

        // Scroll bar
        if(totalItems>visible){
            int trackH=avH;
            int thumbH=std::max(8,trackH*visible/totalItems);
            int thumbY=TOP+pad+(trackH-thumbH)*macroScroll/std::max(1,maxScroll);
            canvas.fillRect(W-4,TOP+pad,4,trackH,COL_BORDER);
            canvas.fillRect(W-4,thumbY,4,thumbH,COL_DIM2);
        }

        int bw=W-2*pad-6;
        int drawn=0;
        for(int vi=0;vi<visible&&(macroScroll+vi)<totalItems;vi++){
            int itemIdx=macroScroll+vi;
            int by=TOP+pad+vi*(bh+gap);
            int rh=std::min(bh,NAV_Y-by-2);
            if(rh<12) break;
            _macroBtns[vi]={pad,by,bw,rh};
            drawn++;

            if(itemIdx==0){
                // Spindle entry
                canvas.fillRoundRect(pad,by,bw,rh,4,COL_PANEL2);
                canvas.drawRoundRect(pad,by,bw,rh,4,0xF81F);
                canvas.fillRect(pad+1,by+2,5,rh-4,0xF81F);
                canvas.setFont(&fonts::Font0); canvas.setTextDatum(middle_center);
                canvas.setTextColor(0xF81F);
                canvas.drawString("S",pad+16,by+rh/2);
                canvas.setFont(&fonts::Font2); canvas.setTextDatum(middle_left);
                canvas.setTextColor(COL_WHITE);
                canvas.drawString("Spindle / Coolant",pad+28,by+rh*36/100);
                canvas.setFont(&fonts::Font0); canvas.setTextDatum(middle_left);
                canvas.setTextColor(COL_DIM2);
                canvas.drawString("CW / CCW / Stop / Flood / M-off",pad+28,by+rh*70/100);
                canvas.setTextDatum(middle_right);
                canvas.setTextColor(0xF81F);
                canvas.drawString("▶",pad+bw-4,by+rh/2);
            } else {
                int mi=itemIdx-1;
                bool empty=MACROS[mi].cmd.empty();
                int mcol=empty?COL_DIM:MACROS[mi].col;
                bool isP6=(_enableMode==3&&mi==_enableMacro);
                if(isP6){ canvas.fillRoundRect(pad,by,bw,rh,4,0x0019); canvas.drawRoundRect(pad,by,bw,rh,4,mcol);}
                else{ canvas.fillRoundRect(pad,by,bw,rh,4,COL_PANEL2); canvas.drawRoundRect(pad,by,bw,rh,4,COL_BORDER2);}
                canvas.fillRect(pad+1,by+2,5,rh-4,mcol);
                char idx2[4]; snprintf(idx2,sizeof(idx2),"%d",mi+1);
                canvas.setFont(&fonts::Font0); canvas.setTextDatum(middle_center);
                canvas.setTextColor(mcol);
                canvas.drawString(idx2,pad+16,by+rh/2);
                canvas.setFont(&fonts::Font2); canvas.setTextDatum(middle_left);
                canvas.setTextColor(empty?COL_DIM:COL_WHITE);
                canvas.drawString(MACROS[mi].name.c_str(),pad+28,by+rh*36/100);
                canvas.setFont(&fonts::Font0); canvas.setTextDatum(middle_left);
                canvas.setTextColor(empty?COL_DIM:COL_DIM2);
                canvas.drawString(empty?"(empty)":MACROS[mi].cmd.substr(0,30).c_str(),pad+28,by+rh*70/100);
                if(isP6){ canvas.setTextDatum(middle_right); canvas.setTextColor(CYAN); canvas.drawString("P6",pad+bw-4,by+rh/2);}
            }
        }
        for(int vi=drawn;vi<8;vi++) _macroBtns[vi]={0,0,0,0};
    }

    // ── Disconnected overlay ─────────────────────────────────────────────────
    void drawDisconnectedOverlay() {
        // Semi-transparent dark overlay
        for (int y2 = TOP; y2 < NAV_Y; y2 += 2)
            canvas.drawFastHLine(0, y2, W, 0x0000);

        // Central panel
        int px=20, py=70, pw=W-40, ph=90;
        canvas.fillRoundRect(px, py, pw, ph, 8, 0x1082);
        canvas.drawRoundRect(px, py, pw, ph, 8, 0xF800);

        // Warning triangle
        canvas.fillTriangle(W/2, py+8, W/2-14, py+34, W/2+14, py+34, 0xFD20);
        canvas.fillRect(W/2-2, py+13, 4, 11, 0x1082);
        canvas.fillRect(W/2-2, py+27, 4, 4,  0x1082);

        // Text
        canvas.setFont(&fonts::Font2);
        canvas.setTextDatum(middle_center);
        canvas.setTextColor(0xF800);
        canvas.drawString("DISCONNECTED", W/2, py+46);

        canvas.setFont(&fonts::Font0);
        canvas.setTextColor(0x9D17);
        canvas.drawString("FluidNC not responding", W/2, py+62);
        canvas.drawString("Check UART connection", W/2, py+74);

        canvas.setTextColor(0xFD20);
        canvas.drawString("Reconnecting...", W/2, py+86);
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
        vizCacheInit();
        // Default axis letters XYZABC
        const char* defLetters = "XYZABC";
        for (int i=0;i<6;i++) _axisLetters[i] = defLetters[i];
        _axisAutoDetected = false;
        fnc_term_inject("> $I");
        termLines.push_back({ "[MSG:FluidDial UI]", GREEN   });
        fnc_realtime(StatusReport);
        // Request axis count and names from FluidNC
        send_line("$axes/count");
        for (int i=0;i<6;i++) {
            char cmd[24]; snprintf(cmd,sizeof(cmd),"$axes/%d/name",i);
            send_line(cmd);
        }
        request_file_list("/sd");
    }

    void onDROChange()    override { if (_tab == 0) reDisplay(); }
    void onLineReceived()  override { if (_tab == 2) reDisplay(); }
    void onLimitsChange() override { if (_tab == 1) reDisplay(); }

    void onStateChange(state_t old_state) override {
        if (state == Alarm) _alarmOpen = true;
        else _alarmOpen = false;
        if (state == Idle) { _jobSentToFluidNC = false; }
        // Reconnection: re-request axis config when coming back online
        if (old_state == Disconnected && state != Disconnected) {
            send_line("$axes/count");
            for (int i=0;i<6;i++) {
                char cmd[24]; snprintf(cmd,sizeof(cmd),"$axes/%d/name",i);
                send_line(cmd);
            }
        }
        // Job timer
        if (state == Cycle && old_state != Cycle) {
            // Started or resumed
            _jobStartTime = millis();
        } else if (state != Cycle && old_state == Cycle) {
            // Paused or ended — accumulate elapsed
            _jobElapsed += millis() - _jobStartTime;
            _jobStartTime = 0;
        }
        if (state == Idle || state == Alarm) {
            // Job ended — keep elapsed for display
            _jobStartTime = 0;
        }
        reDisplay();
    }

    void onMessage(char* command, char* arguments) override {
        // Parse $axes/N/name=X responses
        if (command && strncmp(command, "axes/", 5) == 0) {
            // Format: axes/N/name  arguments = X (or Y, Z, A, B, C)
            const char* p = command + 5;
            int axIdx = atoi(p);
            const char* slash = strchr(p, '/');
            if (slash && strncmp(slash, "/name", 5) == 0 && axIdx >= 0 && axIdx < 6) {
                if (arguments && arguments[0] >= 'A' && arguments[0] <= 'Z') {
                    _axisLetters[axIdx] = arguments[0];
                    _axisAutoDetected = true;
                    // Auto-detect mode from axis letters
                    if (n_axes == 4) {
                        bool hasA = false, hasY2 = false;
                        for (int i=0;i<n_axes;i++) {
                            if (_axisLetters[i]=='A') hasA=true;
                            if (i>0 && _axisLetters[i]=='Y') hasY2=true;
                        }
                        if (hasY2) { _droAxesMode=3; tabui_setAxes(3); }
                        else if (hasA) { _droAxesMode=1; tabui_setAxes(1); }
                    } else if (n_axes == 2) {
                        _droAxesMode=2; tabui_setAxes(2);
                    } else if (n_axes == 3) {
                        _droAxesMode=0; tabui_setAxes(0);
                    }
                }
            }
        }
        std::string line = std::string("[MSG:") + arguments + "]";
        termLines.push_back({ line, GREEN });
        if (termLines.size() > 200) termLines.erase(termLines.begin());
        if (termScroll == 0 && _tab == 2) reDisplay();  // live update when pinned to bottom
        if (_tab == 2) reDisplay();
    }

    void onFileLines(int firstLine, const std::vector<std::string>& lines) override {
        static const int BATCH = 500; // Try large batch; FluidNC may truncate to its limit

        if (firstLine == 0) {
            // First batch — reset accumulator
            allFileLines.clear();
            _loadingDone = false;
            // Set text preview from first batch
            previewLines = lines;
            previewFirstLine = 0;
            previewScroll = 0;
            filePreviewMode = true;
        }

        // Accumulate into allFileLines
        for (auto& l : lines) allFileLines.push_back(l);

        if ((int)lines.size() >= 60 && !_loadingPath.empty()) {
            // More lines likely available — request next batch
            _loadingBatch = firstLine + (int)lines.size();
            request_file_preview(_loadingPath.c_str(), _loadingBatch, BATCH);
        } else {
            // End of file reached — parse full accumulated path
            _loadingDone = true;
            _loadingBatch = 0;
            if (fileSelected>=0 && fileSelected<(int)fileList.size()) {
                parseGcodeToVizPath(allFileLines, fileList[fileSelected].name);
                // Save to LittleFS cache for instant load next time
                vizCacheEvictOld();
                vizCacheSave(fileList[fileSelected].name,
                             fileList[fileSelected].size, vizPath);
            }
        }

        // Update viz path as we accumulate (progressive)
        if (!allFileLines.empty() && fileSelected>=0 && fileSelected<(int)fileList.size()) {
            if ((int)allFileLines.size() > (int)vizPath.size())
                parseGcodeToVizPath(allFileLines, fileList[fileSelected].name);
        }

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
                float dist = delta * mpgSteps[(int)mpgStepIdx];
                static const int jogFeed[] = { 100, 500, 2000 };
                int feed = jogFeed[(int)mpgStepIdx];

                if (_droAxesMode == 3) {
                    // XYYZ mode: axis 0=X, 1=Y(both), 2=Z, 3=Y2(independent)
                    // In FluidNC, Y2 ganged axis uses same 'Y' command — both move together
                    // For Y1 (axis 1): move Y — Y2 slave follows automatically
                    // For Y2 (axis 3): also move Y — note: true independent Y2 requires
                    //   FluidNC config with separate axis letter (check your machine config)
                    if (mpgAxis == 3) {
                        // Y2 independent — try 'B' first (common FluidNC ganged axis letter)
                        // If your machine uses a different letter, change 'B' here
                        send_linef("$J=G91 B%.3f F%d", dist, feed);
                    } else {
                        static const char axCharYYZ[] = { 'X', 'Y', 'Z', 'Y' };
                        send_linef("$J=G91 %c%.3f F%d", axCharYYZ[mpgAxis], dist, feed);
                    }
                } else {
                    // Use auto-detected axis letters from FluidNC config
                    char axC = _axisAutoDetected ? _axisLetters[(int)mpgAxis]
                                                 : "XYZA"[(int)mpgAxis];
                    send_linef("$J=G91 %c%.3f F%d", axC, dist, feed);
                }
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
            // Throttle reDisplay during job to avoid CPU overload
            static uint32_t _lastOvrRedraw = 0;
            if (millis() - _lastOvrRedraw > 150) {
                _lastOvrRedraw = millis();
                reDisplay();
            }
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
            int maxScroll = std::max(0, fnc_term_count() - ((NAV_Y-CMD_H-TOP-4-2)/13));
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
            if (!*p || *p==';' || *p=='(' || *p=='%' || *p=='N' || *p=='n') {
                // Check for G90/G91 after line number N
                if (*p=='N'||*p=='n') { while(*p && *p!=' ') p++; while(*p==' ') p++; }
                else continue;
            }
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

        // Nav bar — Hold/Abort (when job active) or tab switch
        if (y >= NAV_Y) {
            // Check Hold/Abort buttons first (shown when job running on DRO tab)
            if (hit(_holdBtn, x, y)) {
                if (state == Hold) {
                    fnc_realtime((realtime_cmd_t)'~');  // Resume
                    fnc_term_inject("> Resume");
                } else {
                    fnc_realtime((realtime_cmd_t)'!');  // Hold
                    fnc_term_inject("> Hold");
                }
                reDisplay(); return;
            }
            if (hit(_abortBtn, x, y)) {
                fnc_realtime((realtime_cmd_t)0x18);    // Soft Reset
                fnc_term_inject("> Abort: Reset sent");
                simJobRunning = false; vizPathExecuted = 0;
                reDisplay(); return;
            }
            for (int i = 0; i < N_TABS; i++) {
                if (hit(_navTabs[i], x, y)) {
                    _tab = i;
                    if (_tab == 2) {
                        request_file_list(filePath.c_str());
                    }
                    reDisplay(); return;
                }
            }
            return;
        }

        // DRO screen
        if (_tab == 0) {
            // Clear [×] button — top-right of viz area (clears loaded G-code path)
            if (!vizPath.empty() && x >= VIZ_X+VIZ_W-16 && x < VIZ_X+VIZ_W
                && y >= VIZ_Y+1 && y <= VIZ_Y+14) {
                vizPath.clear(); vizJobName.clear(); vizPathExecuted=0;
                reDisplay(); return;
            }
            // Mode toggle pills: WPos/MPos and mm/in (top of viz area)
            if (y >= TOP+2 && y <= TOP+12 && x >= DROW+2) {
                int px=DROW+2, pw=36, g=3;
                if (x < px+pw)   { _showMPos=false; reDisplay(); return; }  // WPos
                if (x < px+pw+g+pw) { _showMPos=true; reDisplay(); return; }  // MPos
                px+=pw+g+pw+g+4;
                if (x < px+24)   { send_line("G21"); reDisplay(); return; }  // mm
                if (x < px+51)   { send_line("G20"); reDisplay(); return; }  // in
            }
            // Simulation mode: tap a DRO axis row to simulate +0.1mm jog
            if (false && x < DROW && y >= TOP && y < NAV_Y - FEED_H) {  // sim removed
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
                _homePressedId=0; _homePressTime=millis();
                send_line("$H"); fnc_term_inject("> $H");
                reDisplay(); return;
            }
            if (hit(_probeBtnR, x, y)) {
                _homePressedId=1; _homePressTime=millis();
                _probeOpen=true; reDisplay(); return;
            }
            const char* axcmds[]={"$HX","$HY","$HZ"};
            for(int i=0;i<3;i++){
                if(hit(_axisHomeBtns[i],x,y)){
                    _homePressedId=10+i; _homePressTime=millis();
                    send_line(axcmds[i]);
                    fnc_term_inject((std::string("> ")+axcmds[i]).c_str());
                    reDisplay(); return;
                }
            }
            const char* zcmds[]={"G10 L20 P1 X0","G10 L20 P1 Y0","G10 L20 P1 Z0","G10 L20 P1 X0 Y0 Z0"};
            for(int k=0;k<4;k++){
                if(hit(_zeroWcsBtns[k],x,y)){
                    _homePressedId=20+k; _homePressTime=millis();
                    send_line(zcmds[k]);
                    fnc_term_inject((std::string("> ")+zcmds[k]).c_str());
                    termLines.push_back({"ok",GREEN});
                    reDisplay(); return;
                }
            }
        }

        // Files screen
        if (_tab == 2) {
            int hdrH=24, abH=38, rowH=30;
            int listY=TOP+hdrH, abY=NAV_Y-abH;
            // Up button
            if (filePath != "/sd" && y>=TOP && y<TOP+hdrH && x>=W-42) {
                size_t pos = filePath.rfind('/');
                if (pos != std::string::npos && pos > 0) filePath = filePath.substr(0, pos);
                else filePath = "/sd";
                fileScroll = 0; fileSelected = -1;
                request_file_list(filePath.c_str()); return;
            }
            // Action bar buttons
            if (y >= abY && fileSelected>=0 && fileSelected<(int)fileList.size()
                && !fileList[fileSelected].isDir) {
                int btnY=abY+14, btnH=abH-16, btnW=(W-12)/2;
                if (y>=btnY && y<btnY+btnH) {
                    if (x<=4+btnW) {
                        // View button
                        previewScroll=0; _previewShowPath=(!vizPath.empty());
                        filePreviewMode=true; reDisplay(); return;
                    } else if (x>=8+btnW) {
                        // Run button
                        simPath.clear(); simPathIdx=0;
                        parseGcodeToVizPath(previewLines,fileList[fileSelected].name);
                        simPath=vizPath;
                        simJobName=fileList[fileSelected].name;
                        {
                            std::string path=filePath+"/"+fileList[fileSelected].name;
                            send_linef("$Localfs/Run=%s",path.c_str());
                            termLines.push_back({"> Run: "+simJobName,COL_DIM2});
                            _jobSentToFluidNC=true;
                        }
                        filePreviewMode=false; _tab=0; reDisplay(); return;
                    }
                }
                return;
            }
            // File row taps
            int maxR=(abY-listY)/rowH;
            if (y >= listY && y < abY) {
                int ri = (y - listY) / rowH;
                int fi = fileScroll + ri;
                if (fi < (int)fileList.size()) {
                    if (fileList[fi].isDir) {
                        filePath += "/" + fileList[fi].name;
                        fileScroll = 0; fileSelected = -1;
                        request_file_list(filePath.c_str());
                    } else {
                        fileSelected     = fi;
                        filePreviewMode  = false;
                        previewLines.clear();
                        previewScroll    = 0;
                        // Load file: chain-load for viz, first batch for preview
                        {
                            std::string path = filePath + "/" + fileList[fi].name;
                            vizJobName = fileList[fi].name;
                            vizPathExecuted = 0;
                            // Try LittleFS cache first (instant)
                            int fsize = fileList[fi].size;
                            if (vizCacheLoad(fileList[fi].name, fsize, vizPath)) {
                                // Cache hit — viz ready immediately, just need text preview
                                _loadingPath = "";
                                _loadingDone = true;
                                request_file_preview(path.c_str(), 0, 60); // text preview only
                            } else {
                                // Cache miss — chain-load from SD
                                _loadingPath = path;
                                _loadingBatch = 0;
                                allFileLines.clear();
                                request_file_preview(path.c_str(), 0, 500);
                            }
                        }
                        reDisplay();
                    }
                }
                return;
            }
            // G-code preview overlay touches (full screen)
            if (filePreviewMode) {
                // Action bar at bottom: [Exit] [Path/G-code] [Run]
                int abY=NAV_Y-28, abH2=28, bw3=(W-16)/3, by3=abY+3, bh3=22;
                if (y >= abY) {
                    if (x < 4+bw3) {
                        // Exit → back to file list
                        filePreviewMode=false; _previewShowPath=false;
                        reDisplay(); return;
                    }
                    if (x < 8+2*bw3) {
                        // Toggle G-code / Path
                        if (!vizPath.empty()) { _previewShowPath=!_previewShowPath; reDisplay(); return; }
                        return;
                    }
                    // Run button
                    {
                        // Parse gcode XY path
                        simPath.clear(); simPathIdx = 0;
                        parseGcodeToVizPath(previewLines, fileList[fileSelected].name);
                        simPath = vizPath;  // share for sim animation
                                                simJobName = fileList[fileSelected].name;
                        if (true) {
                            std::string path = filePath + "/" + fileList[fileSelected].name;
                            send_linef("$Localfs/Run=%s", path.c_str());
                            termLines.push_back({ "> Run: " + simJobName, COL_DIM2 });
                            _jobSentToFluidNC = true;


                        filePreviewMode = false; _previewShowPath=false;
                        _tab = 0; reDisplay(); return;
                    }
                    return;
                }
                // Scroll in content area (swipe handled by flick, tap scrolls 1 line here)
                return;  // consume all touches inside preview
            }
            // (Run handled above in action bar section)
            if (false && fileSelected >= 0 && y >= NAV_Y - 20 && x >= W - 42) {
                std::string path = filePath + "/" + fileList[fileSelected].name;
                send_linef("$Localfs/Run=%s", path.c_str());
                termLines.push_back({ std::string("> Run: ") + fileList[fileSelected].name, COL_DIM2 });
                _jobSentToFluidNC = true;
                _tab = 2; reDisplay();
                return;
            }
        }

        // Terminal screen
        // Probe screen touches (tab 2)

        if (_tab == 3) {
            // Top row: Soft Reset
            if (y>=TOP && y<TOP+22) {
                fnc_realtime((realtime_cmd_t)0x18);
                fnc_term_inject("> Soft Reset (Ctrl-X)");
                reDisplay(); return;
            }
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
            // Spindle sub-menu open
            if (_spindleMenuOpen) {
                int sy=TOP+20, sbh=26, spad=6, sbw=W-12;
                for(int ci=0;ci<N_SPINDLE;ci++){
                    if(y>=sy&&y<sy+sbh&&x>=spad&&x<spad+sbw){
                        send_line(SPINDLE_CMDS[ci]);
                        fnc_term_inject((std::string("> ")+SPINDLE_CMDS[ci]).c_str());
                        _spindleMenuOpen=false; reDisplay(); return;
                    }
                    sy+=sbh+4;
                }
                _spindleMenuOpen=false; reDisplay(); return;
            }
            // Main macro list
            for (int vi = 0; vi < 8; vi++) {
                if (_macroBtns[vi].w == 0) break;
                if (hit(_macroBtns[vi], x, y)) {
                    int itemIdx = macroScroll + vi;
                    if (itemIdx == 0) {
                        _spindleMenuOpen=true; reDisplay(); return;
                    }
                    int mi = itemIdx - 1;
                    if (mi < (int)MACROS.size() && !MACROS[mi].cmd.empty()) {
                        send_line(MACROS[mi].cmd.c_str());
                        fnc_term_inject((std::string("> ")+MACROS[mi].cmd).c_str());
                        reDisplay();
                    }
                    return;
                }
            }
        }
    }  // closes onTouchClick body
    }  // end onTouchClick

    void onLeftFlick()  override {}
    void onRightFlick() override {}
    void onUpFlick() override {
        if (_tab == 1) { _homeScroll = std::max(0, _homeScroll - 20); reDisplay(); }
        if (_tab == 2 && filePreviewMode) { previewScroll = std::max(0, previewScroll - 5); reDisplay(); }
        if (_tab == 3) { termScroll = std::max(0, termScroll - 5); reDisplay(); }
        if (_tab == 4) { if(_spindleMenuOpen){_spindleMenuOpen=false;}else{macroScroll=std::max(0,macroScroll-2);} reDisplay(); }
    }
    void onDownFlick() override {
        if (_tab == 1) { _homeScroll += 20; reDisplay(); }
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
        if (_tab == 4) { macroScroll+=2; reDisplay(); }
    }

    void reDisplay() override {
        static bool _inRedisplay = false;
        if (_inRedisplay) return;  // prevent reentrant calls
        _inRedisplay = true;
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
        if (state == Disconnected)    drawDisconnectedOverlay();
        drawNav();
        refreshDisplay();
        _inRedisplay = false;
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
