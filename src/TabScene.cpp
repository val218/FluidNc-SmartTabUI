// TabScene.cpp  — 5-tab landscape UI for FluidDial JC2432W328C (320x240)
// Copyright (c) 2024 FluidDial contributors. GPLv3 licence.

#include "Scene.h"
#include "driver/gpio.h"
#include "FileParser.h"
#include "System.h"
#include <vector>
#include <string>
#include <algorithm>

// ─────────────────────────────────────────────────────────────────────────────
// Palette — RGB565 literals
// ─────────────────────────────────────────────────────────────────────────────
static const int COL_BG      = 0x0862;
static const int COL_PANEL   = 0x0883;
static const int COL_PANEL2  = 0x10A3;
static const int COL_PANEL3  = 0x0863;
static const int COL_BORDER  = 0x1906;
static const int COL_BORDER2 = 0x2147;
static const int COL_DIM     = 0x31C9;
static const int COL_DIM2    = 0x52F0;
static const int COL_WHITE   = 0xDF1D;
static const int COL_WHITE2  = 0x9D17;
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
static int termScroll = 0;  // 0 = pinned to bottom, >0 = scrolled up by N lines

// ── MPG handwheel state ───────────────────────────────────────────────────────
// Step size switch: gpio16=1mm, gpio4=10mm, neither=0.1mm
// Axis switch:      gpio17=X, gpio35=Y, gpio21=Z, gpio22=A, neither=off
// E-stop:           gpio8 active-low
static int   mpgAxis     = -1;   // -1=off, 0=X,1=Y,2=Z,3=A
static int   mpgStepIdx  = 0;    // 0=0.1, 1=1.0, 2=10.0
static float mpgSteps[]  = { 0.1f, 1.0f, 10.0f };
static const char* mpgStepLabels[] = { "0.1", "1", "10" };
static bool  mpgEstopLast = true;   // debounce

void readMpgSwitches() {
    // Throttle to once every 50ms — no need to read every loop iteration
    static uint32_t lastRead = 0;
    uint32_t now = millis();
    if (now - lastRead < 50) return;
    lastRead = now;

    // Step switch (active-low, internal pullup)
    // gpio_get_level is safe on any pin including PCNT-managed ones
    bool s16 = (gpio_get_level(GPIO_NUM_16) == 0);
    bool s4  = (gpio_get_level(GPIO_NUM_4)  == 0);
    if      (s4)  mpgStepIdx = 2;   // 10mm
    else if (s16) mpgStepIdx = 1;   // 1mm
    else          mpgStepIdx = 0;   // 0.1mm

    // Axis switch (active-low)
    // GPIO21/22 are shared with PCNT encoder - gpio_get_level is read-only, safe
    // GPIO35 is input-only, no pullup - reads reliably only when externally pulled
    bool a17 = (gpio_get_level(GPIO_NUM_17) == 0);
    bool a35 = (gpio_get_level(GPIO_NUM_35) == 0);
    bool a21 = (gpio_get_level(GPIO_NUM_21) == 0);
    bool a22 = (gpio_get_level(GPIO_NUM_22) == 0);
    if      (a22) mpgAxis = 3;   // A
    else if (a21) mpgAxis = 2;   // Z
    else if (a35) mpgAxis = 1;   // Y
    else if (a17) mpgAxis = 0;   // X
    else          mpgAxis = -1;  // off
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
struct MacroBtn { const char* name; int col; const char* cmd; };
static const MacroBtn MACROS[] = {
    { "Tool Change", ORANGE,  "M6"                    },
    { "Park",        CYAN,    "G53 G0 Z0"             },
    { "Coolant On",  BLUE,    "M8"                    },
    { "Coolant Off", BLUE,    "M9"                    },
    { "Probe Z",     YELLOW,  "G38.2 Z-50 F100"       },
    { "Zero X",      COL_AX_X,   "G10 L20 P1 X0"         },
    { "Zero Y",      COL_AX_Y,   "G10 L20 P1 Y0"         },
    { "Zero Z",      COL_AX_Z,   "G10 L20 P1 Z0"         },
    { "Zero All",    ORANGE,  "G10 L20 P1 X0 Y0 Z0"  },
    { "Custom 1",    COL_DIM2,    ""                       },
    { "Custom 2",    COL_DIM2,    ""                       },
    { "Custom 3",    COL_DIM2,    ""                       },
};
static const int N_MACROS = 12;

// ─────────────────────────────────────────────────────────────────────────────
// File list
// ─────────────────────────────────────────────────────────────────────────────
struct FileEntry { std::string name; bool isDir; int size; };
static std::vector<FileEntry> fileList;
static int  fileScroll   = 0;
static int  fileSelected = -1;
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
    Rect _macroBtns[N_MACROS];

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

        // screen name centred
        hdrTxt(TAB_LABELS[_tab], W / 2, 10, COL_WHITE2);

        // MPG status: axis + step shown when active
        if (mpgAxis >= 0) {
            static const char* axNames[] = { "X", "Y", "Z", "A" };
            static const int   axCols[]  = { COL_AX_X, COL_AX_Y, COL_AX_Z, COL_AX_A };
            char mpgStr[12];
            snprintf(mpgStr, sizeof(mpgStr), "JOG %s %smm",
                     axNames[mpgAxis], mpgStepLabels[mpgStepIdx]);
            hdrTxt(mpgStr, W - 4, 10, axCols[mpgAxis], middle_right);
        } else {
            // TX/RX indicators when MPG not active
            canvas.fillCircle(W - 64, 10, 2, COL_DIM2);
            hdrTxt("TX", W - 60, 10, COL_DIM2, middle_left);
            canvas.fillCircle(W - 40, 10, 2, COL_DIM2);
            hdrTxt("RX", W - 36, 10, COL_DIM2, middle_left);
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
        for (int i = 0; i < 4; i++) {
            int ry   = TOP + i * DRO_ROW;
            int axcol = AX_STYLES[i].col;
            int rowbg = (i % 2 == 0) ? 0x0883 : 0x0863;
            canvas.fillRect(0, ry, DROW, DRO_ROW, rowbg);
            canvas.fillRect(0, ry, 3, DRO_ROW, axcol);
            hline(0, ry + DRO_ROW - 1, DROW, COL_BORDER);
            txt(AX_STYLES[i].letter, 7, ry + DRO_ROW * 3 / 10, axcol, TINY, middle_left);
            txt(pos_to_cstr(myAxes[i], num_digits()), DROW - 5, ry + DRO_ROW * 6 / 10, COL_WHITE, TINY, middle_right);
        }
        vline(DROW - 1, TOP, NAV_Y - TOP - FEED_H, COL_BORDER);

        // Visualizer area
        canvas.fillRect(VIZ_X, VIZ_Y, VIZ_W, VIZ_H, COL_PANEL2);
        txt("G-code Visualizer", VIZ_X + VIZ_W / 2, VIZ_Y + VIZ_H / 2 - 8, COL_DIM, TINY, middle_center);
        txt("(connect to machine)", VIZ_X + VIZ_W / 2, VIZ_Y + VIZ_H / 2 + 8, COL_DIM, TINY, middle_center);

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

    // ── Files screen ─────────────────────────────────────────────────────────
    void drawFilesScreen() {
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
        // Action bar
        if (fileSelected >= 0 && fileSelected < (int)fileList.size() && !fileList[fileSelected].isDir) {
            int abY = NAV_Y - 20;
            canvas.fillRect(0, abY, W, 20, COL_PANEL);
            hline(0, abY, W, COL_BORDER);
            f2s(fileList[fileSelected].name, 6, abY + 10, CYAN, middle_left);
            tintStrokeR(W - 42, abY + 2, 38, 16, 2, GREEN, GREEN, 40);
            f2("Run", W - 23, abY + 10, GREEN);
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
        int cols = 2, rows = (N_MACROS + 1) / 2;
        int pad = 8, gap = 6;
        int bw2 = (W - 2 * pad - gap) / cols;
        int avH = NAV_Y - TOP - pad * 2;
        int bh  = (avH - (rows - 1) * gap) / rows;
        for (int mi = 0; mi < N_MACROS; mi++) {
            int col2 = mi % cols, row = mi / cols;
            int bx   = pad + col2 * (bw2 + gap);
            int by   = TOP + pad + row * (bh + gap);
            _macroBtns[mi] = { bx, by, bw2, bh };
            tintStrokeR(bx, by, bw2, bh, 3, MACROS[mi].col, MACROS[mi].col, 24);
            canvas.fillCircle(bx + 10, by + bh / 2, 3, MACROS[mi].col);
            f2(MACROS[mi].name, bx + 18, by + bh / 2, COL_WHITE, middle_left);
        }
    }

    // ── Probe overlay ─────────────────────────────────────────────────────────
    void drawProbeOverlay() {
        canvas.fillRect(0, TOP, W, NAV_Y - TOP, 0x0001);
        canvas.fillRect(0, TOP, W, NAV_Y - TOP, 0x0863);

        int pw = W - 24, ph = H - TOP - BOT - 20;
        int px = (W - pw) / 2, py = TOP + 10, titleH = 24;

        fillR(px, py, pw, ph, 6, 0x0882);
        strokeR(px, py, pw, ph, 6, CYAN);
        canvas.fillRect(px, py, pw, titleH, 0x0863);
        hline(px, py + titleH, pw, COL_BORDER);
        canvas.fillCircle(px + 14, py + titleH / 2, 4, CYAN);
        f2("Select Probe Operation", px + 26, py + titleH / 2, COL_WHITE, middle_left);

        _probeClose = { px + pw - 22, py, 22, titleH };
        f2("X", px + pw - 11, py + titleH / 2, COL_DIM2);

        int listY = py + titleH + 4;
        int rowH  = (ph - titleH - 8) / N_PROBE_OPTS;
        for (int i = 0; i < N_PROBE_OPTS; i++) {
            int rx = px + 10, ry = listY + i * rowH;
            int rw = pw - 20, rh = rowH - 3;
            _probeRows[i] = { rx, ry, rw, rh };
            canvas.fillRoundRect(rx, ry, rw, rh, 3, i % 2 == 0 ? 0x0883 : 0x0863);
            canvas.fillRect(rx, ry, 3, rh, PROBE_OPTS[i].col);
            f2(PROBE_OPTS[i].label, rx + 10, ry + rh * 38 / 100, COL_WHITE, middle_left);
            f2(PROBE_OPTS[i].sub,   rx + 10, ry + rh * 72 / 100, COL_DIM2,  middle_left);
            f2(">",  rx + rw - 6,   ry + rh / 2, PROBE_OPTS[i].col, middle_right);
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
        f2("Send $X to unlock", cx, cy - ph / 2 + 40, COL_WHITE2);

        int ubw = 160, ubh = 44;
        int ubx = cx - ubw / 2, uby = cy - ubh / 2 + 12;
        _unlockBtn = { ubx, uby, ubw, ubh };
        tintStrokeR(ubx, uby, ubw, ubh, 6, RED, RED, 60);
        f2("UNLOCK  ($X)", cx, uby + ubh / 2, COL_WHITE);
    }

public:
    TabScene() : Scene("TabUI") {}

    void onEntry(void* arg = nullptr) override {
        sprite_offset = { 0, 0 };
        termLines.clear();
        termLines.push_back({ "> $I",              COL_DIM2 });
        termLines.push_back({ "[MSG:FluidDial UI]", GREEN   });
        fnc_realtime(StatusReport);
        request_file_list("/sd");
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
        // MPG handwheel: always takes priority when an axis is selected
        if (mpgAxis >= 0) {
            static const char axChar[] = { 'X', 'Y', 'Z', 'A' };
            float dist = delta * mpgSteps[mpgStepIdx];
            // Feed rate scales with step size: 0.1mm→100, 1mm→500, 10mm→2000
            static const int jogFeed[] = { 100, 500, 2000 };
            int feed = jogFeed[mpgStepIdx];
            send_linef("$J=G91 %c%.3f F%d", axChar[mpgAxis], dist, feed);
            reDisplay();
            return;
        }
        if (_tab == 0 && (state == Cycle || state == Hold)) {
            if (delta > 0 && myFro < 200) fnc_realtime(FeedOvrFinePlus);
            if (delta < 0 && myFro > 10)  fnc_realtime(FeedOvrFineMinus);
            reDisplay();
        }
        if (_tab == 2) {
            fileScroll = std::max(0, fileScroll + delta);
            reDisplay();
        }
        if (_tab == 3) {
            termScroll = std::max(0, termScroll - delta);
            int cmdY   = NAV_Y - CMD_H;
            int outH   = cmdY - TOP - 4;
            int maxL   = (outH - 4) / OUT_LH;
            int maxScroll = std::max(0, (int)termLines.size() - maxL);
            termScroll = std::min(termScroll, maxScroll);
            reDisplay();
        }
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
                    if (_tab == 2) request_file_list(filePath.c_str());
                    reDisplay(); return;
                }
            }
            return;
        }

        // DRO screen
        if (_tab == 0) {
            if (hit(_feedMinus, x, y)) {
                fnc_realtime(FeedOvrCoarseMinus);  // -10% per press
                reDisplay(); return;
            }
            if (hit(_feedPlus, x, y)) {
                fnc_realtime(FeedOvrCoarsePlus);   // +10% per press
                reDisplay(); return;
            }
            // Viz tabs - use wider touch area (40px) for easier tapping
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
                        request_file_list(filePath.c_str());
                    } else {
                        fileSelected = (fileSelected == fi) ? -1 : fi;
                        reDisplay();
                    }
                }
                return;
            }
            // Run button
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

        // Macros screen
        if (_tab == 4) {
            for (int mi = 0; mi < N_MACROS; mi++) {
                if (hit(_macroBtns[mi], x, y) && MACROS[mi].cmd[0]) {
                    send_line(MACROS[mi].cmd);
                    termLines.push_back({ std::string("> ") + MACROS[mi].cmd, COL_DIM2 });
                    reDisplay(); return;
                }
            }
        }
    }

    void onLeftFlick()  override {}
    void onRightFlick() override {}
    void onUpFlick() override {
        // Flick up = finger moves up = scroll down toward latest
        if (_tab == 3) {
            termScroll = std::max(0, termScroll - 3);
            reDisplay();
        }
    }
    void onDownFlick() override {
        // Flick down = finger moves down = scroll up into history
        if (_tab == 3) {
            termScroll += 3;
            int cmdY  = NAV_Y - CMD_H;
            int outH  = cmdY - TOP - 4;
            int maxL  = (outH - 4) / OUT_LH;
            int maxSc = std::max(0, (int)termLines.size() - maxL);
            termScroll = std::min(termScroll, maxSc);
            reDisplay();
        }
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
