// ardmain.cpp — Tab UI boot for FluidDial JC2432W328C
#include "System.h"
#include "FileParser.h"
#include "Scene.h"
#include "TabScene.h"
#include "SimMode.h"
#include "Settings.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "Encoder.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdio>

extern void        show_logo();
extern const char* git_info;

// ── Colours used before theme is applied ─────────────────────────────────────
#define S_BG      0x0862
#define S_PANEL   0x10A3
#define S_BORDER  0x2147
#define S_DIM     0x52F0
#define S_DIM2    0x31C9
#define S_WHITE   0xDF1D
#define S_CYAN    0x07FF
#define S_GREEN   0x07E0
#define S_ORANGE  0xFD20
#define S_RED     0xF800
#define S_BLUE    0x001F

// ── Helpers ───────────────────────────────────────────────────────────────────
static void sBtn(int x, int y, int w, int h, int bg, int border, const char* label, int col) {
    canvas.fillRoundRect(x, y, w, h, 5, bg);
    canvas.drawRoundRect(x, y, w, h, 5, border);
    canvas.setTextDatum(middle_center);
    canvas.setTextColor(col);
    canvas.setFont(&fonts::Font2);
    canvas.drawString(label, x + w/2, y + h/2);
}

static bool touchIn(int tx, int ty, int x, int y, int w, int h) {
    return tx >= x && tx < x+w && ty >= y && ty < y+h;
}

// ── Check enable button ───────────────────────────────────────────────────────
static bool readEnableNow() {
    uint8_t val = 0xFF;
    i2c_master_read_from_device(I2C_NUM_1, 0x20, &val, 1, pdMS_TO_TICKS(50));
    return !(val & 0x40);  // P6 active-LOW
}

// ── Settings menu ─────────────────────────────────────────────────────────────
static void drawSettingsMenu(const AppSettings& s, bool mpgOk) {
    canvas.fillScreen(S_BG);
    int cx = display.width() / 2;

    // Title
    canvas.setFont(&fonts::Font2);
    canvas.setTextDatum(middle_center);
    canvas.setTextColor(S_WHITE);
    canvas.drawString("Settings", cx, 16);

    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(S_DIM);
    canvas.drawString(mpgOk ? "MPG: Connected" : "MPG: Not detected", cx, 30);

    int y = 44, rowH = 36, pad = 8, optW = 80, optH = 26;

    // ── Mode ─────────────────────────────────────────────────────────────────
    canvas.setFont(&fonts::Font0);
    canvas.setTextDatum(middle_left);
    canvas.setTextColor(S_DIM);
    canvas.drawString("MODE", pad, y + rowH/2);

    sBtn(80,  y+5, optW, optH, s.simMode ? S_PANEL : 0x0019, s.simMode ? S_BORDER : S_BLUE,  "Normal",     s.simMode ? S_DIM : S_WHITE);
    sBtn(168, y+5, optW, optH, s.simMode ? 0x2800  : S_PANEL, s.simMode ? S_ORANGE : S_BORDER, "Simulation", s.simMode ? S_ORANGE : S_DIM);
    y += rowH;

    // ── Theme ────────────────────────────────────────────────────────────────
    canvas.setTextDatum(middle_left);
    canvas.setTextColor(S_DIM);
    canvas.setFont(&fonts::Font0);
    canvas.drawString("THEME", pad, y + rowH/2);

    const char* themes[] = { "Dark", "Neutral", "Light" };
    int tw = 72, tx0 = 76;
    for (int i = 0; i < 3; i++) {
        bool sel = ((int)s.theme == i);
        int bc = sel ? S_CYAN : S_BORDER;
        int bg = sel ? 0x0019 : S_PANEL;
        sBtn(tx0 + i*(tw+4), y+5, tw, optH, bg, bc, themes[i], sel ? S_WHITE : S_DIM);
    }
    y += rowH;

    // ── DRO Axes ─────────────────────────────────────────────────────────────
    canvas.setTextDatum(middle_left);
    canvas.setTextColor(S_DIM);
    canvas.setFont(&fonts::Font0);
    canvas.drawString("AXES", pad, y + rowH/2);

    const char* axOpts[] = { "XYZ", "XYZA", "XY", "XYYZ" };
    int aw = 66, ax0 = 76;
    for (int i = 0; i < 4; i++) {
        bool sel = ((int)s.axes == i);
        sBtn(ax0 + i*(aw+3), y+5, aw, optH, sel ? 0x0019 : S_PANEL,
             sel ? S_GREEN : S_BORDER, axOpts[i], sel ? S_WHITE : S_DIM);
    }
    y += rowH;

    // ── P6 Enable button mode ────────────────────────────────────────────────
    canvas.setTextDatum(middle_left);
    canvas.setTextColor(S_DIM);
    canvas.setFont(&fonts::Font0);
    canvas.drawString("P6 BTN", pad, y + rowH/2);

    const char* enOpts[] = { "Gate All", "Touch", "Jog", "Macro", "Off" };
    int enw = 52, enx0 = 76;
    for (int i = 0; i < 5; i++) {
        bool sel = ((int)s.enableMode == i);
        sBtn(enx0 + i*(enw+2), y+5, enw, optH, sel ? 0x0019 : S_PANEL,
             sel ? S_CYAN : S_BORDER, enOpts[i], sel ? S_WHITE : S_DIM);
    }
    y += rowH;

    // Macro index selector — only visible in Macro mode
    if (s.enableMode == EnableMode::MacroBtn) {
        canvas.setTextDatum(middle_left);
        canvas.setTextColor(S_DIM);
        canvas.setFont(&fonts::Font0);
        canvas.drawString("MACRO#", pad, y + rowH/2);
        // Show current macro name
        char mname[24];
        // We can't access MACROS vector here, just show index
        snprintf(mname, sizeof(mname), "Macro %d", s.enableMacro + 1);
        sBtn(enx0, y+5, 120, optH, S_PANEL, S_BORDER, mname, S_DIM2);
        sBtn(enx0 + 124, y+5, 30, optH, S_PANEL, S_BORDER, "-", S_WHITE);
        sBtn(enx0 + 158, y+5, 30, optH, S_PANEL, S_BORDER, "+", S_WHITE);
        y += rowH;
    }

    // ── Monitor + Save buttons ─────────────────────────────────────────────
    y += 4;
    int cx2 = display.width() / 2;
    sBtn(cx2 - 156, y, 96, 28, S_PANEL, S_CYAN,   "Inputs",      S_CYAN);
    sBtn(cx2 - 54,  y, 96, 28, S_PANEL, S_ORANGE, "UART",        S_ORANGE);
    sBtn(cx2 + 50,  y, 96, 28, 0x0C00,  S_GREEN,  "Save & Boot", S_GREEN);
    canvas.pushSprite(0, 0);
}


// ── UART Terminal Monitor ─────────────────────────────────────────────────────
// Shows live raw UART data from FluidNC and lets user test baud rate
static void runUartMonitor() {
    extern uart_port_t fnc_uart_port;

    // Get current baud rate
    uint32_t baud = 0;
    uart_get_baudrate(fnc_uart_port, &baud);

    // Scrolling line buffer
    static char lines[12][64];
    static int  nLines = 0;
    static char lineBuf[64];
    static int  lineLen = 0;
    memset(lines, 0, sizeof(lines));
    nLines  = 0;
    lineLen = 0;

    // Baud rate options
    const uint32_t baudOpts[] = { 9600, 19200, 38400, 57600, 115200, 250000, 500000, 1000000 };
    const int      nBauds     = 8;
    int selBaud = -1;
    for (int i = 0; i < nBauds; i++) if (baudOpts[i] == baud) { selBaud = i; break; }

    uint32_t lastDraw = 0;

    for (;;) {
        // Drain incoming UART bytes into line buffer
        uint8_t c;
        while (uart_read_bytes(fnc_uart_port, &c, 1, 0) == 1) {
            if (c == '\n' || c == '\r') {
                if (lineLen > 0) {
                    lineBuf[lineLen] = 0;
                    // Scroll up
                    if (nLines < 12) {
                        strncpy(lines[nLines++], lineBuf, 63);
                    } else {
                        memmove(lines[0], lines[1], 11 * 64);
                        strncpy(lines[11], lineBuf, 63);
                    }
                    lineLen = 0;
                }
            } else if (lineLen < 63 && c >= 32 && c < 128) {
                lineBuf[lineLen++] = (char)c;
            }
        }

        // Redraw every 150ms
        uint32_t now = millis();
        if (now - lastDraw < 150) { delay(10); }
        else {
            lastDraw = now;
            canvas.fillScreen(S_BG);
            int cx = display.width() / 2;
            int W  = display.width();

            // Title
            canvas.setFont(&fonts::Font2);
            canvas.setTextDatum(middle_center);
            canvas.setTextColor(S_WHITE);
            canvas.drawString("UART Monitor", cx, 11);

            // Baud rate selector
            int bx = 4, by = 23, bw = 36, bh = 14, bg = 6;
            canvas.setFont(&fonts::Font0);
            canvas.setTextDatum(middle_left);
            canvas.setTextColor(S_DIM);
            canvas.drawString("Baud:", bx, by + bh/2);
            bx += 30;
            for (int i = 0; i < nBauds; i++) {
                bool sel = (i == selBaud);
                canvas.fillRoundRect(bx, by, bw, bh, 2, sel ? 0x0019 : S_PANEL);
                canvas.drawRoundRect(bx, by, bw, bh, 2, sel ? S_CYAN : S_BORDER);
                char bb[12];
                if (baudOpts[i] >= 1000000)
                    snprintf(bb, sizeof(bb), "%dM", (int)(baudOpts[i]/1000000));
                else if (baudOpts[i] >= 1000)
                    snprintf(bb, sizeof(bb), "%dk", (int)(baudOpts[i]/1000));
                else
                    snprintf(bb, sizeof(bb), "%d", (int)baudOpts[i]);
                canvas.setTextDatum(middle_center);
                canvas.setTextColor(sel ? S_WHITE : S_DIM);
                canvas.drawString(bb, bx + bw/2, by + bh/2);
                bx += bw + bg;
            }

            // Current baud confirmation
            char baudStr[24];
            snprintf(baudStr, sizeof(baudStr), "Active: %lu", (unsigned long)baud);
            canvas.setTextDatum(middle_right);
            canvas.setTextColor(S_DIM);
            canvas.drawString(baudStr, W - 4, by + bh/2);

            // Divider
            canvas.drawFastHLine(0, 40, W, S_BORDER);

            // Received lines
            int lineY = 42, lh = 13;
            canvas.setFont(&fonts::Font0);
            for (int i = 0; i < nLines; i++) {
                // Colour by first char
                int col = S_DIM2;
                if (lines[i][0] == '<') col = S_GREEN;
                else if (lines[i][0] == '[') col = S_CYAN;
                else if (lines[i][0] == 'e' || lines[i][0] == 'E') col = S_RED;
                else if (lines[i][0] == 'o') col = S_WHITE;
                canvas.setTextDatum(middle_left);
                canvas.setTextColor(col);
                canvas.drawString(lines[i], 2, lineY + i * lh);
            }

            // Partial current line (dim)
            if (lineLen > 0) {
                lineBuf[lineLen] = 0;
                canvas.setTextColor(S_DIM);
                canvas.drawString(lineBuf, 2, lineY + nLines * lh);
            }

            // Exit hint
            canvas.setTextDatum(middle_center);
            canvas.setTextColor(S_DIM);
            canvas.drawString("Hold e-stop 2s to exit", cx, display.height() - 6);
        }

        // Touch for baud rate selection
        touch.update(millis());
        auto t = touch.getDetail();
        if (t.wasClicked()) {
            int tx = t.x, ty = t.y;
            int bx2 = 34, by2 = 23, bw2 = 36, bh2 = 14, bg2 = 6;
            for (int i = 0; i < nBauds; i++) {
                if (touchIn(tx, ty, bx2, by2, bw2, bh2)) {
                    selBaud = i;
                    baud = baudOpts[i];
                    // Reconfigure UART baud rate on the fly
                    uart_set_baudrate(fnc_uart_port, baud);
                    nLines = 0; lineLen = 0;  // clear buffer after baud change
                }
                bx2 += bw2 + bg2;
            }
        }

        // Exit on e-stop held 2s
        static uint32_t estopHeld2 = 0;
        bool estop2 = (gpio_get_level(GPIO_NUM_17) == 0);
        if (estop2) { if (!estopHeld2) estopHeld2 = millis(); if (millis()-estopHeld2>2000) return; }
        else estopHeld2 = 0;
    }
}




// ── UART Terminal Monitor ─────────────────────────────────────────────────────

// ── Input Monitor ─────────────────────────────────────────────────────────────
// Live display of all inputs: PCF8574 bits, encoder count, touch, e-stop
static void runInputMonitor() {
    int16_t encBase = get_encoder();
    int     lastTx = -1, lastTy = -1;
    uint8_t lastPins = 0xFF;

    for (;;) {
        uint8_t pins = 0xFF;
        i2c_master_read_from_device(I2C_NUM_1, 0x20, &pins, 1, pdMS_TO_TICKS(20));
        int16_t encNow  = get_encoder();
        int16_t encDelta = encNow - encBase;
        bool estop = (gpio_get_level(GPIO_NUM_17) == 0);

        touch.update(millis());
        auto t = touch.getDetail();
        if (t.state == m5::touch_state_t::touch) { lastTx = t.x; lastTy = t.y; }

        canvas.fillScreen(S_BG);
        int cx = display.width() / 2;

        // Title + back hint
        canvas.setFont(&fonts::Font2);
        canvas.setTextDatum(middle_center);
        canvas.setTextColor(S_WHITE);
        canvas.drawString("Input Monitor", cx, 12);
        canvas.setFont(&fonts::Font0);
        canvas.setTextColor(S_DIM);
        canvas.drawString("Hold e-stop 2s to exit", cx, 24);

        canvas.setFont(&fonts::Font0);
        int y = 36, lh = 18;

        // ── PCF8574 bits ──────────────────────────────────────────────────
        canvas.setTextDatum(middle_left);
        canvas.setTextColor(S_DIM);
        canvas.drawString("PCF8574 raw:", 6, y);
        char hexbuf[12]; snprintf(hexbuf, sizeof(hexbuf), "0x%02X", pins);
        canvas.setTextColor(S_WHITE);
        canvas.drawString(hexbuf, 100, y);
        y += lh;

        // Individual bits with labels
        const char* bitLabels[] = { "P0 X-Axis", "P1 Y-Axis", "P2 Z-Axis", "P3 A-Axis",
                                    "P4 Step x10","P5 Step x100","P6 Enable","P7 unused" };
        for (int i = 0; i < 8; i++) {
            bool active = !(pins & (1 << i));
            int col = active ? S_GREEN : S_DIM;
            // Dot indicator
            canvas.fillCircle(10, y, 4, col);
            canvas.setTextDatum(middle_left);
            canvas.setTextColor(col);
            canvas.drawString(bitLabels[i], 18, y);
            canvas.setTextColor(active ? S_GREEN : S_DIM);
            canvas.drawString(active ? "LOW (active)" : "HIGH", 130, y);
            y += lh;
        }

        // ── E-stop ────────────────────────────────────────────────────────
        y += 2;
        canvas.fillCircle(10, y, 4, estop ? S_RED : S_DIM);
        canvas.setTextDatum(middle_left);
        canvas.setTextColor(estop ? S_RED : S_DIM);
        canvas.drawString("E-stop GPIO17", 18, y);
        canvas.drawString(estop ? "PRESSED" : "open", 130, y);
        y += lh;

        // ── Encoder ───────────────────────────────────────────────────────
        char encbuf[20];
        snprintf(encbuf, sizeof(encbuf), "Encoder delta: %+d", (int)encDelta);
        canvas.setTextColor(S_CYAN);
        canvas.drawString(encbuf, 6, y);
        y += lh;

        // ── Touch ─────────────────────────────────────────────────────────
        if (lastTx >= 0) {
            char tbuf[28];
            snprintf(tbuf, sizeof(tbuf), "Touch: x=%d y=%d", lastTx, lastTy);
            canvas.setTextColor(S_CYAN);
            canvas.drawString(tbuf, 6, y);
        } else {
            canvas.setTextColor(S_DIM);
            canvas.drawString("Touch: none", 6, y);
        }

        canvas.pushSprite(0, 0);  // flip buffer — no flicker

        // Exit: e-stop held for 2s
        static uint32_t estopHeldSince = 0;
        if (estop) {
            if (!estopHeldSince) estopHeldSince = millis();
            if (millis() - estopHeldSince > 2000) return;
        } else {
            estopHeldSince = 0;
        }

        delay(80);
    }
}

static void runSettingsMenu(AppSettings& s) {
    uint8_t pcfVal = 0xFF;
    bool mpgOk = (i2c_master_read_from_device(
        I2C_NUM_1, 0x20, &pcfVal, 1, pdMS_TO_TICKS(20)) == ESP_OK);

    // Use 16-bit canvas for settings menu (better colour accuracy than 8-bit)
    canvas.deleteSprite();
    canvas.setColorDepth(16);
    canvas.createSprite(display.width(), display.height());

    drawSettingsMenu(s, mpgOk);

    int cx = display.width() / 2;
    for (;;) {
        touch.update(millis());
        auto t = touch.getDetail();
        if (!t.wasClicked()) { delay(20); continue; }
        int tx = t.x, ty = t.y;

        int y = 44, rowH = 36, optW = 80, optH = 26;
        int tw = 72, tx0 = 76;
        int aw = 66, ax0 = 76;

        // Mode row
        if (touchIn(tx,ty, 80, y+5, optW, optH))  { s.simMode = false; }
        if (touchIn(tx,ty, 168,y+5, optW, optH))  { s.simMode = true;  }
        y += rowH;

        // Theme row
        for (int i=0;i<3;i++)
            if (touchIn(tx,ty, tx0+i*(tw+4), y+5, tw, optH)) s.theme = (Theme)i;
        y += rowH;

        // Axes row
        for (int i=0;i<4;i++)
            if (touchIn(tx,ty, ax0+i*(aw+3), y+5, aw, optH)) s.axes = (DROAxes)i;
        y += rowH;

        // P6 enable mode row
        { int enw=52, enx0b=76;
          for (int i=0;i<5;i++)
            if (touchIn(tx,ty, enx0b+i*(enw+2), y+5, enw, optH)) s.enableMode=(EnableMode)i;
        }
        y += rowH;

        // Macro index row (only shown in Macro mode)
        if (s.enableMode == EnableMode::MacroBtn) {
            int enx0b=76;
            if (touchIn(tx,ty, enx0b+124, y+5, 30, optH) && s.enableMacro > 0)  s.enableMacro--;
            if (touchIn(tx,ty, enx0b+158, y+5, 30, optH) && s.enableMacro < 15) s.enableMacro++;
            y += rowH;
        }
        y += 4;

        // Input Monitor
        if (touchIn(tx,ty, cx-156, y, 96, 28)) {
            runInputMonitor();
            drawSettingsMenu(s, mpgOk);
            continue;
        }
        // UART Monitor
        if (touchIn(tx,ty, cx-54, y, 96, 28)) {
            runUartMonitor();
            drawSettingsMenu(s, mpgOk);
            continue;
        }
        // Save & Boot
        if (touchIn(tx,ty, cx+50, y, 96, 28)) {
            settings_save(s);
            return;
        }

        drawSettingsMenu(s, mpgOk);
    }

    // Restore 8-bit canvas for main UI
    canvas.deleteSprite();
    canvas.setColorDepth(8);
    canvas.createSprite(display.width(), display.height());
}

// ── Background task: MPG switches + e-stop ────────────────────────────────────
static void mpgTask(void*) {
    static bool     lastEstop    = true;
    static bool     resetPending = false;
    static uint32_t resetTime    = 0;
    static uint32_t unlockTime   = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(30));  // 30ms: leaves headroom for 20ms I2C timeout
        readMpgSwitches();
        uint32_t now = millis();
        bool estopHigh = (gpio_get_level(GPIO_NUM_17) != 0);
        simMode_setEstop(!estopHigh);
        mpgSetEstop(!estopHigh);  // update header indicator
        if (!estopHigh && lastEstop) {
            fnc_realtime(FeedHold);
            resetPending = true;
            resetTime = now;
        }
        if (estopHigh && !lastEstop) {
            unlockTime = now + 200;
        }
        lastEstop = estopHigh;
        if (resetPending && now - resetTime  >= 50) { resetPending = false; fnc_realtime(Reset); }
        if (unlockTime   && now >= unlockTime)       { unlockTime = 0; send_line("$X"); }

        // E-stop + enable held 3s → hard reboot
        static uint32_t rebootHeld = 0;
        bool en2 = mpgEnableHeld();
        if (!estopHigh && en2) {
            if (!rebootHeld) rebootHeld = now;
            if (now - rebootHeld >= 3000) esp_restart();
        } else {
            rebootHeld = 0;
        }
    }
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    init_system();
    force_landscape();

    canvas.deleteSprite();
    canvas.setColorDepth(8);
    canvas.createSprite(display.width(), display.height());

    // Check enable BEFORE showing logo (catches early press)
    bool enterSettings = readEnableNow();

    show_logo();

    // ── Boot progress bar on logo screen ────────────────────────────────────
    int  W = display.width(), H = display.height();
    int  barX = 20, barY = H - 22, barW = W - 40, barH = 8, barR = 4;

    // Helper: draw progress bar frame + fill
    auto drawProgress = [&](int pct, const char* label, int barCol) {
        // Bar track
        display.fillRoundRect(barX, barY, barW, barH, barR, 0x2104);
        // Fill
        int fillW = barW * pct / 100;
        if (fillW > 0)
            display.fillRoundRect(barX, barY, fillW, barH, barR, barCol);
        // Label above bar
        display.fillRect(barX, barY - 14, barW, 12, 0x0000);
        display.setFont(&fonts::Font0);
        display.setTextDatum(middle_left);
        display.setTextColor(0x9D17);
        display.drawString(label, barX, barY - 8);
    };

    // Enable button hint (right of bar)
    display.setFont(&fonts::Font0);
    display.setTextDatum(middle_right);
    display.setTextColor(0x52F0);
    display.drawString("Hold EN for Settings", W - barX, barY - 8);

    // Step 1: Poll window — progress bar fills over 2 seconds
    uint32_t logoStart = millis();
    uint32_t rebootHeld = 0;
    while (millis() - logoStart < 2000) {
        uint32_t elapsed = millis() - logoStart;
        int pct = elapsed * 60 / 2000;  // fills to 60% during poll window

        bool en    = readEnableNow();
        bool estop = (gpio_get_level(GPIO_NUM_17) == 0);
        if (en) enterSettings = true;

        int barCol = enterSettings ? 0x07E0   // green: settings confirmed
                   : en            ? 0xFD20   // orange: button held
                                   : 0x065F;  // cyan: normal boot

        drawProgress(pct, enterSettings ? "Settings mode - release to enter"
                        : en            ? "Hold for Settings..."
                                       : "Booting...", barCol);

        // E-stop + enable → reboot countdown
        if (en && estop) {
            if (!rebootHeld) rebootHeld = millis();
            uint32_t held = millis() - rebootHeld;
            int rPct = held * 100 / 3000;
            display.fillRoundRect(barX, barY, barW * rPct / 100, barH, barR, 0xF800);
            display.setFont(&fonts::Font0);
            display.setTextDatum(middle_center);
            display.setTextColor(0xF800);
            display.drawString("REBOOTING...", W/2, barY - 8);
            if (held >= 3000) esp_restart();
        } else {
            rebootHeld = 0;
        }
        delay(50);
    }

    // Step 2: Load settings (60-75%)
    drawProgress(65, "Loading settings...", 0x065F);
    AppSettings s;
    settings_load(s);

    if (enterSettings) {
        drawProgress(75, "Entering settings...", 0x07E0);
        delay(300);
        runSettingsMenu(s);
    }

    // Step 3: Apply theme and config (75-90%)
    drawProgress(80, "Applying theme...", 0x065F);
    tabui_setTheme((int)s.theme);
    tabui_setAxes((int)s.axes);
    tabui_setEnableMode((int)s.enableMode, s.enableMacro);
    if (s.simMode) simMode_enable();

    // Step 4: Start UI (90-100%)
    drawProgress(92, "Starting UI...", 0x065F);
    dbg_printf("FluidNC Pendant %s  [%s | theme=%d | axes=%d]\n",
               git_info, s.simMode ? "SIM" : "Normal", (int)s.theme, (int)s.axes);
    fnc_realtime(StatusReport);
    activate_scene(getTabScene());

    drawProgress(100, "Ready", 0x07E0);
    delay(300);

    xTaskCreatePinnedToCore(mpgTask, "mpg", 4096, nullptr, 1, nullptr, 0);
    display.fillScreen(0x0862);
}

void loop() {
    fnc_poll();
    dispatch_events();
}
