// ardmain.cpp — Tab UI boot for FluidDial JC2432W328C
#include "System.h"
#include "FileParser.h"
#include "Scene.h"
#include "TabScene.h"
#include "SimMode.h"
#include "Settings.h"
#include "driver/gpio.h"
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
#define S_WHITE   0xDF1D
#define S_CYAN    0x07FF
#define S_GREEN   0x07E0
#define S_ORANGE  0xFD20
#define S_RED     0xF800
#define S_BLUE    0x001F

// ── Helpers ───────────────────────────────────────────────────────────────────
static void sBtn(int x, int y, int w, int h, int bg, int border, const char* label, int col) {
    display.fillRoundRect(x, y, w, h, 5, bg);
    display.drawRoundRect(x, y, w, h, 5, border);
    display.setTextDatum(middle_center);
    display.setTextColor(col);
    display.setFont(&fonts::Font2);
    display.drawString(label, x + w/2, y + h/2);
}

static bool touchIn(int tx, int ty, int x, int y, int w, int h) {
    return tx >= x && tx < x+w && ty >= y && ty < y+h;
}

// ── Check enable button ───────────────────────────────────────────────────────
static bool readEnableNow() {
    uint8_t val = 0xFF;
    i2c_master_read_from_device(I2C_NUM_1, 0x20, &val, 1, pdMS_TO_TICKS(20));
    return !(val & 0x40);  // P6 active-LOW
}

// ── Settings menu ─────────────────────────────────────────────────────────────
static void drawSettingsMenu(const AppSettings& s, bool mpgOk) {
    display.fillScreen(S_BG);
    int cx = display.width() / 2;

    // Title
    display.setFont(&fonts::Font2);
    display.setTextDatum(middle_center);
    display.setTextColor(S_WHITE);
    display.drawString("Settings", cx, 16);

    display.setFont(&fonts::Font0);
    display.setTextColor(S_DIM);
    display.drawString(mpgOk ? "MPG: Connected" : "MPG: Not detected", cx, 30);

    int y = 44, rowH = 36, pad = 8, optW = 80, optH = 26;

    // ── Mode ─────────────────────────────────────────────────────────────────
    display.setFont(&fonts::Font0);
    display.setTextDatum(middle_left);
    display.setTextColor(S_DIM);
    display.drawString("MODE", pad, y + rowH/2);

    sBtn(80,  y+5, optW, optH, s.simMode ? S_PANEL : 0x0019, s.simMode ? S_BORDER : S_BLUE,  "Normal",     s.simMode ? S_DIM : S_WHITE);
    sBtn(168, y+5, optW, optH, s.simMode ? 0x2800  : S_PANEL, s.simMode ? S_ORANGE : S_BORDER, "Simulation", s.simMode ? S_ORANGE : S_DIM);
    y += rowH;

    // ── Theme ────────────────────────────────────────────────────────────────
    display.setTextDatum(middle_left);
    display.setTextColor(S_DIM);
    display.setFont(&fonts::Font0);
    display.drawString("THEME", pad, y + rowH/2);

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
    display.setTextDatum(middle_left);
    display.setTextColor(S_DIM);
    display.setFont(&fonts::Font0);
    display.drawString("AXES", pad, y + rowH/2);

    const char* axOpts[] = { "XYZ", "XYZA", "XY", "XYYZ" };
    int aw = 66, ax0 = 76;
    for (int i = 0; i < 4; i++) {
        bool sel = ((int)s.axes == i);
        sBtn(ax0 + i*(aw+3), y+5, aw, optH, sel ? 0x0019 : S_PANEL,
             sel ? S_GREEN : S_BORDER, axOpts[i], sel ? S_WHITE : S_DIM);
    }
    y += rowH;

    // ── Save / Boot button ───────────────────────────────────────────────────
    y += 6;
    sBtn(cx - 80, y, 160, 32, 0x0C00, S_GREEN, "Save & Boot", S_GREEN);
}

static void runSettingsMenu(AppSettings& s) {
    uint8_t pcfVal = 0xFF;
    bool mpgOk = (i2c_master_read_from_device(
        I2C_NUM_1, 0x20, &pcfVal, 1, pdMS_TO_TICKS(20)) == ESP_OK);

    drawSettingsMenu(s, mpgOk);

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
        y += rowH + 6;

        // Save & Boot
        if (touchIn(tx,ty, display.width()/2-80, y, 160, 32)) {
            settings_save(s);
            return;
        }

        drawSettingsMenu(s, mpgOk);
    }
}

// ── Background task: MPG switches + e-stop ────────────────────────────────────
static void mpgTask(void*) {
    static bool     lastEstop    = true;
    static bool     resetPending = false;
    static uint32_t resetTime    = 0;
    static uint32_t unlockTime   = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(50));
        readMpgSwitches();
        uint32_t now = millis();
        bool estopHigh = (gpio_get_level(GPIO_NUM_17) != 0);
        simMode_setEstop(!estopHigh);
        if (!estopHigh && lastEstop) { fnc_realtime(FeedHold); resetPending = true; resetTime = now; }
        if (estopHigh  && !lastEstop) unlockTime = now + 200;
        lastEstop = estopHigh;
        if (resetPending && now - resetTime  >= 50) { resetPending = false; fnc_realtime(Reset); }
        if (unlockTime   && now >= unlockTime)       { unlockTime = 0; send_line("$X"); }
    }
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    init_system();
    force_landscape();

    canvas.deleteSprite();
    canvas.setColorDepth(8);
    canvas.createSprite(display.width(), display.height());

    show_logo();
    delay_ms(1500);

    // Load saved settings
    AppSettings s;
    settings_load(s);

    // Check enable button during logo — if held, show settings menu
    if (readEnableNow()) {
        runSettingsMenu(s);
    }

    // Apply settings
    tabui_setTheme((int)s.theme);
    tabui_setAxes((int)s.axes);
    if (s.simMode) simMode_enable();

    display.fillScreen(0x0862);
    dbg_printf("FluidNC Pendant %s  [%s | theme=%d | axes=%d]\n",
               git_info, s.simMode ? "SIM" : "Normal", (int)s.theme, (int)s.axes);
    fnc_realtime(StatusReport);
    activate_scene(getTabScene());

    xTaskCreatePinnedToCore(mpgTask, "mpg", 2048, nullptr, 1, nullptr, 0);
}

void loop() {
    fnc_poll();
    dispatch_events();
}
