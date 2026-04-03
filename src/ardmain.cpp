// ardmain.cpp — Tab UI boot for FluidDial JC2432W328C
// Copyright (c) 2024 FluidDial contributors. GPLv3 licence.

#include "System.h"
#include "FileParser.h"
#include "Scene.h"
#include "TabScene.h"
#include "SimMode.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern void        show_logo();
extern const char* git_info;

// ── Boot choice screen ────────────────────────────────────────────────────────
// Always shown after logo. Returns true = simulation mode selected.
// TO REMOVE SIMULATION: delete the Simulation button and return false always.
static bool bootChoiceScreen() {
    display.fillScreen(0x0862);

    int cx = display.width()  / 2;
    int cy = display.height() / 2;

    // Check PCF8574 presence and show status
    uint8_t val = 0xFF;
    bool mpgFound = (i2c_master_read_from_device(
        I2C_NUM_1, 0x20, &val, 1, pdMS_TO_TICKS(20)) == ESP_OK);

    display.setFont(&fonts::Font2);
    display.setTextDatum(middle_center);
    display.setTextColor(mpgFound ? 0x07E0 : 0xFD20);  // green or orange
    display.drawString(mpgFound ? "MPG: Connected" : "MPG: Not detected", cx, cy - 68);

    // Buttons
    int bw = 120, bh = 48, gap = 16;
    int nx = cx - bw - gap / 2;   // Normal (left)
    int sx = cx + gap / 2;         // Simulation (right)
    int by = cy - 14;

    // Normal button
    display.fillRoundRect(nx, by, bw, bh, 6, 0x0883);
    display.drawRoundRect(nx, by, bw, bh, 6, 0x9D17);
    display.setTextColor(0xDF1D);
    display.drawString("Normal", nx + bw / 2, by + bh / 2);

    // Simulation button — TO REMOVE: delete these 4 lines
    display.fillRoundRect(sx, by, bw, bh, 6, 0x2800);
    display.drawRoundRect(sx, by, bw, bh, 6, 0xFD20);
    display.setTextColor(0xFD20);
    display.drawString("Simulation", sx + bw / 2, by + bh / 2);

    // Hints
    display.setFont(&fonts::Font0);
    display.setTextColor(0x52F0);
    display.setTextDatum(middle_center);
    display.drawString("Tap DRO rows to simulate axis movement", cx, by + bh + 20);
    display.drawString("Auto-boots Normal in 10 seconds", cx, by + bh + 34);

    // Wait for tap, 10s timeout → Normal
    uint32_t deadline = millis() + 10000;
    while (millis() < deadline) {
        touch.update(millis());
        auto t = touch.getDetail();
        if (t.wasClicked()) {
            int tx = t.x, ty = t.y;
            if (tx >= nx && tx < nx + bw && ty >= by && ty < by + bh) return false;
            if (tx >= sx && tx < sx + bw && ty >= by && ty < by + bh) return true;
        }
        // Countdown refresh
        uint32_t secs = (deadline - millis()) / 1000 + 1;
        char buf[8]; snprintf(buf, sizeof(buf), "%lus", secs);
        display.setTextColor(0x0862);  // erase
        display.drawString("   ", cx + 68, by + bh + 34);
        display.setTextColor(0x52F0);
        display.drawString(buf, cx + 68, by + bh + 34);
        delay(250);
    }
    return false;
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

        if (!estopHigh && lastEstop) {
            fnc_realtime(FeedHold);
            resetPending = true;
            resetTime    = now;
        }
        if (estopHigh && !lastEstop) unlockTime = now + 200;
        lastEstop = estopHigh;

        if (resetPending && now - resetTime >= 50)
            { resetPending = false; fnc_realtime(Reset); }
        if (unlockTime && now >= unlockTime)
            { unlockTime = 0; send_line("$X"); }
    }
}

void setup() {
    init_system();
    force_landscape();

    canvas.deleteSprite();
    canvas.setColorDepth(8);
    canvas.createSprite(display.width(), display.height());

    show_logo();
    delay_ms(1500);

    bool sim = bootChoiceScreen();
    if (sim) simMode_enable();

    display.fillScreen(0x0862);
    dbg_printf("FluidNC Pendant %s  [%s]\n", git_info, sim ? "SIM" : "TabUI");
    fnc_realtime(StatusReport);
    activate_scene(getTabScene());

    xTaskCreatePinnedToCore(mpgTask, "mpg", 2048, nullptr, 1, nullptr, 0);
}

void loop() {
    fnc_poll();
    dispatch_events();
}
