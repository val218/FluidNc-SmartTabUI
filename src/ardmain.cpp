// ardmain.cpp — Tab UI boot for FluidDial JC2432W328C
// Copyright (c) 2024 FluidDial contributors. GPLv3 licence.

#include "System.h"
#include "FileParser.h"
#include "Scene.h"
#include "TabScene.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern void        show_logo();
extern const char* git_info;

// ── Background task: MPG switches + e-stop ────────────────────────────────────
// Runs on Core 0 at low priority, completely separate from the UI loop (Core 1).
// Reads PCF8574 over I2C and GPIO17 e-stop. Any I2C delay stays on Core 0.
static void mpgTask(void*) {
    static bool     lastEstop    = true;
    static bool     resetPending = false;
    static uint32_t resetTime    = 0;
    static uint32_t unlockTime   = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(50));  // 20Hz — plenty for switch positions

        // Read MPG switches via PCF8574
        readMpgSwitches();

        // E-stop: GPIO17 active-LOW
        uint32_t now = millis();
        bool estopHigh = (gpio_get_level(GPIO_NUM_17) != 0);

        if (!estopHigh && lastEstop) {       // pressed
            fnc_realtime(FeedHold);
            resetPending = true;
            resetTime    = now;
        }
        if (estopHigh && !lastEstop) {       // released
            unlockTime = now + 200;
        }
        lastEstop = estopHigh;

        if (resetPending && now - resetTime >= 50) {
            resetPending = false;
            fnc_realtime(Reset);
        }
        if (unlockTime && now >= unlockTime) {
            unlockTime = 0;
            send_line("$X");
        }
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
    display.fillScreen(0x0862);

    dbg_printf("FluidNC Pendant %s  [TabUI]\n", git_info);
    fnc_realtime(StatusReport);
    activate_scene(getTabScene());

    // Start MPG background task on Core 0 at low priority
    xTaskCreatePinnedToCore(
        mpgTask,    // function
        "mpg",      // name
        2048,       // stack bytes
        nullptr,    // parameter
        1,          // priority (1=low)
        nullptr,    // handle
        0           // Core 0
    );
}

void loop() {
    fnc_poll();
    dispatch_events();
}
