// ardmain.cpp — Tab UI boot for FluidDial JC2432W328C
// Copyright (c) 2024 FluidDial contributors. GPLv3 licence.

#include "System.h"
#include "FileParser.h"
#include "Scene.h"
#include "TabScene.h"

extern void        show_logo();
extern const char* git_info;

void setup() {
    // 1. Initialise hardware (capacitive touch, no calibration screen)
    init_system();

    // 2. Force landscape rotation - must be before canvas creation
    force_landscape();

    // 3. Create canvas at correct landscape size (320x240 x 8-bit = 75KB)
    canvas.setColorDepth(8);
    if (!canvas.createSprite(display.width(), display.height())) {
        // Allocation failed - try smaller fallback
        canvas.createSprite(240, 240);
    }

    // 4. Boot logo
    show_logo();
    delay_ms(1500);

    // Clear to dark background using direct display write (not canvas)
    display.fillScreen(0x0862);

    dbg_printf("FluidNC Pendant %s  [TabUI]\n", git_info);
    fnc_realtime(StatusReport);

    // 5. Start Tab UI
    activate_scene(getTabScene());
}

void loop() {
    fnc_poll();
    dispatch_events();
}
