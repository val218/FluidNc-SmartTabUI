// Settings.h — Persistent user settings stored in NVS flash.
#pragma once

enum class Theme   { Dark = 0, Neutral = 1, Light = 2 };
enum class DROAxes { XYZ = 0, XYZA = 1, XY = 2, XYYZ = 3 };

// P6 enable button behaviour
enum class EnableMode {
    EnableGate  = 0,  // must hold to allow jogging + touch (default)
    TouchOnly   = 1,  // gates touch only, not jogging
    JogOnly     = 2,  // gates jogging only, not touch
    MacroBtn    = 3,  // acts as a macro button (no gating)
    Disabled    = 4,  // P6 ignored entirely
};

struct AppSettings {
    bool       simMode    = false;
    Theme      theme      = Theme::Dark;
    DROAxes    axes       = DROAxes::XYZ;
    EnableMode enableMode = EnableMode::EnableGate;
    int        enableMacro = 0;  // macro index to fire when MacroBtn mode
};

void settings_load(AppSettings& s);
void settings_save(const AppSettings& s);
void settings_applyTheme(Theme t);
