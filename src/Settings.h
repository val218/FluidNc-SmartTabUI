// Settings.h — Persistent user settings stored in NVS flash.
#pragma once

enum class Theme   { Dark = 0, Neutral = 1, Light = 2 };
enum class DROAxes { XYZ = 0, XYZA = 1, XY = 2, XYYZ = 3 };

enum class EnableMode {
    EnableGate  = 0,
    TouchOnly   = 1,
    JogOnly     = 2,
    MacroBtn    = 3,
    Disabled    = 4,
};

// Machine home corner position (which corner is X0,Y0)
enum class HomeCorner {
    BottomLeft  = 0,  // X0,Y0 = bottom-left  (typical)
    BottomRight = 1,  // X0,Y0 = bottom-right
    TopLeft     = 2,  // X0,Y0 = top-left
    TopRight    = 3,  // X0,Y0 = top-right
};

struct AppSettings {
    bool       simMode     = false;
    Theme      theme       = Theme::Dark;
    DROAxes    axes        = DROAxes::XYZ;
    EnableMode enableMode  = EnableMode::EnableGate;
    int        enableMacro = 0;
    // Machine work area
    int        workX       = 1250;   // mm — short axis
    int        workY       = 2500;   // mm — long axis
    HomeCorner homeCorner  = HomeCorner::BottomLeft;
};

void settings_load(AppSettings& s);
void settings_save(const AppSettings& s);
void settings_applyTheme(Theme t);
