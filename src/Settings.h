// Settings.h — Persistent user settings stored in NVS flash.
// Loaded at boot, saved when changed in the settings menu.
#pragma once

enum class Theme  { Dark = 0, Neutral = 1, Light = 2 };
enum class DROAxes { XYZ = 0, XYZA = 1, XY = 2, XYYZ = 3 };  // XYYZ = dual Y

struct AppSettings {
    bool     simMode  = false;
    Theme    theme    = Theme::Dark;
    DROAxes  axes     = DROAxes::XYZ;
};

void     settings_load(AppSettings& s);
void     settings_save(const AppSettings& s);

// Apply theme colours globally — call after load or change
void     settings_applyTheme(Theme t);
