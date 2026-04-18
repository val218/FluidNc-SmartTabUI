// Settings.cpp — Load/save app settings via NVS.
#include "Settings.h"
#include "NVS.h"
#include "TabScene.h"

static nvs_handle_t _nvs = 0;

void settings_load(AppSettings& s) {
    if (!_nvs) _nvs = nvs_init("tabui_cfg");
    int v;
    v = 0;               nvs_get_i32(_nvs, "simMode",     &v); s.simMode    = (v != 0);
    v = (int)Theme::Dark;  nvs_get_i32(_nvs, "theme",     &v); s.theme      = (Theme)v;
    v = (int)DROAxes::XYZ; nvs_get_i32(_nvs, "axes",      &v); s.axes       = (DROAxes)v;
    v = 0;               nvs_get_i32(_nvs, "enableMode",  &v); s.enableMode  = (EnableMode)v;
    v = 0;               nvs_get_i32(_nvs, "enableMacro", &v); s.enableMacro = v;
    v = 1250;            nvs_get_i32(_nvs, "workX",       &v); s.workX       = v;
    v = 2500;            nvs_get_i32(_nvs, "workY",       &v); s.workY       = v;
    v = 0;               nvs_get_i32(_nvs, "homeCorner",  &v); s.homeCorner  = (HomeCorner)v;
}

void settings_save(const AppSettings& s) {
    if (!_nvs) _nvs = nvs_init("tabui_cfg");
    nvs_set_i32(_nvs, "simMode",     s.simMode ? 1 : 0);
    nvs_set_i32(_nvs, "theme",       (int)s.theme);
    nvs_set_i32(_nvs, "axes",        (int)s.axes);
    nvs_set_i32(_nvs, "enableMode",  (int)s.enableMode);
    nvs_set_i32(_nvs, "enableMacro", s.enableMacro);
    nvs_set_i32(_nvs, "workX",       s.workX);
    nvs_set_i32(_nvs, "workY",       s.workY);
    nvs_set_i32(_nvs, "homeCorner",  (int)s.homeCorner);
}

void settings_applyTheme(Theme t) { tabui_setTheme((int)t); }
