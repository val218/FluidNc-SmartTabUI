// Settings.cpp — Load/save app settings via NVS.
#include "Settings.h"
#include "NVS.h"

static nvs_handle_t _nvs = nullptr;

void settings_load(AppSettings& s) {
    if (!_nvs) _nvs = nvs_init("tabui_cfg");
    int v = 0;
    nvs_get_i32(_nvs, "simMode", &v); s.simMode = (v != 0);
    v = (int)Theme::Dark;
    nvs_get_i32(_nvs, "theme",   &v); s.theme   = (Theme)v;
    v = (int)DROAxes::XYZ;
    nvs_get_i32(_nvs, "axes",    &v); s.axes    = (DROAxes)v;
}

void settings_save(const AppSettings& s) {
    if (!_nvs) _nvs = nvs_init("tabui_cfg");
    nvs_set_i32(_nvs, "simMode", s.simMode ? 1 : 0);
    nvs_set_i32(_nvs, "theme",   (int)s.theme);
    nvs_set_i32(_nvs, "axes",    (int)s.axes);
}

#include "TabScene.h"

void settings_applyTheme(Theme t) {
    tabui_setTheme((int)t);
}
