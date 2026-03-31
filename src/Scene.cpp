// Copyright (c) 2023 Mitch Bradley
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Scene.h"
#include "driver/gpio.h"
#include "System.h"
#include "TabScene.h"

#include <vector>

Scene* current_scene = nullptr;

int touchX;
int touchY;
int touchDeltaX;
int touchDeltaY;

std::vector<Scene*> scene_stack;

void activate_scene(Scene* scene, void* arg) {
    if (current_scene) current_scene->onExit();
    current_scene = scene;
    current_scene->onEntry(arg);
    current_scene->reDisplay();
}
void push_scene(Scene* scene, void* arg) {
    scene_stack.push_back(current_scene);
    activate_scene(scene, arg);
}
void pop_scene(void* arg) {
    if (scene_stack.size()) {
        Scene* last_scene = scene_stack.back();
        scene_stack.pop_back();
        activate_scene(last_scene, arg);
    }
}
void activate_at_top_level(Scene* scene, void* arg) {
    scene_stack.clear();
    activate_scene(scene, arg);
}
Scene* parent_scene() {
    return scene_stack.size() ? scene_stack.back() : nullptr;
}

bool touchIsCenter() {
    Point ctr = Point { touchX, touchY }.from_display();
    int center_radius = display_short_side() / 6;
    return (ctr.x * ctr.x + ctr.y * ctr.y) < (center_radius * center_radius);
}

void dispatch_button(bool pressed, int button) {
    switch (button) {
        case 0: pressed ? current_scene->onRedButtonPress()   : current_scene->onRedButtonRelease();   break;
        case 1: pressed ? current_scene->onDialButtonPress()  : current_scene->onDialButtonRelease();  break;
        case 2: pressed ? current_scene->onGreenButtonPress() : current_scene->onGreenButtonRelease(); break;
    }
}

void dispatch_touch() {
    static m5::touch_state_t last_touch_state = {};
    auto t = touch.getDetail();
    if (t.state != last_touch_state) {
        last_touch_state = t.state;
        touchX = t.x - sprite_offset.x;
        touchY = t.y - sprite_offset.y;
        int delta;
        if (screen_encoder(t.x, t.y, delta) && t.state == m5::touch_state_t::touch) {
            current_scene->onEncoder(delta);
            return;
        }
        int button;
        if (screen_button_touched(t.state == m5::touch_state_t::touch, t.x, t.y, button)) {
            if (t.state == m5::touch_state_t::touch)       dispatch_button(true,  button);
            else if (t.state == m5::touch_state_t::none)   dispatch_button(false, button);
            return;
        }
        if (touchX < 0) return;
        if (t.state == m5::touch_state_t::touch)         current_scene->onTouchPress();
        else if (t.state == m5::touch_state_t::none)     current_scene->onTouchRelease();
        if (t.wasClicked())      current_scene->onTouchClick();
        else if (t.wasHold())    current_scene->onTouchHold();
        else if (t.state == m5::touch_state_t::flick_end) {
            touchDeltaX = t.distanceX();
            touchDeltaY = t.distanceY();
            int absX = abs(touchDeltaX), absY = abs(touchDeltaY);
            if      (absY > 60 && absX < absY * 2)  touchDeltaY > 0 ? current_scene->onDownFlick()  : current_scene->onUpFlick();
            else if (absX > 60 && absY < absX * 2)  touchDeltaX > 0 ? current_scene->onRightFlick() : current_scene->onLeftFlick();
            else current_scene->onTouchFlick();
        }
    }
}

ActionHandler action = nullptr;
void schedule_action(ActionHandler _action) { action = _action; }

void dispatch_events() {
    // Poll MPG switches and e-stop every loop
    readMpgSwitches();

    // E-stop: GPIO8 is unusable (ESP32 internal flash SPI bus GPIO6-11).
    // GPIO34 is taken by lockout_pin. Use GPIO36 or GPIO39 for e-stop in hardware.
    // Disabled until hardware is confirmed safe.

    update_events();

    static int16_t oldEncoder = 0;
    int16_t newEncoder   = get_encoder();
    int16_t encoderDelta = newEncoder - oldEncoder;
    if (encoderDelta) {
        oldEncoder = newEncoder;
        int16_t scaledDelta = current_scene->scale_encoder(encoderDelta);
        if (scaledDelta && !ui_locked()) current_scene->onEncoder(scaledDelta);
    }

    if (!ui_locked()) {
        bool pressed; int button;
        if (switch_button_touched(pressed, button)) dispatch_button(pressed, button);
        dispatch_touch();
    }

    // On disconnect, stay on TabScene (it shows Disconnected state in header)
    if (!fnc_is_connected()) {
        if (state != Disconnected) {
            set_disconnected_state();
            activate_at_top_level(getTabScene());
        }
    }

    if (action) { action(); action = nullptr; }
}

static const char* setting_name(const char* base_name, int axis) {
    static char name[32];
    if (axis == -1) return base_name;
    sprintf(name, "%s%c", base_name, axisNumToChar(axis));
    return name;
}

void Scene::setPref(const char* name, int value)                               { setPref(name, -1, value); }
void Scene::getPref(const char* name, int* value)                              { getPref(name, -1, value); }
void Scene::setPref(const char* base_name, int axis, int value)                { if (_prefs) nvs_set_i32(_prefs, setting_name(base_name, axis), value); }
void Scene::getPref(const char* base_name, int axis, int* value)               { if (_prefs) nvs_get_i32(_prefs, setting_name(base_name, axis), reinterpret_cast<int32_t*>(value)); }
void Scene::setPref(const char* base_name, int axis, const char* value)        { if (_prefs) nvs_set_str(_prefs, setting_name(base_name, axis), value); }
void Scene::getPref(const char* base_name, int axis, char* value, int maxlen)  { if (_prefs) { size_t len = maxlen; nvs_get_str(_prefs, setting_name(base_name, axis), value, &len); } }

bool Scene::initPrefs() {
    if (_prefs) return false;
    _prefs = nvs_init(name());
    return _prefs;
}
int Scene::scale_encoder(int delta) {
    _encoder_accum += delta;
    int res = _encoder_accum / _encoder_scale;
    _encoder_accum %= _encoder_scale;
    return res;
}
void Scene::background() { system_background(); }

void act_on_state_change() { current_scene->onStateChange(previous_state); }
