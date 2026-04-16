// TabScene.h
// 5-tab landscape UI for FluidDial JC2432W328C (320x240)
// Copyright (c) 2024 - FluidDial contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once
#include "Scene.h"

// Returns the singleton TabScene instance.
// Call activate_scene(getTabScene()) from ardmain.cpp instead of initMenus()
// to use the new tab-based UI.
Scene* getTabScene();
void   readMpgSwitches();
bool   mpgEnableHeld();   // true when P6 enable button is held
bool   tabui_touchGated();  // true when touch input should be blocked
void   tabui_setTheme(int theme);  // 0=Dark 1=Neutral 2=Light
void   tabui_setAxes(int axes);    // 0=XYZ 1=XYZA 2=XY 3=XYYZ
void   tabui_setEnableMode(int mode, int macro);  // configure P6 behaviour
int    tabui_getAxes();
