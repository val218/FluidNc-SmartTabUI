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
