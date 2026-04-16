// Copyright (c) 2023 Mitch Bradley
// Tab UI build: stripped to essentials only.
#pragma once
#include "FluidNCModel.h"
#include "Text.h"

// draw primitives
LGFX_Sprite* createPngBackground(const char* filename);
void drawBackground(LGFX_Sprite* sprite);
void drawBackground(int color);

void drawFilledCircle(int x, int y, int radius, int fillcolor);
void drawFilledCircle(Point xy, int radius, int fillcolor);
void drawCircle(int x, int y, int radius, int thickness, int outlinecolor);
void drawCircle(Point xy, int radius, int thickness, int outlinecolor);
void drawOutlinedCircle(int x, int y, int radius, int fillcolor, int outlinecolor);
void drawOutlinedCircle(Point xy, int radius, int fillcolor, int outlinecolor);
void drawRect(int x, int y, int width, int height, int radius, int bgcolor);
void drawRect(Point xy, int width, int height, int radius, int bgcolor);
void drawRect(Point xy, Point wh, int radius, int bgcolor);
void drawOutlinedRect(int x, int y, int width, int height, int bgcolor, int outlinecolor);
void drawOutlinedRect(Point xy, int width, int height, int bgcolor, int outlinecolor);
void drawPngFile(const char* filename, Point xy);
void drawPngBackground(const char* filename);
void refreshDisplay();
void drawError();

extern Point sprite_offset;
