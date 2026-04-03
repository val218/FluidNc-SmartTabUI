// Copyright (c) 2023 Mitch Bradley
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.
// Tab UI build: stripped to only what TabScene needs.

#include "System.h"
#include "Drawing.h"

void drawBackground(int color)          { canvas.fillSprite(color); }
void drawBackground(LGFX_Sprite* sprite){ sprite->pushSprite(0, 0); }

LGFX_Sprite* createPngBackground(const char* filename) {
    LGFX_Sprite* sprite = new LGFX_Sprite(&canvas);
    sprite->setColorDepth(canvas.getColorDepth());
    sprite->createSprite(canvas.width(), canvas.height());
    drawPngFile(sprite, filename, 0, 0);
    return sprite;
}

void drawFilledCircle(int x, int y, int radius, int fillcolor)                          { canvas.fillCircle(x, y, radius, fillcolor); }
void drawFilledCircle(Point xy, int radius, int fillcolor)                              { Point d = xy.to_display(); drawFilledCircle(d.x, d.y, radius, fillcolor); }
void drawCircle(int x, int y, int radius, int thickness, int outlinecolor)              { for (int i = 0; i < thickness; i++) canvas.drawCircle(x, y, radius - i, outlinecolor); }
void drawCircle(Point xy, int radius, int thickness, int outlinecolor)                  { Point d = xy.to_display(); drawCircle(d.x, d.y, radius, thickness, outlinecolor); }
void drawOutlinedCircle(int x, int y, int radius, int fillcolor, int outlinecolor)      { canvas.fillCircle(x, y, radius, fillcolor); canvas.drawCircle(x, y, radius, outlinecolor); }
void drawOutlinedCircle(Point xy, int radius, int fillcolor, int outlinecolor)          { Point d = xy.to_display(); drawOutlinedCircle(d.x, d.y, radius, fillcolor, outlinecolor); }
void drawRect(int x, int y, int width, int height, int radius, int bgcolor)             { canvas.fillRoundRect(x, y, width, height, radius, bgcolor); }
void drawRect(Point xy, int width, int height, int radius, int bgcolor)                 { Point off = { width/2, -height/2 }; Point d = (xy - off).to_display(); drawRect(d.x, d.y, width, height, radius, bgcolor); }
void drawRect(Point xy, Point wh, int radius, int bgcolor)                              { drawRect(xy, wh.x, wh.y, radius, bgcolor); }
void drawOutlinedRect(int x, int y, int width, int height, int bgcolor, int outlinecolor){ canvas.fillRoundRect(x, y, width, height, 5, bgcolor); canvas.drawRoundRect(x, y, width, height, 5, outlinecolor); }
void drawOutlinedRect(Point xy, int width, int height, int bgcolor, int outlinecolor)   { Point d = xy.to_display(); drawOutlinedRect(d.x, d.y, width, height, bgcolor, outlinecolor); }
void drawPngFile(const char* filename, Point xy)                                        { drawPngFile(filename, xy.x, xy.y); }
void drawPngBackground(const char* filename)                                            { drawPngFile(filename, 0, 0); }

void refreshDisplay() {
    display.startWrite();
    canvas.pushSprite(sprite_offset.x, sprite_offset.y);
    display.endWrite();
}

void drawError() {}  // Not used in TabUI

