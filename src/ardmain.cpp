// ardmain.cpp — Tab UI boot for FluidDial JC2432W328C
#include "System.h"
#include "Hardware2432.hpp"
#include "JobRecovery.h"
#include "FluidNCModel.h"
#include "FileParser.h"
#include "Scene.h"
#include "TabScene.h"
#include "SimMode.h"
#include "Settings.h"
#include "driver/gpio.h"
#include "esp_system.h"
extern "C" void fnc_term_inject(const char* line);
#include "driver/uart.h"
#include "Encoder.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdio>

extern void        show_logo();
extern const char* git_info;

// ── Colours used before theme is applied ─────────────────────────────────────
#define S_BG      0x0862
#define S_PANEL   0x10A3
#define S_BORDER  0x2147
#define S_DIM     0x52F0
#define S_DIM2    0x31C9
#define S_WHITE   0xDF1D
#define S_CYAN    0x07FF
#define S_GREEN   0x07E0
#define S_ORANGE  0xFD20
#define S_RED     0xF800
#define S_BLUE    0x001F

// ── Helpers ───────────────────────────────────────────────────────────────────
static void sBtn(int x, int y, int w, int h, int bg, int border, const char* label, int col) {
    display.fillRoundRect(x, y, w, h, 5, bg);
    display.drawRoundRect(x, y, w, h, 5, border);
    display.setTextDatum(middle_center);
    display.setTextColor(col);
    display.setFont(&fonts::Font2);
    display.drawString(label, x + w/2, y + h/2);
}

static bool touchIn(int tx, int ty, int x, int y, int w, int h) {
    return tx >= x && tx < x+w && ty >= y && ty < y+h;
}

// ── Check enable button ───────────────────────────────────────────────────────
static bool readEnableNow() {
    uint8_t val = 0xFF;
    i2c_master_read_from_device(I2C_NUM_1, 0x20, &val, 1, pdMS_TO_TICKS(50));
    return !(val & 0x40);  // P6 active-LOW
}

// ── Settings menu ─────────────────────────────────────────────────────────────
static lgfx::LGFX_Sprite* _sBuf = nullptr;  // settings back-buffer

static void drawSettingsMenu(const AppSettings& s, bool mpgOk, int scrollY = 0) {
    int W = display.width(), H = display.height(), cx = W/2;
    int rowH = 48, pad = 6, optH = 36;
    int bx0 = 68, bW = W - bx0 - 4;

    // Create back-buffer at screen size on first call (no flicker)
    if (!_sBuf) {
        _sBuf = new lgfx::LGFX_Sprite(&display);
        _sBuf->setColorDepth(8);
        _sBuf->createSprite(W, H);
    }
    _sBuf->fillSprite(S_BG);

    auto sB = [&](int x, int y2, int w, int h2, uint16_t bg, uint16_t bc,
                  const char* lb, uint16_t tc) {
        if (y2+h2 < 0 || y2 > H) return;
        _sBuf->fillRoundRect(x, y2, w, h2, 3, bg);
        _sBuf->drawRoundRect(x, y2, w, h2, 3, bc);
        _sBuf->setFont(&fonts::Font0);
        _sBuf->setTextDatum(middle_center);
        _sBuf->setTextColor(tc);
        _sBuf->drawString(lb, x+w/2, y2+h2/2);
    };

    auto lbTxt = [&](int ry, const char* t) {
        if (ry+rowH < 0 || ry > H) return;
        _sBuf->setFont(&fonts::Font0);
        _sBuf->setTextDatum(middle_left);
        _sBuf->setTextColor(S_DIM);
        _sBuf->drawString(t, pad, ry+rowH/2);
    };

    int y = 36 - scrollY;

    // Title
    if (16-scrollY > -16 && 16-scrollY < H+16) {
        _sBuf->setFont(&fonts::Font2);
        _sBuf->setTextDatum(middle_center);
        _sBuf->setTextColor(S_WHITE);
        _sBuf->drawString("Settings", cx, 16-scrollY);
        _sBuf->setFont(&fonts::Font0);
        _sBuf->setTextColor(S_DIM);
        _sBuf->drawString(mpgOk?"MPG: Connected":"MPG: Not detected", cx, 30-scrollY);
    }

    // HOUR METER (read-only display)
    lbTxt(y, "JOB HRS");
    if (y+rowH > 0 && y < H) {
        uint32_t secs = tabui_getJobSeconds();
        uint32_t hrs  = secs / 3600;
        uint32_t mins = (secs % 3600) / 60;
        char hbuf[24]; snprintf(hbuf,sizeof(hbuf),"%uh %02um",hrs,mins);
        // Show with maintenance status
        bool due = tabui_maintDue();
        uint16_t hcol = due ? ORANGE : GREEN;
        _sBuf->setFont(&fonts::Font2); _sBuf->setTextDatum(middle_left);
        _sBuf->setTextColor(hcol);
        _sBuf->drawString(hbuf, bx0+4, y+rowH/2);
        if(due){
            _sBuf->setTextColor(ORANGE);
            _sBuf->drawString("MAINT DUE!", bx0+100, y+rowH/2);
        }
    } y += rowH;

    // MAINTENANCE INTERVAL
    lbTxt(y, "MAINT");
    if (y+rowH > 0 && y < H) {
        int bw2=36, lx=bx0;
        sB(lx,       y+8,bw2,optH-8,S_PANEL,S_BORDER,"-",S_WHITE); lx+=bw2+4;
        char mb[16]; snprintf(mb,sizeof(mb),"%dh",s.maintInterval);
        _sBuf->setFont(&fonts::Font2); _sBuf->setTextDatum(middle_center);
        _sBuf->setTextColor(S_WHITE);
        _sBuf->drawString(mb, lx+30, y+rowH/2);
        lx+=64;
        sB(lx,       y+8,bw2,optH-8,S_PANEL,S_BORDER,"+",S_WHITE); lx+=bw2+12;
        sB(lx,       y+8,bW-lx+bx0-4,optH-8,0x0C00,GREEN,"Reset Maint",GREEN);
    } y += rowH;

    // MACHINE TYPE
    lbTxt(y, "MACHINE");
    if (y+rowH > 0 && y < H) {
        int mw=(bW-8)/3;
        const char* ml[]={"CNC","Plotter","Laser"};
        uint16_t mc[]={0x06BF, 0x07E0, ORANGE};
        for(int i=0;i<3;i++){
            bool sel=((int)s.machineType==i);
            sB(bx0+i*(mw+4),y+4,mw,optH,sel?mc[i]:S_PANEL,sel?mc[i]:S_BORDER,ml[i],sel?S_BG:S_DIM);
        }
    } y += rowH;

    // SIM
    lbTxt(y, "SIM");
    { int bw2=(bW-4)/2;
      sB(bx0,       y+4,bw2,optH,s.simMode?S_PANEL:0x0400,s.simMode?S_BORDER:GREEN,"OFF",s.simMode?S_DIM:S_WHITE);
      sB(bx0+bw2+4, y+4,bw2,optH,s.simMode?0x0019:S_PANEL,s.simMode?S_CYAN:S_BORDER,"SIM ON",s.simMode?S_WHITE:S_DIM);
    } y += rowH;

    // THEME
    lbTxt(y, "THEME");
    { int tw=(bW-8)/3; const char* th[]={"Dark","Neutral","Light"};
      for(int i=0;i<3;i++){bool s2=((int)s.theme==i);
        sB(bx0+i*(tw+4),y+4,tw,optH,s2?0x0019:S_PANEL,s2?S_CYAN:S_BORDER,th[i],s2?S_WHITE:S_DIM);}
    } y += rowH;

    // P6 BTN
    lbTxt(y, "P6 BTN");
    { int enw=(bW-12)/5; const char* en[]={"Gate","Touch","Jog","Macro","Off"};
      for(int i=0;i<5;i++){bool s2=((int)s.enableMode==i);
        sB(bx0+i*(enw+3),y+4,enw,optH,s2?0x0019:S_PANEL,s2?S_CYAN:S_BORDER,en[i],s2?S_WHITE:S_DIM);}
    } y += rowH;

    // WORK X (row)
    lbTxt(y, "X mm");
    if (y+rowH > 0 && y < H) {
        char xb[8]; snprintf(xb,8,"%d",s.workX);
        sB(bx0,       y+8,44,optH-8,S_PANEL,S_BORDER,"-",S_WHITE);
        _sBuf->setFont(&fonts::Font2); _sBuf->setTextDatum(middle_center);
        _sBuf->setTextColor(S_WHITE);
        _sBuf->drawString(xb, bx0+44+bW/2-44, y+rowH/2);
        sB(bx0+bW-44, y+8,44,optH-8,S_PANEL,S_BORDER,"+",S_WHITE);
    } y += rowH;
    // WORK Y (row)
    lbTxt(y, "Y mm");
    if (y+rowH > 0 && y < H) {
        char yb[8]; snprintf(yb,8,"%d",s.workY);
        sB(bx0,       y+8,44,optH-8,S_PANEL,S_BORDER,"-",S_WHITE);
        _sBuf->setFont(&fonts::Font2); _sBuf->setTextDatum(middle_center);
        _sBuf->setTextColor(S_WHITE);
        _sBuf->drawString(yb, bx0+44+bW/2-44, y+rowH/2);
        sB(bx0+bW-44, y+8,44,optH-8,S_PANEL,S_BORDER,"+",S_WHITE);
    } y += rowH;

    // HOME
    lbTxt(y, "HOME");
    { const char* hc[]={"Bot-L","Bot-R","Top-L","Top-R"}; int hw=(bW-12)/4;
      for(int i=0;i<4;i++){bool s2=((int)s.homeCorner==i);
        sB(bx0+i*(hw+4),y+4,hw,optH,s2?0x0400:S_PANEL,s2?GREEN:S_BORDER,hc[i],s2?S_WHITE:S_DIM);}
    } y += rowH;

    // BRIGHT slider
    lbTxt(y, "BRIGHT");
    if (y+rowH > 0 && y < H) {
        int barX=bx0, barW=bW;
        _sBuf->fillRoundRect(barX,y+8,barW,optH-8,3,S_PANEL);
        _sBuf->drawRoundRect(barX,y+8,barW,optH-8,3,S_BORDER);
        _sBuf->fillRoundRect(barX+1,y+9,std::max(4,(s.brightness*barW)/255)-2,optH-10,3,S_CYAN);
        char bb[8]; snprintf(bb,8,"%d%%",(s.brightness*100)/255);
        _sBuf->setFont(&fonts::Font0); _sBuf->setTextDatum(middle_center);
        _sBuf->setTextColor(S_WHITE); _sBuf->drawString(bb,barX+barW/2,y+rowH/2);
    } y += rowH;

    // VOL slider
    lbTxt(y, "VOL");
    if (y+rowH > 0 && y < H) {
        int barX=bx0, barW=bW;
        _sBuf->fillRoundRect(barX,y+8,barW,optH-8,3,S_PANEL);
        _sBuf->drawRoundRect(barX,y+8,barW,optH-8,3,S_BORDER);
        _sBuf->fillRoundRect(barX+1,y+9,std::max(4,(s.volume*barW)/9)-2,optH-10,3,S_ORANGE);
        char vb[8]; snprintf(vb,8,"%d/9",s.volume);
        _sBuf->setFont(&fonts::Font0); _sBuf->setTextDatum(middle_center);
        _sBuf->setTextColor(S_WHITE); _sBuf->drawString(vb,barX+barW/2,y+rowH/2);
    } y += rowH;

    // Bottom buttons — drawn at FIXED screen position (always visible)
    { int btnY = H - 34;
      sB(cx-156,btnY,96,28,S_PANEL,S_CYAN,  "Inputs",      S_CYAN);
      sB(cx-54, btnY,96,28,S_PANEL,S_ORANGE,"UART",        S_ORANGE);
      sB(cx+50, btnY,96,28,0x0C00, S_GREEN, "Save & Boot", S_GREEN);
    }

    // Scroll indicator
    { int totalH = 36 + 11*rowH + 40;  // 11 rows + 8px bottom padding: JOB HRS,MAINT,MACHINE,SIM,THEME,P6,Xmm,Ymm,HOME,BRIGHT,VOL
      if (totalH > H) {
        int tH = std::max(8, H*H/totalH);
        int tY = scrollY*(H-tH)/std::max(1,totalH-H);
        _sBuf->fillRect(W-4, 0, 4, H, 0x2104);
        _sBuf->fillRect(W-4, tY, 4, tH, S_DIM);
      }
    }

    _sBuf->pushSprite(0, 0);  // single DMA push — no flicker
}


// ── UART Terminal Monitor ─────────────────────────────────────────────────────
// Shows live raw UART data from FluidNC and lets user test baud rate
static void runUartMonitor() {
    extern uart_port_t fnc_uart_port;

    // Get current baud rate
    uint32_t baud = 0;
    uart_get_baudrate(fnc_uart_port, &baud);

    // Scrolling line buffer
    static char lines[12][64];
    static int  nLines = 0;
    static char lineBuf[64];
    static int  lineLen = 0;
    memset(lines, 0, sizeof(lines));
    nLines  = 0;
    lineLen = 0;

    // Baud rate options
    const uint32_t baudOpts[] = { 9600, 19200, 38400, 57600, 115200, 250000, 500000, 1000000 };
    const int      nBauds     = 8;
    int selBaud = -1;
    for (int i = 0; i < nBauds; i++) if (baudOpts[i] == baud) { selBaud = i; break; }

    uint32_t lastDraw = 0;

    for (;;) {
        // Drain incoming UART bytes into line buffer
        uint8_t c;
        while (uart_read_bytes(fnc_uart_port, &c, 1, 0) == 1) {
            if (c == '\n' || c == '\r') {
                if (lineLen > 0) {
                    lineBuf[lineLen] = 0;
                    // Scroll up
                    if (nLines < 12) {
                        strncpy(lines[nLines++], lineBuf, 63);
                    } else {
                        memmove(lines[0], lines[1], 11 * 64);
                        strncpy(lines[11], lineBuf, 63);
                    }
                    lineLen = 0;
                }
            } else if (lineLen < 63 && c >= 32 && c < 128) {
                lineBuf[lineLen++] = (char)c;
            }
        }

        // Redraw every 150ms
        uint32_t now = millis();
        if (now - lastDraw < 150) { delay(10); }
        else {
            lastDraw = now;
            if (!_sBuf) { _sBuf = new lgfx::LGFX_Sprite(&display); _sBuf->setColorDepth(8); _sBuf->createSprite(display.width(), display.height()); }
            _sBuf->fillSprite(S_BG);
            int cx = display.width() / 2;
            int W  = display.width();

            // Title
            _sBuf->setFont(&fonts::Font2);
            _sBuf->setTextDatum(middle_center);
            _sBuf->setTextColor(S_WHITE);
            _sBuf->drawString("UART Monitor", cx, 11);

            // Baud rate selector
            int bx = 4, by = 23, bw = 36, bh = 14, bg = 6;
            _sBuf->setFont(&fonts::Font0);
            _sBuf->setTextDatum(middle_left);
            _sBuf->setTextColor(S_DIM);
            _sBuf->drawString("Baud:", bx, by + bh/2);
            bx += 30;
            for (int i = 0; i < nBauds; i++) {
                bool sel = (i == selBaud);
                _sBuf->fillRoundRect(bx, by, bw, bh, 2, sel ? 0x0019 : S_PANEL);
                _sBuf->drawRoundRect(bx, by, bw, bh, 2, sel ? S_CYAN : S_BORDER);
                char bb[12];
                if (baudOpts[i] >= 1000000)
                    snprintf(bb, sizeof(bb), "%dM", (int)(baudOpts[i]/1000000));
                else if (baudOpts[i] >= 1000)
                    snprintf(bb, sizeof(bb), "%dk", (int)(baudOpts[i]/1000));
                else
                    snprintf(bb, sizeof(bb), "%d", (int)baudOpts[i]);
                _sBuf->setTextDatum(middle_center);
                _sBuf->setTextColor(sel ? S_WHITE : S_DIM);
                _sBuf->drawString(bb, bx + bw/2, by + bh/2);
                bx += bw + bg;
            }

            // Current baud confirmation
            char baudStr[24];
            snprintf(baudStr, sizeof(baudStr), "Active: %lu", (unsigned long)baud);
            _sBuf->setTextDatum(middle_right);
            _sBuf->setTextColor(S_DIM);
            _sBuf->drawString(baudStr, W - 4, by + bh/2);

            // Divider
            _sBuf->drawFastHLine(0, 40, W, S_BORDER);

            // Received lines
            int lineY = 42, lh = 13;
            _sBuf->setFont(&fonts::Font0);
            for (int i = 0; i < nLines; i++) {
                // Colour by first char
                int col = S_DIM2;
                if (lines[i][0] == '<') col = S_GREEN;
                else if (lines[i][0] == '[') col = S_CYAN;
                else if (lines[i][0] == 'e' || lines[i][0] == 'E') col = S_RED;
                else if (lines[i][0] == 'o') col = S_WHITE;
                _sBuf->setTextDatum(middle_left);
                _sBuf->setTextColor(col);
                _sBuf->drawString(lines[i], 2, lineY + i * lh);
            }

            // Partial current line (dim)
            if (lineLen > 0) {
                lineBuf[lineLen] = 0;
                _sBuf->setTextColor(S_DIM);
                _sBuf->drawString(lineBuf, 2, lineY + nLines * lh);
            }

            // Exit hint
            _sBuf->setTextDatum(middle_center);
            _sBuf->setTextColor(S_DIM);
            _sBuf->drawString("Hold e-stop 2s to exit", cx, display.height()-6);
            _sBuf->pushSprite(0,0);  // single DMA push — no flicker
        }

        // Touch for baud rate selection
        touch.update(millis());
        auto t = touch.getDetail();
        if (t.wasClicked()) {
            int tx = t.x, ty = t.y;
            int bx2 = 34, by2 = 23, bw2 = 36, bh2 = 14, bg2 = 6;
            for (int i = 0; i < nBauds; i++) {
                if (touchIn(tx, ty, bx2, by2, bw2, bh2)) {
                    selBaud = i;
                    baud = baudOpts[i];
                    // Reconfigure UART baud rate on the fly
                    uart_set_baudrate(fnc_uart_port, baud);
                    nLines = 0; lineLen = 0;  // clear buffer after baud change
                }
                bx2 += bw2 + bg2;
            }
        }

        // Exit on e-stop held 2s
        static uint32_t estopHeld2 = 0;
        bool estop2 = (gpio_get_level(GPIO_NUM_17) == 0);
        if (estop2) { if (!estopHeld2) estopHeld2 = millis(); if (millis()-estopHeld2>2000) return; }
        else estopHeld2 = 0;
    }
}




// ── UART Terminal Monitor ─────────────────────────────────────────────────────

// ── Input Monitor ─────────────────────────────────────────────────────────────
// Live display of all inputs: PCF8574 bits, encoder count, touch, e-stop
static void runInputMonitor() {
    int16_t encBase = get_encoder();
    int     lastTx = -1, lastTy = -1;
    uint8_t lastPins = 0xFF;

    for (;;) {
        uint8_t pins = 0xFF;
        i2c_master_read_from_device(I2C_NUM_1, 0x20, &pins, 1, pdMS_TO_TICKS(20));
        int16_t encNow  = get_encoder();
        int16_t encDelta = encNow - encBase;
        bool estop = (gpio_get_level(GPIO_NUM_17) == 0);

        touch.update(millis());
        auto t = touch.getDetail();
        if (t.state == m5::touch_state_t::touch) { lastTx = t.x; lastTy = t.y; }

        display.fillScreen(S_BG);
        int cx = display.width() / 2;

        // Title + back hint
        display.setFont(&fonts::Font2);
        display.setTextDatum(middle_center);
        display.setTextColor(S_WHITE);
        display.drawString("Input Monitor", cx, 12);
        display.setFont(&fonts::Font0);
        display.setTextColor(S_DIM);
        display.drawString("Hold e-stop 2s to exit", cx, 24);

        display.setFont(&fonts::Font0);
        int y = 36, lh = 18;

        // ── PCF8574 bits ──────────────────────────────────────────────────
        display.setTextDatum(middle_left);
        display.setTextColor(S_DIM);
        display.drawString("PCF8574 raw:", 6, y);
        char hexbuf[12]; snprintf(hexbuf, sizeof(hexbuf), "0x%02X", pins);
        display.setTextColor(S_WHITE);
        display.drawString(hexbuf, 100, y);
        y += lh;

        // Individual bits with labels
        const char* bitLabels[] = { "P0 X-Axis", "P1 Y-Axis", "P2 Z-Axis", "P3 A-Axis",
                                    "P4 Step x10","P5 Step x100","P6 Enable","P7 unused" };
        for (int i = 0; i < 8; i++) {
            bool active = !(pins & (1 << i));
            int col = active ? S_GREEN : S_DIM;
            // Dot indicator
            display.fillCircle(10, y, 4, col);
            display.setTextDatum(middle_left);
            display.setTextColor(col);
            display.drawString(bitLabels[i], 18, y);
            display.setTextColor(active ? S_GREEN : S_DIM);
            display.drawString(active ? "LOW (active)" : "HIGH", 130, y);
            y += lh;
        }

        // ── E-stop ────────────────────────────────────────────────────────
        y += 2;
        display.fillCircle(10, y, 4, estop ? S_RED : S_DIM);
        display.setTextDatum(middle_left);
        display.setTextColor(estop ? S_RED : S_DIM);
        display.drawString("E-stop GPIO17", 18, y);
        display.drawString(estop ? "PRESSED" : "open", 130, y);
        y += lh;

        // ── Encoder ───────────────────────────────────────────────────────
        char encbuf[20];
        snprintf(encbuf, sizeof(encbuf), "Encoder delta: %+d", (int)encDelta);
        display.setTextColor(S_CYAN);
        display.drawString(encbuf, 6, y);
        y += lh;

        // ── Touch ─────────────────────────────────────────────────────────
        if (lastTx >= 0) {
            char tbuf[28];
            snprintf(tbuf, sizeof(tbuf), "Touch: x=%d y=%d", lastTx, lastTy);
            display.setTextColor(S_CYAN);
            display.drawString(tbuf, 6, y);
        } else {
            display.setTextColor(S_DIM);
            display.drawString("Touch: none", 6, y);
        }

        // no-op: drawing direct to display  // flip buffer — no flicker

        // Exit: e-stop held for 2s
        static uint32_t estopHeldSince = 0;
        if (estop) {
            if (!estopHeldSince) estopHeldSince = millis();
            if (millis() - estopHeldSince > 2000) return;
        } else {
            estopHeldSince = 0;
        }

        delay(150);
    }
}

static void runSettingsMenu(AppSettings& s) {
    int W = display.width();
    uint8_t pcfVal = 0xFF;
    bool mpgOk = (i2c_master_read_from_device(
        I2C_NUM_1, 0x20, &pcfVal, 1, pdMS_TO_TICKS(20)) == ESP_OK);

    int _settingsScroll = 0;
    int _lastTouchY = -1;
    bool _didDrag = false;
    auto drawSettings = [&]() {
        drawSettingsMenu(s, mpgOk, _settingsScroll);
    };
    drawSettings();

    int cx = display.width() / 2;
    for (;;) {
        touch.update(millis());
        // E-stop + enable → reboot (check every loop iteration)
        {
            bool en2   = readEnableNow();
            bool est2  = (gpio_get_level(GPIO_NUM_17) == 0);
            static uint32_t rbHeld = 0;
            if (en2 && est2) {
                if (!rbHeld) rbHeld = millis();
                if (millis() - rbHeld >= 3000) esp_restart();
                // Show countdown bar at bottom of screen
                uint32_t h = millis() - rbHeld;
                display.fillRect(0, display.height()-4, display.width()*h/3000, 4, 0xF800);
            } else {
                rbHeld = 0;
            }
        }
        auto t = touch.getDetail();

        int rowH2 = 48, bx0_2 = 68, bW2 = W - bx0_2 - 4;
        // MPG encoder scroll
        { int16_t enc = get_encoder();
          if (enc != 0) {
              { int maxS=std::max(0,36+11*rowH2+40-240);  // 11 rows + 8px pad
              _settingsScroll = std::max(0, std::min(maxS, _settingsScroll + enc * 4)); }
              drawSettings(); delay(8); continue;
          }
        }
        // Touch handling — track drag and tap separately
        bx0_2 = 68; bW2 = W - bx0_2 - 4;
        int maxScroll = std::max(0, 36 + 11*rowH2 + 40 - 240);  // 11 rows + 8px pad

        if (t.wasPressed()) {
            _lastTouchY = t.y;
            _didDrag = false;
            delay(5); continue;
        }
        if (t.isPressed() && _lastTouchY >= 0) {
            int delta = _lastTouchY - t.y;
            int ty2 = t.y + _settingsScroll;
            // Brightness slider drag (row 11 = index from top: 0=JOB,1=MAINT,2=MACH,3=SIM,4=THEME,5=P6,6=Xmm,7=Ymm,8=HOME,9=BRIGHT,10=VOL)
            { int yBr = 36+9*rowH2+8;
              if (ty2>=yBr && ty2<yBr+38 && t.x>=bx0_2 && t.x<bx0_2+bW2) {
                  _didDrag = true;  // prevent click firing after slider
                  s.brightness = std::max(10,std::min(255,(t.x-bx0_2)*255/bW2));
                  display.setBrightness(s.brightness);
                  drawSettings(); delay(5); continue;
              }
            }
            // Volume slider drag (row 10 = after BRIGHT)
            { int yVol = 36+10*rowH2+8;
              if (ty2>=yVol && ty2<yVol+38 && t.x>=bx0_2 && t.x<bx0_2+bW2) {
                  _didDrag = true;  // prevent click firing after slider
                  s.volume = std::max(0,std::min(9,(t.x-bx0_2)*9/bW2));
                  tabui_setVolume(s.volume);
                  drawSettings(); delay(5); continue;
              }
            }
            // Scroll drag
            if (abs(delta) >= 4) {
                _didDrag = true;
                _settingsScroll = std::max(0, std::min(maxScroll, _settingsScroll + delta));
                _lastTouchY = t.y;
                drawSettings();
            }
            delay(5); continue;
        }
        if (t.isReleased()) { _lastTouchY = -1; }
        // Skip click if this was a drag
        if (_didDrag) { _didDrag = false; delay(5); continue; }
        if (!t.wasClicked()) { delay(10); continue; }
        int tx = t.x, ty = t.y + _settingsScroll;

        int y = 36, rowH = 48, optH = 36;
        int bx0 = 68, bW = W - bx0 - 4;

        // Hour meter row — read only, no touch needed
        y += rowH;

        // Maintenance interval row
        { int bw2=36, lx=bx0;
          if (touchIn(tx,ty, lx, y+8, bw2, optH-8) && s.maintInterval > 1)  { s.maintInterval--; tabui_setMaintInterval(s.maintInterval); }
          lx+=bw2+4+64;
          if (touchIn(tx,ty, lx, y+8, bw2, optH-8) && s.maintInterval < 999) { s.maintInterval++; tabui_setMaintInterval(s.maintInterval); }
          lx+=bw2+12;
          int resetW=bW-lx+bx0-4;
          if (touchIn(tx,ty, lx, y+8, resetW, optH-8)) {
              tabui_setMaintInterval(s.maintInterval);
              tabui_resetMaintenance();
          }
        }
        y += rowH;

        // Machine type row
        { int mw=(bW-8)/3;
          for(int i=0;i<3;i++)
            if(touchIn(tx,ty,bx0+i*(mw+4),y+4,mw,optH)){ s.machineType=(MachineType)i; tabui_setMachineType(i); }
        }
        y += rowH;

        // Sim mode row
        { int bw2=(bW-4)/2;
          if (touchIn(tx,ty, bx0,       y+4, bw2, optH)) s.simMode = false;
          if (touchIn(tx,ty, bx0+bw2+4, y+4, bw2, optH)) s.simMode = true;
        }
        y += rowH;

        // Theme row
        { int tw=(bW-8)/3;
          for(int i=0;i<3;i++)
            if (touchIn(tx,ty,bx0+i*(tw+4),y+4,tw,optH)) s.theme=(Theme)i;
        }
        y += rowH;

        // P6 enable mode row
        { int enw=(bW-12)/5;
          for(int i=0;i<5;i++)
            if (touchIn(tx,ty,bx0+i*(enw+3),y+4,enw,optH)) s.enableMode=(EnableMode)i;
        }
        y += rowH;

        // Macro index row (only shown in Macro mode)
        if (s.enableMode == EnableMode::MacroBtn) {
            int enx0b=76;
            if (touchIn(tx,ty, enx0b+124, y+5, 30, optH) && s.enableMacro > 0)  s.enableMacro--;
            if (touchIn(tx,ty, enx0b+158, y+5, 30, optH) && s.enableMacro < 15) s.enableMacro++;
            y += rowH;
        }
        // WORK X row
        if (touchIn(tx,ty, bx0,       y+8, 44, optH-8) && s.workX > 50)   s.workX -= 50;
        if (touchIn(tx,ty, bx0+bW-44, y+8, 44, optH-8) && s.workX < 9999) s.workX += 50;
        y += rowH;
        // WORK Y row
        if (touchIn(tx,ty, bx0,       y+8, 44, optH-8) && s.workY > 50)   s.workY -= 50;
        if (touchIn(tx,ty, bx0+bW-44, y+8, 44, optH-8) && s.workY < 9999) s.workY += 50;
        y += rowH;

        // Home corner
        { int hw=(bW-12)/4;
          for(int i=0;i<4;i++)
            if(touchIn(tx,ty,bx0+i*(hw+4),y+4,hw,optH)) s.homeCorner=(HomeCorner)i;
        }
        y += rowH;

        // Bottom buttons at FIXED screen position (not scrollable)
        // Use t.x, t.y directly (not scrolled ty)
        // Bottom buttons — use same position as draw (btnY = H-34)
        { int btnY = display.height() - 34;
          if (touchIn(t.x, t.y, cx-156, btnY, 96, 28)) {
              runInputMonitor(); drawSettings(); continue;
          }
          if (touchIn(t.x, t.y, cx-54,  btnY, 96, 28)) {
              runUartMonitor(); drawSettings(); continue;
          }
          if (touchIn(t.x, t.y, cx+50,  btnY, 96, 28)) {
              settings_save(s); esp_restart();
          }
        }

        drawSettings();
    }

}

// ── Background task: MPG switches + e-stop ────────────────────────────────────
static void mpgTask(void*) {
    static bool     lastEstop    = true;
    static uint32_t resumeTime   = 0;   // delayed action timer
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(30));
        readMpgSwitches();
        uint32_t now = millis();
        bool estopHigh = (gpio_get_level(GPIO_NUM_17) != 0);
        simMode_setEstop(!estopHigh);
        mpgSetEstop(!estopHigh);
        if (estopHigh != lastEstop) { mpgSignalChanged(); }

        if (!estopHigh && lastEstop) {
            // E-stop PRESSED: Soft Reset → FluidNC immediately enters Alarm state
            extern volatile bool _forceAlarm;
            _forceAlarm = true;
            fnc_realtime((realtime_cmd_t)0x18);  // Ctrl-X Soft Reset → Alarm
            fnc_term_inject("> E-STOP: Reset → Alarm");
            resumeTime = 0;
            // Alarm siren: 3 fast two-tone beeps
            for (int bi = 0; bi < 3; bi++) {
                beep(880,  80, 128);  vTaskDelay(pdMS_TO_TICKS(90));
                beep(1400, 80, 128);  vTaskDelay(pdMS_TO_TICKS(110));
            }
        }

        // While e-stop held: repeat Soft Reset every 2s so FluidNC cannot recover
        static uint32_t holdRepeat = 0;
        if (!estopHigh) {
            if (holdRepeat == 0) holdRepeat = now;
            if (now - holdRepeat >= 2000) {
                holdRepeat = now;
                fnc_realtime((realtime_cmd_t)0x18);  // keep machine in Alarm
            }
        } else {
            holdRepeat = 0;
        }

        if (estopHigh && !lastEstop) {
            // E-stop RELEASED: show recovery menu immediately, then wait for FluidNC boot
            extern void tabui_setEstopRecovery();
            tabui_setEstopRecovery();   // show recovery options right away
            resumeTime = now + 800;
            fnc_term_inject("> E-STOP released — choose recovery option");
        }

        lastEstop = estopHigh;

        // E-stop released: set _estopRecovery so UI shows recovery menu
        if (resumeTime && now >= resumeTime && estopHigh) {
            resumeTime = 0;
            // Do NOT auto-send $X — let user choose recovery action from overlay
            // Just signal the UI to show recovery options
            extern void tabui_setEstopRecovery();
            tabui_setEstopRecovery();
            mpgSignalChanged();
        }

        // E-stop + enable held 3s → hard reboot
        static uint32_t rebootHeld = 0;
        bool en2 = mpgEnableHeld();
        if (!estopHigh && en2) {
            if (!rebootHeld) rebootHeld = now;
            if (now - rebootHeld >= 3000) esp_restart();
        } else {
            rebootHeld = 0;
        }
    }
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    init_system();
    force_landscape();

    // Check enable BEFORE showing logo (catches early press)
    bool enterSettings = readEnableNow();

    show_logo();

    // ── Build timestamp — shown on logo so you can confirm new firmware loaded
    {
        display.setFont(&fonts::Font0);
        display.setTextDatum(middle_center);
        display.setTextColor(0x07FF);  // cyan
        // __DATE__ = "May 17 2026", __TIME__ = "18:53:01" — set at compile time
        display.drawString("Build: " __DATE__ " " __TIME__, display.width()/2, 16);
    }

    // ── Boot progress bar on logo screen ────────────────────────────────────
    int  W = display.width(), H = display.height();
    int  barX = 20, barY = H - 22, barW = W - 40, barH = 8, barR = 4;

    // Helper: draw progress bar frame + fill
    auto drawProgress = [&](int pct, const char* label, int barCol) {
        // Bar track
        display.fillRoundRect(barX, barY, barW, barH, barR, 0x2104);
        // Fill
        int fillW = barW * pct / 100;
        if (fillW > 0)
            display.fillRoundRect(barX, barY, fillW, barH, barR, barCol);
        // Label above bar
        display.fillRect(barX, barY - 14, barW, 12, 0x0000);
        display.setFont(&fonts::Font0);
        display.setTextDatum(middle_left);
        display.setTextColor(0x9D17);
        display.drawString(label, barX, barY - 8);
    };

    // Enable button hint (right of bar)
    display.setFont(&fonts::Font0);
    display.setTextDatum(middle_right);
    display.setTextColor(0x52F0);
    display.drawString("Hold EN for Settings", W - barX, barY - 8);

    // ── Step 1: EN button poll window (1 second, 0→50%) ──────────────────────
    {
        uint32_t t0 = millis();
        uint32_t rebootHeld = 0;
        int lastPct = -1;
        const char* lastLabel = "";
        while (millis() - t0 < 1000) {
            uint32_t el = millis() - t0;
            bool en    = readEnableNow();
            bool estop = (gpio_get_level(GPIO_NUM_17) == 0);
            if (en) enterSettings = true;
            int pct = el * 50 / 1000;
            int barCol = enterSettings ? 0x07E0 : en ? 0xFD20 : 0x065F;
            const char* lbl = enterSettings ? "Settings mode"
                            : en ? "Hold for Settings..." : "Booting...";
            // Only redraw when pct or label changes
            if (pct != lastPct || lbl != lastLabel) {
                lastPct = pct; lastLabel = lbl;
                display.fillRoundRect(barX, barY, barW, barH, barR, 0x2104);
                if (pct > 0)
                    display.fillRoundRect(barX, barY, barW * pct / 100, barH, barR, barCol);
                display.fillRect(barX, barY - 14, barW, 12, 0x0000);
                display.setFont(&fonts::Font0);
                display.setTextDatum(middle_left);
                display.setTextColor(0x9D17);
                display.drawString(lbl, barX, barY - 8);
            }
            // EN+ESTOP reboot — only if BOTH clearly held (debounced)
            if (en && estop) {
                if (!rebootHeld) rebootHeld = millis();
                if (millis() - rebootHeld >= 3000) esp_restart();
            } else { rebootHeld = 0; }
            vTaskDelay(pdMS_TO_TICKS(30));
        }
    }

    // ── Step 2: Load settings (50→65%) ────────────────────────────────────────
    drawProgress(52, "Loading settings...", 0x065F);
    AppSettings s;
    settings_load(s);
    drawProgress(58, "Applying config...", 0x065F);
    tabui_setTheme((int)s.theme);
    tabui_setAxes((int)s.axes);
    tabui_setEnableMode((int)s.enableMode, s.enableMacro);
    tabui_setWorkArea(s.workX, s.workY, (int)s.homeCorner);
    tabui_setVolume(s.volume);
    tabui_setMachineType((int)s.machineType);
    tabui_setMaintInterval(s.maintInterval);
    tabui_loadHourMeter();
    display.setBrightness(s.brightness);

    if (enterSettings) {
        drawProgress(65, "Entering settings...", 0x07E0);
        delay(200);
        runSettingsMenu(s);
        settings_load(s);
        tabui_setTheme((int)s.theme);
        tabui_setAxes((int)s.axes);
        tabui_setEnableMode((int)s.enableMode, s.enableMacro);
        tabui_setWorkArea(s.workX, s.workY, (int)s.homeCorner);
    }

    // ── Step 3: Connect or sim ───────────────────────────────────────────────
    if (!s.simMode) {
        drawProgress(85, "Connecting...", 0x065F);
        request_status_report();
    } else {
        drawProgress(100, "Sim Mode", 0x07E0);
        simMode_enable();
    }

    // Create canvas — no fillScreen to avoid black flash
    // The first reDisplay() from activate_scene covers the display immediately
    canvas.deleteSprite();
    canvas.setColorDepth(8);
    canvas.createSprite(display.width(), display.height());
    canvas.fillSprite(0x0862);  // pre-fill canvas with bg colour (no black frame)

    dbg_printf("FluidNC Pendant %s  [%s | theme=%d | axes=%d]\n",
               git_info, "Normal", (int)s.theme, (int)s.axes);
    if (!s.simMode) fnc_realtime(StatusReport);
    activate_scene(getTabScene());
    // Check for interrupted job on boot
    jobrecov_init();
    if (jobrecov_hasDirty() && !s.simMode) {
        jobrecov_showPrompt();  // will show on first reDisplay
    }
    if (s.simMode) {
        simMode_injectState();   // inject Idle after scene is ready — no lag
        tabui_resetJobState();   // clear any stale job flags
        markDirty();
    }

    xTaskCreatePinnedToCore(mpgTask, "mpg", 4096, nullptr, 1, nullptr, 0);
}

void loop() {
    dispatch_events();  // touch/encoder first — keeps UI responsive
    if (simMode_active()) {
        // Sim mode: inject fake state periodically, skip UART polling
        static uint32_t _simLast = 0;
        if (millis() - _simLast > 80) {
            _simLast = millis();
            simMode_tick();
            simMode_injectState();
        }
    } else {
        fnc_poll();     // UART parse — only when not in sim mode
    }
}
