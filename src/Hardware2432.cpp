// Hardware2432.cpp — JC2432W328C capacitive touch display
// Tab UI build: capacitive only, no touch-type selection screen, no restart loop.
// Copyright (c) 2023 Mitch Bradley / 2024 TabUI contributors. GPLv3 licence.

#include "System.h"

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <driver/i2c.h>

#include "Hardware2432.hpp"
#include "Drawing.h"
#include "NVS.h"
#include "driver/gpio.h"

#include <driver/uart.h>
#include "hal/uart_hal.h"

int lockout_pin = GPIO_NUM_34;

m5::Touch_Class  xtouch;
m5::Touch_Class& touch = xtouch;

static uint32_t read_panel_reg(lgfx::IBus* bus, int32_t pin_cs, uint_fast16_t cmd, uint8_t dummy_bits, uint8_t real_reads) {
    size_t        dlen     = 8;
    uint_fast16_t read_cmd = cmd;
    lgfx::pinMode(pin_cs, lgfx::pin_mode_t::output);
    bus->beginTransaction();
    lgfx::gpio_hi(pin_cs);
    bus->writeCommand(0, dlen);
    bus->wait();
    lgfx::gpio_lo(pin_cs);
    bus->writeCommand(read_cmd, dlen);
    bus->beginRead(dummy_bits);
    uint32_t res = 0;
    for (size_t i = 0; i < real_reads; ++i) {
        auto data = bus->readData(dlen);
        res += ((data >> (dlen - 8)) & 0xFF) << (i * 8);
    }
    bus->endTransaction();
    lgfx::gpio_hi(pin_cs);
    return res;
}

LGFX_Device  xdisplay;
LGFX_Device& display = xdisplay;

LGFX_Sprite canvas(&display);
LGFX_Sprite buttons[3] = { &display, &display, &display };
LGFX_Sprite locked_button(&display);

uint8_t base_rotation = 2;

lgfx::Bus_SPI bus;
static void init_bus() {
    auto cfg       = bus.config();
    cfg.freq_write = 55000000;
    cfg.freq_read  = 16000000;
    cfg.use_lock   = true;
    cfg.dma_channel = SPI_DMA_CH_AUTO;
    cfg.spi_host    = HSPI_HOST;
    cfg.pin_mosi    = GPIO_NUM_13;
    cfg.pin_miso    = GPIO_NUM_12;
    cfg.pin_sclk    = GPIO_NUM_14;
    cfg.pin_dc      = GPIO_NUM_2;
    cfg.spi_mode    = 0;
    cfg.spi_3wire   = false;
    bus.config(cfg);
    bus.init();
}

lgfx::Light_PWM light;

void init_light() {
    auto cfg        = light.config();
    cfg.pin_bl      = GPIO_NUM_21;
    cfg.freq        = 12000;
    cfg.pwm_channel = 7;
    cfg.offset      = 0;
    cfg.invert      = false;
    light.config(cfg);
}

void setBacklightPin(uint8_t pinnum) {
    auto cfg   = light.config();
    cfg.pin_bl = pinnum;
    light.config(cfg);
    light.init(255);
}

lgfx::Panel_ST7789  _panel_st7789;
lgfx::Panel_ILI9341 _panel_ili9341;

static void init_panel_st7789() {
    base_rotation = 0;
    auto& p = _panel_st7789;
    p.bus(&bus);
    auto cfg            = p.config();
    cfg.pin_cs          = GPIO_NUM_15;
    cfg.offset_rotation = base_rotation;
    cfg.bus_shared      = false;
    p.config(cfg);
    p.light(&light);
    display.setPanel(&p);
}

static void init_panel_ili9341() {
    base_rotation = 2;
    auto& p = _panel_ili9341;
    p.bus(&bus);
    auto cfg            = p.config();
    cfg.pin_cs          = GPIO_NUM_15;
    cfg.offset_rotation = base_rotation;
    cfg.bus_shared      = false;
    p.config(cfg);
    p.light(&light);
    display.setPanel(&p);
}

#ifdef DEBUG_TO_USB
Stream& debugPort = Serial;
#endif

int red_button_pin   = -1;
int dial_button_pin  = -1;
int green_button_pin = -1;
int enc_a, enc_b;

// ── Capacitive touch (CST816S) ───────────────────────────────────────────────
lgfx::Touch_CST816S _touch_cst816s;

static void init_capacitive_cyd() {
    auto cfg            = _touch_cst816s.config();
    cfg.i2c_port        = I2C_NUM_0;
    cfg.pin_sda         = GPIO_NUM_33;
    cfg.pin_scl         = GPIO_NUM_32;
    cfg.pin_rst         = GPIO_NUM_25;
    cfg.pin_int         = -1;
    cfg.offset_rotation = base_rotation;
    cfg.freq            = 400000;
    cfg.x_max           = 240;
    cfg.y_max           = 320;
    _touch_cst816s.config(cfg);
    display.getPanel()->setTouch(&_touch_cst816s);
    display.getPanel()->initTouch();

    setBacklightPin(GPIO_NUM_27);
    pinMode(lockout_pin, INPUT);

    // MPG encoder
    enc_a = GPIO_NUM_22;
    enc_b = GPIO_NUM_21;

    // Step switch: single GPIO16 — NC=0.1mm, PD=1mm, PU=10mm
    // Axis switch: GPIO17(NC=off,PD=X,PU=Z), GPIO35(PU=Y), GPIO4(PD=A)
    // All configured as plain INPUT (no internal pull) so NC/PD/PU are distinguishable
    // GPIO35 is input-only — already has external 10k pullup to 3.3V
    gpio_set_direction(GPIO_NUM_16, GPIO_MODE_INPUT);
    gpio_set_direction(GPIO_NUM_4,  GPIO_MODE_INPUT);
    gpio_set_direction(GPIO_NUM_17, GPIO_MODE_INPUT);
    gpio_set_direction(GPIO_NUM_35, GPIO_MODE_INPUT);

    // E-stop: GPIO8 is flash SPI (unusable). GPIO34 is lockout_pin.
    // Wire e-stop to GPIO36 or GPIO39 in hardware revision.

    red_button_pin   = -1;
    dial_button_pin  = -1;
    green_button_pin = -1;
}

bool round_display = false;

const int n_buttons      = 3;
const int button_w       = 80;
const int button_h       = 80;
const int button_half_wh = button_w / 2;
const int sprite_wh      = 240;
Point     button_wh(button_w, button_h);
int       button_colors[] = { RED, YELLOW, GREEN };

class Layout {
private:
    int _rotation;
public:
    Point buttonsXY, buttonsWidth, buttonsHeight, buttonsWH, spritePosition;
    Layout(int rotation, Point spritePosition, Point firstButtonPosition)
        : _rotation(rotation), spritePosition(spritePosition), buttonsXY(firstButtonPosition) {
        buttonsWH = (_rotation & 1) ? Point{ button_w, sprite_wh } : Point{ sprite_wh, button_h };
    }
    Point buttonOffset(int n) { return (_rotation & 1) ? Point(0, n * button_h) : Point(n * button_w, 0); }
    int   rotation() { return _rotation; }
};

// clang-format off
Layout layouts[] = {
    { 0, { 0, 0 },        { 0, sprite_wh } },
    { 0, { 0, button_h }, { 0, 0 }         },
    { 1, { 0, 0 },        { sprite_wh, 0 } },
    { 1, { button_w, 0 }, { 0, 0 }         },
    { 2, { 0, 0 },        { 0, sprite_wh } },
    { 2, { 0, button_h }, { 0, 0 }         },
    { 3, { button_w, 0 }, { 0, 0 }         },
    { 3, { 0, 0 },        { sprite_wh, 0 } },
};
// clang-format on
int     num_layouts = sizeof(layouts) / sizeof(layouts[0]);
Layout* layout;
int32_t layout_num = 0;
Point   sprite_offset;

void set_layout(int n) {
    layout        = &layouts[n];
    sprite_offset = layout->spritePosition;
    display.setRotation(layout->rotation());
}

nvs_handle_t hw_nvs;
int32_t      display_num = 0;

void init_hardware() {
#ifdef DEBUG_TO_USB
    Serial.begin(115200);
#endif
    hw_nvs = nvs_init("hardware");

    init_light();
    init_bus();

    if (read_panel_reg(&bus, GPIO_NUM_15, 0xda, 0, 1) == 0x0) {
        dbg_printf("ILI9341 panel\n");
        init_panel_ili9341();
    } else {
        dbg_printf("ST7789 panel\n");
        init_panel_st7789();
    }
    display.init();

    // Capacitive touch only — no selection screen, no restart
    init_capacitive_cyd();

    nvs_get_i32(hw_nvs, "layout", &layout_num);
    set_layout(layout_num);

    touch.begin(&display);
    init_encoder(enc_a, enc_b);
    init_fnc_uart(FNC_UART_NUM, PND_TX_FNC_RX_PIN, PND_RX_FNC_TX_PIN);
    touch.setFlickThresh(10);
}

void initButton(int n) {
    buttons[n].setColorDepth(display.getColorDepth());
    buttons[n].createSprite(button_w, button_h);
    buttons[n].fillRect(0, 0, 80, 80, BLACK);
    const int   radius = 28;
    const char* filename;
    int         color;
    switch (n) {
        case 0: color = RED;    filename = "/red_button.png";    break;
        case 1: color = YELLOW; filename = "/orange_button.png"; break;
        case 2: color = GREEN;  filename = "/green_button.png";  break;
        default: return;
    }
    buttons[n].fillCircle(button_half_wh, button_half_wh, radius, color);
    buttons[n].drawPngFile(LittleFS, filename, 10, 10, 60, 60, 0, 0, 0.0f, 0.0f, datum_t::top_left);
}

void initLockedButton() {
    locked_button.setColorDepth(display.getColorDepth());
    locked_button.createSprite(button_w, button_h);
    locked_button.fillRect(0, 0, button_w, button_h, BLACK);
    locked_button.fillCircle(button_half_wh, button_half_wh, 28, DARKGREY);
}

static void initButtons() {
    for (int i = 0; i < 3; i++) initButton(i);
    initLockedButton();
}

int last_locked = -1;

void redrawButtons() {
    display.startWrite();
    for (int i = 0; i < n_buttons; i++) {
        Point     position = layout->buttonsXY + layout->buttonOffset(i);
        auto&     sprite   = last_locked == 1 ? locked_button : buttons[i];
        sprite.pushSprite(position.x, position.y);
    }
    display.endWrite();
}

void show_logo() {
    display.clear();
    // Logo is 300x100, centred on 320x240 landscape: x=(320-300)/2=10, y=(240-100)/2=70
    display.drawPngFile(LittleFS, "/fluid_dial.png",
                        10, 70, 300, 100, 0, 0, 0.0f, 0.0f, datum_t::top_left);
}

void base_display() {
    initButtons();
    redrawButtons();
}

void force_landscape() {
    layout_num = 2;  // rotation 1 = landscape
    nvs_set_i32(hw_nvs, "layout", layout_num);
    set_layout(layout_num);
    sprite_offset = { 0, 0 };
    // Move on-screen button strip off-screen so it never intercepts touches.
    // Layout 2 places buttons at x=240 which blocks the right 80px (viz tabs, macros).
    layout->buttonsXY = { 400, 0 };
}

void next_layout(int delta) {
    layout_num += delta;
    while (layout_num >= num_layouts) layout_num -= num_layouts;
    while (layout_num < 0)            layout_num += num_layouts;
    set_layout(layout_num);
    nvs_set_i32(hw_nvs, "layout", layout_num);
    redrawButtons();
}

void system_background() { drawBackground(BLACK); }

bool switch_button_touched(bool& pressed, int& button) {
    static int last_red = -1, last_green = -1, last_dial = -1;
    bool state;
    if (red_button_pin != -1) {
        state = digitalRead(red_button_pin);
        if ((int)state != last_red)   { last_red   = state; button = 0; pressed = !state; return true; }
    }
    if (dial_button_pin != -1) {
        state = digitalRead(dial_button_pin);
        if ((int)state != last_dial)  { last_dial  = state; button = 1; pressed = !state; return true; }
    }
    if (green_button_pin != -1) {
        state = digitalRead(green_button_pin);
        if ((int)state != last_green) { last_green = state; button = 2; pressed = !state; return true; }
    }
    return false;
}

bool screen_encoder(int x, int y, int& delta) { return false; }

bool    touch_debounce = false;
int32_t touch_timeout  = 0;

bool ui_locked() {
    bool locked = digitalRead(lockout_pin);
    if ((int)locked != last_locked) { last_locked = locked; redrawButtons(); }
    return locked;
}

bool in_rect(Point test, Point xy, Point wh) {
    return test.x >= xy.x && test.x < (xy.x + wh.x) && test.y >= xy.y && test.y < (xy.y + wh.y);
}
bool in_button_stripe(Point xy) { return in_rect(xy, layout->buttonsXY, layout->buttonsWH); }

bool screen_button_touched(bool pressed, int x, int y, int& button) {
    Point xy(x, y);
    if (!in_button_stripe(xy)) return false;
    xy -= layout->buttonsXY;
    for (int i = 0; i < n_buttons; i++) {
        if (in_rect(xy, layout->buttonOffset(i), button_wh)) {
            button = i;
            if (!pressed) { touch_debounce = true; touch_timeout = milliseconds() + 100; }
            return true;
        }
    }
    return false;
}

void update_events() {
    auto ms = lgfx::millis();
    if (touch.isEnabled()) {
        if (touch_debounce) {
            if ((ms - touch_timeout) < 0) return;
            touch_debounce = false;
        }
        touch.update(ms);
    }
}

void ackBeep() {}
void deep_sleep(int us) {}
