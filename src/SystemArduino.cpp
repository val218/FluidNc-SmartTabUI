// Copyright (c) 2023 Mitch Bradley
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

// System interface routines for the Arduino framework

#include "System.h"
#include "FluidNCModel.h"
#include "NVS.h"

#include <Esp.h>  // ESP.restart()

#include <driver/uart.h>
#include <string.h>
#include "hal/uart_hal.h"

uart_port_t fnc_uart_port;

// We use the ESP-IDF UART driver instead of the Arduino
// HardwareSerial driver so we can use software (XON/XOFF)
// flow control.  The ESP-IDF driver supports the ESP32's
// hardware implementation of XON/XOFF, but Arduino does not.

// TX/RX activity counters — incremented on each byte, read by header display
volatile uint32_t fnc_tx_count = 0;
volatile uint32_t fnc_rx_count = 0;

// ── Software ring buffer for UART RX ──────────────────────────────────────────
// A dedicated FreeRTOS task drains the hardware UART buffer into this ring buffer
// continuously on Core 0. fnc_getchar() reads from here — bytes are NEVER lost
// due to LCD redraw delays blocking loop() on Core 1.
#define UART_RING_SIZE 4096
static uint8_t  _uartRing[UART_RING_SIZE];
static volatile uint32_t _uartRingHead = 0;  // written by uart_task (Core 0)
static volatile uint32_t _uartRingTail = 0;  // read by fnc_getchar (Core 1)

static inline uint32_t uart_ring_used() {
    return (_uartRingHead - _uartRingTail) & (UART_RING_SIZE - 1);
}
static inline bool uart_ring_empty() {
    return _uartRingHead == _uartRingTail;
}
static inline void uart_ring_push(uint8_t c) {
    uint32_t next = (_uartRingHead + 1) & (UART_RING_SIZE - 1);
    if (next != _uartRingTail) {  // don't overwrite if full
        _uartRing[_uartRingHead] = c;
        _uartRingHead = next;
    }
}
static inline int uart_ring_pop() {
    if (_uartRingTail == _uartRingHead) return -1;
    uint8_t c = _uartRing[_uartRingTail];
    _uartRingTail = (_uartRingTail + 1) & (UART_RING_SIZE - 1);
    return c;
}

// ── UART reader task (Core 0, high priority) ──────────────────────────────────
// Runs every 1ms, reads ALL available bytes from hardware UART into ring buffer.
// This runs INDEPENDENTLY of loop() so LCD redraws never cause UART byte loss.
void uart_reader_task(void*) {
    uint8_t buf[128];
    uint32_t taskLoops = 0;
    for (;;) {
        int n = uart_read_bytes(fnc_uart_port, buf, sizeof(buf), 0);
        if (n > 0) {
            for (int i = 0; i < n; i++) uart_ring_push(buf[i]);
            fnc_rx_count += n;
            update_rx_time();  // now a no-op but kept for compat
        }
        taskLoops++;
        // Watchdog: log task health every 10 seconds via a shared counter
        // (readable from Core 1 for diagnostics)
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// Expose uart_reader_task loop count for health monitoring
// If this stops incrementing, the task has died
volatile uint32_t uart_task_loops = 0;

// Raw UART line capture for terminal display
#define TERM_LINE_BUF 64
#define TERM_LINE_MAX 40
static char  _termBuf[TERM_LINE_BUF];
static int   _termBufLen = 0;
static char  _termLines[TERM_LINE_MAX][TERM_LINE_BUF];
static int   _termHead = 0;  // index of next write slot (circular)
static int   _termCount = 0; // how many lines are available
volatile int fnc_term_gen = 0;  // incremented each new line

extern "C" int fnc_term_count() { return _termCount; }
extern "C" void fnc_term_inject(const char* line) {
    // Inject a TX command echo into the terminal buffer
    strncpy(_termLines[_termHead], line, TERM_LINE_BUF-1);
    _termLines[_termHead][TERM_LINE_BUF-1] = 0;
    _termHead = (_termHead + 1) % TERM_LINE_MAX;
    if (_termCount < TERM_LINE_MAX) _termCount++;
    fnc_term_gen++;
}
extern "C" const char* fnc_term_line(int idx) {
    // idx=0 = oldest, idx=_termCount-1 = newest
    if (_termCount == 0) return "";
    int n = _termCount < TERM_LINE_MAX ? _termCount : TERM_LINE_MAX;
    int oldest = (_termHead - n + TERM_LINE_MAX) % TERM_LINE_MAX;
    int slot = (oldest + idx) % TERM_LINE_MAX;
    return _termLines[slot];
}

extern "C" void fnc_putchar(uint8_t c) {
    uart_write_bytes(fnc_uart_port, (const char*)&c, 1);
    fnc_tx_count++;
#ifdef ECHO_FNC_TO_DEBUG
    dbg_write(c);
#endif
}

void ledcolor(int n) {
    digitalWrite(4, !(n & 1));
    digitalWrite(16, !(n & 2));
    digitalWrite(17, !(n & 4));
}
extern "C" int fnc_getchar() {
    // Read from software ring buffer (filled by uart_reader_task on Core 0)
    // This is always fast — no hardware UART access, no blocking
    int res = uart_ring_pop();
    if (res >= 0) {
        char c = (char)res;
        // Accumulate into terminal line buffer
        if (c == '\n' || c == '\r') {
            if (_termBufLen > 0) {
                _termBuf[_termBufLen] = 0;
                strncpy(_termLines[_termHead], _termBuf, TERM_LINE_BUF-1);
                _termHead = (_termHead + 1) % TERM_LINE_MAX;
                if (_termCount < TERM_LINE_MAX) _termCount++;
                _termBufLen = 0;
                fnc_term_gen++;
            }
        } else if (_termBufLen < TERM_LINE_BUF-1 && c >= 32) {
            _termBuf[_termBufLen++] = c;
        }
        // update_rx_time() called in uart_reader_task when bytes arrive — not here
#ifdef ECHO_FNC_TO_DEBUG
        dbg_write(c);
#endif
        return (uint8_t)c;
    }
    return -1;
}

extern "C" void poll_extra() {
#ifdef DEBUG_TO_USB
    if (debugPort.available()) {
        char c = debugPort.read();
        if (c == 0x12) {  // CTRL-R
            ESP.restart();
            while (1) {}
        }
        fnc_putchar(c);  // So you can type commands to FluidNC
    }
#endif
}

void drawPngFile(const char* filename, int x, int y) {
    drawPngFile(&canvas, filename, x, y);
}
void drawPngFile(LGFX_Sprite* sprite, const char* filename, int x, int y) {
    // When datum is middle_center, the origin is the center of the canvas and the
    // +Y direction is down.
    std::string fn { "/" };
    fn += filename;
    sprite->drawPngFile(LittleFS, fn.c_str(), x, -y, 0, 0, 0, 0, 1.0f, 1.0f, datum_t::middle_center);
}

#define FORMAT_LITTLEFS_IF_FAILED true

// Baud rates up to 10M work
#ifndef FNC_BAUD
#    define FNC_BAUD 115200
#endif

extern void init_hardware();

// UART recovery — called when rx has been frozen too long
extern "C" void fnc_uart_reinit() {
    uart_driver_delete(fnc_uart_port);
    uart_driver_install(fnc_uart_port, 1024, 0, 0, NULL, ESP_INTR_FLAG_IRAM);
    // Reset ring buffer
    _uartRingHead = 0;
    _uartRingTail = 0;
    dbg_println("UART reinit");
}

void init_fnc_uart(int uart_num, int tx_pin, int rx_pin) {
    fnc_uart_port = (uart_port_t)uart_num;
    int baudrate  = FNC_BAUD;
    uart_driver_delete(fnc_uart_port);
    uart_set_pin(fnc_uart_port, (gpio_num_t)tx_pin, (gpio_num_t)rx_pin, -1, -1);
    uart_config_t conf;
#if defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S2)
    conf.source_clk = UART_SCLK_APB;  // ESP32, ESP32S2
#endif
#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3)
    // UART_SCLK_XTAL is independent of the APB frequency
    conf.source_clk = UART_SCLK_XTAL;  // ESP32C3, ESP32S3
#endif
    conf.baud_rate = baudrate;

    conf.data_bits           = UART_DATA_8_BITS;
    conf.parity              = UART_PARITY_DISABLE;
    conf.stop_bits           = UART_STOP_BITS_1;
    conf.flow_ctrl           = UART_HW_FLOWCTRL_DISABLE;
    conf.rx_flow_ctrl_thresh = 0;
    if (uart_param_config(fnc_uart_port, &conf) != ESP_OK) {
        dbg_println("UART config failed");
        while (1) {}
        return;
    };
    // Hardware UART buffer: 512 bytes (bridging gap during uart_reader_task wakeup).
    // XON/XOFF is DISABLED: FluidNC ignores XOFF by default, and with the ring buffer
    // approach we no longer need flow control — bytes go:
    //   hardware RX buffer (512B) → uart_reader_task → software ring (4096B) → fnc_getchar()
    // The 4096B ring buffer holds 40ms of maximum-rate UART data — far more than
    // any LCD redraw can take.
    uart_driver_install(fnc_uart_port, 1024, 0, 0, NULL, ESP_INTR_FLAG_IRAM);
    // No XON/XOFF — ring buffer makes it unnecessary and it caused compatibility issues
    uint32_t baud;
    uart_get_baudrate(fnc_uart_port, &baud);
    // Send XON to ensure FluidNC starts sending if it was previously paused
    fnc_putchar(0x11);
}

void init_system() {
    init_hardware();

    if (!LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED)) {
        dbg_println("LittleFS Mount Failed");
        return;
    }

    // Canvas created by ardmain.cpp after force_landscape() sets correct rotation
}
void resetFlowControl() {
    fnc_putchar(0x11);
    uart_ll_force_xon(fnc_uart_port);
}

extern "C" int milliseconds() {
    return millis();
}

void delay_ms(uint32_t ms) {
    delay(ms);
}

void dbg_write(uint8_t c) {
#ifdef DEBUG_TO_USB
    if (debugPort.availableForWrite() > 1) {
        debugPort.write(c);
    }
#endif
}

void dbg_print(const char* s) {
#ifdef DEBUG_TO_USB
    if (debugPort.availableForWrite() > strlen(s)) {
        debugPort.print(s);
    }
#endif
}

nvs_handle_t nvs_init(const char* name) {
    nvs_handle_t handle;
    esp_err_t    err = nvs_open(name, NVS_READWRITE, &handle);
    return err == ESP_OK ? handle : 0;
}
