/**
 * gfx_conf.h — LovyanGFX Configuration for CrowPanel Advance 4.3" V1.0
 *
 * Display:  ST7265 RGB 16-bit Parallel (800×480)
 * Touch:    GT911 Capacitive (I2C @ 0x5D, GPIO 15/16)
 * IO Exp:   TCA9534 @ 0x18 (backlight, touch reset)
 *
 * Matched exactly to the official Elecrow V1.0 example code:
 * https://github.com/Elecrow-RD/CrowPanel-Advance-4.3-HMI-ESP32-S3-AI-Powered-IPS-Touch-Screen-800x480
 */

#pragma once

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>
#include <driver/i2c.h>

// ============== Dimensions ==============
#define DISPLAY_WIDTH  800
#define DISPLAY_HEIGHT 480

// ============== TCA9534 Port Expander ==============
// Official Elecrow V1.0: address 0x18
//   Pin 1 = Backlight enable (HIGH = on)
//   Pin 2 = Touch RST (active LOW)
#define TCA9534_ADDR  0x18

// ============== LGFX Configuration Class ==============
class LGFX : public lgfx::LGFX_Device {
public:
    lgfx::Bus_RGB     _bus_instance;
    lgfx::Panel_RGB   _panel_instance;
    lgfx::Touch_GT911 _touch_instance;

    LGFX(void) {
        // ---- Panel Configuration ----
        {
            auto cfg = _panel_instance.config();
            cfg.memory_width  = 800;
            cfg.memory_height = 480;
            cfg.panel_width   = 800;
            cfg.panel_height  = 480;
            cfg.offset_x = 0;
            cfg.offset_y = 0;
            _panel_instance.config(cfg);
        }

        // ---- PSRAM Framebuffer ----
        {
            auto cfg = _panel_instance.config_detail();
            cfg.use_psram = 1;
            _panel_instance.config_detail(cfg);
        }

        // ---- Bus (RGB Parallel 16-bit) ----
        {
            auto cfg = _bus_instance.config();
            cfg.panel = &_panel_instance;

            // Data pins — exact Elecrow V1.0 mapping
            cfg.pin_d0  = GPIO_NUM_21;   // B0
            cfg.pin_d1  = GPIO_NUM_47;   // B1
            cfg.pin_d2  = GPIO_NUM_48;   // B2
            cfg.pin_d3  = GPIO_NUM_45;   // B3
            cfg.pin_d4  = GPIO_NUM_38;   // B4
            cfg.pin_d5  = GPIO_NUM_9;    // G0
            cfg.pin_d6  = GPIO_NUM_10;   // G1
            cfg.pin_d7  = GPIO_NUM_11;   // G2
            cfg.pin_d8  = GPIO_NUM_12;   // G3
            cfg.pin_d9  = GPIO_NUM_13;   // G4
            cfg.pin_d10 = GPIO_NUM_14;   // G5
            cfg.pin_d11 = GPIO_NUM_7;    // R0
            cfg.pin_d12 = GPIO_NUM_17;   // R1
            cfg.pin_d13 = GPIO_NUM_18;   // R2
            cfg.pin_d14 = GPIO_NUM_3;    // R3
            cfg.pin_d15 = GPIO_NUM_46;   // R4

            // Sync signals
            cfg.pin_henable = GPIO_NUM_42;
            cfg.pin_vsync   = GPIO_NUM_41;
            cfg.pin_hsync   = GPIO_NUM_40;
            cfg.pin_pclk    = GPIO_NUM_39;

            // Elecrow V1.0 reference value. 14 MHz produced only ~34 Hz panel
            // scan and was visibly unstable on the physical spare panel.
            cfg.freq_write = 21000000;

            // Timing — from Elecrow official V1.0 example
            cfg.hsync_polarity    = 0;
            cfg.hsync_front_porch = 8;
            cfg.hsync_pulse_width = 4;
            cfg.hsync_back_porch  = 8;
            cfg.vsync_polarity    = 0;
            cfg.vsync_front_porch = 8;
            cfg.vsync_pulse_width = 4;
            cfg.vsync_back_porch  = 8;
            cfg.pclk_idle_high    = 1;

            _bus_instance.config(cfg);
        }
        _panel_instance.setBus(&_bus_instance);

        // ---- Touch (GT911) — I2C port 0, GPIO 15/16 ----
        {
            auto cfg = _touch_instance.config();
            cfg.x_min = 0;
            cfg.x_max = 800;
            cfg.y_min = 0;
            cfg.y_max = 480;
            cfg.pin_int = -1;
            // This flag describes sharing the panel's RGB bus, not sharing I2C
            // with the IO expander/Pico. Elecrow's V1.0 reference sets false.
            cfg.bus_shared = false;
            cfg.offset_rotation = 0;
            cfg.i2c_port = I2C_NUM_0;
            cfg.pin_sda  = GPIO_NUM_15;
            cfg.pin_scl  = GPIO_NUM_16;
            cfg.pin_rst  = -1;     // RST via TCA9534, not direct GPIO
            // 100 kHz is intentionally conservative for the off-board Pico cable.
            cfg.freq = 100000;
            cfg.i2c_addr = 0x5D;
            _touch_instance.config(cfg);
            _panel_instance.setTouch(&_touch_instance);
        }

        setPanel(&_panel_instance);
    }
};
