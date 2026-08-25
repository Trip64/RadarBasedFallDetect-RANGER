/**
 * LovyanGFX Hardware Configuration for CrowPanel Advance 4.3" (ESP32-S3)
 * 
 * Hardware:
 *   - Display: ST7265 RGB 16-bit Parallel (800x480 IPS)
 *   - Touch Controller: GT911 Capacitive (I2C @ 0x5D, GPIO15/16)
 *   - Port Expander: TCA9534 (I2C @ 0x18, Pin 1=Backlight, Pin 2=Touch RST)
 */

#pragma once

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>
#include <driver/i2c.h>

#define DISPLAY_WIDTH  800
#define DISPLAY_HEIGHT 480
#define TCA9534_ADDR   0x18

class LGFX : public lgfx::LGFX_Device {
public:
    lgfx::Bus_RGB     _bus_instance;
    lgfx::Panel_RGB   _panel_instance;
    lgfx::Touch_GT911 _touch_instance;

    LGFX(void) {
        // Panel geometry
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

        // Allocate framebuffer in Octal-SPI PSRAM
        {
            auto cfg = _panel_instance.config_detail();
            cfg.use_psram = 1;
            _panel_instance.config_detail(cfg);
        }

        // RGB 16-bit parallel bus pinout
        {
            auto cfg = _bus_instance.config();
            cfg.panel = &_panel_instance;

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

            cfg.pin_henable = GPIO_NUM_42;
            cfg.pin_vsync   = GPIO_NUM_41;
            cfg.pin_hsync   = GPIO_NUM_40;
            cfg.pin_pclk    = GPIO_NUM_39;

            // Pixel clock set to 14MHz to prevent DMA bus contention with PSRAM
            cfg.freq_write = 14000000;

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
        setPanel(&_panel_instance);

        // Capacitive touch controller
        {
            auto cfg = _touch_instance.config();
            cfg.x_min = 0;
            cfg.x_max = 799;
            cfg.y_min = 0;
            cfg.y_max = 479;
            cfg.pin_int = -1;
            cfg.bus_shared = true;
            cfg.offset_rotation = 0;
            cfg.i2c_port = 1;
            cfg.i2c_addr = 0x5D;
            cfg.pin_sda = GPIO_NUM_15;
            cfg.pin_scl = GPIO_NUM_16;
            cfg.freq = 400000;
            _touch_instance.config(cfg);
            _panel_instance.setTouch(&_touch_instance);
        }
    }
};
