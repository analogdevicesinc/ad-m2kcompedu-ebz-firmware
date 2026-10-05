/*
 * Display.h - OLED display driver (SH1107 64x128)
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include <Adafruit_SH110X.h>

#define CEN   "\x01"
#define CEN_E "\x02"
#define HL    "\x03"
#define HL_E  "\x04"
#define ST    "\x05"
#define ST_E  "\x06"

class Display
{
    public:
        Display(uint8_t i2c_address, uint16_t width, uint16_t height);
        void begin();
        void draw_text(const char text[]);

    private:
        Adafruit_SH1107 oled;
        uint8_t i2c_address;
        uint16_t width;
        uint16_t height;
};

#endif
