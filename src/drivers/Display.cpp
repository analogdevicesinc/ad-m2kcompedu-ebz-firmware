/*
 * Display.cpp - OLED display driver (SH1107 64x128)
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#include "Display.h"
#include <Wire.h>

constexpr uint8_t TEXT_SCALE   = 1;
constexpr uint8_t CHAR_WIDTH   = 6 * TEXT_SCALE;
constexpr uint8_t CHAR_HEIGHT  = 8 * TEXT_SCALE;
constexpr uint8_t LINE_SPACING = 7;
constexpr uint8_t HL_PADDING   = 4;
constexpr uint8_t Y_OFFSET     = 7;

static void draw_highlight(Adafruit_SH1107& oled, uint8_t x, uint8_t y, uint8_t w, uint8_t h)
{
    oled.drawRect(x - HL_PADDING, y - HL_PADDING,
                  w + HL_PADDING * 2, h + HL_PADDING * 2,
                  SH110X_WHITE);
}

Display::Display(uint8_t i2c_address, uint16_t width, uint16_t height)
    : oled(width, height, &Wire, -1), i2c_address(i2c_address), width(width), height(height)
{
}

void Display::begin()
{
    if (!oled.begin(i2c_address, true)) {
        Serial.println("Display init failed");
        for (;;);
    }
    oled.display();
    delay(1000);
    oled.setRotation(1);
    oled.clearDisplay();
    oled.setTextSize(TEXT_SCALE);
    oled.setTextColor(SH110X_WHITE);
    oled.setCursor(0, 0);
    oled.display();
}

void Display::draw_text(const char text[])
{
    oled.clearDisplay();

    uint8_t x = 0;
    uint8_t y = Y_OFFSET;
    bool centering = false;
    bool highlighting = false;
    uint8_t hl_start_x = 0;
    bool striking = false;
    uint8_t st_start_x = 0;

    // First pass for centering: need line length to compute offset
    // We'll handle it inline by buffering per-line when centering is active
    for (uint8_t i = 0; text[i] != '\0'; i++) {
        char c = text[i];

        switch (c) {
            case '\n':
                if (highlighting) {
                    draw_highlight(oled, hl_start_x, y, x - hl_start_x, CHAR_HEIGHT);
                    highlighting = false;
                }
                if (striking) {
                    oled.drawFastHLine(st_start_x, y + CHAR_HEIGHT / 2, x - st_start_x, SH110X_WHITE);
                    striking = false;
                }
                x = 0;
                y += CHAR_HEIGHT + LINE_SPACING;
                break;

            case '\x01':
                centering = true;
                {
                    uint8_t len = 0;
                    for (uint8_t j = i + 1; text[j] != '\0' && text[j] != '\x02' && text[j] != '\n'; j++) {
                        if (text[j] < '\x20') continue;
                        len++;
                    }
                    x = (height - len * CHAR_WIDTH) / 2;
                }
                break;

            case '\x02':
                centering = false;
                break;

            case '\x03':
                highlighting = true;
                hl_start_x = x;
                break;

            case '\x04':
                if (highlighting) {
                    draw_highlight(oled, hl_start_x, y, x - hl_start_x, CHAR_HEIGHT);
                    highlighting = false;
                }
                break;

            case '\x05':
                striking = true;
                st_start_x = x;
                break;

            case '\x06':
                if (striking) {
                    oled.drawFastHLine(st_start_x, y + CHAR_HEIGHT / 2, x - st_start_x, SH110X_WHITE);
                    striking = false;
                }
                break;

            default:
                oled.drawChar(x, y, c, SH110X_WHITE, SH110X_BLACK, TEXT_SCALE);
                x += CHAR_WIDTH;
                break;
        }
    }

    if (highlighting) {
        draw_highlight(oled, hl_start_x, y, x - hl_start_x, CHAR_HEIGHT);
    }
    if (striking) {
        oled.drawFastHLine(st_start_x, y + CHAR_HEIGHT / 2, x - st_start_x, SH110X_WHITE);
    }

    oled.display();
}

void Display::popup(const char text[], uint16_t secs)
{
    // Snapshot the current framebuffer so we can restore it afterwards.
    const size_t bufSize = (size_t)width * height / 8;
    uint8_t* fb = oled.getBuffer();
    uint8_t saved[bufSize];
    memcpy(saved, fb, bufSize);

    // Show the popup (draw_text wipes the screen and pushes to the panel).
    draw_text(text);

    // Blocking wait: the main loop stalls here, so no input is processed and
    // no parameter can change while the popup is visible.
    delay((uint32_t)secs * 1000UL);

    // Restore the previous screen exactly as it was.
    memcpy(fb, saved, bufSize);
    oled.display();
}
