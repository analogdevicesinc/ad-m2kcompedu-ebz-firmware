/*
 * LedMatrix.h - UNO R4 12x8 LED matrix startup/idle animation driver
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LED_MATRIX_H
#define LED_MATRIX_H

#include <Arduino.h>
#include <Arduino_LED_Matrix.h>

class LedMatrix
{
    public:
        LedMatrix();
        void begin();
        void tick();

        static constexpr uint8_t ROWS = 8;
        static constexpr uint8_t COLS = 12;

    private:
        enum class Phase : uint8_t { Reveal, Scroll };

        void renderReveal(uint16_t f);
        void renderScroll(uint16_t f);

        ArduinoLEDMatrix matrix;
        Phase         phase;
        uint16_t      frame;
        unsigned long lastStep;

        static constexpr uint8_t  REVEAL_FRAMES      = 9;
        static constexpr unsigned long REVEAL_INTERVAL_MS = 100;
        static constexpr unsigned long SCROLL_INTERVAL_MS = 80;
};

#endif
