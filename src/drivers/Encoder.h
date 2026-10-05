/*
 * Encoder.h - Rotary encoder driver
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

enum class Event : uint8_t {
    None,
    TurnCW,
    TurnCCW,
    Press,
    LongPress
};

class Encoder
{
    public:
        Encoder(uint8_t clk, uint8_t dt, uint8_t sw);
        void begin();
        void tick();
        Event poll();

    private:
        uint8_t pin_clk;
        uint8_t pin_dt;
        uint8_t pin_sw;

        bool lastClk = HIGH;
        bool lastSw = HIGH;
        bool swPressed = false;
        unsigned long pressStart = 0;
        unsigned long lastRotation = 0;

        static constexpr unsigned long DEBOUNCE_MS = 3;
        static constexpr unsigned long LONG_PRESS_MS = 600;

        Event lastEvent = Event::None;
};

#endif
