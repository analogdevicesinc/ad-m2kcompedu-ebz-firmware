/*
 * Encoder.cpp - Rotary encoder driver
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "Encoder.h"

Encoder::Encoder(uint8_t clk, uint8_t dt, uint8_t sw)
    : pin_clk(clk), pin_dt(dt), pin_sw(sw)
{
}

void Encoder::begin()
{
    pinMode(pin_clk, INPUT_PULLUP);
    pinMode(pin_dt, INPUT_PULLUP);
    pinMode(pin_sw, INPUT_PULLUP);
    lastClk = digitalRead(pin_clk);
    lastSw = HIGH;
}

void Encoder::tick()
{
    unsigned long now = millis();

    // --- Rotation ---
    bool clk = digitalRead(pin_clk);
    if (clk != lastClk && clk == LOW) {
        if (now - lastRotation >= DEBOUNCE_MS) {
            bool dt = digitalRead(pin_dt);
            if (dt != clk) {
                lastEvent = Event::TurnCCW;
            } else {
                lastEvent = Event::TurnCW;
            }
            lastRotation = now;
        }
    }
    lastClk = clk;

    // --- Button ---
    bool sw = digitalRead(pin_sw);
    if (sw == LOW && lastSw == HIGH) {
        swPressed = true;
        pressStart = now;
    } else if (sw == HIGH && lastSw == LOW) {
        if (swPressed) {
            if (now - pressStart >= LONG_PRESS_MS) {
                lastEvent = Event::LongPress;
            } else {
                lastEvent = Event::Press;
            }
            swPressed = false;
        }
    }
    lastSw = sw;
}

Event Encoder::poll()
{
    Event ev = lastEvent;
    lastEvent = Event::None;
    return ev;
}
