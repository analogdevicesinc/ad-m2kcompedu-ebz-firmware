/*
 * AD9833.h - programmable waveform generator driver
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef AD9833_H
#define AD9833_H

#include <Arduino.h>

constexpr uint8_t AD9833_OFF      = 0;
constexpr uint8_t AD9833_SINE     = 1;
constexpr uint8_t AD9833_SQUARE1  = 2;
constexpr uint8_t AD9833_SQUARE2  = 3;
constexpr uint8_t AD9833_TRIANGLE = 4;

class AD9833
{
    public:
        AD9833(uint8_t fsyncPin);

        void begin();
        void setCrystalFrequency(float mclk);
        void setWave(uint8_t wave);
        void setFrequency(float frequency, uint8_t reg);
        void setPhase(float degrees, uint8_t reg);

    private:
        void writeData(uint16_t value);

        uint8_t  fsyncPin;
        uint16_t control;
        float    freqConst;
};

#endif
