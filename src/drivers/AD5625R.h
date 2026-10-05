/*
 * AD5625R.h - Quad 12-bit DAC driver (DC offset control)
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef AD5625R_H
#define AD5625R_H

#include <Arduino.h>

class AD5625R
{
    public:
        AD5625R(uint8_t i2cAddr);
        void begin();
        void write(uint16_t data, uint8_t channel);

    private:
        uint8_t addr;
};

#endif
