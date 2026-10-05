/*
 * AD5443.h - 12-bit MDAC driver (amplitude control)
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef AD5443_H
#define AD5443_H

#include <Arduino.h>

class AD5443
{
    public:
        AD5443(uint8_t syncPin);
        void begin();
        uint16_t write(uint16_t data);

    private:
        uint8_t syncPin;
};

#endif
