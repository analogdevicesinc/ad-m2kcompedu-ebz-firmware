/*
 * AD5443.cpp - 12-bit MDAC driver (amplitude control)
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "AD5443.h"
#include <SPI.h>

static constexpr uint16_t CMD_LOAD_UPDATE = 0x1000;
static constexpr uint16_t CMD_READBACK    = 0x2000;
static constexpr uint16_t CMD_NOP         = 0x0000;

AD5443::AD5443(uint8_t syncPin)
    : syncPin(syncPin)
{
}

void AD5443::begin()
{
    pinMode(syncPin, OUTPUT);
    digitalWrite(syncPin, HIGH);
}

uint16_t AD5443::write(uint16_t data)
{
    data &= 0x0FFF;

    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));

    digitalWrite(syncPin, LOW);
    SPI.transfer16(CMD_LOAD_UPDATE | data);
    digitalWrite(syncPin, HIGH);

    digitalWrite(syncPin, LOW);
    SPI.transfer16(CMD_READBACK);
    digitalWrite(syncPin, HIGH);

    digitalWrite(syncPin, LOW);
    uint16_t readback = SPI.transfer16(CMD_NOP);
    digitalWrite(syncPin, HIGH);

    SPI.endTransaction();

    return readback;
}
