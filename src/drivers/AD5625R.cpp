/*
 * AD5625R.cpp - Quad 12-bit DAC driver (DC offset control)
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "AD5625R.h"
#include <Wire.h>

static constexpr uint8_t CMD_WRITE_UPDATE_N = 0b011;
static constexpr uint8_t CMD_PWR_REG        = 0b100;
static constexpr uint8_t CMD_LDAC_REG       = 0b110;
static constexpr uint8_t CMD_INTERNAL_REF   = 0b111;

AD5625R::AD5625R(uint8_t i2cAddr)
    : addr(i2cAddr)
{
}

void AD5625R::begin()
{
    // Power down unused channels C and D
    Wire.beginTransmission(addr);
    Wire.write((CMD_PWR_REG << 3) | 0x00);
    Wire.write(0x00);
    Wire.write(0x3C);
    Wire.endTransmission();

    // Auto LDAC on channels A and B
    Wire.beginTransmission(addr);
    Wire.write((CMD_LDAC_REG << 3) | 0x00);
    Wire.write(0x00);
    Wire.write(0x03);
    Wire.endTransmission();

    // Internal reference on
    Wire.beginTransmission(addr);
    Wire.write((CMD_INTERNAL_REF << 3) | 0x00);
    Wire.write(0x00);
    Wire.write(0x01);
    Wire.endTransmission();

    // Both channels to midscale (zero offset)
    write(2048, 0);
    write(2048, 1);
}

void AD5625R::write(uint16_t data, uint8_t channel)
{
    Wire.beginTransmission(addr);
    Wire.write((CMD_WRITE_UPDATE_N << 3) | (channel & 0x07));
    Wire.write((data & 0x0FF0) >> 4);
    Wire.write((data & 0x0F) << 4);
    Wire.endTransmission();
}
