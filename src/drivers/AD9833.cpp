/*
 * AD9833.cpp - programmable waveform generator driver
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "AD9833.h"
#include <SPI.h>

static constexpr uint16_t CTRL_B28     = 1 << 13;
static constexpr uint16_t CTRL_RESET   = 1 << 8;
static constexpr uint16_t CTRL_SLEEP1  = 1 << 7;
static constexpr uint16_t CTRL_SLEEP12 = 1 << 6;
static constexpr uint16_t CTRL_OPBITEN = 1 << 5;
static constexpr uint16_t CTRL_DIV2    = 1 << 3;
static constexpr uint16_t CTRL_MODE    = 1 << 1;

static constexpr uint16_t REG_FREQ0  = 0x4000;
static constexpr uint16_t REG_FREQ1  = 0x8000;
static constexpr uint16_t REG_PHASE0 = 0xC000;
static constexpr uint16_t REG_PHASE1 = 0xE000;

static constexpr uint32_t FREQ_WORD_MAX = 0x0FFFFFFF;
static constexpr float    POW2TO28      = 268435456.0f;

AD9833::AD9833(uint8_t fsyncPin)
    : fsyncPin(fsyncPin),
      control(CTRL_B28),
      freqConst(POW2TO28 / 25000000.0f)
{
}

void AD9833::begin()
{
    pinMode(fsyncPin, OUTPUT);
    digitalWrite(fsyncPin, HIGH);

    control = CTRL_B28;
    writeData(control | CTRL_RESET);
    delay(10);
    writeData(control);

    setFrequency(0.0f, 0);
    setFrequency(0.0f, 1);
    setPhase(0.0f, 0);
    setPhase(0.0f, 1);
}

void AD9833::setCrystalFrequency(float mclk)
{
    freqConst = POW2TO28 / mclk;
}

void AD9833::setWave(uint8_t wave)
{
    if (wave > AD9833_TRIANGLE)
    {
        return;
    }

    control &= ~(CTRL_SLEEP1 | CTRL_SLEEP12 | CTRL_OPBITEN | CTRL_DIV2 | CTRL_MODE);

    switch (wave)
    {
        case AD9833_OFF:      control |= CTRL_SLEEP1 | CTRL_SLEEP12; break;
        case AD9833_SINE:                                            break;
        case AD9833_SQUARE1:  control |= CTRL_OPBITEN | CTRL_DIV2;   break;
        case AD9833_SQUARE2:  control |= CTRL_OPBITEN;               break;
        case AD9833_TRIANGLE: control |= CTRL_MODE;                  break;
    }

    writeData(control);
}

void AD9833::setFrequency(float frequency, uint8_t reg)
{
    if (reg > 1)
    {
        return;
    }

    if (frequency < 0.0f)
    {
        frequency = 0.0f;
    }

    uint32_t word = (uint32_t)round(frequency * freqConst);
    if (word > FREQ_WORD_MAX)
    {
        word = FREQ_WORD_MAX;
    }

    control |= CTRL_B28;
    writeData(control);

    uint16_t address = (reg == 0) ? REG_FREQ0 : REG_FREQ1;
    writeData(address | (word & 0x3FFF));
    writeData(address | ((word >> 14) & 0x3FFF));
}

void AD9833::setPhase(float degrees, uint8_t reg)
{
    if (reg > 1)
    {
        return;
    }

    uint16_t word = (uint16_t)round(degrees * (4096.0f / 360.0f)) & 0x0FFF;
    writeData(((reg == 0) ? REG_PHASE0 : REG_PHASE1) | word);
}

void AD9833::writeData(uint16_t value)
{
    SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE2));
    digitalWrite(fsyncPin, LOW);
    SPI.transfer16(value);
    digitalWrite(fsyncPin, HIGH);
    SPI.endTransaction();
}
