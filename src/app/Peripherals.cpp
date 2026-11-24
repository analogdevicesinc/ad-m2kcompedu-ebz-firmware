/*
 * Peripherals.cpp
 * Scope: Bridges AppState to hardware drivers.
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "Peripherals.h"
#include <SPI.h>

constexpr float SIN_GAIN_V_PER_LSB = 0.0053311;
constexpr float SQW_GAIN_V_PER_LSB = 0.03889;
constexpr float DC_OFF_V_PER_LSB   = 0.0031640;
constexpr float CRYSTAL_FREQ       = 20000000.0;
constexpr uint32_t ADC_PERIOD_MS   = 100;

static void claimSpi()
{
    SPI.begin();
    SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE2));
    SPI.transfer16(0x0000);
    SPI.endTransaction();
}

static void releaseSpi()
{
    SPI.end();
}

Peripherals::Peripherals(AD9833& waveformGen, AD5443& amplitudeCtrl,
                         AD5625R& dcOffsetCtrl, PwmOut& pwmOut,
                         uint8_t sqwSwitchPin, Messenger& messenger)
    : waveformGen(waveformGen),
      amplitudeCtrl(amplitudeCtrl),
      dcOffsetCtrl(dcOffsetCtrl),
      pwmOut(pwmOut),
      sqwSwitchPin(sqwSwitchPin),
      pwmRunning(false),
      messenger(messenger),
      applied{},
      lastOwner(SpiCanBusOwner::Can),
      lastAdcMs(0)
{
}

void Peripherals::init()
{
    beginAdc();
    beginPwm();
    dcOffsetCtrl.begin();

    const AppState& state = messenger.state();

    if (messenger.readSpiCanBusOwner() == SpiCanBusOwner::SignalGen)
    {
        beginSignalGen();
        claimSpi();
        updateFrequency(state);
        updateWave(state);
        updateAmplitude(state);
        releaseSpi();
    }

    updateDcOffset(state);
    applyPwm(state);

    applied = state;
    lastOwner = messenger.readSpiCanBusOwner();
}

void Peripherals::run()
{
    const AppState& state = messenger.state();
    SpiCanBusOwner owner = messenger.readSpiCanBusOwner();

    if (owner == SpiCanBusOwner::SignalGen)
    {
        bool regained = (lastOwner != SpiCanBusOwner::SignalGen);
        if (regained)
        {
            beginSignalGen();
        }

        if (regained ||
            state.waveform  != applied.waveform  || state.amplitude != applied.amplitude ||
            state.freqOrder != applied.freqOrder || state.frequency != applied.frequency)
        {
            claimSpi();
            updateFrequency(state);
            updateWave(state);
            updateAmplitude(state);
            releaseSpi();

            applied.waveform  = state.waveform;
            applied.amplitude = state.amplitude;
            applied.freqOrder = state.freqOrder;
            applied.frequency = state.frequency;
        }
    }
    lastOwner = owner;

    if (state.dcOffset != applied.dcOffset)
    {
        updateDcOffset(state);
        applied.dcOffset = state.dcOffset;
    }

    if (state.pwmEnabled   != applied.pwmEnabled   || state.pwmFreqOrder != applied.pwmFreqOrder ||
        state.pwmFrequency != applied.pwmFrequency || state.dutyCycle    != applied.dutyCycle)
    {
        applyPwm(state);
        applied.pwmEnabled   = state.pwmEnabled;
        applied.pwmFreqOrder = state.pwmFreqOrder;
        applied.pwmFrequency = state.pwmFrequency;
        applied.dutyCycle    = state.dutyCycle;
    }

    updateAdc();
}

void Peripherals::beginSignalGen()
{
    pinMode(sqwSwitchPin, OUTPUT);
    digitalWrite(sqwSwitchPin, HIGH);

    claimSpi();
    amplitudeCtrl.begin();
    waveformGen.begin();
    waveformGen.setCrystalFrequency(CRYSTAL_FREQ);
    releaseSpi();
}

void Peripherals::beginPwm()
{
    pwmOut.begin(1000.0f, 50.0f);
    pwmOut.suspend();
}

void Peripherals::beginAdc()
{
    pinMode(A2, INPUT);
    pinMode(A3, INPUT);

    AppState& state = messenger.state();
    for (uint8_t i = 0; i < 2; i++)
    {
        state.voltmeterScale[i] = (float)ADC_FULL_SCALE_MV / (float)ADC_MAX_COUNT;
        state.audioScale[i] = 0.001f;
    }
}

void Peripherals::updateWave(const AppState& state)
{
    uint8_t mode;
    switch (state.waveform)
    {
        case 0:  mode = AD9833_OFF;      break;
        case 1:  mode = AD9833_OFF;      break;
        case 2:  mode = AD9833_SINE;     break;
        case 3:  mode = AD9833_SQUARE1;  break;
        case 4:  mode = AD9833_TRIANGLE; break;
        default: mode = AD9833_OFF;      break;
    }

    if (state.waveform == 3)
    {
        digitalWrite(sqwSwitchPin, LOW);
    }
    else
    {
        digitalWrite(sqwSwitchPin, HIGH);
    }

    waveformGen.setWave(mode);
    waveformGen.setWave(mode);
}

void Peripherals::updateFrequency(const AppState& state)
{
    float freq = state.frequency;
    if (state.freqOrder == 1)
    {
        freq *= 1000.0f;
    }
    else if (state.freqOrder == 2)
    {
        freq *= 1000000.0f;
    }

    if (state.waveform != 0)
    {
        waveformGen.setFrequency(freq, 0);
    }
}

void Peripherals::updateAmplitude(const AppState& state)
{
    float vpp = state.amplitude / 1000.0;

    float gain = SIN_GAIN_V_PER_LSB;
    if (state.waveform == 3)
    {
        gain = SQW_GAIN_V_PER_LSB;
    }

    int32_t lsb = (int32_t)round(vpp / gain);

    if (state.waveform == 3)
    {
        if (state.amplitude <= 1000)
        {
            lsb += state.calSqw1V;
        }
        else
        {
            lsb += state.calSqw10V;
        }
    }
    else
    {
        if (state.amplitude <= 1000)
        {
            lsb += state.calSine1V;
        }
        else
        {
            lsb += state.calSine10V;
        }
    }

    if (lsb < 0) { lsb = 0; }
    if (lsb > 4095) { lsb = 4095; }

    amplitudeCtrl.write((uint16_t)lsb);
}

void Peripherals::updateDcOffset(const AppState& state)
{
    float vdc = state.dcOffset / 1000.0;
    int32_t lsb = (int32_t)round(vdc / DC_OFF_V_PER_LSB);
    lsb += state.calOffset;

    uint16_t dacA, dacB;
    if (lsb >= 0)
    {
        dacA = 2048 + (lsb + 1) / 2;
        dacB = 2048 - lsb / 2;
    }
    else
    {
        dacA = 2048 + lsb / 2;
        dacB = 2048 - (lsb - 1) / 2;
    }

    dcOffsetCtrl.write(dacA, 0);
    dcOffsetCtrl.write(dacB, 1);
}

void Peripherals::updateAdc()
{
    if (messenger.audioActive())
    {
        return;
    }

    uint32_t now = millis();
    if (now - lastAdcMs < ADC_PERIOD_MS)
    {
        return;
    }
    lastAdcMs = now;

    AppState& state = messenger.state();
    const uint8_t pins[2] = { A2, A3 };
    for (uint8_t i = 0; i < 2; i++)
    {
        uint16_t raw = analogRead(pins[i]);
        state.voltmeterRaw[i] = raw;
        state.voltmeterMv[i]  = ((uint32_t)raw * ADC_FULL_SCALE_MV + ADC_MAX_COUNT / 2) / ADC_MAX_COUNT;
    }
}

void Peripherals::applyPwm(const AppState& state)
{
    if (state.pwmEnabled && !pwmRunning)
    {
        pwmOut.resume();
        pwmRunning = true;
    }
    else if (!state.pwmEnabled && pwmRunning)
    {
        pwmOut.suspend();
        pwmRunning = false;
    }

    float freq = state.pwmFrequency;
    if (state.pwmFreqOrder == 1)
    {
        freq *= 1000.0f;
    }
    else if (state.pwmFreqOrder == 2)
    {
        freq *= 1000000.0f;
    }

    pwmOut.end();
    pwmOut.begin(freq, (float)state.dutyCycle);

    if (!state.pwmEnabled)
    {
        pwmOut.suspend();
    }
    else
    {
        pwmRunning = true;
    }
}
