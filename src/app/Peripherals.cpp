/*
 * Peripherals.cpp
 * Scope: Bridges AppState to hardware drivers.
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "Peripherals.h"
#include "Controler.h"

constexpr float SIN_GAIN_V_PER_LSB = 0.0053311;
constexpr float SQW_GAIN_V_PER_LSB = 0.03889;
constexpr float DC_OFF_V_PER_LSB   = 0.0031640;
constexpr float CRYSTAL_FREQ       = 20000000.0;

Peripherals::Peripherals(AD9833& waveformGen, AD5443& amplitudeCtrl,
                         AD5625R& dcOffsetCtrl, PwmOut& pwmOut,
                         uint8_t sqwSwitchPin)
    : waveformGen(waveformGen),
      amplitudeCtrl(amplitudeCtrl),
      dcOffsetCtrl(dcOffsetCtrl),
      pwmOut(pwmOut),
      sqwSwitchPin(sqwSwitchPin),
      pwmRunning(false)
{
}

void Peripherals::begin()
{
    pinMode(sqwSwitchPin, OUTPUT);
    digitalWrite(sqwSwitchPin, HIGH);

    amplitudeCtrl.begin();
    dcOffsetCtrl.begin();

    waveformGen.begin();
    waveformGen.setCrystalFrequency(CRYSTAL_FREQ);

    pwmOut.begin(1000.0f, 50.0f);
    pwmOut.suspend();
}

void Peripherals::update(const AppState& state, bool* refreshFlags)
{
    bool waveChanged = refreshFlags[0];

    if (refreshFlags[2] || refreshFlags[3] || waveChanged)
    {
        updateFrequency(state);
        refreshFlags[2] = false;
        refreshFlags[3] = false;
    }
    if (refreshFlags[0])
    {
        updateWave(state);
        refreshFlags[0] = false;
        refreshFlags[1] = true;
    }
    if (refreshFlags[1])
    {
        updateAmplitude(state);
        refreshFlags[1] = false;
    }
    if (refreshFlags[4])
    {
        updateDcOffset(state);
        refreshFlags[4] = false;
    }
    if (refreshFlags[5] || refreshFlags[6] || refreshFlags[7] || refreshFlags[8])
    {
        updatePwm(state);
        refreshFlags[5] = refreshFlags[6] = refreshFlags[7] = refreshFlags[8] = false;
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
    float gain = (state.waveform == 3) ? SQW_GAIN_V_PER_LSB : SIN_GAIN_V_PER_LSB;
    int32_t lsb = (int32_t)round(vpp / gain);

    if (state.waveform == 3)
    {
        lsb += (state.amplitude <= 1000) ? state.calSqw1V : state.calSqw10V;
    }
    else
    {
        lsb += (state.amplitude <= 1000) ? state.calSine1V : state.calSine10V;
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

void Peripherals::updatePwm(const AppState& state)
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
