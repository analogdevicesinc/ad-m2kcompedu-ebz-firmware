/*
 * CalibrationWizard.cpp
 * Scope: Headless calibration model. Steps the signal generator through the
 *        shared state, trims cal values, persists them to EEPROM, and publishes
 *        its view state to the bus. Does no rendering of its own.
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "CalibrationWizard.h"
#include <EEPROM.h>

constexpr uint16_t EEPROM_ADDR  = 0x10;
constexpr char     EEPROM_MAGIC = 'O';

struct CalStorage {
    char    id;
    int16_t calOffset;
    int16_t calSine1V;
    int16_t calSine10V;
    int16_t calSqw1V;
    int16_t calSqw10V;
};

CalibrationWizard::CalibrationWizard(Messenger& messenger)
    : messenger(messenger),
      _active(false),
      done(false),
      calType(CalType::Offset),
      step(0),
      totalSteps(0),
      savedWaveform(0),
      savedAmplitude(0),
      savedFreqOrder(0),
      savedFrequency(0.0f),
      savedDcOffset(0)
{
}

void CalibrationWizard::init()
{
    loadFromEEPROM();
}

void CalibrationWizard::run()
{
    if (!_active)
    {
        if (messenger.calibrationPending())
        {
            CalType type = messenger.calibrationRequest();
            messenger.clearCalibrationRequest();
            if (messenger.readSpiCanBusOwner() == SpiCanBusOwner::SignalGen)
            {
                start(type);
            }
        }
        return;
    }

    CalCmd cmd = messenger.takeCalCommand();
    if (cmd != CalCmd::None)
    {
        process(cmd);
    }
}

void CalibrationWizard::loadFromEEPROM()
{
    AppState& s = messenger.state();
    CalStorage stored;
    EEPROM.get(EEPROM_ADDR, stored);

    if (stored.id == EEPROM_MAGIC)
    {
        s.calOffset  = stored.calOffset;
        s.calSine1V  = stored.calSine1V;
        s.calSine10V = stored.calSine10V;
        s.calSqw1V   = stored.calSqw1V;
        s.calSqw10V  = stored.calSqw10V;
    }
    else
    {
        s.calOffset  = 0;
        s.calSine1V  = 0;
        s.calSine10V = 0;
        s.calSqw1V   = 0;
        s.calSqw10V  = 0;
    }
}

void CalibrationWizard::start(CalType type)
{
    AppState& s = messenger.state();

    _active = true;
    done    = false;
    calType = type;
    step    = 0;

    switch (type)
    {
        case CalType::Offset:  totalSteps = 1; break;
        case CalType::SineAmp: totalSteps = 2; break;
        case CalType::SqwAmp:  totalSteps = 2; break;
    }

    savedWaveform  = s.waveform;
    savedAmplitude = s.amplitude;
    savedFreqOrder = s.freqOrder;
    savedFrequency = s.frequency;
    savedDcOffset  = s.dcOffset;

    enterStep();
}

void CalibrationWizard::enterStep()
{
    AppState& s = messenger.state();

    s.freqOrder = 1;
    s.frequency = 1.0f;
    s.dcOffset  = 0;

    switch (calType)
    {
        case CalType::Offset:
            s.waveform  = 0;
            s.amplitude = 0;
            break;
        case CalType::SineAmp:
            s.waveform  = 2;
            if (step == 0)
            {
                s.amplitude = 1000;
            }
            else
            {
                s.amplitude = 10000;
            }
            break;
        case CalType::SqwAmp:
            s.waveform  = 3;
            if (step == 0)
            {
                s.amplitude = 1000;
            }
            else
            {
                s.amplitude = 10000;
            }
            break;
    }

    publishView();
}

void CalibrationWizard::process(CalCmd cmd)
{
    AppState& s = messenger.state();

    if (done)
    {
        if (cmd == CalCmd::Next || cmd == CalCmd::Abort)
        {
            done    = false;
            _active = false;
            publishView();
        }
        return;
    }

    switch (cmd)
    {
        case CalCmd::AdjustUp:
            adjust(+1);
            break;
        case CalCmd::AdjustDown:
            adjust(-1);
            break;
        case CalCmd::Next:
            step++;
            if (step >= totalSteps)
            {
                finish();
            }
            else
            {
                enterStep();
            }
            break;
        case CalCmd::Abort:
            s.waveform  = savedWaveform;
            s.amplitude = savedAmplitude;
            s.freqOrder = savedFreqOrder;
            s.frequency = savedFrequency;
            s.dcOffset  = savedDcOffset;
            _active = false;
            publishView();
            break;
        case CalCmd::None:
            break;
    }
}

void CalibrationWizard::adjust(int8_t dir)
{
    AppState& s = messenger.state();
    int16_t* target = nullptr;

    switch (calType)
    {
        case CalType::Offset:
            target = &s.calOffset;
            break;
        case CalType::SineAmp:
            if (step == 0)
            {
                target = &s.calSine1V;
            }
            else
            {
                target = &s.calSine10V;
            }
            break;
        case CalType::SqwAmp:
            if (step == 0)
            {
                target = &s.calSqw1V;
            }
            else
            {
                target = &s.calSqw10V;
            }
            break;
    }

    *target += dir;
    if (*target > 500)  { *target = 500; }
    if (*target < -500) { *target = -500; }

    publishView();
}

void CalibrationWizard::finish()
{
    AppState& s = messenger.state();

    CalStorage stored;
    stored.id         = EEPROM_MAGIC;
    stored.calOffset  = s.calOffset;
    stored.calSine1V  = s.calSine1V;
    stored.calSine10V = s.calSine10V;
    stored.calSqw1V   = s.calSqw1V;
    stored.calSqw10V  = s.calSqw10V;
    EEPROM.put(EEPROM_ADDR, stored);

    s.waveform  = savedWaveform;
    s.amplitude = savedAmplitude;
    s.freqOrder = savedFreqOrder;
    s.frequency = savedFrequency;
    s.dcOffset  = savedDcOffset;

    done = true;
    publishView();
}

int16_t CalibrationWizard::currentValue() const
{
    const AppState& s = messenger.state();

    switch (calType)
    {
        case CalType::Offset:
            return s.calOffset;
        case CalType::SineAmp:
            if (step == 0)
            {
                return s.calSine1V;
            }
            return s.calSine10V;
        case CalType::SqwAmp:
            if (step == 0)
            {
                return s.calSqw1V;
            }
            return s.calSqw10V;
    }
    return 0;
}

void CalibrationWizard::publishView()
{
    CalView v;
    v.active = _active;
    v.done   = done;
    v.type   = calType;
    v.step   = step;
    v.value  = currentValue();
    messenger.publishCalView(v);
}
