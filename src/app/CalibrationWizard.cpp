/*
 * CalibrationWizard.cpp
 * Scope: Takes over input/display during calibration.
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "CalibrationWizard.h"
#include "Controler.h"
#include "Peripherals.h"
#include <EEPROM.h>

constexpr uint16_t EEPROM_ADDR = 0x10;
static constexpr char EEPROM_MAGIC = 'O';

struct CalStorage {
    char id;
    int16_t calOffset;
    int16_t calSine1V;
    int16_t calSine10V;
    int16_t calSqw1V;
    int16_t calSqw10V;
};

CalibrationWizard::CalibrationWizard(Display& display, Peripherals& peripherals)
    : display(display), peripherals(peripherals),
      _active(false), done(false), calType(CalType::Offset), step(0), totalSteps(0)
{
}

void CalibrationWizard::loadFromEEPROM(AppState& state)
{
    CalStorage stored;
    EEPROM.get(EEPROM_ADDR, stored);

    if (stored.id == EEPROM_MAGIC)
    {
        state.calOffset  = stored.calOffset;
        state.calSine1V  = stored.calSine1V;
        state.calSine10V = stored.calSine10V;
        state.calSqw1V   = stored.calSqw1V;
        state.calSqw10V  = stored.calSqw10V;
    }
    else
    {
        state.calOffset  = 0;
        state.calSine1V  = 0;
        state.calSine10V = 0;
        state.calSqw1V   = 0;
        state.calSqw10V  = 0;
    }
}

void CalibrationWizard::start(CalType type, AppState& state, bool* refreshFlags)
{
    _active = true;
    calType = type;
    step = 0;

    switch (type)
    {
        case CalType::Offset:  totalSteps = 1; break;
        case CalType::SineAmp: totalSteps = 2; break;
        case CalType::SqwAmp:  totalSteps = 2; break;
    }

    savedWaveform  = state.waveform;
    savedAmplitude = state.amplitude;
    savedFreqOrder = state.freqOrder;
    savedFrequency = state.frequency;
    savedDcOffset  = state.dcOffset;

    enterStep(state, refreshFlags);
}

void CalibrationWizard::enterStep(AppState& state, bool* refreshFlags)
{
    switch (calType)
    {
        case CalType::Offset:
            state.waveform  = 0;
            state.amplitude = 0;
            state.freqOrder = 1;
            state.frequency = 1.0f;
            state.dcOffset  = 0;
            break;

        case CalType::SineAmp:
            state.waveform  = 2;
            state.freqOrder = 1;
            state.frequency = 1.0f;
            state.dcOffset  = 0;
            state.amplitude = (step == 0) ? 1000 : 10000;
            break;

        case CalType::SqwAmp:
            state.waveform  = 3;
            state.freqOrder = 1;
            state.frequency = 1.0f;
            state.dcOffset  = 0;
            state.amplitude = (step == 0) ? 1000 : 10000;
            break;
    }

    refreshFlags[0] = true;
    refreshFlags[1] = true;
    refreshFlags[2] = true;
    refreshFlags[3] = true;
    refreshFlags[4] = true;

    peripherals.update(state, refreshFlags);
    render(state);
}

void CalibrationWizard::process(Event ev, AppState& state, bool* refreshFlags)
{
    if (done)
    {
        if (ev == Event::Press || ev == Event::LongPress)
        {
            done = false;
            _active = false;
        }
        return;
    }

    switch (ev)
    {
        case Event::TurnCW:
            adjust(+1, state, refreshFlags);
            break;

        case Event::TurnCCW:
            adjust(-1, state, refreshFlags);
            break;

        case Event::Press:
            step++;
            if (step >= totalSteps)
            {
                finish(state, refreshFlags);
            }
            else
            {
                enterStep(state, refreshFlags);
            }
            break;

        case Event::LongPress:
            state.waveform  = savedWaveform;
            state.amplitude = savedAmplitude;
            state.freqOrder = savedFreqOrder;
            state.frequency = savedFrequency;
            state.dcOffset  = savedDcOffset;

            refreshFlags[0] = refreshFlags[1] = refreshFlags[2] = refreshFlags[3] = refreshFlags[4] = true;
            peripherals.update(state, refreshFlags);
            _active = false;
            break;

        case Event::None:
            break;
    }
}

void CalibrationWizard::adjust(int8_t dir, AppState& state, bool* refreshFlags)
{
    switch (calType)
    {
        case CalType::Offset:
            state.calOffset += dir;
            if (state.calOffset > 500)  { state.calOffset = 500; }
            if (state.calOffset < -500) { state.calOffset = -500; }
            refreshFlags[4] = true;
            break;

        case CalType::SineAmp:
            if (step == 0)
            {
                state.calSine1V += dir;
                if (state.calSine1V > 500)  { state.calSine1V = 500; }
                if (state.calSine1V < -500) { state.calSine1V = -500; }
            }
            else
            {
                state.calSine10V += dir;
                if (state.calSine10V > 500)  { state.calSine10V = 500; }
                if (state.calSine10V < -500) { state.calSine10V = -500; }
            }
            refreshFlags[1] = true;
            break;

        case CalType::SqwAmp:
            if (step == 0)
            {
                state.calSqw1V += dir;
                if (state.calSqw1V > 500)  { state.calSqw1V = 500; }
                if (state.calSqw1V < -500) { state.calSqw1V = -500; }
            }
            else
            {
                state.calSqw10V += dir;
                if (state.calSqw10V > 500)  { state.calSqw10V = 500; }
                if (state.calSqw10V < -500) { state.calSqw10V = -500; }
            }
            refreshFlags[1] = true;
            break;
    }

    peripherals.update(state, refreshFlags);
    render(state);
}

void CalibrationWizard::render(const AppState& state)
{
    char buf[80];
    uint8_t pos = 0;

    int16_t val;
    switch (calType)
    {
        case CalType::Offset:  val = state.calOffset; break;
        case CalType::SineAmp: val = (step == 0) ? state.calSine1V : state.calSine10V; break;
        case CalType::SqwAmp:  val = (step == 0) ? state.calSqw1V  : state.calSqw10V;  break;
    }

    switch (calType)
    {
        case CalType::Offset:
            buf[pos++] = '\x01';
            pos += sprintf(&buf[pos], "Cal Offset");
            buf[pos++] = '\x02';
            buf[pos++] = '\n';
            pos += sprintf(&buf[pos], "Vmeter->SigOut\nAdj Vout~0\n");
            break;

        case CalType::SineAmp:
            buf[pos++] = '\x01';
            pos += sprintf(&buf[pos], "Cal Sine");
            buf[pos++] = '\x02';
            buf[pos++] = '\n';
            pos += sprintf(&buf[pos], "Scope->SigOut\nAdj Vout=%sVpp\n",
                           (step == 0) ? "1" : "10");
            break;

        case CalType::SqwAmp:
            buf[pos++] = '\x01';
            pos += sprintf(&buf[pos], "Cal SQW");
            buf[pos++] = '\x02';
            buf[pos++] = '\n';
            pos += sprintf(&buf[pos], "Scope->SigOut\nAdj Vout=%sVpp\n",
                           (step == 0) ? "1" : "10");
            break;
    }

    buf[pos++] = '\x03';
    pos += sprintf(&buf[pos], "Offset: %d", val);
    buf[pos++] = '\x04';
    buf[pos] = '\0';

    display.draw_text(buf);

    Serial.print(F("[CAL] step="));
    Serial.print(step);
    Serial.print(F(" val="));
    Serial.println(val);
}

void CalibrationWizard::finish(AppState& state, bool* refreshFlags)
{
    CalStorage stored;
    stored.id         = EEPROM_MAGIC;
    stored.calOffset  = state.calOffset;
    stored.calSine1V  = state.calSine1V;
    stored.calSine10V = state.calSine10V;
    stored.calSqw1V   = state.calSqw1V;
    stored.calSqw10V  = state.calSqw10V;
    EEPROM.put(EEPROM_ADDR, stored);

    state.waveform  = savedWaveform;
    state.amplitude = savedAmplitude;
    state.freqOrder = savedFreqOrder;
    state.frequency = savedFrequency;
    state.dcOffset  = savedDcOffset;

    refreshFlags[0] = refreshFlags[1] = refreshFlags[2] = refreshFlags[3] = refreshFlags[4] = true;
    peripherals.update(state, refreshFlags);

    char buf[64];
    uint8_t pos = 0;
    buf[pos++] = '\x01';
    pos += sprintf(&buf[pos], "Calibration");
    buf[pos++] = '\x02';
    buf[pos++] = '\n';
    buf[pos++] = '\n';
    buf[pos++] = '\x01';
    pos += sprintf(&buf[pos], "Done!");
    buf[pos++] = '\x02';
    buf[pos] = '\0';
    display.draw_text(buf);

    done = true;
}
