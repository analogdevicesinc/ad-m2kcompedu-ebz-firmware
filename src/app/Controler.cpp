/*
 * Controler.cpp
 * Scope: Handles input events, manages navigation state,
 * dispatches actions to hardware and display layers.
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "Controler.h"
#include "../iio/iio_server.h"

static const char* waveNames[]  = { "OFF", "DC", "Sine", "Square", "Triangle" };
static const char* orderNames[] = { "Hz", "KHz", "MHz" };
static const char* onOffNames[] = { "OFF", "ON" };

// { label, children, numChildren, paramIndex }
static constexpr MenuNode freqChildren[] = {
    { "Value", nullptr, 0, P_FREQ_VAL },
    { "Order", nullptr, 0, P_FREQ_ORD },
};

static constexpr MenuNode siggenItems[] = {
    { "WaveForm",  nullptr,      0, P_WAVE },
    { "Vampl-pp",  nullptr,      0, P_AMPL },
    { "Frequency", freqChildren, 2, 0 },
    { "DC Offset", nullptr,      0, P_DC },
};

static constexpr MenuNode pwmFreqChildren[] = {
    { "Value", nullptr, 0, P_PWM_FREQ_VAL },
    { "Order", nullptr, 0, P_PWM_FREQ_ORD },
};

static constexpr MenuNode pwmItems[] = {
    { "PWM Out",   nullptr,         0, P_PWM_EN },
    { "Frequency", pwmFreqChildren, 2, 0 },
    { "Duty Cyc.", nullptr,         0, P_PWM_DUTY },
};

static constexpr MenuNode calItems[] = {
    { "Calibrate Offset",   nullptr, 0, P_CAL_OFF },
    { "Calibrate Sine Amp", nullptr, 0, P_CAL_SINE },
    { "Calibrate SQW Amp",  nullptr, 0, P_CAL_SQW },
};

static constexpr MenuNode mainItems[] = {
    { "Signal Gen", siggenItems, 4, 0 },
    { "PWM Gen",    pwmItems,    3, 0 },
    { "Calibrate",  calItems,    3, 0 },
};

static constexpr MenuNode root = { "Main", mainItems, 3, 0 };

static Param params[P_COUNT];

static bool isValueGroup(const MenuNode* node)
{
    if (!node->children || node->numChildren == 0)
    {
        return false;
    }
    for (uint8_t i = 0; i < node->numChildren; i++)
    {
        if (node->children[i].children != nullptr)
        {
            return false;
        }
        if (params[node->children[i].paramIndex].isAction)
        {
            return false;
        }
    }
    return true;
}

Controler::Controler(Encoder& encoder, Display& display, Peripherals& peripherals)
    : encoder(encoder), display(display), peripherals(peripherals), menu(display),
      calWizard(display, peripherals),
      current(&root), cursor(0), editing(false), subCursor(0), depth(0),
      //    {wave, ampl, fOrd, freq,   dc, pwmEn, pOrd, pFreq,  duty, calOff, sin1V, sin10V, sqw1V, sqw10V}
      state{0, 1000, 1, 1.0f, 0, false, 0, 100.0f, 50, 0, 0, 0, 0, 0},
      // refreshFlags[0..8]: wave, ampl, freqOrd, freqVal, dc, pwmEn, pwmOrd, pwmVal, duty
      refreshFlags{}
{
}

void Controler::begin()
{
    display.begin();
    encoder.begin();
    peripherals.begin();
    calWizard.loadFromEEPROM(state);

    //                    { value,              min,  max, step, wrap, unit,    names,      nCnt, action, float }
    params[P_WAVE]         = { &state.waveform,      0,      4,    1, true,  nullptr, waveNames,  5, false, false };
    params[P_AMPL]         = { &state.amplitude,     0,   5000,  100, false, "V",     nullptr,    0, false, false };
    params[P_FREQ_ORD]     = { &state.freqOrder,     0,      2,    1, false, nullptr, orderNames, 3, false, false };
    params[P_FREQ_VAL]     = { &state.frequency,     0,      0,    0, false, nullptr, nullptr,    0, false, true  };
    params[P_DC]           = { &state.dcOffset,  -2500,   2500,  100, false, "V",     nullptr,    0, false, false };
    params[P_PWM_EN]       = { &state.pwmEnabled,    0,      1,    1, true,  nullptr, onOffNames, 2, false, false };
    params[P_PWM_FREQ_ORD] = { &state.pwmFreqOrder,  0,      2,    1, false, nullptr, orderNames, 3, false, false };
    params[P_PWM_FREQ_VAL] = { &state.pwmFrequency,  0,      0,    0, false, nullptr, nullptr,    0, false, true  };
    params[P_PWM_DUTY]     = { &state.dutyCycle,     0,    100,    1, false, "%",     nullptr,    0, false, false };
    params[P_CAL_OFF]      = { nullptr,              0,      0,    0, false, nullptr, nullptr,    0, true,  false };
    params[P_CAL_SINE]     = { nullptr,              0,      0,    0, false, nullptr, nullptr,    0, true,  false };
    params[P_CAL_SQW]      = { nullptr,              0,      0,    0, false, nullptr, nullptr,    0, true,  false };

    menu.render(current, cursor, editing, subCursor, params);

    for (uint8_t i = 0; i <= P_PWM_DUTY; i++) // only prime the flags Peripherals::update() actually consumes.
    {
        refreshFlags[i] = true;
    }
    peripherals.update(state, refreshFlags);

    iio_server_begin(params, refreshFlags);
}

void Controler::tick()
{
    encoder.tick();

    Event ev = encoder.poll();
    if (ev != Event::None)
    {
        process(ev);
    }

    /* Refresh display when settings were adjusted, either via UI or IIO */
    if (!calWizard.active())
    {
        for (uint8_t i = 0; i < P_COUNT; i++)
        {
            if (refreshFlags[i])
            {
                menu.render(current, cursor, editing, subCursor, params);
                break;
            }
        }
    }

    /* Picks up refreshFlags set by IIO writes, not just UI events. */
    peripherals.update(state, refreshFlags);
}

static void adjustFreqValue(float& freq, uint8_t& order, int8_t dir, float minHz)
{
    float step;
    float ref = (dir > 0) ? freq : freq - 0.01f;
    if (ref < 10.0f)
    {
        step = 0.1f;
    }
    else if (ref < 100.0f)
    {
        step = 1.0f;
    }
    else
    {
        step = 10.0f;
    }

    if (dir > 0)
    {
        if (order == 2 && freq >= 2.0f)
        {
            return;
        }
        freq += step;
        if (freq >= 1000.0f && order < 2)
        {
            freq = 1.0f;
            order++;
        }
        if (order == 2 && freq > 2.0f)
        {
            freq = 2.0f;
        }
    }
    else
    {
        freq -= step;
        if (freq < 1.0f)
        {
            if (order > 0)
            {
                freq = 990.0f;
                order--;
            }
            else
            {
                freq = minHz;
            }
        }
        if (order == 0 && freq < minHz)
        {
            freq = minHz;
        }
    }
}

static void adjustFreqOrder(float& freq, uint8_t& order, int8_t dir, float minHz)
{
    if (dir > 0)
    {
        freq *= 10.0f;
        if (freq >= 1000.0f && order < 2)
        {
            freq /= 1000.0f;
            order++;
        }
        if (order == 2 && freq > 2.0f)
        {
            freq = 2.0f;
        }
    }
    else
    {
        freq /= 10.0f;
        if (freq < 1.0f)
        {
            if (order > 0)
            {
                freq *= 1000.0f;
                order--;
            }
            else
            {
                freq = minHz;
            }
        }
        if (order == 0 && freq < minHz)
        {
            freq = minHz;
        }
    }
}

void Controler::adjust(int8_t dir)
{
    const MenuNode* item = &current->children[cursor];
    uint8_t pIdx;

    if (isValueGroup(item))
    {
        pIdx = item->children[subCursor].paramIndex;
    }
    else
    {
        pIdx = item->paramIndex;
    }

    const Param* p = &params[pIdx];
    if (p->isAction)
    {
        return;
    }

    if (pIdx == P_FREQ_VAL)
    {
        adjustFreqValue(state.frequency, state.freqOrder, dir, 10.0f);
        refreshFlags[P_FREQ_VAL] = true;
        refreshFlags[P_FREQ_ORD] = true;
        return;
    }
    if (pIdx == P_PWM_FREQ_VAL)
    {
        adjustFreqValue(state.pwmFrequency, state.pwmFreqOrder, dir, 1.0f);
        refreshFlags[P_PWM_FREQ_VAL] = true;
        refreshFlags[P_PWM_FREQ_ORD] = true;
        return;
    }

    if (pIdx == P_FREQ_ORD)
    {
        adjustFreqOrder(state.frequency, state.freqOrder, dir, 10.0f);
        refreshFlags[P_FREQ_VAL] = true;
        refreshFlags[P_FREQ_ORD] = true;
        return;
    }
    if (pIdx == P_PWM_FREQ_ORD)
    {
        adjustFreqOrder(state.pwmFrequency, state.pwmFreqOrder, dir, 1.0f);
        refreshFlags[P_PWM_FREQ_VAL] = true;
        refreshFlags[P_PWM_FREQ_ORD] = true;
        return;
    }

    refreshFlags[pIdx] = true;

    int32_t val;
    if (p->min >= 0 && p->max <= 255)
    {
        val = *((uint8_t*)p->value);
    }
    else if (p->min >= 0 && p->max <= 65535)
    {
        val = *((uint16_t*)p->value);
    }
    else if (p->min < 0)
    {
        val = *((int16_t*)p->value);
    }
    else
    {
        val = *((uint32_t*)p->value);
    }

    val += dir * p->step;

    if (p->wrap)
    {
        if (val > p->max) { val = p->min; }
        if (val < p->min) { val = p->max; }
    }
    else
    {
        if (val > p->max) { val = p->max; }
        if (val < p->min) { val = p->min; }
    }

    if (p->min >= 0 && p->max <= 255)
    {
        *((uint8_t*)p->value) = (uint8_t)val;
    }
    else if (p->min >= 0 && p->max <= 65535)
    {
        *((uint16_t*)p->value) = (uint16_t)val;
    }
    else if (p->min < 0)
    {
        *((int16_t*)p->value) = (int16_t)val;
    }
    else
    {
        *((uint32_t*)p->value) = (uint32_t)val;
    }
}

void Controler::process(Event ev)
{
    if (calWizard.active())
    {
        calWizard.process(ev, state, refreshFlags);
        if (!calWizard.active())
        {
            current = &root;
            cursor = 0;
            depth = 0;
            menu.render(current, cursor, editing, subCursor, params);
        }
        return;
    }

    switch (ev)
    {
        case Event::TurnCW:
            if (editing)
            {
                adjust(+1);
            }
            else
            {
                cursor = cursor + 1;
                if (cursor > current->numChildren - 1)
                {
                    cursor = 0;
                }
            }
            break;

        case Event::TurnCCW:
            if (editing)
            {
                adjust(-1);
            }
            else
            {
                if (cursor == 0)
                {
                    cursor = current->numChildren - 1;
                }
                else
                {
                    cursor--;
                }
            }
            break;

        case Event::Press:
            if (editing)
            {
                const MenuNode* item = &current->children[cursor];
                if (isValueGroup(item))
                {
                    subCursor++;
                    if (subCursor >= item->numChildren)
                    {
                        editing = false;
                        subCursor = 0;
                    }
                }
                else
                {
                    editing = false;
                }
            }
            else
            {
                const MenuNode* child = &current->children[cursor];

                bool blocked = false;
                if (state.waveform == 1)
                {
                    if (!child->children && child->paramIndex == P_AMPL)
                    {
                        blocked = true;
                    }
                    if (child->children && child->numChildren > 0
                        && child->children[0].paramIndex == P_FREQ_VAL)
                    {
                        blocked = true;
                    }
                }
                if (blocked)
                {
                    break;
                }

                if (child->children)
                {
                    if (isValueGroup(child))
                    {
                        editing = true;
                        subCursor = 0;
                    }
                    else
                    {
                        parentStack[depth] = current;
                        cursorStack[depth] = cursor;
                        depth++;
                        current = child;
                        cursor = 0;
                    }
                }
                else
                {
                    uint8_t pIdx = child->paramIndex;
                    if (params[pIdx].isAction)
                    {
                        if (pIdx == P_CAL_OFF)
                        {
                            calWizard.start(CalType::Offset, state, refreshFlags);
                        }
                        else if (pIdx == P_CAL_SINE)
                        {
                            calWizard.start(CalType::SineAmp, state, refreshFlags);
                        }
                        else if (pIdx == P_CAL_SQW)
                        {
                            calWizard.start(CalType::SqwAmp, state, refreshFlags);
                        }
                        return;
                    }
                    editing = true;
                }
            }
            break;

        case Event::LongPress:
            current = &root;
            cursor = 0;
            editing = false;
            subCursor = 0;
            depth = 0;
            break;

        case Event::None:
            break;
    }

    menu.render(current, cursor, editing, subCursor, params);
}
