/*
 * Menu.cpp
 * Scope: Menu navigation FSM + rendering. Reads/writes AppState on the bus.
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include "Menu.h"

static constexpr uint32_t DRAW_INTERVAL_MS = 33;
static constexpr uint8_t  MENU_VISIBLE_ROWS = 4;

static const char* waveNames[]  = { "OFF", "DC", "Sine", "Square", "Triangle" };
static const char* orderNames[] = { "Hz", "KHz", "MHz" };
static const char* onOffNames[] = { "OFF", "ON" };

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

static constexpr MenuNode vmeterItems[] = {
    { "A2",   nullptr, 0, P_VOLTMETER_CH1 },
    { "A3",   nullptr, 0, P_VOLTMETER_CH2 },
};

static constexpr MenuNode mainItems[] = {
    { "Signal Gen",    siggenItems, 4, 0 },
    { "PWM Gen",       pwmItems,    3, 0 },
    { "Calibrate",     calItems,    3, 0 },
    { "CAN Messenger", nullptr,     0, P_CAN_MSG },
    { "Voltmeter",     vmeterItems, 2, 0, true },
};

static constexpr MenuNode root = { "Main", mainItems, 5, 0 };

static int32_t readParam(const Param& p)
{
    if (p.min >= 0 && p.max <= 255)
    {
        return *((uint8_t*)p.value);
    }
    if (p.min >= 0 && p.max <= 65535)
    {
        return *((uint16_t*)p.value);
    }
    if (p.min < 0)
    {
        return *((int16_t*)p.value);
    }
    return *((uint32_t*)p.value);
}

static void append(char* buf, uint8_t& pos, const char* str)
{
    pos += sprintf(&buf[pos], "%s", str);
}

static void appendHL(char* buf, uint8_t& pos, const char* str)
{
    buf[pos++] = '\x03';
    append(buf, pos, str);
    buf[pos++] = '\x04';
}

static void appendStrike(char* buf, uint8_t& pos, const char* str)
{
    buf[pos++] = '\x05';
    append(buf, pos, str);
    buf[pos++] = '\x06';
}

static void appendCentered(char* buf, uint8_t& pos, const char* line)
{
    buf[pos++] = '\x01';
    append(buf, pos, line);
    buf[pos++] = '\x02';
    buf[pos++] = '\n';
}

static void adjustFreqValue(float& freq, uint8_t& order, int8_t dir, float minHz)
{
    float step;
    float ref = freq;
    if (dir <= 0)
    {
        ref = freq - 0.01f;
    }
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

Menu::Menu(Encoder& encoder, Display& display, Messenger& messenger)
    : encoder(encoder), display(display), messenger(messenger), params{},
      current(&root), cursor(0), editing(false), subCursor(0),
      viewingCan(false), scrollTop(0), depth(0),
      buf{}, lastBuf{}, popupText(nullptr), popupUntil(0), lastDraw(0)
{
}

void Menu::init()
{
    display.begin();
    encoder.begin();

    AppState& s = messenger.state();

    params[P_WAVE]          = { &s.waveform,     0,    4,   1, true,  nullptr, waveNames,  5, false, false };
    params[P_AMPL]          = { &s.amplitude,    0, 5000, 100, false, "V",     nullptr,    0, false, false };
    params[P_FREQ_ORD]      = { &s.freqOrder,    0,    2,   1, false, nullptr, orderNames, 3, false, false };
    params[P_FREQ_VAL]      = { &s.frequency,    0,    0,   0, false, nullptr, nullptr,    0, false, true  };
    params[P_DC]            = { &s.dcOffset, -2500, 2500, 100, false, "V",     nullptr,    0, false, false };
    params[P_PWM_EN]        = { &s.pwmEnabled,   0,    1,   1, true,  nullptr, onOffNames, 2, false, false };
    params[P_PWM_FREQ_ORD]  = { &s.pwmFreqOrder, 0,    2,   1, false, nullptr, orderNames, 3, false, false };
    params[P_PWM_FREQ_VAL]  = { &s.pwmFrequency, 0,    0,   0, false, nullptr, nullptr,    0, false, true  };
    params[P_PWM_DUTY]      = { &s.dutyCycle,    0,  100,   1, false, "%",     nullptr,    0, false, false };
    params[P_CAL_OFF]       = { nullptr,         0,    0,   0, false, nullptr, nullptr,    0, true,  false };
    params[P_CAL_SINE]      = { nullptr,         0,    0,   0, false, nullptr, nullptr,    0, true,  false };
    params[P_CAL_SQW]       = { nullptr,         0,    0,   0, false, nullptr, nullptr,    0, true,  false };
    params[P_CAN_MSG]       = { nullptr,         0,    0,   0, false, nullptr, nullptr,    0, true,  false };
    params[P_VOLTMETER_CH1] = { &s.voltmeterMv[0],     0,    5000,   0, false, "V", nullptr, 0, false,  false };
    params[P_VOLTMETER_CH2] = { &s.voltmeterMv[1],     0,    5000,   0, false, "V", nullptr, 0, false,  false };
    buildScreen();
    draw();
    lastDraw = millis();
}

void Menu::run()
{
    encoder.tick();
    Event ev = encoder.poll();

    handleEvent(ev);

    uint32_t now = millis();
    if (ev != Event::None || now - lastDraw >= DRAW_INTERVAL_MS)
    {
        lastDraw = now;
        buildScreen();
        draw();
    }
}

void Menu::handleEvent(Event ev)
{
    if (popupActive())
    {
        return;
    }

    const CalView& cal = messenger.calView();
    if (cal.active)
    {
        if (ev != Event::None)
        {
            CalCmd cmd = CalCmd::None;
            if (ev == Event::TurnCW)
            {
                cmd = CalCmd::AdjustUp;
            }
            else if (ev == Event::TurnCCW)
            {
                cmd = CalCmd::AdjustDown;
            }
            else if (ev == Event::Press)
            {
                cmd = CalCmd::Next;
            }
            else if (ev == Event::LongPress)
            {
                cmd = CalCmd::Abort;
            }
            messenger.postCalCommand(cmd);
        }
        return;
    }

    if (viewingCan)
    {
        if (ev == Event::LongPress)
        {
            viewingCan = false;
            messenger.setCanActive(false);
        }
        return;
    }

    if (ev != Event::None)
    {
        process(ev);
    }
}

void Menu::buildScreen()
{
    if (popupActive())
    {
        strcpy(buf, popupText);
        return;
    }

    const CalView& cal = messenger.calView();
    if (cal.active)
    {
        buildCalibration(cal);
        return;
    }

    if (viewingCan)
    {
        buildCan(messenger.canState());
        return;
    }

    buildMenu();
}

void Menu::draw()
{
    if (strcmp(buf, lastBuf) == 0)
    {
        return;
    }
    strcpy(lastBuf, buf);
    display.draw_text(buf);
}

void Menu::showPopup(const char* text, uint16_t secs)
{
    popupText  = text;
    popupUntil = millis() + (uint32_t)secs * 1000UL;
}

bool Menu::popupActive()
{
    return popupUntil != 0 && millis() < popupUntil;
}

void Menu::buildCalibration(const CalView& cal)
{
    if (cal.done)
    {
        strcpy(buf, CEN "Calibration" CEN_E "\n\n" CEN "Saved!" CEN_E);
        return;
    }

    switch (cal.type)
    {
        case CalType::Offset:
            sprintf(buf, CEN "Cal Offset" CEN_E "\n"
                         "Vmeter->SigOut\n"
                         "Adj Vout ~ 0\n"
                         HL "val: %d" HL_E, cal.value);
            break;
        case CalType::SineAmp:
        {
            const char* vpp = "1";
            if (cal.step != 0)
            {
                vpp = "10";
            }
            sprintf(buf, CEN "Cal Sine" CEN_E "\n"
                         "Scope->SigOut\n"
                         "Adj Vout = %sVpp\n"
                         HL "val: %d" HL_E,
                    vpp, cal.value);
            break;
        }
        case CalType::SqwAmp:
        {
            const char* vpp = "1";
            if (cal.step != 0)
            {
                vpp = "10";
            }
            sprintf(buf, CEN "Cal SQW" CEN_E "\n"
                         "Scope->SigOut\n"
                         "Adj Vout = %sVpp\n"
                         HL "val: %d" HL_E,
                    vpp, cal.value);
            break;
        }
    }
}

void Menu::buildCan(const CanState& can)
{
    if (!can.active)
    {
        sprintf(buf, CEN "CAN Bridge" CEN_E "\n\n"
                     CEN "inactive" CEN_E "\n"
                     CEN "fit jumper P22" CEN_E);
    }
    else
    {
        const char* status = "closed";
        if (can.open)
        {
            status = "OPEN";
        }
        sprintf(buf, CEN "CAN Bridge" CEN_E "\n\n"
                     CEN "%luk" CEN_E "\n"
                     CEN "%s" CEN_E,
                (unsigned long)(can.bitrate / 1000),
                status);
    }
}

bool Menu::isValueGroup(const MenuNode* node)
{
    if (!node->children || node->numChildren == 0 || node->submenu)
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

void Menu::adjust(int8_t dir)
{
    AppState& s = messenger.state();
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

    if (messenger.readSpiCanBusOwner() == SpiCanBusOwner::Can)
    {
        if (pIdx == P_WAVE || pIdx == P_AMPL ||
            pIdx == P_FREQ_VAL || pIdx == P_FREQ_ORD)
        {
            showPopup(CEN "CAN active" CEN_E "\n\n"
                      CEN "Remove the P22, P21 jumpers" CEN_E, 2);
            return;
        }
    }

    if (pIdx == P_FREQ_VAL)
    {
        adjustFreqValue(s.frequency, s.freqOrder, dir, 10.0f);
        return;
    }
    if (pIdx == P_PWM_FREQ_VAL)
    {
        adjustFreqValue(s.pwmFrequency, s.pwmFreqOrder, dir, 1.0f);
        return;
    }

    if (pIdx == P_FREQ_ORD)
    {
        adjustFreqOrder(s.frequency, s.freqOrder, dir, 10.0f);
        return;
    }
    if (pIdx == P_PWM_FREQ_ORD)
    {
        adjustFreqOrder(s.pwmFrequency, s.pwmFreqOrder, dir, 1.0f);
        return;
    }

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

void Menu::process(Event ev)
{
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
                if (messenger.state().waveform == 1)
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
                        if (pIdx == P_CAL_OFF || pIdx == P_CAL_SINE || pIdx == P_CAL_SQW)
                        {
                            if (messenger.readSpiCanBusOwner() == SpiCanBusOwner::Can)
                            {
                                showPopup(CEN "CAN active" CEN_E "\n\n"
                                          CEN "Remove jumpers" CEN_E, 2);
                                return;
                            }

                            CalType type = CalType::SqwAmp;
                            if (pIdx == P_CAL_OFF)
                            {
                                type = CalType::Offset;
                            }
                            else if (pIdx == P_CAL_SINE)
                            {
                                type = CalType::SineAmp;
                            }
                            messenger.requestCalibration(type);
                        }
                        else if (pIdx == P_CAN_MSG)
                        {
                            viewingCan = true;
                            messenger.setCanActive(true);
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
}

void Menu::formatValue(char* out, uint8_t& pos, const Param& p, uint8_t pIdx)
{
    if (p.isAction)
    {
        return;
    }

    if (p.isFloat)
    {
        float val = *((float*)p.value);
        if (val < 10.0f)
        {
            int16_t whole = (int16_t)val;
            int16_t frac = (int16_t)((val - (float)whole) * 10.0f);
            if (frac < 0)
            {
                frac = -frac;
            }
            pos += sprintf(&out[pos], "%d.%d", whole, frac);
        }
        else
        {
            pos += sprintf(&out[pos], "%d", (int)val);
        }
        return;
    }

    int32_t val = readParam(p);

    if (p.names)
    {
        pos += sprintf(&out[pos], "%s", p.names[val]);
    }
    else if (pIdx == P_AMPL || pIdx == P_DC)
    {
        int32_t whole = val / 1000;
        int32_t absVal = val;
        if (val < 0)
        {
            absVal = -val;
        }
        int32_t frac = absVal % 1000 / 100;
        const char* unit = "";
        if (p.unit)
        {
            unit = p.unit;
        }
        pos += sprintf(&out[pos], "%ld.%ld%s", (long)whole, (long)frac, unit);
    }
    else if (pIdx == P_VOLTMETER_CH1 || pIdx == P_VOLTMETER_CH2)
    {
        const char* unit = "";
        if (p.unit)
        {
            unit = p.unit;
        }
        pos += sprintf(&out[pos], "%ld.%02ld%s", (long)(val / 1000), (long)(val % 1000 / 10), unit);
    }
    else
    {
        const char* unit = "";
        if (p.unit)
        {
            unit = p.unit;
        }
        pos += sprintf(&out[pos], "%ld%s", (long)val, unit);
    }
}

void Menu::buildLine(char* line, const MenuNode* item, bool selected)
{
    uint8_t pos = 0;
    char valueBuf[32];
    uint8_t vpos = 0;

    if (!item->children)
    {
        const Param& p = params[item->paramIndex];

        if (p.isAction)
        {
            if (selected)
            {
                appendHL(line, pos, item->label);
            }
            else
            {
                append(line, pos, item->label);
            }
        }
        else
        {
            formatValue(valueBuf, vpos, p, item->paramIndex);
            valueBuf[vpos] = '\0';

            if (selected && editing)
            {
                append(line, pos, item->label);
                append(line, pos, ": ");
                appendHL(line, pos, valueBuf);
            }
            else if (selected)
            {
                appendHL(line, pos, item->label);
                append(line, pos, ": ");
                append(line, pos, valueBuf);
            }
            else
            {
                append(line, pos, item->label);
                append(line, pos, ": ");
                append(line, pos, valueBuf);
            }
        }
    }
    else if (isValueGroup(item))
    {
        bool hasValue = false;
        for (uint8_t j = 0; j < item->numChildren; j++)
        {
            if (!params[item->children[j].paramIndex].isAction)
            {
                hasValue = true;
                break;
            }
        }

        if (selected && !editing)
        {
            appendHL(line, pos, item->label);
        }
        else
        {
            append(line, pos, item->label);
        }

        if (hasValue)
        {
            append(line, pos, ": ");
        }

        bool first = true;
        for (uint8_t j = 0; j < item->numChildren; j++)
        {
            uint8_t cpIdx = item->children[j].paramIndex;
            if (params[cpIdx].isAction)
            {
                continue;
            }

            if (!first)
            {
                line[pos++] = ' ';
            }
            first = false;

            vpos = 0;
            formatValue(valueBuf, vpos, params[cpIdx], cpIdx);
            valueBuf[vpos] = '\0';

            if (selected && editing && j == subCursor)
            {
                appendHL(line, pos, valueBuf);
            }
            else
            {
                append(line, pos, valueBuf);
            }
        }
    }
    else
    {
        if (selected)
        {
            appendHL(line, pos, item->label);
        }
        else
        {
            append(line, pos, item->label);
        }
    }

    line[pos] = '\0';
}

void Menu::buildMenu()
{
    uint8_t pos = 0;
    char line[64];

    uint8_t wave = *((uint8_t*)params[P_WAVE].value);

    if (cursor < scrollTop)
    {
        scrollTop = cursor;
    }
    else if (cursor >= scrollTop + MENU_VISIBLE_ROWS)
    {
        scrollTop = cursor - MENU_VISIBLE_ROWS + 1;
    }

    uint8_t end = scrollTop + MENU_VISIBLE_ROWS;
    if (end > current->numChildren)
    {
        end = current->numChildren;
    }

    for (uint8_t i = scrollTop; i < end; i++)
    {
        const MenuNode* item = &current->children[i];
        bool selected = (i == cursor);

        bool struck = (wave == 1) &&
            ((!item->children && item->paramIndex == P_AMPL) ||
             (item->children && item->numChildren > 0
              && item->children[0].paramIndex == P_FREQ_VAL));

        buildLine(line, item, selected);

        if (struck)
        {
            char strikeBuf[64];
            uint8_t spos = 0;
            appendStrike(strikeBuf, spos, line);
            strikeBuf[spos] = '\0';
            appendCentered(buf, pos, strikeBuf);
        }
        else
        {
            appendCentered(buf, pos, line);
        }
    }

    buf[pos] = '\0';
}
