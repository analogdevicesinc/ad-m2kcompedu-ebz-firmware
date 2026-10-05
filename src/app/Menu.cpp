/*
 * Menu.cpp
 * Scope: Builds markup strings from menu state, calls Display::draw_text() to render.
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "Menu.h"
#include "Controler.h"
#include <string.h>

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
    }
    return true;
}

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
//highlight
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

Menu::Menu(Display& display)
    : display(display)
{
}

void Menu::formatValue(char* buf, uint8_t& pos, const Param& p, uint8_t pIdx)
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
            pos += sprintf(&buf[pos], "%d.%d", whole, frac);
        }
        else
        {
            pos += sprintf(&buf[pos], "%d", (int)val);
        }
        return;
    }

    int32_t val = readParam(p);

    if (p.names)
    {
        pos += sprintf(&buf[pos], "%s", p.names[val]);
    }
    else if (pIdx == P_AMPL || pIdx == P_DC)
    {
        int32_t whole = val / 1000;
        int32_t frac = (val < 0 ? -val : val) % 1000 / 100;
        pos += sprintf(&buf[pos], "%ld.%ld%s", (long)whole, (long)frac, p.unit ? p.unit : "");
    }
    else
    {
        pos += sprintf(&buf[pos], "%ld%s", (long)val, p.unit ? p.unit : "");
    }
}

void Menu::buildLine(char* line, const MenuNode* item, bool selected,
                     bool editing, uint8_t subCursor, const Param* params)
{
    uint8_t pos = 0;
    char valueBuf[32];
    uint8_t vpos = 0;

    if (!item->children)
    {
        const Param& p = params[item->paramIndex];

        if (p.isAction)
        {
            // Action items have no value; render the bare (optionally
            // highlighted) label so the selection box wraps it cleanly.
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
        // A group only shows a value (and therefore a "label: " separator)
        // if at least one child is a real parameter rather than an action.
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

        // Separator is emitted outside the highlight so the box wraps only
        // the label; skipped entirely for action-only groups (e.g. Calibrate).
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

void Menu::render(const MenuNode* current, uint8_t cursor,
                  bool editing, uint8_t subCursor, const Param* params)
{
    char buf[256];
    uint8_t pos = 0;
    char line[64];

    uint8_t wave = *((uint8_t*)params[P_WAVE].value);

    for (uint8_t i = 0; i < current->numChildren; i++)
    {
        const MenuNode* item = &current->children[i];
        bool selected = (i == cursor);

        bool struck = (wave == 1) &&
            ((!item->children && item->paramIndex == P_AMPL) ||
             (item->children && item->numChildren > 0
              && item->children[0].paramIndex == P_FREQ_VAL));

        buildLine(line, item, selected, editing, subCursor, params);

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
    display.draw_text(buf);
}
