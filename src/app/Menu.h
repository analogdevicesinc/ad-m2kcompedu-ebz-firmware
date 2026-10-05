/*
 * Menu.h
 * Scope: Translates menu state into formatted text for the Display driver.
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MENU_H
#define MENU_H

#include "../drivers/Display.h"

struct MenuNode;
struct Param;

class Menu
{
    public:
        Menu(Display& display);
        void render(const MenuNode* current, uint8_t cursor,
                    bool editing, uint8_t subCursor, const Param* params);

    private:
        Display& display;
        void formatValue(char* buf, uint8_t& pos, const Param& p, uint8_t pIdx);
        void buildLine(char* line, const MenuNode* item, bool selected,
                       bool editing, uint8_t subCursor, const Param* params);
};

#endif
