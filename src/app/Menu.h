/*
 * Menu.h
 * Scope: Owns the rotary encoder, menu navigation and rendering. Pumps the
 *        encoder, writes value changes to the shared state, draws the screen.
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MENU_H
#define MENU_H

#include "../drivers/Display.h"
#include "../drivers/Encoder.h"
#include "Messenger.h"

enum ParamIdx : uint8_t {
    P_WAVE = 0,
    P_AMPL,
    P_FREQ_ORD,
    P_FREQ_VAL,
    P_DC,
    P_PWM_EN,
    P_PWM_FREQ_ORD,
    P_PWM_FREQ_VAL,
    P_PWM_DUTY,
    P_CAL_OFF,
    P_CAL_SINE,
    P_CAL_SQW,
    P_ADC,
    P_CAN_MSG,
    P_VOLTMETER_CH1,
    P_VOLTMETER_CH2,
    P_COUNT
};

struct Param {
    void*        value;
    int32_t      min;
    int32_t      max;
    int16_t      step;
    bool         wrap;
    const char*  unit;
    const char** names;
    uint8_t      nameCount;
    bool         isAction;
    bool         isFloat;
};

struct MenuNode {
    const char*     label;
    const MenuNode* children;
    uint8_t         numChildren;
    uint8_t         paramIndex;
    bool            submenu;
};

class Menu
{
    public:
        Menu(Encoder& encoder, Display& display, Messenger& messenger);
        void init();
        void run();
        Param* paramTable() { return params; }

    private:
        void handleEvent(Event ev);
        void updateVoltmeterAdc();
        void process(Event ev);
        void adjust(int8_t dir);
        void buildScreen();
        void buildMenu();
        void buildCalibration(const CalView& cal);
        void buildCan(const CanState& can);
        void buildAdc();
        void draw();
        void showPopup(const char* text, uint16_t secs);
        bool popupActive();
        bool isValueGroup(const MenuNode* node);
        void formatValue(char* out, uint8_t& pos, const Param& p, uint8_t pIdx);
        void buildLine(char* line, const MenuNode* item, bool selected);

        Encoder&   encoder;
        Display&   display;
        Messenger& messenger;
        Param      params[P_COUNT];

        const MenuNode* current;
        uint8_t cursor;
        bool    editing;
        uint8_t subCursor;
        bool    viewingCan;
        uint8_t scrollTop;
        bool    viewingAdc;
        bool    voltmeterAdc;

        const MenuNode* parentStack[4];
        uint8_t cursorStack[4];
        uint8_t depth;

        char        buf[256];
        char        lastBuf[256];
        const char* popupText;
        uint32_t    popupUntil;
        uint32_t    lastDraw;
};

#endif
