/*
 * Controler.h
 * Scope: Owns the encoder, runs the FSM, calls menu/display.
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CONTROLER_H
#define CONTROLER_H

#include "../drivers/Encoder.h"
#include "../drivers/Display.h"
#include "Menu.h"
#include "Peripherals.h"
#include "CalibrationWizard.h"
#include "CanMessenger.h"

struct AppState {
    uint8_t waveform;
    uint16_t amplitude;
    uint8_t freqOrder;
    float frequency;
    int16_t dcOffset;

    bool pwmEnabled;
    uint8_t pwmFreqOrder;
    float pwmFrequency;
    uint8_t dutyCycle;

    int16_t calOffset;
    int16_t calSine1V;
    int16_t calSine10V;
    int16_t calSqw1V;
    int16_t calSqw10V;
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
    P_CAN_MSG,
    P_COUNT
};

struct MenuNode {
    const char* label;
    const MenuNode* children;
    uint8_t numChildren;
    uint8_t paramIndex;
};

enum class Mode : uint8_t {
    SigGen,
    Can,
};

class Controler
{
    public:
        Controler(Encoder& encoder, Display& display, Peripherals& peripherals);
        void begin(uint8_t canRxPin);
        void tick();

    private:
        void process(Event ev);
        void adjust(int8_t dir);
        Mode detectMode();

        Mode mode;
        uint8_t canRxPin;

        Encoder& encoder;
        Display& display;
        Peripherals& peripherals;
        Menu menu;
        CalibrationWizard calWizard;
        CanMessenger canMessenger;

        AppState state;
        bool refreshFlags[P_COUNT];

        const MenuNode* current;
        uint8_t cursor;
        bool editing;
        uint8_t subCursor;

        const MenuNode* parentStack[4];
        uint8_t cursorStack[4];
        uint8_t depth;
};

#endif
