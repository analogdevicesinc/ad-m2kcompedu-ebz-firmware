/*
 * CalibrationWizard.h
 * Scope: Takes over input/display during calibration.
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CALIBRATION_WIZARD_H
#define CALIBRATION_WIZARD_H

#include <Arduino.h>
#include "../drivers/Encoder.h"
#include "../drivers/Display.h"

struct AppState;
class Peripherals;

enum class CalType : uint8_t {
    Offset,
    SineAmp,
    SqwAmp
};

class CalibrationWizard
{
    public:
        CalibrationWizard(Display& display, Peripherals& peripherals);

        void start(CalType type, AppState& state, bool* refreshFlags);
        void process(Event ev, AppState& state, bool* refreshFlags);
        bool active() const { return _active; }
        void loadFromEEPROM(AppState& state);

    private:
        void enterStep(AppState& state, bool* refreshFlags);
        void adjust(int8_t dir, AppState& state, bool* refreshFlags);
        void render(const AppState& state);
        void finish(AppState& state, bool* refreshFlags);

        Display& display;
        Peripherals& peripherals;

        bool _active;
        bool done;
        CalType calType;
        uint8_t step;
        uint8_t totalSteps;

        uint8_t savedWaveform;
        uint16_t savedAmplitude;
        uint8_t savedFreqOrder;
        float savedFrequency;
        int16_t savedDcOffset;

    friend class Controler;
};

#endif
