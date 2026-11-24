/*
 * CalibrationWizard.h
 * Scope: Headless calibration model. Steps the signal generator through the
 *        shared state, trims cal values, persists them to EEPROM, and publishes
 *        its view state to the bus. Does no rendering of its own.
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CALIBRATION_WIZARD_H
#define CALIBRATION_WIZARD_H

#include <Arduino.h>
#include "Messenger.h"

class CalibrationWizard
{
    public:
        CalibrationWizard(Messenger& messenger);

        void init();
        void run();

    private:
        void    loadFromEEPROM();
        void    start(CalType type);
        void    process(CalCmd cmd);
        void    enterStep();
        void    adjust(int8_t dir);
        void    finish();
        void    publishView();
        int16_t currentValue() const;

        Messenger& messenger;

        bool    _active;
        bool    done;
        CalType calType;
        uint8_t step;
        uint8_t totalSteps;

        uint8_t  savedWaveform;
        uint16_t savedAmplitude;
        uint8_t  savedFreqOrder;
        float    savedFrequency;
        int16_t  savedDcOffset;
};

#endif
