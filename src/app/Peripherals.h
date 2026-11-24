/*
 * Peripherals.h
 * Scope: Bridges AppState to hardware drivers.
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include <Arduino.h>
#include <AD9833.h>
#include "pwm.h"
#include "../drivers/AD5443.h"
#include "../drivers/AD5625R.h"
#include "Messenger.h"

class Peripherals
{
    public:
        Peripherals(AD9833& waveformGen, AD5443& amplitudeCtrl,
                    AD5625R& dcOffsetCtrl, PwmOut& pwmOut,
                    uint8_t sqwSwitchPin, Messenger& messenger);

        void init();
        void run();

    private:
        AD9833&   waveformGen;
        AD5443&   amplitudeCtrl;
        AD5625R&  dcOffsetCtrl;
        PwmOut&   pwmOut;
        uint8_t   sqwSwitchPin;
        bool      pwmRunning;
        Messenger& messenger;
        AppState  applied;
        SpiCanBusOwner lastOwner;
        uint32_t  lastAdcMs;

        void beginSignalGen();
        void beginPwm();
        void beginAdc();

        void updateWave(const AppState& state);
        void updateFrequency(const AppState& state);
        void updateAmplitude(const AppState& state);
        void updateDcOffset(const AppState& state);
        void updateAdc();
        void applyPwm(const AppState& state);
};

#endif
